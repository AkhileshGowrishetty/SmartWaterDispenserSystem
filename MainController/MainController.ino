/*
 * Smart Water Dispenser System
 * 
 * HARDWARE:
 *  - ARDUINO UNO
 *  - 20X4 LCD DISPLAY WITH I2C CONTROLLER
 *  - DHT11 TEMPERATURE AND HUMIDITY
 *  - CAPACITIVE SOIL MOISTURE SENSOR
 *  - YF-S201 WATER FLOW SENSOR
 *  - FLOAT LEVEL CONTROL SWITCH (WATER LEVEL SWITCH)
 *  - PUSH BUTTON FOR EMERGENCY STOP
 *  - 4 CHANNEL RELAY
 *  - ARDUINO PRO MICRO
 *  - 4X3 KEYPAD MATRIX
 *  
 * LIBRARIES:
 *  - Wire Library for I2C Communication.
 *  - LiquidCrystal_I2C Library for controlling LCD Display and I2C controller.
 *  - DHT Library for reading Data from DHT11 Sensor.
 * 
 * - An Arduino Pro Micro has been used to interface the Keypad with the controller (Arduino Uno).
 * - I2C communication is used to interface the keypad. This was used as 7 pins are needed to interface the keypad.
 * - Both the I2C Controller for LCD Display and Arduino Pro Micro uses the same pins for I2C bus.
 * - The Pump will not switch ON if all Valves are closed. In this case, open atleast one Valve and then switch ON the Pump.
 * - The Pump will switch OFF if the last open Valve is closed.
 * - The delay between Pump and Valve Control is selected using the "DELAY_TIME" macro.
 * - The Arduino Pro Micro sends an interrupt signal to the controller. This is to prevent frequent polling of the keypad for new message.
 * - The controller acknowledges by sending a request using I2C and the Arduino Pro Micro sends the keypad data to the controller.
 *
 * Keypad Control:
 *  "1" - Valve 1 ON/OFF
 *  "2" - Valve 2 ON/OFF
 *  "3" - Valve 3 ON/OFF
 *  "4" - Pump 1 ON/OFF
 *  "*" - All Valves and Pumps OFF
 *  "#" - All Valves and Pumps ON
 */



#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <AltSoftSerial.h>


// Pin definitions
#define WATER_FLOW_SENSOR   2
#define KEYPAD_INT          3
#define EMERGENCY_STOP      4
#define WATER_LEVEL_SENSOR  5
#define RAINFALL_SENSOR     6
#define DHT_PIN             7
#define VALVE1              10
#define VALVE2              11
#define VALVE3              12
#define PUMP1               13
#define SOIL_SENSOR         14
#define GSM_PIN             17

// Constants
#define KEYPAD_ADDRESS 0x08
#define DELAY_TIME 500
#define NETWORK_WAIT_TIME 10000

// Calibration Constants
#define WATERFLOW_CALIBRATION_FACTOR 7.5
#define SOIL_MOISTURE_LOW   680
#define SOIL_MOISTURE_HIGH  230

// Keywords for Help and Status messages
#define helpCommand       "help"
#define statusCommand     "status"
#define allON             "on"
#define allOFF            "off"
// Enter the phone number below.
#define phoneNumber       "+91xxxxxxxxxx"
#define emergencyMessage  "Emergency Stop is pressed."
#define waterLevelMessage "Water Level is LOW."
#define rainfallMessage   "It is Raining."

// Custom variable for Future Status of Valves
typedef enum {
  ALL_OFF,
  ONE_ON,
  TWO_ON,
  ALL_ON
} valveStatus_t;

// Custom variable for the change in State of the Pump
typedef enum {
  OFF_TO_ON = -1,
  NO_CHANGE,
  ON_TO_OFF
} stateChange_t;

// Custom variable for parsing the SMS from GSM modem.
enum _parseState {
  PS_DETECT_MSG_TYPE,

  PS_COMMAND_ECHO,
  
  PS_CMT_NUMBER,
  PS_CMT_NAME,
  PS_CMT_DATE,
  PS_CMT_CONTENT
  
};

/* 
 * The following are "volatile" variables as they are used in Interrupt Service Routines (ISR).
 * ISRs need to be short and fast. Further interrupts are disabled in ISRs.
 * Serial print can not be used as they require interrupts.
 */

// Variables to store status for interrupts. 
volatile bool intKeypad = false;
volatile bool emergencyStopPressed = false;
volatile bool waterLevel = false;
volatile bool rainfall = false;
volatile uint8_t currentPIND;
volatile unsigned long waterFlowPulse = 0;

// Boolean variable to check if manual control or automatic control.
bool automatic = false;
// An array to hold the Future state of Valves and Pump.
// {Valve1, Valve2, Valve3, Pump1}
bool valvePumpState[4] = {0, 0, 0, 0};
// An array to hold the Current state of Valves and Pump.
bool valvePump[4] = {0, 0, 0, 0};
// A character array to hold the Temperature and Humidity values as Strings.
char sensorData[2][8] = {
  "----",
  "----"
};
// A character array to hold Flow Rate value as String.
char flowRateData[5];
// A character array to store the raw message from the GSM modem.
char bufferSMS[100]; 
// A character array to store the parsed message from the GSM modem.
char messageSMS[30];
// Variable to index through the raw message.
unsigned int bufferIndex = 0;
// Boolean variable to check if there is new message.
bool newMessage= false;

// An array for the custom character.
byte degreeC[] = {0x1C, 0x14, 0x1C, 0x03, 0x04, 0x04, 0x04, 0x03};
// Unsigned 8bit integer to store Soil Moisture Level as percentage.
uint8_t soilMoistureLevel;
// Float type to hold the water flow rate.
float flowRate = 0.0;
// Unsigned 16bit integer to store the flow rate in millilitres.
unsigned int flowMilliLitres = 0;
// Unsigned 32bit integer to store the total amount of water flowed from the start of the controller.
unsigned long totalMilliLitres = 0;

// The following variables are used for timing of different processess without blocking.

// For LCD Display updation process.
unsigned long lcdUpdateTime;
// For Sensor Data updation process.
unsigned long sensorUpdateTime;
// For Valve and Pump Control process.
unsigned long valvePumpUpdateTime;
// For Flow Rate calculation process.
unsigned long flowUpdateTime = 0;

// Declaration of the custom variable that holds Future status of the Valves.
valveStatus_t valveStatus = ALL_OFF;
// Declaration of the custom variable that holds the parse state.
byte state = PS_DETECT_MSG_TYPE;

// Object initialization for LCD Display. The I2C Address of the I2C controller is 0x27 (HEX).
// The other parameters are pin mappings of the I2C controller to the LCD Display.
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
// Object initialization for DHT11 sensor. The parameters are the pin number where DHT11 sensor is connected
// and the type of sensor as the library supports various sensors.
DHT dht11(DHT_PIN, DHT11);
// Object initialization for the Software Serial port. This is supported only on Pins 8(RX) & 9(TX).
AltSoftSerial GPRS;


// Setup function of the code. This executes only once.
void setup() {
  
  // Initialization of Serial communication for debugging.
  Serial.begin(9600);
  
  // Initialization of the Software Serial communication and setting the baud rate.
  GPRS.begin(9600);

  // Switching ON the GSM modem using a digital pin.
  powerGSM();
  // Waiting for the GSM module to connect to network.
  delay(NETWORK_WAIT_TIME);
  
  // Initialization of I2C communication. Used for LCD I2C controller and Keypad.
  Wire.begin();
  // Initialization of the DHT11 sensor.
  dht11.begin();
  
  // Setting the size of the LCD Display.
  lcd.begin(20, 4);
  // Switching ON the backlight of the LCD Display.
  lcd.setBacklight(HIGH);

  lcd.clear();
  lcd.print(F("Smart Water"));
  lcd.setCursor(0, 1);
  lcd.print(F("Dispenser System"));
  // Creating the custom character for the degree centigrade symbol.
  lcd.createChar(0, degreeC);

  // Setting the Emergency Stop, Water Level, Rainfall sensor pins to input with pullups.
  // The state of the pins is HIGH unless it is shorted to Logic 0.
  for (uint8_t i = EMERGENCY_STOP; i <= RAINFALL_SENSOR; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  // Setting the Valve and Pump pins to output.
  for (uint8_t i = VALVE1; i <= PUMP1; i++) {
    pinMode(i, OUTPUT);
  }

  // Setting the interrupt pins as input with pullups.
  // Setting the ISRs for the respective pins. Here the interrupt is triggered when the state changes
  // from HIGH (logic 1) to LOW (logic 0).  
  pinMode(WATER_FLOW_SENSOR, INPUT_PULLUP);
  attachInterrupt( digitalPinToInterrupt(WATER_FLOW_SENSOR), waterFlowSensorISR, FALLING);
  pinMode(KEYPAD_INT, INPUT_PULLUP);
  attachInterrupt( digitalPinToInterrupt(KEYPAD_INT), sendRequest, FALLING);

  // Disabling the interrupts.
  cli();
  
  // PCICR - Pin Change Interrupt Control Register.
  // It is used to enable Pin Change Interrupts in a particular PORT (set of pins).
  // Bit0 is for PORTB
  // Bit1 is for PORTC
  // Bit2 is for PORTD
  PCICR   |= 0b00000100;
  
  // PCMSK2 - Pin Change Mask Register 2.
  // It is used to enable Pin Change Interrupts on particular pins and mask others.
  PCMSK2  |= 0b01110000;
  
  // Enabling the interrupts.
  sei();

  // Sending the command "AT" as the GSM modem is in auto-baud mode. The controller waits for response from the modem.
  GPRS.println("AT");
  waitForResponse("OK", 2000);
  // Sending the command "ATE 0" to disable echo from the modem. The controller waits for response from the modem.
  GPRS.println("ATE 0");
  waitForResponse("OK", 2000);
  // Sending the command "AT+CREG?" to query if the gsm modem got network. The controller waits for response from the modem.
  GPRS.println("AT+CREG?");
  waitForResponse("OK", 2000);
  // Sending the command "AT+CMGF=1" to set the modem to Text Mode for SMS. The controller waits for response from the modem.
  GPRS.println("AT+CMGF=1");
  waitForResponse("OK", 2000);

  Serial.println("Ready");

  
  // Printing the names of the valves and pumps.
  lcd.clear();
  lcd.setCursor(14, 0);
  lcd.print(F("V1-"));
  lcd.setCursor(14, 1);
  lcd.print(F("V2-"));
  lcd.setCursor(14, 2);
  lcd.print(F("V3-"));
  lcd.setCursor(14, 3);
  lcd.print(F("P1-"));
  
  // Updating the time variables.
  valvePumpUpdateTime = sensorUpdateTime = lcdUpdateTime = millis();
}

// Loop function of the code. This executes continuously.
void loop() {
  
  // Setting the future status of the pump based on number of valves to be opened/closed once every second.  
  if (millis() - valvePumpUpdateTime > 1000) {

    // Condition is satisfied if state of valves is to be changed.
    if ( memcmp(valvePump, valvePumpState, 3) ) {

      // Variable to store the number of Valves to be open.
      uint8_t count = 0;
      // Counting the number of Valves to be open
      for (uint8_t i = 0; i <= 2; i++) {
        if (valvePumpState[i] == 1) {
          count++;
        }
      }

      // Assign the count to the valveStatus.      
      switch (count) {
        case 0:
          valveStatus = ALL_OFF;
          break;
        case 1:
          valveStatus = ONE_ON;
          break;
        case 2:
          valveStatus = TWO_ON;
          break;
        case 3:
          valveStatus = ALL_ON;
          break;
        default:
          break;
      }

    }

    // Compare the current state and future state.
    switch ( (stateChange_t) memcmp(&valvePump[3], &valvePumpState[3], 1) ) {
      case NO_CHANGE:
      // If there is no change, the pump either remains OFF or ON.
      // Here, only states of valves are changed.
      // If the future state of the valves is ALL_OFF, then the pump is also switched OFF.
        if ( (valvePump[3] == 1) && ( valveStatus == ALL_OFF ) ) {
          valvePumpState[3] = 0;
        }
        valvePumpControl(NO_CHANGE);
        break;

      case OFF_TO_ON:
      // If the pump is going to be switched ON, there must be atleast one Valve opened.
      // The "if" condition ensures that.
      // This condition is also used to ensure delay after opening valves.
        if ( valveStatus == ALL_OFF ) {
          valvePumpState[3] = 0;
        }
        valvePumpControl(OFF_TO_ON);
        break;

      case ON_TO_OFF:
      // If the pump is going to be switched OFF, this ensures delay before closing of Valves.
        valvePumpControl(ON_TO_OFF);
        break;

      default:
        break;
    }

    // Updating the time variable.
    valvePumpUpdateTime = millis();
  }

  // Checking for Emergency Stops, Water Level and Rainfall Variables are updated in ISRs.
  if (emergencyStopPressed || waterLevel || rainfall) {

    char stopMessage[100] = "";
    // Clear the LCD Display.
    lcd.clear();

    // Switch OFF the Valves and Pumps.
    for ( uint8_t i = VALVE1; i <= PUMP1; i++) {
      digitalWrite(i, LOW);
    }

    // Handling the Emergency Stop and displaying on the LCD.
    if (emergencyStopPressed) {
      lcd.setCursor(0, 0);
      lcd.print(F("EMERGENCY STOP"));
      lcd.setCursor(0, 1);
      lcd.print(F("PRESSED"));
      strcat( stopMessage, emergencyMessage);
    }

    // Handling the Low Water Level signal and displaying on the LCD.
    if (waterLevel) {
      lcd.setCursor(0, 2);
      lcd.print(F("WATER LEVEL LOW"));
      strcat( stopMessage, waterLevelMessage);
    }

    // Handling the Rainfall sensor signal and displaying on the LCD.
    if (rainfall) {
      lcd.setCursor(0, 3);
      lcd.print(F("IT IS RAINING"));
      strcat( stopMessage, rainfallMessage);
    }

    // Send message stating the kind of disruption.
    sendMessage(stopMessage);
    // Switching OFF the GSM modem using a digital pin. 
    powerGSM();
    
    // Disabling the interrupts.
    cli();
    // Infinite loop to prevent the change of states.
    // To return to normal functioning, reset or power cycle the controller.
    while (true) {
      delay(100);
    }
  }


  // If no Emergency Stop or Low Water Level or Rainfall
  else {

    // If there is something from the GSM modem and we don't have a message, then we parse it.
    while( GPRS.available() && !newMessage) {
      parseAT(GPRS.read());
    }

    // If there is a new message, we use the parsed message to control the state of the valves and pump.
    // If the message contains any keywords, the controller sends a message to the number.
    if (newMessage) {
      if ( strstr(messageSMS, "v1") != NULL ) {
        Serial.println("V1");
        valvePumpState[0] ^= 1;
      }
      if ( strstr(messageSMS, "v2") != NULL ) {
        Serial.println("V2");
        valvePumpState[1] ^= 1;
      }
      if ( strstr(messageSMS, "v3") != NULL ) {
        Serial.println("V1");
        valvePumpState[2] ^= 1;
      }
      if ( strstr(messageSMS, "p1") != NULL ) {
        Serial.println("P1");
        valvePumpState[3] ^= 1;
      }
      if ( strstr(messageSMS, "auto") != NULL ) {
        Serial.println("auto");
        automatic = 1;
      }
      if ( strstr(messageSMS, "manual") != NULL ) {
        Serial.println("manual");
        automatic = 0;
      }
      if ( strstr(messageSMS, helpCommand) != NULL ) {
        Serial.println("help");
        sendMessage("Send V1 or V2 or V3 or P1 to toggle the valves or pumps. Send auto or manual or on or off to use them accordingly. Send status for the data.");
      }
      if ( strstr(messageSMS, statusCommand) != NULL ) {
        Serial.println("status");
        char statusMessage[180];
        sprintf(statusMessage, "Valve1: %s, Valve2: %s, Valve3: %s, Pump1: %s.\nTemp: %s C, Humidity: %s %%\nFlow Rate: %s L/Min Soil Moisture: %d %%",
                valvePump[0] ? "ON":"OFF", valvePump[1] ? "ON":"OFF", valvePump[2] ? "ON":"OFF", valvePump[3] ? "ON":"OFF",
                sensorData[0], sensorData[1], flowRateData, soilMoistureLevel); 
        sendMessage(statusMessage);
      }
      if ( strstr(messageSMS, allON) != NULL ) {
        Serial.println("all on");
        for (uint8_t i=0; i<=3; i++) {
          valvePumpState[i] = 1;
        }
      }
      if ( strstr(messageSMS, allOFF) != NULL ) {
        Serial.println("all off");
        for (uint8_t i=0; i<=3; i++) {
          valvePumpState[i] = 0;
        }
      }
      // Set new message status to false.
      newMessage = false;
      // Clear or reset the parsed message.
      memset(messageSMS, 0, sizeof(messageSMS));
    }
    
    // To calculate the flow rate of water once every second.
    if ( (millis() - flowUpdateTime) > 1000) {

      // Disabling the particular interrupt.
      detachInterrupt( digitalPinToInterrupt(WATER_FLOW_SENSOR));
      // Calculation of water flow rate.      
      flowRate = ( (1000.0 / (millis() - flowUpdateTime)) * waterFlowPulse ) / WATERFLOW_CALIBRATION_FACTOR;
      // Updating the time variable.      
      flowUpdateTime = millis();
      // Conversion of units.
      flowMilliLitres = (flowRate / 60) * 1000;
      totalMilliLitres += flowMilliLitres;

      // Resetting the pulse count.
      waterFlowPulse = 0;
      // Enabling the particular interrupt.
      attachInterrupt( digitalPinToInterrupt(WATER_FLOW_SENSOR), waterFlowSensorISR, FALLING);
    }

    // Checking if there is some data from the I2C devices.
    while (Wire.available() ) {
      // Read the character.
      char msg = Wire.read();
      // Print the character in the Serial Monitor for debugging.
      Serial.println(msg);

      // Assigning them to the future states according to the character received.      
      switch (msg) {
        case '1':
          valvePumpState[0] ^= 1;
          break;
        case '2':
          valvePumpState[1] ^= 1;
          break;
        case '3':
          valvePumpState[2] ^= 1;
          break;
        case '4':
          valvePumpState[3] ^= 1;
          break;
        case '5':
          automatic ^= 1;
          break;
        case '*':
          for (uint8_t i = 0; i <= 3; i++) {
            valvePumpState[i] = 0;
          }
          break;
        case '#':
          for (uint8_t i = 0; i <= 3; i++) {
            valvePumpState[i] = 1;
          }
          break;
        default:
          break;
      }
    }

    // If the keypad interrupt is triggered, then we reset the interrupt variable and request character from keypad.
    if (intKeypad) {
      intKeypad = false;
      Wire.requestFrom(KEYPAD_ADDRESS, 1);
    }

    // Updating the LCD Display every half second.
    while ( millis() - lcdUpdateTime > 500) {

      // Printing the Temperature and Humidity data.
      lcd.home();
      lcd.print(sensorData[0]);
      lcd.write(byte(0));
      lcd.print(F(" "));
      lcd.print(sensorData[1]);
      lcd.print(F("%"));

      // Printing the current type of irrigation.
      lcd.setCursor(0, 1);
      lcd.print( automatic ? "Automatic" : "Manual   " );
      
      // Printing the flow rate data.
      lcd.setCursor(0, 2);
      
      dtostrf(flowRate, 4, 1, flowRateData);
      lcd.print(flowRateData);
      lcd.print(F("L/M"));

      // Printing the Soil moisture level data.
      lcd.setCursor(0, 3);
      if (soilMoistureLevel < 100) {
        lcd.print(" ");
      }
      lcd.print(soilMoistureLevel);
      lcd.print(F("% "));

      // Printing the current states of the Valves and Pump.
      for (byte i = 0; i <= 3; i++) {
        switch (valvePump[i]) {
          case 0:
            lcd.setCursor(17, i);
            lcd.print(F("OFF"));
            break;
          case 1:
            lcd.setCursor(17, i);
            lcd.print(F("ON "));
            break;
          default:
            lcd.setCursor(17, i);
            lcd.print(F("---"));
            break;
        }
      }
      // Updating the time variable.
      lcdUpdateTime = millis();
    }

    // Read the sensors once every second.    
    while ( millis() - sensorUpdateTime > 1000) {

      // Convert floats to character array.
      dtostrf(dht11.computeHeatIndex(dht11.readTemperature(), dht11.readHumidity(), false) , 5, 2, &sensorData[0][0]);
      dtostrf(dht11.readHumidity(), 5, 2, &sensorData[1][0]);
      // Read the volatge of the soil moisture sensor.
      unsigned int soilMoistureReading = analogRead(SOIL_SENSOR);
      Serial.println(soilMoistureReading);
      
      // Convert it to percentage using the calibrated values and constrain between 0 and 100.
      soilMoistureLevel = map( soilMoistureReading, SOIL_MOISTURE_LOW, SOIL_MOISTURE_HIGH, 0, 100);
      soilMoistureLevel = constrain(soilMoistureLevel, 0 , 100);
      //Let us assume soil moisture sensor is near valve 1. So we control valve 1 based on the reading of the sensor.
      if ( soilMoistureLevel < 20 && automatic ) {
        valvePumpState[0] = 1;
        valvePumpState[3] = 1;
      }
      if ( soilMoistureLevel > 80 && automatic ) {
        valvePumpState[0] = 0;
        valvePumpState[3] = 0;
      }
      // Updating the time variable.
      sensorUpdateTime = millis();
    }
  }
}


// Function for Valve and Pump control.
void valvePumpControl(stateChange_t stateChange) {
  // Set the future state to current state.
  for (uint8_t i = 0; i <= 3; i++) {
    valvePump[i] = valvePumpState[i];
  }
  // Adding delay before Valve control if there is no change or switching OFF.
  // First Pump is switched ON/OFF, then Valves.
  if (stateChange == NO_CHANGE || stateChange == ON_TO_OFF) {
    digitalWrite(PUMP1, valvePump[3]);
    delay(DELAY_TIME);
    for (uint8_t i = VALVE1; i <= VALVE3; i++) {
      digitalWrite(i, valvePump[i - VALVE1]);
    }
    return;
  }
  // Adding delay after Valve control if Pump is switched ON.
  // First Valves are opened/closed, then Pump is swithced ON.
  if (stateChange == OFF_TO_ON) {
    for (uint8_t i = VALVE1; i <= VALVE3; i++) {
      digitalWrite(i, valvePump[i - VALVE1]);
    }
    delay(DELAY_TIME);
    digitalWrite(PUMP1, valvePump[3]);
    return;
  }

}

// ISR for sending request to keypad.
void sendRequest() {
  intKeypad = true;
}

// ISR for water flow sensor. Increments on every pulse.
void waterFlowSensorISR() {
  waterFlowPulse++;
}

// ISR for Pin Change Interrupts.
// If the transition is from HIGH (logic 1)  to LOW (logic 0), then the corresponding pin is mapped to the variable.
ISR(PCINT2_vect) {
  currentPIND = PIND;
  if (!( currentPIND & (1 << PD4) ))
    emergencyStopPressed ^= 1;
  if (!( currentPIND & (1 << PD5) ))
    waterLevel ^= 1;
  if (!( currentPIND & (1 << PD6) ))
    rainfall ^= 1;
}

// Funtion to reset or clear the raw buffer/message.
void resetBuffer() {
  memset(bufferSMS, 0, sizeof(bufferSMS));
  bufferIndex = 0;
}

// Function to wait for response from the GSM modem.
void waitForResponse(char *str, uint16_t timeout) {
  unsigned long startTime = millis();
  
  while( (strstr(bufferSMS, str) == NULL) && (millis() - startTime < timeout) ) {
    while( GPRS.available() ) {
      bufferSMS[bufferIndex++] = GPRS.read();
      Serial.print(bufferSMS[bufferIndex-1]);
      delay(50);
    }
  }
  resetBuffer();
}

// Function to parse the GSM modem output.
void parseAT(char b) {
  bufferSMS[bufferIndex] = b;
  bufferIndex += 1;
  if ( bufferIndex >= 100 ) {
    resetBuffer();
  }
   
  switch (state) {
    case PS_DETECT_MSG_TYPE:
      {
        if ( b == "\n" )
          resetBuffer();
        else {
          if ( bufferIndex == 3 && strcmp(bufferSMS, "AT+") == 0 ) {
            state = PS_COMMAND_ECHO;
            resetBuffer();
          }
          else if (strstr(bufferSMS, "+CMT:") != NULL ) {
            state = PS_CMT_NUMBER;
            resetBuffer();
          }
//          resetBuffer();
        }
      }
      break;

    case PS_CMT_NUMBER:
      {
        if( b == ',' ) {
          state = PS_CMT_NAME;
          resetBuffer();
        }
      }
      break;

    case PS_CMT_NAME:
      {
        if ( b == ',' ) {
          state = PS_CMT_DATE;
          resetBuffer();
        }
      }
      break;

    case PS_CMT_DATE:
      {
        if (b == '\n') {
          state = PS_CMT_CONTENT;
          resetBuffer();
        }
      }
      break;

    case PS_CMT_CONTENT:
      {
        if (b == '\n') {
          state = PS_DETECT_MSG_TYPE;
          strcpy(messageSMS, strlwr(bufferSMS));
          newMessage = true;
//          Serial.println("MSG content: ");
//          Serial.println(bufferSMS);
          resetBuffer();
        }
      }
      break;

      
  }  
}

// Function to send message. The variable to be passed is the address of the start of the character array of the message.
void sendMessage(char *payload) {
  GPRS.print("AT+CMGS=\"");
  GPRS.print(phoneNumber);
  GPRS.println("\"");
  waitForResponse(">", 1000);
  GPRS.println(payload);
  delay(250);
  waitForResponse(">", 1000);
  GPRS.write(26);
  waitForResponse("OK", 2000);
  Serial.println("Sent status");
  delay(200);
}

// Function to switch ON/OFF GSM module.
void powerGSM() {
  pinMode(GSM_PIN, OUTPUT);
  digitalWrite(GSM_PIN, LOW);
  delay(1000);
  digitalWrite(GSM_PIN, HIGH);
  delay(1500);
  digitalWrite(GSM_PIN, LOW);
  Serial.println(F("ON/OFF"));
  delay(1000);
}
