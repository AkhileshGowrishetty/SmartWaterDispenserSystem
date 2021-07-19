/*
 * Smart Water Dispenser System
 *
 * This is the sketch for the Arduino Pro Micro.
 * 
 * When a button is pressed, the pin connected to the interrupt of the controller (Arduino Uno)
 * is set to 'LOW' (Logic 0).
 * The Arduino Pro Micro receives a request from the controller to send the keypad message.
 *
 * LIBRARIES:
 *  - Keypad Library for keypad matrix handling.
 *  - Wire Library for I2C Communication.
 *
 * Keypad Control:
 *  "1" - Valve 1 ON/OFF
 *  "2" - Valve 2 ON/OFF
 *  "3" - Valve 3 ON/OFF
 *  "4" - Pump 1 ON/OFF
 *  "*" - All Valves and Pumps OFF
 *  "#" - All Valves and Pumps ON
 */


#include <Keypad.h>
#include <Wire.h>

// Number of Rows and Columns
#define ROWS 4
#define COLUMNS 3

// Value of the trigger pin that is connected to the controller's interrupt
#define INT_PIN 4
// I2C Address.
#define KEYPAD_ADDRESS 0x08

// A 2-dimensional array that holds the characters to be sent.
// It is with respect to the position of buttons in the matrix.
char keys[ROWS][COLUMNS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}  
};

// Variable to hold the character of the pressed key.
char pressedKey;

// Pin definitions of the rows and columns.
uint8_t rowPins[ROWS] = {15, 14, 16, 10};
uint8_t columnPins[COLUMNS] = {A0, A1, A2};

// Object initialization of the keypad. The parameters are pointer to the character map,
// pin definitions of rows and columns and number of rows and columns.
Keypad keypad = Keypad( makeKeymap(keys), rowPins, columnPins, ROWS, COLUMNS);

// Setup function of the code. This executes only once.
void setup() {
  
  // Initialization of Serial communication for debugging.
//  Serial.begin(9600);
  // Initialization of I2C communication. Used for Keypad. Need to pass the address here.
  Wire.begin(KEYPAD_ADDRESS);

  // Linking the Callback function if there is a I2C request.
  Wire.onRequest(requestEvent);
  
  // Setup of the interrupt pin.
  pinMode(INT_PIN, OUTPUT);
  // Setting the value of the interrupt trigger pin.
  digitalWrite(INT_PIN, HIGH);
  
}

// Loop function of the code. This executes continuously.
void loop() {
  
  // Local variable for storing the keypad response. This is an example of polling.
  char key = keypad.getKey();

  // If we get a response
  if (key) {

    // Trigger the interrupt in the controller.
    digitalWrite(INT_PIN, LOW);
    // Store the key in a global variable as we do not know the timing of the I2C request.
    pressedKey = key;
    // Print the key in Serial Monitor for debugging.
//    Serial.println(key);
  }
}

// Callback function for I2C request.
void requestEvent() {

  // Reset the interrupt trigger.
  digitalWrite(INT_PIN, HIGH);
  // Send the keypad response to the controller.
  Wire.write(pressedKey);
}
