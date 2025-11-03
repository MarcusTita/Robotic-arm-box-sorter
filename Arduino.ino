// Define pin assignments for Actuator 1
const int analogPin1 = A2;  // Analog input pin for actuator 1
const int pwmPin1 = 3;      // PWM enable pin for actuator 1
const int cwPin1 = 10;      // Clockwise control pin
const int ccwPin1 = 11;     // Counter-clockwise control pin
int currentPWM1 = 0;
float targetVoltage1 = 1.5;  // Default voltage for actuator 1
bool flip1 = true, flop1 = true;



// Define pin assignments for Actuator 2
const int analogPin2 = A3;  // Analog input pin for actuator 2
const int pwmPin2 = 4;      // PWM enable pin for actuator 2
const int cwPin2 = 9;       // Clockwise control pin
const int ccwPin2 = 8;      // Counter-clockwise control pin
int currentPWM2 = 0;
float targetVoltage2 = 2.2;  // Default voltage for actuator 2
bool flip2 = true, flop2 = true;

// Define pin assignments for Actuator 3
const int analogPin1 = A4;  // Analog input pin for actuator 1
const int pwmPin1 = 5;      // PWM enable pin for actuator 1
const int cwPin1 = 6;      // Clockwise control pin
const int ccwPin1 = 7;     // Counter-clockwise control pin
int currentPWM3 = 0;
float targetVoltage3 = 2.2;  // Default voltage for actuator 3
bool flip3 = true, flop3 = true;

//global motor pwm
const int maxPWM = 255;    // Maximum PWM power
const int minPWM = 50;     // Minimum PWM to overcome inertia
const int pwmStep = 5;     // Step size for smoother ramping
bool motorsEnabled = true;


void setup() {
   Serial.begin(115200);


   // Initialize actuator 1 pins
   pinMode(pwmPin1, OUTPUT);
   pinMode(cwPin1, OUTPUT);
   pinMode(ccwPin1, OUTPUT);
   pinMode(analogPin1, INPUT);


   // Initialize actuator 2 pins
   pinMode(pwmPin2, OUTPUT);
   pinMode(cwPin2, OUTPUT);
   pinMode(ccwPin2, OUTPUT);
   pinMode(analogPin2, INPUT);


   // Ensure actuators are off at start
   stopActuators();


   Serial.println("Enter two voltage targets (e.g., 1.5,2.2) or 'disable'/'enable'.");
}


void loop() {
   // Check for Serial Input
   if (Serial.available()) {
       String input = Serial.readStringUntil('\n'); // Read user input
       input.trim();
      
       if (input.equalsIgnoreCase("disable")) {
           motorsEnabled = false;
           stopActuators();
           Serial.println("Motors DISABLED.");
       } else if (input.equalsIgnoreCase("enable")) {
           motorsEnabled = true;
           Serial.println("Motors ENABLED.");
       } else {
           int commaIndex = input.indexOf(',');
           if (commaIndex > 0) {
               float newVoltage1 = input.substring(0, commaIndex).toFloat();
               float newVoltage2 = input.substring(commaIndex + 1).toFloat();
              
               if (newVoltage1 >= 0 && newVoltage1 <= 5 && newVoltage2 >= 0 && newVoltage2 <= 5) {
                   targetVoltage1 = newVoltage1;
                   targetVoltage2 = newVoltage2;


                   Serial.print("New Voltage Targets Set: ");
                   Serial.print(targetVoltage1);
                   Serial.print("V, ");
                   Serial.println(targetVoltage2);
               } else {
                   Serial.println("Invalid input! Enter two numbers between 0 and 5.");
               }
           }
       }
   }


   if (motorsEnabled) {
       // Read analog voltages for each actuator
       float voltage1 = analogRead(analogPin1) * (5.0 / 1023.0);
       float voltage2 = analogRead(analogPin2) * (5.0 / 1023.0);


       // Print continuous updates
       Serial.print("Go to Voltage X: ");
       Serial.print(targetVoltage1);
       Serial.print("V, Y: ");
       Serial.println(targetVoltage2);


       Serial.print("Current Voltage 1: ");
       Serial.print(voltage1);
       Serial.print("V | Target: ");
       Serial.print(targetVoltage1);
       Serial.print("V | ");


       Serial.print("Current Voltage 2: ");
       Serial.print(voltage2);
       Serial.print("V | Target: ");
       Serial.println(targetVoltage2);


       // Control each actuator independently
       controlActuator1(voltage1);
       controlActuator2(voltage2);
   }


   delay(3); // Small delay for smooth response
}


// Control function for Actuator 1
void controlActuator1(float currentVoltage) {
   if (currentVoltage < targetVoltage1) { // Move Forward
       if (flop1) { // Reset PWM when switching direction
           currentPWM1 = 0;
           flip1 = true;
           flop1 = false;
       }
       Serial.println("Actuator 1 Moving FORWARD");
       rampPWM(currentPWM1);
       digitalWrite(cwPin1, HIGH);
       digitalWrite(ccwPin1, LOW);
       analogWrite(pwmPin1, currentPWM1);
   }
   else if (currentVoltage > targetVoltage1) { // Move Backward
       if (flip1) { // Reset PWM when switching direction
           currentPWM1 = 0;
           flip1 = false;
           flop1 = true;
       }
       Serial.println("Actuator 1 Moving BACKWARD");
       rampPWM(currentPWM1);
       digitalWrite(cwPin1, LOW);
       digitalWrite(ccwPin1, HIGH);
       analogWrite(pwmPin1, currentPWM1);
   }
   else { // Stop at target voltage
       Serial.println("Actuator 1 Holding Position");
       digitalWrite(cwPin1, LOW);
       digitalWrite(ccwPin1, LOW);
       analogWrite(pwmPin1, 0);
       currentPWM1 = 0;
   }
}


// Control function for Actuator 2
void controlActuator2(float currentVoltage) {
   if (currentVoltage < targetVoltage2) { // Move Forward
       if (flop2) { // Reset PWM when switching direction
           currentPWM2 = 0;
           flip2 = true;
           flop2 = false;
       }
       Serial.println("Actuator 2 Moving FORWARD");
       rampPWM(currentPWM2);
       digitalWrite(cwPin2, HIGH);
       digitalWrite(ccwPin2, LOW);
       analogWrite(pwmPin2, currentPWM2);
   }
   else if (currentVoltage > targetVoltage2) { // Move Backward
       if (flip2) { // Reset PWM when switching direction
           currentPWM2 = 0;
           flip2 = false;
           flop2 = true;
       }
       Serial.println("Actuator 2 Moving BACKWARD");
       rampPWM(currentPWM2);
       digitalWrite(cwPin2, LOW);
       digitalWrite(ccwPin2, HIGH);
       analogWrite(pwmPin2, currentPWM2);
   }
   else { // Stop at target voltage
       Serial.println("Actuator 2 Holding Position");
       digitalWrite(cwPin2, LOW);
       digitalWrite(ccwPin2, LOW);
       analogWrite(pwmPin2, 0);
       currentPWM2 = 0;
   }
}


// Smooth PWM ramping function
void rampPWM(int &currentPWM) {
   if (currentPWM < maxPWM) {
       currentPWM += pwmStep;
   }
   currentPWM = constrain(currentPWM, minPWM, maxPWM);
}


// Stop all actuators
void stopActuators() {
   digitalWrite(cwPin1, LOW);
   digitalWrite(ccwPin1, LOW);
   analogWrite(pwmPin1, 0);


   digitalWrite(cwPin2, LOW);
   digitalWrite(ccwPin2, LOW);
   analogWrite(pwmPin2, 0);
}



