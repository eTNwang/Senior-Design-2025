#include <Servo.h>

// Pin assignments
const int encoderLeftPinA = 19;
const int encoderLeftPinB = 18;
const int encoderRightPinA = 2;
const int encoderRightPinB = 3;
const int motorLeftPWMPin = 5; // 1 and 2
const int motorRightPWMPin = 7; // 3 and 4
const int sprayerPinA = 13;       // GPIO pin for toggling (e.g., an LED)
const int sprayerPinB = 12;       // one of these pins should be low and the other high to spray

int desiredSpeedR = 0;  // Target speed for right wheel (from serial input)
int desiredSpeedL = 0;  // Target speed for left wheel (from serial input)
int toggleValue = 0;    // Third value for toggling GPIO pin

bool toggleState = false;  // State of the toggle pin

// Encoder counters
volatile int encoderCountLeft = 0;
volatile int encoderCountRight = 0;

unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 100;  // 100 ms sampling time

Servo motorLeft;
Servo motorRight;

void setup() {
    Serial.begin(115200);  // Set the baud rate for serial communication
    while (!Serial) {
        // Wait for serial connection
    }

    // Set up motor control pins
    motorLeft.attach(motorLeftPWMPin);
    motorRight.attach(motorRightPWMPin);

    // Set up encoder pins
    pinMode(encoderLeftPinA, INPUT);
    pinMode(encoderLeftPinB, INPUT);
    pinMode(encoderRightPinA, INPUT);
    pinMode(encoderRightPinB, INPUT);

    // Set up toggle pin
    pinMode(sprayerPinA, OUTPUT);
    pinMode(sprayerPinB, OUTPUT);
    stopSprayer();

    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(encoderLeftPinA), updateEncoderLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderRightPinA), updateEncoderRight, CHANGE);
}

void loop() {
    // 1. Check for new serial input
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        parseSerialInput(input);
    }

    // 2. Run PID control at the set sampling interval
    if (millis() - lastSampleTime >= sampleInterval) {
        lastSampleTime = millis();
        controlMotors();
        
        // 3. Get actual speeds from encoders and send over serial
        int actualSpeedR = encoderCountRight;
        int actualSpeedL = encoderCountLeft;
        
        // Reset encoder counts after reading
        encoderCountRight = 0;
        encoderCountLeft = 0;
        
        Serial.print(actualSpeedR);
        Serial.print(" ");
        Serial.print(-1*actualSpeedL); // invert left encoder too
        Serial.print(" ");
        Serial.println(toggleState ? 1 : 0); // Send toggle state
    }
}

// Function to parse the serial input "SpeedR SpeedL"
void parseSerialInput(String input) {
    int firstSpaceIndex = input.indexOf(' ');
    int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);
      
    if (firstSpaceIndex != -1 && secondSpaceIndex != -1) {
        String rightStr = input.substring(0, firstSpaceIndex);
        String leftStr = input.substring(firstSpaceIndex + 1, secondSpaceIndex);
        String toggleStr = input.substring(secondSpaceIndex + 1);

        desiredSpeedR = rightStr.toInt();  // Convert string to integer
        desiredSpeedL = leftStr.toInt();  // Convert string to integer
        toggleValue = toggleStr.toInt();  // Convert string to integer

        // Check toggle value and update GPIO pin state
        if (toggleValue == 1) {
            toggleState = !toggleState; // Toggle the state
            if (toggleState == 1) {
              startSprayer();
            } else {
              stopSprayer();
            }
        }
    }
}

void startSprayer() {
  digitalWrite(sprayerPinA, LOW);
  digitalWrite(sprayerPinB, HIGH);
}

void stopSprayer() {
  // black is closed when off and red is open 
  digitalWrite(sprayerPinA, HIGH);
  digitalWrite(sprayerPinB, LOW);
}

// Function to control motors using PWM
void controlMotors() {
    const int offset = 250;
    // Rev: 1000<p<1475 for: 1525<p<2000
    // invewrt the Left speed 
    int pwmL = map(-1*desiredSpeedL, -255, 255, 1000+offset, 2000-offset); // convert ot microseconds
    int pwmR = map(desiredSpeedR, -255, 255, 1000+offset, 2000-offset);

    // Set motor PWM for left motor
    motorLeft.writeMicroseconds(pwmL);

    // Set motor PWM for right motor
    motorRight.writeMicroseconds(pwmR);
}

// Interrupt Service Routine (ISR) for the left encoder
void updateEncoderLeft() {
    bool stateA = digitalRead(encoderLeftPinA);
    bool stateB = digitalRead(encoderLeftPinB);
    if (stateA == stateB) {
        encoderCountLeft++;
    } else {
        encoderCountLeft--;
    }
}

// Interrupt Service Routine (ISR) for the right encoder
void updateEncoderRight() {
    bool stateA = digitalRead(encoderRightPinA);
    bool stateB = digitalRead(encoderRightPinB);
    if (stateA == stateB) {
        encoderCountRight++;
    } else {
        encoderCountRight--;
    }
}
