// Pin assignments
const int encoderLeftPinA = 19;
const int encoderLeftPinB = 18;
const int encoderRightPinA = 2;
const int encoderRightPinB = 3;
const int motorLeftPWMPin = 6;
const int motorRightPWMPin = 7;

// PID control variables
float Kp = 0.5, Ki = 0.0, Kd = 0.0; // Tune these values as necessary
float previousErrorR = 0, integralR = 0;
float previousErrorL = 0, integralL = 0;

int desiredSpeedR = 0;  // Target speed for right wheel (from serial input)
int desiredSpeedL = 0;  // Target speed for left wheel (from serial input)

// Encoder counters
volatile int encoderCountLeft = 0;
volatile int encoderCountRight = 0;

unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 100;  // 100 ms sampling time

void setup() {
    Serial.begin(115200);  // Set the baud rate for serial communication
    while (!Serial) {
        // Wait for serial connection
    }

    // Set up motor control pins
    pinMode(motorLeftPWMPin, OUTPUT);
    pinMode(motorRightPWMPin, OUTPUT);

    // Set up encoder pins
    pinMode(encoderLeftPinA, INPUT);
    pinMode(encoderLeftPinB, INPUT);
    pinMode(encoderRightPinA, INPUT);
    pinMode(encoderRightPinB, INPUT);

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
        Serial.println(actualSpeedL);
    }
}

// Function to parse the serial input "SpeedR SpeedL"
void parseSerialInput(String input) {
    int spaceIndex = input.indexOf(' ');
    
    if (spaceIndex != -1) {
        String rightStr = input.substring(0, spaceIndex);
        String leftStr = input.substring(spaceIndex + 1);

        desiredSpeedR = rightStr.toInt();  // Convert string to integer
        desiredSpeedL = leftStr.toInt();   // Convert string to integer
    }
}

// Function to control motors using PWM
void controlMotors() {
    // Convert desired speed to PWM values (mapping speed range if necessary)
    int pwmL = constrain(desiredSpeedL, -255, 255);
    int pwmR = constrain(desiredSpeedR, -255, 255);

    // Set motor PWM for left motor
    analogWrite(motorLeftPWMPin, abs(pwmL));
    if (pwmL < 0) {
        digitalWrite(motorLeftPWMPin, LOW);  // Reverse direction if needed
    } else {
        digitalWrite(motorLeftPWMPin, HIGH);
    }

    // Set motor PWM for right motor
    analogWrite(motorRightPWMPin, abs(pwmR));
    if (pwmR < 0) {
        digitalWrite(motorRightPWMPin, LOW);  // Reverse direction if needed
    } else {
        digitalWrite(motorRightPWMPin, HIGH);
    }
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
