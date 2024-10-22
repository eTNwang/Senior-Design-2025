#include <Romi32U4.h>

// Create motor objects for left and right wheels
Romi32U4Motors motors;
Romi32U4Encoders encoders;
Romi32U4LCD lcd; // LCD object for displaying data

// PID control variables
float Kp = 0.5, Ki = 0.0, Kd = 0.0; // Tune these values as necessary
float previousErrorR = 0, integralR = 0;
float previousErrorL = 0, integralL = 0;

int desiredSpeedR = 0;  // Target speed for right wheel (from serial input)
int desiredSpeedL = 0;  // Target speed for left wheel (from serial input)

unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 100;  // 100 ms sampling time

void setup() {
    Serial.begin(115200);  // Set the baud rate for serial communication
    while (!Serial) {
        // Wait for serial connection
    }

    encoders.init(); // Initialize encoders
    motors.setSpeeds(0, 0);  // Start with motors off

    lcd.clear();  // Clear the LCD at the start
}

void loop() {
    // 1. Check for new serial input
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        parseSerialInput(input);

        // Display the desired speeds on the LCD
        displayDesiredSpeedsOnLCD();
    }

    // 2. Run PID control at the set sampling interval
    if (millis() - lastSampleTime >= sampleInterval) {
        lastSampleTime = millis();
        controlMotors();
        
        // 3. Get actual speeds from encoders and send over serial
        int actualSpeedR = encoders.getCountsAndResetRight();
        int actualSpeedL = encoders.getCountsAndResetLeft();
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

// Function to display desired speeds on the LCD
void displayDesiredSpeedsOnLCD() {
    lcd.clear();  // Clear the previous content on the LCD

    // Display desired speed for right wheel on line 1
    lcd.gotoXY(0, 0);  // Move to the first line
    lcd.print("R:");
    lcd.print(desiredSpeedR);

    // Display desired speed for left wheel on line 2
    lcd.gotoXY(0, 1);  // Move to the second line
    lcd.print("L:");
    lcd.print(desiredSpeedL);
}

// PID control for both motors
void controlMotors() {
    // 1. Calculate current speeds (encoder counts per sampling interval)
    int actualSpeedR = encoders.getCountsRight();
    int actualSpeedL = encoders.getCountsLeft();

    // 2. Right wheel PID
    float errorR = desiredSpeedR - actualSpeedR;
    integralR += errorR * (sampleInterval / 1000.0);
    float derivativeR = (errorR - previousErrorR) / (sampleInterval / 1000.0);
    float outputR = (Kp * errorR) + (Ki * integralR) + (Kd * derivativeR);
    previousErrorR = errorR;

    // 3. Left wheel PID
    float errorL = desiredSpeedL - actualSpeedL;
    integralL += errorL * (sampleInterval / 1000.0);
    float derivativeL = (errorL - previousErrorL) / (sampleInterval / 1000.0);
    float outputL = (Kp * errorL) + (Ki * integralL) + (Kd * derivativeL);
    previousErrorL = errorL;

    // 4. Apply the PID output as PWM efforts to the motors
    motors.setSpeeds(constrain(outputL, -255, 255), constrain(outputR, -255, 255));
    // motors.setSpeeds(desiredSpeedR, desiredSpeedR);
}
