#include <arduino.h>
#include <ZumoReflectanceSensorArray.h>

// --- Sensor setup ---
ZumoReflectanceSensorArray reflectanceSensors;
#define NUM_SENSORS 6

unsigned char sensorPins[] = {4, A3, 11, A0, A2, 5};
// unsigned char numSensors = 6; // Remove this line
unsigned int sensorValues[NUM_SENSORS];

//unsigned char numSensors = 6;

unsigned int timeout = 3000; // Increase this value for higher sensor mounting
unsigned char emitterPin = 2;

//unsigned int sensorValues[NUM_SENSORS];

// --- SparkFun TB6612FNG motor driver pins ---
const int AIN1 = 7; // Left motor IN1
const int AIN2 = 8; // Left motor IN2
const int PWMA = 6; // Left motor PWM

const int BIN1 = 9; // Right motor IN1
const int BIN2 = 12; // Right motor IN2
const int PWMB = 10; // Right motor PWM

// --- PD control parameters ---
float Kp = 0.1; //weight form error
float Kd = 0.1; //weight from difference in error
int baseSpeed = 125;
int maxSpeed = 255;
int recovery = 100;

int lastError = 0;

void setup() {
    reflectanceSensors.init();

    // Motor pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    // Calibration (10s, move sensors across line)
    delay(500);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
        reflectanceSensors.calibrate();
    }
    digitalWrite(13, LOW);

    Serial.begin(9600);
    Serial.println("Calibration done. Starting line following...");
}

//funksjon for å sette motorfarten, tar inn negativ for bakover, positiv tall for forover. forventet verdier mellom -255 og 255
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Left motor
    if (leftSpeed >= 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        leftSpeed = -leftSpeed;
    }
    if (leftSpeed > maxSpeed) leftSpeed = maxSpeed; //keeps under max speed 255
    analogWrite(PWMA, leftSpeed);

    // Right motor
    if (rightSpeed >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        rightSpeed = -rightSpeed;
    }
    if (rightSpeed > maxSpeed) rightSpeed = maxSpeed; //keeps under max speed 255
    analogWrite(PWMB, rightSpeed);
}

void loop() {
    // read calibrated line position (0–5000)
    int position = reflectanceSensors.readLine(sensorValues); //was unsigned, may need to put in again after test.

    // detect if line is lost (all sensors white)
    bool lineDetected = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] < 500 and (position == 0 or position == 5000)) {
            // threshold for black
            lineDetected = false;
            break;
        }
    }
    //finner tilbake til linjen basert på kva siden den mista den på
    if (!lineDetected) {
        Serial.println("⚠️ Line lost! Recovering...");
        if (lastError < 0) {
            setMotorSpeeds(recovery, -recovery); // spin left
            Serial.println("Recovering LEFT");
        } else {
            setMotorSpeeds(-recovery, recovery); // spin right
            Serial.println("Recovering RIGHT");
        }
        delay(50);
        return;
    }

    // Logisk kontroll for hvordan/hvor mye den skal svinge baser på senor input
    int error = (int) position - 2500; // center = 2500
    int derivative = error - lastError;
    int correction = Kp * error + Kd * derivative;

    int leftSpeed = baseSpeed - (correction/4 );
    int rightSpeed = baseSpeed + (correction/4 );

    if (leftSpeed > maxSpeed) leftSpeed = maxSpeed;
    if (leftSpeed < -maxSpeed) leftSpeed = -maxSpeed;
    if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;
    if (rightSpeed < -maxSpeed) rightSpeed = -maxSpeed;



    setMotorSpeeds(leftSpeed, rightSpeed);

    // --- Debug output --- Kun visuelt
    Serial.print("Sensors: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        Serial.print(" ");
    }
    Serial.print(" | Pos: ");
    Serial.print(position);
    Serial.print(" | Err: ");
    Serial.print(error);
    Serial.print(" | Deriv: ");
    Serial.print(derivative);
    Serial.print(" | Corr: ");
    Serial.print(correction);
    Serial.print(" | L: ");
    Serial.print(leftSpeed);
    Serial.print(" | R: ");
    Serial.println(rightSpeed);

    lastError = error;
    delay(50); // adjust for how fast you want updates
}
