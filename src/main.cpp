#include <arduino.h>
#include <ZumoReflectanceSensorArray.h>


// --- Sensor setup ---
#define NUM_SENSORS 6
ZumoReflectanceSensorArray reflectanceSensors;
unsigned int sensorValues[NUM_SENSORS];

// --- SparkFun TB6612FNG motor driver pins ---
const int AIN1 = 7; // Left motor IN1
const int AIN2 = 8; // Left motor IN2
const int PWMA = 6; // Left motor PWM

const int BIN1 = 9; // Right motor IN1
const int BIN2 = 12; // Right motor IN2
const int PWMB = 10; // Right motor PWM

// --- PD control parameters ---

float kp_e(int err){
    if (err > 2000 || err < -2000){
        return 0.21;
        }
    else{
        return 0.1;
    }

}
float kd_e(int diff){
    if (diff > 2000 || diff < -2000){
        return 0.25;
        }
    else{
        return 0.17;
    }
}


//float Kp = 0.2; //weight form error
//float Kd = 0.22; //weight from difference in error
int baseSpeed = 110;
int maxSpeed = baseSpeed*2;

int diff = 0;



int lastError = 0;

void setup() {
    reflectanceSensors.init();
    if (maxSpeed > 255){
    maxSpeed = 255;
    }
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


    // Logisk kontroll for hvordan/hvor mye den skal svinge baser på senor input
    int error = (int) position - 2500; // center = 2500
    int derivative = error - lastError;
    int correction = kp_e(error) * error + kd_e(derivative) * derivative;

    int leftSpeed = baseSpeed - (correction / 3);
    int rightSpeed = baseSpeed + (correction / 3);

    if (leftSpeed > maxSpeed) leftSpeed = maxSpeed;
    if (leftSpeed < -maxSpeed) leftSpeed = -maxSpeed;
    if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;
    if (rightSpeed < -maxSpeed) rightSpeed = -maxSpeed;

    if(error<0){
        diff = -error;
    }
    else{
        diff = error;
    }

    if (diff < 500) { //justerer farten når den er nær midten
        leftSpeed = baseSpeed;
        rightSpeed = baseSpeed;
    }

    setMotorSpeeds(leftSpeed, rightSpeed);

    bool consoleOutput = false; //Bytt til false for å skru av output, burde være av ved konkuranse

    if (consoleOutput) {
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
    }

    lastError = error;
    delay(50); // adjust for how fast you want updates
}