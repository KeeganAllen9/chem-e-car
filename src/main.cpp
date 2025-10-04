#include <Arduino.h>
#include <EEPROM.h>

/// the amount of samples averaged (and therefore ticks) per evaluation of curLight
constexpr uint16_t lightSampleCount = 200;
/// when curLight goes below this value, power to the wheels will be turned off until reset
constexpr uint16_t brakeActivationThreshold = 350;



/// if car is in a state of being autonomous on the track, whether being powered or braking
bool carActive = false;
/// if the wheels are getting power
bool wheelPower = false;

uint16_t curLight = 0;
uint64_t lightRollingSum = 0;
uint16_t loopCount = 0;

#pragma region logging functions

void createLog() {

}

void clearLogs() {

}

void printLogs() {

}

#pragma endregion


/// when the activation button is pushed
void activateCar() {
    carActive = true;
    wheelPower = true;
}


void setup() {
    Serial.begin(9600);

    // light sensor
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // LED
    pinMode(8, OUTPUT);
}

void loop() {
    lightRollingSum += analogRead(0);
    loopCount++;
    if (loopCount == lightSampleCount) {
        loopCount = 0;

        curLight = lightRollingSum / lightSampleCount;
        lightRollingSum = 0;
        Serial.println(curLight);

        if (curLight < brakeActivationThreshold) {

        }
    }
}