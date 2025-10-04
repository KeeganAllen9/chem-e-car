#include <Arduino.h>
#include <EEPROM.h>

/// the amount of samples averaged (and therefore ticks) per evaluation of curLight
constexpr uint16_t lightSampleCount = 200;
/// when curLight goes below this value, power to the wheels will be turned off until reset
constexpr uint16_t brakeActivationThreshold = 350;



/// if car is in a state of being autonomous on the track, whether being powered or braking
bool carActive = false;
/// the time since initial startup at which the car was activated most recently
unsigned long activationTime = 0;
/// if the wheels are getting power
bool wheelPower = false;
// the time since initial startup at which braking was initiated
unsigned long brakingInitiationTime = 0;
/// the current evaluation of the light sensor, averaged over multiple samples
unsigned int curLight = 0;
/// the running sum of the light sensor's output, used to evaluate curLight
unsigned long long lightRollingSum = 0;
/// loop counter for averaging curLight
unsigned int lightCount = 0;
/// loop counter for determining serial state changes
unsigned int serialCount = 0;
/// if the arduino is attached to an external device
bool serialActive = false;


#pragma region logging functions

/// prints to serial log, if available
void serialPrint(const char *c) {
    if (serialActive) {
        Serial.print(millis());
        Serial.print(" - ");
        Serial.println(c);
    }
}

// log structure -
// first 2B are for current log length
// every 2B after is a log, each consisting of a 2B number of milliseconds of wheel power time

void createLog() {
    uint16_t eeAddress;
    EEPROM.get(0, eeAddress);
    eeAddress += 2;
    if (eeAddress >= EEPROM.length()) {
        return;
    }

    EEPROM.put(eeAddress, brakingInitiationTime - activationTime);
    EEPROM.put(0, eeAddress);
}

void resetLogs() {
    for (uint16_t i = 0; i < EEPROM.length(); i++) {
        EEPROM.update(i, 0);
    }
}

void printLogs() {
    if (!serialActive) {
        return;
    }

    serialPrint("current log data -");
    for (uint16_t eeAddress = 0; eeAddress < EEPROM.length(); eeAddress += 2) {
        uint8_t data;
        EEPROM.get(eeAddress, data);
        Serial.println(data);
    }
}

#pragma endregion


#pragma region state changing functions

/// called when the activation button is pushed
void activateCar() {
    if (carActive && wheelPower) { // prevent accidental double presses
        return;
    }

    carActive = true;
    wheelPower = true;
    activationTime = millis();
    serialPrint("car activated");
}

/// called when the deactivate button is pushed; ends run
void deactivateCar() {
    if (!carActive && !wheelPower) {
        return; // prevent accidental double presses
    }

    if (!wheelPower) { // only create a log if the run was ended early
        createLog();
    }

    carActive = false;
    wheelPower = false;
    serialPrint("car deactivated");
}

/// called when the iodine clock is triggered
void initiateBraking() {
    if (!wheelPower) {
        return;
    }

    wheelPower = false;
    brakingInitiationTime = millis();
    serialPrint("braking activated");
    serialPrint("total wheel power time (s):");
    serialPrint(reinterpret_cast<const char *>((brakingInitiationTime - activationTime) / 1000));
}

#pragma endregion


void setup() {
    Serial.begin(9600);

    // light sensor
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // LED
    pinMode(8, OUTPUT);
}

void loop() {
    // evaluate serial
    serialCount++;
    if (serialCount == 10000) {
        serialActive = Serial.available();
        serialCount = 0;
    }

    // light sensor
    lightRollingSum += analogRead(0);
    lightCount++;
    if (lightCount == lightSampleCount) {
        lightCount = 0;

        curLight = lightRollingSum / lightSampleCount;
        lightRollingSum = 0;
        Serial.println(curLight);

        if (carActive && curLight < brakeActivationThreshold) {
            initiateBraking();
        }
    }
}