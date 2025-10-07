#include <Arduino.h>


// --- constants ---

/// the amount of samples averaged (and therefore ticks) per evaluation of curLight
constexpr unsigned int lightSampleCount = 250;
/// when curLight goes below this value, power to the wheels will be turned off until reset
constexpr unsigned int brakeActivationThreshold = 350;



// --- global vars ---

/// if car is currently in a run
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
/// if we're printing curLight
bool displayLight = false;
/// if the serial port is connected and active
bool isSerialConnected = false;
/// if the serial was connected on the last tick. used to send an initial message if serial becomes connected while arduino is running
bool hadSerialOnLastTick = false;
/// current run timer logs
unsigned long timeLogs[50] = {};
/// current timer log count
unsigned int timeLogNum = 0;



#pragma region logging functions

/// prints to serial log, if available
void serialPrint(const char *c) {
    if (isSerialConnected) {
        Serial.print(millis());
        Serial.print(" - ");
        Serial.println(c);
    }
}

void createLog() {
    timeLogNum++;
    timeLogs[timeLogNum] = brakingInitiationTime - activationTime;
}

void resetLogs() {
    for (unsigned int i = 0; i < timeLogNum; i++) {
        timeLogs[i] = 0;
    }
}

/// print current logs to the serial console
void printLogs() {
    if (!isSerialConnected) {
        return;
    }

    serialPrint("current log data -");
    for (unsigned int i = 0; i < timeLogNum; i++) {
        Serial.print("log #");
        Serial.print(i + 1);
        Serial.print(" - ");
        Serial.print(static_cast<float>(timeLogs[i]) / 1000);
        Serial.println("s");
    }
}

#pragma endregion


#pragma region state changing functions

/// called when the activation button is pushed
void startRun() {
    if (carActive) { // prevent accidental double presses
        return;
    }

    carActive = true;
    wheelPower = true;
    activationTime = millis();
    serialPrint(R"(car activated, beginning run. press "c" to simulate iodine clock triggering, or "e" to end the run)");
}

/// called when the deactivate button is pushed; ends run
void endRun() {
    if (!carActive && !wheelPower) { // prevent accidental double presses
        return;
    }

    if (!wheelPower) { // only create a log if the run wasn't ended early
        createLog();
    }

    carActive = false;
    wheelPower = false;
    serialPrint("car deactivated, run concluded");
}

/// called when the iodine clock is triggered
void triggerClock() {
    if (!wheelPower) { // prevent accidental double presses
        return;
    }

    wheelPower = false;
    brakingInitiationTime = millis();

    if (isSerialConnected) {
        Serial.print(millis());
        Serial.print(" - ");
        Serial.print("clock triggered. ");
        Serial.print(static_cast<float>(brakingInitiationTime - activationTime) / 1000);
        Serial.println(R"( seconds of wheel power. press "e" to end the run)");
    }
}

#pragma endregion


void setup() {
    Serial.begin(9600);
    isSerialConnected = static_cast<bool>(Serial);
    // ReSharper disable once CppDFAConstantConditions
    if (isSerialConnected) {
        serialPrint("initiating setup");
    }

    // light sensor
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // light sensor LED
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);
}

void loop() {
    // serial
    if (isSerialConnected && !hadSerialOnLastTick) {
        serialPrint(R"(connected to serial port. press "s" to start a run, "c" to simulate the iodine clock triggering, "e" to end an active run, "z" to toggle displaying current detected light)");
    }
    hadSerialOnLastTick = isSerialConnected;
    isSerialConnected = static_cast<bool>(Serial);
    // ReSharper disable once CppDFAConstantConditions
    if (isSerialConnected && Serial.available()) {
        const char serialIn = Serial.read();
        if (serialIn == 's') {
            startRun();
        }
        else if (serialIn == 'c') {
            triggerClock();
        }
        else if (serialIn == 'e') {
            endRun();
        }
        else if (serialIn == 'z') {
            displayLight = !displayLight;
            serialPrint(R"(press "z" to toggle displaying currently detected light, "s" to start a run, "c" to simulate the iodine clock triggering, "e" to end an active run)");
        }
    }

    // light sensor
    lightRollingSum += analogRead(0);
    lightCount++;
    if (lightCount == lightSampleCount) {
        lightCount = 0;

        curLight = lightRollingSum / lightSampleCount;
        lightRollingSum = 0;
        if (displayLight && isSerialConnected) {
            Serial.print(millis());
            Serial.print(" - ");
            Serial.print("current light is ");
            Serial.println(curLight);
        }

        if (carActive && curLight < brakeActivationThreshold) {
            triggerClock();
        }
    }
}