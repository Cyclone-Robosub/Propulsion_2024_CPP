// William Barber
#include "Command_Interpreter.h"
#include <wiringPi.h> // this needs to be installed on the pi
#include <iostream>
#include <utility>

DigitalPin::DigitalPin(int gpioNumber, EnableType enableType): Pin(gpioNumber), pinStatus(Disabled), enableType(enableType) {}

void DigitalPin::initialize() {
    pinMode(gpioNumber, OUTPUT);
}

void DigitalPin::enable() {
    switch (enableType) {
        case ActiveHigh:
            digitalWrite(gpioNumber, HIGH);
            break;
        case ActiveLow:
            digitalWrite(gpioNumber, LOW);
            break;
        default:
            std::cerr << "Impossible pin mode!" << std::endl; //throw exception?
    }
    pinStatus = Enabled;
}

void DigitalPin::disable() {
    switch (enableType) {
        case ActiveHigh:
            digitalWrite(gpioNumber, LOW);
            break;
        case ActiveLow:
            digitalWrite(gpioNumber, HIGH);
            break;
        default:
            std::cerr << "Impossible pin mode!" << std::endl; //throw exception?
    }
    pinStatus = Disabled;
}

bool DigitalPin::enabled() {
    return pinStatus == Enabled;
}


PwmPin::PwmPin(int gpioNumber): Pin(gpioNumber), currentPwm(0) {}

void PwmPin::initialize() {
    pinMode(gpioNumber, PWM_OUTPUT);
}

void PwmPin::enable() {
    setPowerAndDirection(255, Forwards);
    currentPwm = 255;
}

void PwmPin::disable() {
    setPowerAndDirection(0, Forwards);
    currentPwm = 255;
}

bool PwmPin::enabled() {
    return currentPwm != 0;
}

void PwmPin::setPowerAndDirection(int pwmValue, Direction direction) {
    //TODO: set direction (not sure how to do this w/ pwm. High/low?)
    pwmWrite(gpioNumber, pwmValue);
    currentPwm = pwmValue;
}



Command_Interpreter_RPi5::Command_Interpreter_RPi5(): thrusterPins(std::vector<PwmPin*>{}), digitalPins(std::vector<DigitalPin*>{}) {}
Command_Interpreter_RPi5::Command_Interpreter_RPi5(std::vector<PwmPin*> thrusterPins, std::vector<DigitalPin*> digitalPins):
                                                thrusterPins(std::move(thrusterPins)), digitalPins(std::move(digitalPins)) {
    allPins = std::vector<Pin*>{};
    allPins.insert(allPins.end(), thrusterPins.begin(), thrusterPins.end());
    allPins.insert(allPins.end(), digitalPins.begin(), digitalPins.end());
}

void Command_Interpreter_RPi5::initializePins() {
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Failure to configure GPIO pins through wiringPi!" << std::endl; //throw exception?
    }
    for (Pin* pin : allPins) {
        pin->initialize();
    }
}

void Command_Interpreter_RPi5::execute(const Command& command) {
    for (int i = 0; i < 8; i++) {
        thrusterPins[i]->setPowerAndDirection(command.thruster_pwms[i], Forwards);
    }
    //TODO: duration
}
