#include "Controller.h"

Controller::Controller()
{

        oldJSValues.resize(8);
        oldButtonValues.resize(20);
        newJSValues.resize(8);
        newButtonValues.resize(20);

        BUTTON_A = 0;
        BUTTON_B = 1;
        BUTTON_X = 2;
        BUTTON_Y = 3;
        RIGHT_TRIGGER = 4;
        RIGHT_BUMPER = 5;
        LEFT_TRIGGER = 6;
        LEFT_BUMPER = 7;
        RIGHT_STICK = 8;
        LEFT_STICK = 9;
        JS_X_AXIS = 10;
        JS_Y_AXIS = 11;

        inputType = 0;
        shouldCalibrate = false;
}

void Controller::update(bool isButton, int index, int value)
{
        controllerLock.lock();
        if (isButton)
        {
                oldButtonValues[index] = newButtonValues[index];
                newButtonValues[index] = value;
        }
        else
        {
                oldJSValues[index] = newJSValues[index];
                newJSValues[index] = value;
        }
        controllerLock.unlock();
}

void Controller::calibrate()
{
        calibrationMonitor.lock();
        cout << "Press button A: ";
        calibrationMonitor.wait();
        BUTTON_A = inputType;

        cout << "Press button B: ";
        calibrationMonitor.wait();
        BUTTON_A = inputType;

        cout << "Press button X: ";
        calibrationMonitor.wait();

        cout << "Press button Y: ";
        calibrationMonitor.wait();

        cout << "Press button A: ";
        calibrationMonitor.wait();

        cout << "Press button A: ";
        calibrationMonitor.wait();

        cout << "Press button A: ";
        calibrationMonitor.wait();

        cout << "Press button A: ";

        shouldCalibrate = false;
        calibrationMonitor.unlock();
}

bool Controller::buttonPressed(int button)
{
        bool pressed = false;
        controllerLock.lock();
        if (oldButtonValues[button] == 1 && newButtonValues[button] == 0)
                pressed = true;
        else
                pressed = false;
        controllerLock.unlock();
        return pressed;
}

bool Controller::buttonHeld(int button)
{
        bool held = false;
        controllerLock.lock();
        if (oldButtonValues[button] == 1 && newButtonValues[button] == 1)
                held = true;
        else
                held = false;
        controllerLock.unlock();
        return held;
}

int Controller::getPreviousJSValue(int axis)
{
        int value = 0;
        controllerLock.lock();
        value = oldJSValues[axis];
        controllerLock.unlock();
        return value;
}

int Controller::getCurrentJSValue(int axis)
{
        int value = 0;
        controllerLock.lock();
        value = newJSValues[axis];
        controllerLock.unlock();
        return value;
}

bool Controller::needCalibration()
{
        bool needed = false;
        calibrationMonitor.lock();
        needed = shouldCalibrate;
        calibrationMonitor.unlock();
        return needed;
}

void Controller::assignInput(int index)
{
        calibrationMonitor.lock();
        inputType = index;
        calibrationMonitor.notify();
        calibrationMonitor.unlock();
}

void Controller::setCalibration(bool needed)
{
        calibrationMonitor.lock();
        shouldCalibrate = needed;
        calibrationMonitor.unlock();
}
