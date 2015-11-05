#include <vector>
#include <IceUtil/Mutex.h>
#include <IceUtil/Monitor.h>

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

using namespace std;

class Controller
{
public:

        static const int BUTTON = 0;
        static const int AXIS = 1;

        // Button identifiers to be used with buttonPressed(int) and buttonHeld(int)
        int BUTTON_A;
        int BUTTON_B;
        int BUTTON_X;
        int BUTTON_Y;
        int RIGHT_TRIGGER;
        int LEFT_TRIGGER;
        int RIGHT_BUMPER;
        int LEFT_BUMPER;
        int RIGHT_STICK;
        int LEFT_STICK;

        // Joystick axis used with joystickValue(int);
        int JS_X_AXIS;
        int JS_Y_AXIS;

        Controller();

        void update(bool isButton, int index, int value);
        void calibrate();
        bool buttonPressed(int button);
        bool buttonHeld(int button);
        int getPreviousJSValue(int axis);
        int getCurrentJSValue(int axis);
        bool needCalibration();
        void assignInput(int index);
        void setCalibration(bool needed);

private:

        IceUtil::Mutex controllerLock;
        IceUtil::Monitor<IceUtil::Mutex> calibrationMonitor;

        vector<int> oldJSValues;
        vector<int> newJSValues;
        vector<int> oldButtonValues;
        vector<int> newButtonValues;

        int inputType;
        bool shouldCalibrate;

};

#endif /* CONTROLLER_H_ */
