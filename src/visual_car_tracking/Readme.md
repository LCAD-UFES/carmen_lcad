# Visual Car Tracking Module

This module is intended to detect cars from an image source and apply the inverse perspective mapping to find the position of the detected object in relation to the source at an 3D environment.

## How to run it?

### Using proccontrol:
With the central running, just execute the proccontrol giving process-visual_car_tracking.ini as the parameter. The corresponding commands in terminal are:

```
./central
./proccontrol process-visual_car_tracking.ini
```
### Standalone:
At terminal, execute it giving the bumblebee camera number that will be used as parameter.
```
./visual_car_tracking [bumblebee-camera-number]
```
##FAQ
1. Error loading cars.xml: No such file or directory
  * Make sure you have a file named cars.xml at the root of bin/ (or the folder where it's been executed from). This is the trained data that will be used by the classifier. [This](https://github.com/andrewssobral/vehicle_detection_haarcascades) github repository has more information about how to train your own data, and he also has a cars.xml trained although I don't know about its use license.
2. The screen doesn't update
  * You need to press any button to the screen to update. I've tried to remove this necessity, but without success.
