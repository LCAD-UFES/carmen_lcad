#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <GL/glew.h>
#include "utils/utils.h"
#include "characters/character.h"

using namespace std;


class Speedometer
{
public:
	Speedometer();
    ~Speedometer();
    void draw(void);
    void update(float speed);

private:

    static const float rVel = 80.0f;
    static const float KMH_MS = 3.6f;    
    static const float ANGLE_CONVERTER = M_PI_2 / 1.65;
    static const float kmMarkerSingleStart = 0.925f;
    static const float kmMarkerSingleEnd = 1.0f;
    static const float angleMarkerMin = M_PI / 30.0f;
    static const float kmMarkerPointStart = 0.85f;
    static const float kmMarkerPointEnd = 1.0f;

    polygon polygonsFloatPoint[9];
    polygon polygonsUnit[9];
    polygon polygonsDozen[9];
    polygon polygonsHundred[9];
    polygon polygonsCambio[9];
    polygon polygonsVelocimeter[9];

    colors colorVelocimeter[5];
    colors colorSpeedometer[5];

    float angleVel;
    float speed;
    int hundredSpeed;
    int dozenSpeed;
    int unitSpeed;
    int floatPointSpeed;
    int cambio;

    Character *character;

    void newLine(float rStart, float rEnd, float angle);
    void drawLabelsVelocimeter(void);
    void drawCambio(void);
    void drawMargin(void);
    void countCambio(void);
};

#endif // SPEEDOMETER_H
