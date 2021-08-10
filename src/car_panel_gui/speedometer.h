#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include "utils.h"
#include "character.h"

using namespace std;


class Speedometer
{
public:
	Speedometer();
    ~Speedometer();
    void draw(void);
    void update(float speed);
    void set_cambio(int cambio);

private:

    static constexpr float rVel = 80.0f;
    static constexpr float KMH_MS = 3.6f;    
    static constexpr float ANGLE_CONVERTER = M_PI_2 / 1.65;
    static constexpr float kmMarkerSingleStart = 0.925f;
    static constexpr float kmMarkerSingleEnd = 1.0f;
    static constexpr float angleMarkerMin = M_PI / 30.0f;
    static constexpr float kmMarkerPointStart = 0.85f;
    static constexpr float kmMarkerPointEnd = 1.0f;

    polygon polygonsFloatPoint[9];
    polygon polygonsUnit[9];
    polygon polygonsDozen[9];
    polygon polygonsHundred[9];
    polygon polygonsCambio[9];
    polygon polygonsVelocimeter[9];

    colors colorVelocimeter[6];
    colors colorSpeedometer[6];

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
