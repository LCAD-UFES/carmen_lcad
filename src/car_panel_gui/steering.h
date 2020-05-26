#ifndef STEERING_H
#define STEERING_H

#include <math.h>
#include "utils.h"
#include "character.h"


class Steering
{
public:
	Steering();
    ~Steering();
    void draw(float angle);

private:
    static constexpr float rVel = 80.0f; // desacoplar
    double colorSteeringR;
    double colorSteeringG;
    polygon polygonsHundred[9];
    polygon polygonsUnit[9];
    polygon polygonsDozen[9];
    colors color[6];
    int hundredAngle;
    int dozenAngle;
    int unitAngle;
    Character *character;

    void newLine(float rStart, float rEnd, float angle);
    void drawArrow(void);
    void drawEdge(void);
    void drawSymbol(void);
    void setColorRotation(float angle);
    void buildColor(void);
    void drawTextAngle(float angle);
};

#endif // STEERING_H
