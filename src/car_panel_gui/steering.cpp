#include <GL/glew.h>
#include "steering.h"


Steering::Steering()
{
    colorSteeringR = 0.0f;
    colorSteeringG = 1.0f;
    hundredAngle = 0;
    dozenAngle = 0;
    unitAngle = 0;
    character = new Character();
    buildColor();
}


Steering::~Steering()
{
    delete character;
}


void
Steering::drawArrow(void)
{
    glColor3f(colorSteeringR, colorSteeringG, 0.0f);
    glLineWidth(1.0f);

    glBegin(GL_POLYGON);
    {
        glVertex3f(-2.5, 75, 0);
        glVertex3f(-5, 75, 0);
        glVertex3f(0, 85, 0);
        glVertex3f(5, 75, 0);
        glVertex3f(2.5, 75, 0);
        glVertex3f(2.5, 65, 0);
        glVertex3f(-2.5, 65, 0);
    }
    glEnd();
}


void
Steering::drawEdge(void)
{
    glLineWidth(1.0f);

    glBegin(GL_POLYGON);
    {
        glColor3f((colorSteeringR), (colorSteeringG), 0.0f);
        glVertex3f(-5, 0, 0);
        glColor3f((colorSteeringR), (colorSteeringG), 0.0f);
        glVertex3f(5, 0, 0);
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(2.5, -50, 0);
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(-2.5, -50, 0);
    }
    glEnd();
}


void
Steering::drawSymbol(void)
{
    glColor3f(1.0f, 1.0f, 0.0f);
    glLineWidth(1.0f);

    glBegin(GL_LINES);
    {
        for (int i = 0; i < 360; i++)
        {
            newLine(0.0f, 0.11f, i);
        }
    }
    glEnd();
}


void
Steering::draw(float angle)
{
    glRotatef(angle, 0, 0, 1);

    glColor3f(0.0f, 0.0f, 1.0f);
    glLineWidth(10.0f);

    glBegin(GL_LINES);
    {
        float x;
        float y;

        int r = 50;

        for (int i = 0; i <= 180; i++)
        {
            x = r * cos(i);
            y = r * sin(i);

            if ((x >= -90) && (x <= 90))
            {
                glVertex3f(x, y, 0);

                x = r * cos(i + 0.1);
                y = r * sin(i + 0.1);
                glVertex3f(x, y, 0);
            }
        }
    }
    glEnd();

    drawEdge();

    glRotatef(90, 0, 0, 1);
    drawEdge();
    glRotatef(-90, 0, 0, 1);

    glRotatef(-90, 0, 0, 1);
    drawEdge();
    glRotatef(90, 0, 0, 1);

    //drawSymbolSteering();

    drawArrow();

    glRotatef(-angle, 0, 0, 1);

    setColorRotation(angle);

    drawTextAngle(angle);
}


void
Steering::setColorRotation(float angle)
{
    angle = fabs(angle);

    this->colorSteeringG = 1.0f;
    this->colorSteeringR = 0.0f;

    if ((angle > 0) && (angle <= 90))
    {
        this->colorSteeringR = angle / 90.0f;
    }
    else if ((angle > 90) && (angle <= 180))
    {
        this->colorSteeringR = 1.0f;
        this->colorSteeringG = angle / 180.0f;
    }
    else if (angle > 180)
    {
        this->colorSteeringR = 1.0f;
        this->colorSteeringG = 0.0f;
    }
}


void
Steering::newLine(float rStart, float rEnd, float angle)
{
    float c = cos(angle);
    float s = sin(angle);

    glVertex2f(this->rVel * rStart * c, this->rVel * rStart * s);
    glVertex2f(this->rVel * rEnd * c, this->rVel * rEnd * s);
}


void
Steering::buildColor(void)
{
    color[0] = character->buildColor(1.0f, 0.0f, 0.0f);
    color[1] = character->buildColor(1.0f, 0.0f, 0.0f);
    color[2] = character->buildColor(0.0f, 1.0f, 0.0f);
    color[3] = character->buildColor(0.0f, 0.0f, 1.0f);
    color[4] = character->buildColor(0.0f, 0.0f, 1.0f);
    color[5] = character->buildColor(0.0f, 1.0f, 1.0f);
}

void
Steering::drawTextAngle(float angle)
{
    glTranslatef(-25, 115, 0);
    character->draw(fabs(Utils::getHundredDigitFromValue(angle)), polygonsHundred, 12, color);
    glTranslatef(25, -115, 0);

    glTranslatef(-4, 115, 0);
    character->draw(fabs(Utils::getDozenDigitFromValue(angle)), polygonsUnit, 12, color);
    glTranslatef(4, -115, 0);

    glTranslatef(18, 115, 0);
    character->draw(fabs(Utils::getUnitDigitFromValue(angle)), polygonsDozen, 12, color);
    glTranslatef(-18, -115, 0);

    glTranslatef(39, 115, 0);
    character->draw(angle > 0 ? 'L' : 'R', polygonsDozen, 12, color);
    glTranslatef(-39, -115, 0);
}
