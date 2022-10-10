#include <GL/glew.h>
#include "speedometer.h"

Speedometer::Speedometer()
{
    angleVel = ANGLE_CONVERTER;
    speed = 0;
    floatPointSpeed = 0;
    hundredSpeed = 0;
    dozenSpeed = 0;
    unitSpeed = 0;
    cambio = 0;
    character = new Character();
}

Speedometer::~Speedometer()
{
    delete character;
}

void Speedometer::draw(void)
{
    drawCambio();

    colorVelocimeter[0] = character->buildColor(1.0f, 0.0f, 0.0f);
    colorVelocimeter[1] = character->buildColor(1.0f, 0.0f, 0.0f);
    colorVelocimeter[2] = character->buildColor(0.0f, 1.0f, 0.0f);
    colorVelocimeter[3] = character->buildColor(0.0f, 0.0f, 1.0f);
    colorVelocimeter[4] = character->buildColor(0.0f, 0.0f, 1.0f);
    colorVelocimeter[5] = character->buildColor(0.0f, 1.0f, 1.0f);

    glTranslatef(-100, 0, 0);

    glTranslatef(-150, 0, 0);

    glTranslatef(125, -40, 0);
    glRotatef(0, 0, 0, 1);
    character->draw(this->dozenSpeed, this->polygonsDozen, 20, this->colorVelocimeter);
    glRotatef(0, 0, 0, 1);
    glTranslatef(-125, 40, 0);

    glTranslatef(155, -40, 0);
    glRotatef(0, 0, 0, 1);
    character->draw(this->unitSpeed, this->polygonsUnit, 20, this->colorVelocimeter);
    glRotatef(0, 0, 0, 1);
    glTranslatef(-155, 40, 0);

    glTranslatef(162.5, -63, 0);
    glRotatef(0, 0, 0, 1);
    character->drawSinglePoint(this->colorVelocimeter);
    glRotatef(0, 0, 0, 1);
    glTranslatef(-162.5, 63, 0);

    glTranslatef(195, -40, 0);
    glRotatef(0, 0, 0, 1);
    character->draw(this->floatPointSpeed, this->polygonsFloatPoint, 20, this->colorVelocimeter);
    glRotatef(0, 0, 0, 1);
    glTranslatef(-195, 40, 0);

    glTranslatef(120, 20, 0);
    glRotatef(0, 0, 0, 1);
    character->draw(this->cambio, this->polygonsCambio, 10, this->colorVelocimeter);
    glRotatef(0, 0, 0, 1);
    glTranslatef(-120, -20, 0);

    glTranslatef(150, 0, 0);

    drawMargin();

    //    glColor3f(1.0f, 1.0f, 1.0f);

    //    glBegin(GL_LINES);
    //    {
    //        for (int i = 0; i < 41; i++)
    //        {
    //            if (i % 5)
    //            {
    //                if ((i % 5) == 1)
    //                {
    //                    glColor3f(1.0f, 1.0f, 1.0f);
    //                }
    //                newLine(kmMarkerSingleStart, kmMarkerSingleEnd, i * angleMarkerMin);
    //            }
    //            else
    //            {
    //                glColor3f(1.0f, 0.0f, 0.0f);
    //                newLine(this->kmMarkerPointStart, kmMarkerPointEnd, i * angleMarkerMin);
    //            }
    //        }
    //    }
    //    glEnd();

    //    glColor3f(0.0f, 1.0f, 0.0f);

    //    glBegin(GL_LINES);
    //    {
    //        newLine(0.0f, -0.8f, this->angleVel + 0.1);
    //    }
    //    glEnd();

    //    glBegin(GL_LINES);
    //    {
    //        for (int i = 0; i < 360; i++)
    //        {
    //            newLine(0.0f, 0.05f, i);
    //        }
    //    }
    //    glEnd();

    //   	drawLabelsVelocimeter();

    glTranslatef(100, 0, 0);
}

void Speedometer::set_cambio(int cambio)
{
    this->cambio = cambio;
}

void Speedometer::update(float speed)
{
    // conversion to km/h
    double speed_km_h = fabs(speed * KMH_MS);
    this->speed = this->speed + (speed_km_h - this->speed) / 15.0; // Media para suavisar o display

    Utils::getDigitsFromValue(this->speed, &hundredSpeed, &dozenSpeed, &unitSpeed, &floatPointSpeed);

    this->angleVel = (this->speed > 0.0) ? ((this->speed * -0.05 * 1.05) + ANGLE_CONVERTER) : ANGLE_CONVERTER;
}

// desacoplar
void Speedometer::newLine(float rStart, float rEnd, float angle)
{
    float c = cos(angle);
    float s = sin(angle);

    glVertex2f(this->rVel * rStart * c, this->rVel * rStart * s);
    glVertex2f(this->rVel * rEnd * c, this->rVel * rEnd * s);
}

void Speedometer::drawLabelsVelocimeter(void)
{
    this->colorVelocimeter[0] = character->buildColor(1.1f, 1.1f, 1.1f);
    this->colorVelocimeter[1] = character->buildColor(1.1f, 1.1f, 1.1f);
    this->colorVelocimeter[2] = character->buildColor(1.1f, 1.1f, 1.1f);
    this->colorVelocimeter[3] = character->buildColor(1.1f, 1.1f, 1.1f);
    this->colorVelocimeter[4] = character->buildColor(1.1f, 1.1f, 1.1f);
    this->colorVelocimeter[5] = character->buildColor(1.1f, 1.1f, 1.1f);

    glTranslatef(-29.0f, -53.0f, 0);
    character->draw(0, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(29.0f, 53.0f, 0);

    glTranslatef(-56.0f, -32.0f, 0);
    character->draw(1, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(5.0f, 0.0f, 0);
    character->draw(0, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(-5.0f, 0.0f, 0);
    glTranslatef(56.0f, 32.0f, 0);

    glTranslatef(-62.0f, 0.0f, 0);
    character->draw(2, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(5.0f, 0.0f, 0);
    character->draw(0, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(-5.0f, 0.0f, 0);
    glTranslatef(62.0f, 0.0f, 0);

    glTranslatef(-53.0f, 33.0f, 0);
    character->draw(3, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(5.0f, 0.0f, 0);
    character->draw(0, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(-5.0f, 0.0f, 0);
    glTranslatef(53.0f, -33.0f, 0);

    glTranslatef(-33.0f, 53.0f, 0);
    character->draw(4, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(5.0f, 0.0f, 0);
    character->draw(0, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(-5.0f, 0.0f, 0);
    glTranslatef(33.0f, -53.0f, 0);

    glTranslatef(0.0f, 62.0f, 0);
    character->draw(5, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(5.0f, 0.0f, 0);
    character->draw(0, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(-5.0f, 0.0f, 0);
    glTranslatef(0.0f, -62.0f, 0);

    glTranslatef(33.0f, 53.0f, 0);
    character->draw(6, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(5.0f, 0.0f, 0);
    character->draw(0, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(-5.0f, 0.0f, 0);
    glTranslatef(-33.0f, -53.0f, 0);

    glTranslatef(51.0f, 33.0f, 0);
    character->draw(7, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(5.0f, 0.0f, 0);
    character->draw(0, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(-5.0f, 0.0f, 0);
    glTranslatef(-51.0f, -33.0f, 0);

    glTranslatef(61.0f, 0.0f, 0);
    character->draw(8, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(5.0f, 0.0f, 0);
    character->draw(0, polygonsVelocimeter, 3, this->colorVelocimeter);
    glTranslatef(-5.0f, 0.0f, 0);
    glTranslatef(-61.0f, -0.0f, 0);
}

void Speedometer::drawCambio(void)
{
    glColor3f(0.0f, 0.0f, 1.0f);
    glLineWidth(2.0f);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);

    glTranslatef(-135, 0, 0);
    glBegin(GL_LINES);
    {
        float x;
        float y;

        int r = 50;

        for (int i = 0; i <= 180; i++)
        {
            x = r * cos(i);
            y = r * sin(i);

            if ((x >= -10) & (y >= 0) & (y <= 105))
            {
                glVertex3f(x, y, 0);

                x = r * cos(i + 0.1);
                y = r * sin(i + 0.1);
                glVertex3f(x, y, 0);
            }
        }
    }
    glEnd();
    glTranslatef(135, 0, 0);
}

void Speedometer::drawMargin(void)
{
    glColor3f(0.0f, 0.0f, 1.0f);
    glLineWidth(2.0f);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);

    //    glRotatef(40, 0, 0, 1);
    //    glBegin(GL_LINES);
    //    {
    //        float x;
    //        float y;
    //
    //        int r = 81;
    //
    //        for (int i = 0; i <= 180; i++)
    //        {
    //            x = r * cos(i);
    //            y = r * sin(i);
    //
    //            if ((x >= -81) & (x <= 81) & (y >= -57) & (y <= 81))
    //            {
    //                glVertex3f(x, y, 0);
    //
    //                x = r * cos(i + 0.1);
    //                y = r * sin(i + 0.1);
    //                glVertex3f(x, y, 0);
    //            }
    //        }
    //    }
    //    glEnd();
    //    glRotatef(-40, 0, 0, 1);

    // TOP
    glTranslatef(-60, 0, 0);
    glBegin(GL_LINES);
    {
        newLine(0, 1.8, 0);
    }
    glEnd();
    glTranslatef(60, 0, 0);

    // BOTTOM
    glTranslatef(-82, -80.9, 0);
    glBegin(GL_LINES);
    {
        newLine(0, 1.80, 0);
    }
    glEnd();
    glTranslatef(82, 80.9, 0);

    // LEFT
    glTranslatef(-50, 48, 0);
    glRotatef(-105, 0, 0, 1);
    glBegin(GL_LINES);
    {
        //newLine(0, 1.045, 0);
        newLine(0, 1.68, 0);
    }
    glEnd();
    glRotatef(105, 0, 0, 1);
    glTranslatef(50, -48, 0);

    // RIGHT
    glTranslatef(85, 0, 0);
    glRotatef(-105, 0, 0, 1);
    glBegin(GL_LINES);
    {
        newLine(0, 1.045, 0);
    }
    glEnd();
    glRotatef(105, 0, 0, 1);
    glTranslatef(-85, 0, 0);
}
