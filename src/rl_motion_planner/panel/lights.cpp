#include "../panel/lights.h"

#include <GL/glew.h>


Lights::Lights()
{
    this->light = 0.1f;
    this->highLight = 0.1f;
}


void
Lights::drawHighLight(void)
{
    glColor3f(0.0f, 0.0f, highLight);
    glLineWidth(5.0f);

    glBegin(GL_LINES);
    {
        float x;
        float y;

        int r = 15;

        for (int i = 0; i <= 180; i++)
        {
            x = r * cos(i);
            y = r * sin(i);

            if ((x >= 0)  & (y <= 14.9f))
            {
                glVertex3f((float)x, (float)y, 0.0f);

                x = r * cos(i + 0.1);
                y = r * sin(i + 0.1);
                glVertex3f((float)x, (float)y, 0.0f);
            }
        }
    }
    glEnd();

    glTranslatef(0.5f, -15.0f, 0.0f);
    glRotatef(90.0f, 0.0f, 0.0f, 1.0f);

    glBegin(GL_LINES);
    {
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(30.0f, 0.0f, 0.0f);
    }
    glEnd();

    glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);
    glTranslatef(-0.5f, 15.0f, 0.0f);

    glBegin(GL_LINES);
    {
        glVertex3f(-20.0f, 12.5f, 0.0f);
        glVertex3f(0.0f, 12.5f, 0.0f);

        glVertex3f(-20.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);

        glVertex3f(-20.0f, -12.5f, 0.0f);
        glVertex3f(0.0f, -12.5f, 0.0f);
    }
    glEnd();
}


void
Lights::drawLight(void)
{
    glColor3f(0.0f, light, 0.0f);
    glLineWidth(5.0f);

    glBegin(GL_LINES);
    {
        float x;
        float y;

        int r = 7.5;

        for (int i = 0; i <= 180; i++)
        {
            x = r * cos(i);
            y = r * sin(i);
            glVertex3f(x, y, 0.0f);

            x = r * cos(i + 0.1);
            y = r * sin(i + 0.1);
            glVertex3f(x, y, 0.0f);
        }
    }
    glEnd();

    glTranslatef(-7.5f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    {
        glVertex3f(-10.0f, 8.5f, 1.0f);
        glVertex3f(0.0f, 0.0f, 1.0f);

        glVertex3f(-10.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 1.0f);

        glVertex3f(-10.0f, -8.5f, 1.0f);
        glVertex3f(0.0f, 0.0f, 1.0f);
    }
    glEnd();
    glTranslatef(7.5f, 0.0f, 0.0f);
}
