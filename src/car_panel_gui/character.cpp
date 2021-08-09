#include <GL/glew.h>
#include "character.h"


Character::Character()
{
    polygonWidth = 0;
}


void
Character::draw(int number, polygon polygons[], int width, colors color[])
{
    polygonWidth = width;

    int length = 0;

    switch (number)
    {
    case 0:
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(0.0, 0.0, 0.0, 0.0, -1.1, 0.0);
        polygons[2] = buildPolygon(90.0, 0.0, 0.0, 1.1, 0.0, 0.0);
        polygons[3] = buildPolygon(180.0, 0.0, 0.0, 1.1, -1.1, 0.0);
        polygons[4] = buildPolygon(180.0, 0.0, 0.0, 1.1, 0.0, 0.0);
        polygons[5] = buildPolygon(90.0, 0.0, 180.0, -1.1, -1.1, 0.0);

        length = 6;
        break;
    case 1:
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(0.0, 0.0, 0.0, 0.0, -1.1, 0.0);

        length = 2;
        break;
    case 2:
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(90.0, 0.0, 0.0, 1.1, 0.0, 0.0);
        polygons[2] = buildPolygon(90.0, 0.0, 180.0, 0.0, -1.1, 0.0);
        polygons[3] = buildPolygon(0.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[4] = buildPolygon(90.0, 0.0, 0.0, -1.1, 0.0, 0.0);

        length = 5;
        break;
    case 3:
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(90.0, 0.0, 0.0, 1.1, 0.0, 0.0);
        polygons[2] = buildPolygon(90.0, 0.0, 180.0, 0.0, -1.1, 0.0);
        polygons[3] = buildPolygon(0.0, 0.0, 0.0, 0.0, -1.1, 0.0);
        polygons[4] = buildPolygon(90.0, 0.0, 0.0, -1.1, 0.0, 0.0);

        length = 5;
        break;
    case 4:
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(180.0, 0.0, 0.0, 1.1, -1.1, 0.0);
        polygons[2] = buildPolygon(90.0, 0.0, 180.0, 0.0, -1.1, 0.0);
        polygons[3] = buildPolygon(0.0, 0.0, 0.0, 0.0, -1.1, 0.0);

        length = 4;
        break;
    case 5:
        polygons[0] = buildPolygon(270.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[1] = buildPolygon(0.0, 0.0, 0.0, -1.1, 0.0, 0.0);
        polygons[2] = buildPolygon(90.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[3] = buildPolygon(0.0, 0.0, 0.0, 0.0, -1.1, 0.0);
        polygons[4] = buildPolygon(270.0, 0.0, 0.0, 1.1, -1.1, 0.0);

        length = 5;
        break;
    case 6:
        polygons[0] = buildPolygon(270.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[1] = buildPolygon(0.0, 0.0, 0.0, -1.1, 0.0, 0.0);
        polygons[2] = buildPolygon(90.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[3] = buildPolygon(0.0, 0.0, 0.0, 0.0, -1.1, 0.0);
        polygons[4] = buildPolygon(270.0, 0.0, 0.0, 1.1, -1.1, 0.0);
        polygons[5] = buildPolygon(0.0, 0.0, 0.0, -1.1, -1.1, 0.0);

        length = 6;
        break;
    case 7:
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(0.0, 0.0, 0.0, 0.0, -1.1, 0.0);
        polygons[2] = buildPolygon(90.0, 0.0, 0.0, 1.1, 0.0, 0.0);

        length = 3;
        break;
    case 8:
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(270.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[2] = buildPolygon(0.0, 0.0, 0.0, -1.1, 0.0, 0.0);
        polygons[3] = buildPolygon(90.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[4] = buildPolygon(0.0, 0.0, 0.0, 0.0, -1.1, 0.0);
        polygons[5] = buildPolygon(270.0, 0.0, 0.0, 1.1, -1.1, 0.0);
        polygons[6] = buildPolygon(0.0, 0.0, 0.0, -1.1, -1.1, 0.0);

        length = 7;
        break;
    case 9:
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(270.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[2] = buildPolygon(0.0, 0.0, 0.0, -1.1, 0.0, 0.0);
        polygons[3] = buildPolygon(90.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[4] = buildPolygon(0.0, 0.0, 0.0, 0.0, -1.1, 0.0);
        polygons[5] = buildPolygon(270.0, 0.0, 0.0, 1.1, -1.1, 0.0);

        length = 6;
        break;
    case 'N':
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, -1.1, 0.0, 0.0);
        polygons[1] = buildPolygon(0.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[2] = buildPolygon(25.0, 0.0, 0.0, -0.4, 0.0, 0.0);
        polygons[3] = buildPolygon(25.0, 0.0, 0.0, -0.3, -1.1, 0.0);
        polygons[4] = buildPolygon(0.0, 0.0, 0.0, 0.4, 0.0, 0.0);
        polygons[5] = buildPolygon(0.0, 0.0, 0.0, 0.4, -1.1, 0.0);

        length = 6;
        break;
    case 'L':
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, -1.1, 0.0, 0.0);
        polygons[1] = buildPolygon(0.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[2] = buildPolygon(90.0, 0.0, 0.0, -1.1, 0.0, 0.0);

        length = 3;
        break;
    case 'o':
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(270.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[2] = buildPolygon(0.0, 0.0, 0.0, -1.1, 0.0, 0.0);
        polygons[3] = buildPolygon(90.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        length = 4;
        break;
    case 'R':
        polygons[0] = buildPolygon(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[1] = buildPolygon(270.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[2] = buildPolygon(0.0, 0.0, 0.0, -1.1, 0.0, 0.0);
        polygons[3] = buildPolygon(90.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        polygons[4] = buildPolygon(0.0, 0.0, 0.0, -1.1, -1.1, 0.0);
        polygons[5] = buildPolygon(45.0, 0.0, 180.0, -0.6, -0.4, 0.0);

        length = 6;
        break;
    }

    for (int i = 0; i < length; i++)
    {
        glRotatef(polygons[i].rx, 0, 0, 1);
        glRotatef(polygons[i].ry, 0, 1, 0);
        glRotatef(polygons[i].rz, 1, 0, 0);

        glTranslatef(polygons[i].tx * polygonWidth, polygons[i].ty * polygonWidth, polygons[i].tz * polygonWidth);

        glBegin(GL_POLYGON);
        {
            glColor3f(color[0].r, color[0].g, color[0].b);
            glVertex2f(polygons[i].ax, polygons[i].ay);

            glColor3f(color[1].r, color[1].g, color[1].b);
            glVertex2f(polygons[i].bx, polygons[i].by);

            glColor3f(color[2].r, color[2].g, color[2].b);
            glVertex2f(polygons[i].cx, polygons[i].cy);

            glColor3f(color[3].r, color[3].g, color[3].b);
            glVertex2f(polygons[i].dx, polygons[i].dy);

            glColor3f(color[4].r, color[4].g, color[4].b);
            glVertex2f(polygons[i].ex, polygons[i].ey);

            glColor3f(color[5].r, color[5].g, color[5].b);
            glVertex2f(polygons[i].fx, polygons[i].fy);
        }
        glEnd();

        glTranslatef(-polygons[i].tx * polygonWidth, -polygons[i].ty * polygonWidth, -polygons[i].tz * polygonWidth);

        glRotatef(-polygons[i].rz, 1, 0, 0);
        glRotatef(-polygons[i].ry, 0, 1, 0);
        glRotatef(-polygons[i].rx, 0, 0, 1);
    }
}


colors
Character::buildColor(float r, float g, float b)
{
    colors color;

    color.r = r;
    color.g = g;
    color.b = b;

    return color;
}


void
Character::drawSinglePoint(colors color[])
{
    float width = 0.2f * this->polygonWidth;
    
    glBegin(GL_POLYGON);
    {
        glColor3f(color[0].r, color[0].g, color[0].b);
        glVertex2f(0.0f, width);

        glColor3f(color[1].r, color[1].g, color[1].b);
        glVertex2f(width / 2, width);

        glColor3f(color[2].r, color[2].g, color[2].b);
        glVertex2f(width, width);

        glColor3f(color[3].r, color[3].g, color[3].b);
        glVertex2f(width, 0.0f);

        glColor3f(color[4].r, color[4].g, color[4].b);
        glVertex2f(width / 2, 0.0f);

        glColor3f(color[5].r, color[5].g, color[5].b);
        glVertex2f(0.0f, 0.0f);
    }
    glEnd();    
}


polygon
Character::buildPolygon(float rx, float ry, float rz, float tx, float ty, float tz)
{
    polygon polygon;

    polygon.ax = -0.05 * polygonWidth;
    polygon.ay = 1.0 * polygonWidth;
    polygon.bx = 0.0 * polygonWidth;
    polygon.by = 1.1 * polygonWidth;
    polygon.cx = 0.05 * polygonWidth;
    polygon.cy = 1.0 * polygonWidth;
    polygon.dx = 0.05 * polygonWidth;
    polygon.dy = 0.1 * polygonWidth;
    polygon.ex = 0.0 * polygonWidth;
    polygon.ey = 0.0 * polygonWidth;
    polygon.fx = -0.05 * polygonWidth;
    polygon.fy = 0.1 * polygonWidth;
    polygon.rx = rx;
    polygon.ry = ry;
    polygon.rz = rz;
    polygon.tx = tx;
    polygon.ty = ty;
    polygon.tz = tz;

    return polygon;
}
