#ifndef CHARACTER_H_
#define CHARACTER_H_

typedef struct
{
    float ax;
    float ay;
    float bx;
    float by;
    float cx;
    float cy;
    float dx;
    float dy;
    float ex;
    float ey;
    float fx;
    float fy;
    float rx;
    float ry;
    float rz;
    float tx;
    float ty;
    float tz;
} polygon;

typedef struct
{
    float r;
    float g;
    float b;
} colors;

class Character
{

public:

	Character();
	void draw(int number, polygon polygons[], int width, colors color[]);
    void drawSinglePoint(colors color[]);
    colors buildColor(float r, float g, float b);

private:

    int polygonWidth;

	polygon buildPolygon(float rx, float ry, float rz, float tx, float ty, float tz);

};

#endif /* CHARACTER_H_ */
