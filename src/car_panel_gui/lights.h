#ifndef LIGHTS_H
#define LIGHTS_H

#include <GL/glew.h>
#include <math.h>

class Lights
{

public:
	Lights();

    float getLight()
    {
        return this->light;
    }

    void setLight(float light)
    {
        this->light = (light == 0.1f) ? 1.0f : 0.1f;
    }

    float getHighLight()
    {
        return this->highLight;
    }

    void setHighLight(float highLight)
    {
        this->highLight = (highLight == 0.1f) ? 1.0f : 0.1f;
    }

    void drawHighLight(void);
    void drawLight(void);

private:
    float highLight;
    float light;

};

#endif // LIGHTS_H
