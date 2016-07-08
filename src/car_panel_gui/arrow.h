#ifndef ARROW_H
#define ARROW_H

#include <GL/glew.h>

class Arrow
{
public:
	Arrow();

	bool getIsFire()
	{
		return this->isFire;
	}
    void draw(void);
    void blink(void);

private:
    static const int timer = 25;

    float colorG;
    int colorTime;
    bool isFire;

    void resetValues(void);
};

#endif // ARROW_H
