#ifndef ARROW_H
#define ARROW_H

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
    double timer;
    float colorG;
    bool isFire;

    void resetValues(void);
};

#endif // ARROW_H
