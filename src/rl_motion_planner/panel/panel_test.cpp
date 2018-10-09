
#include <carmen/carmen.h>
#include "panel.h"

int main()
{
	CarPanel panel;
	double v = 10, phi = 0.45;

	while (1)
	{
		panel.draw(v, phi, carmen_get_time());
		usleep(10);
	}

	return 0;
}
