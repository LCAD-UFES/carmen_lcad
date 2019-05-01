
#include <cstdio>
#include <string>
#include <carmen/tf.h>

using namespace std;
using namespace tf;


int
main()
{
	Transformer transformer;
	Transform t(Quaternion(0, 0, 0), Vector3(0, 0, 0));
	StampedTransform s_t(t, tf::Time(1000000), string("/bola"), string("/agua"));

	for (int n = 0; n < 100 * 100; n++)
	{
		for (int i = 0; i < 100000; i++)
		{
			transformer.setTransform(s_t, "name");
		}
	}

	printf("Press any key to finish...\n");
	getchar();

	return 0;
}

