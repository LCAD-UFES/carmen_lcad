
#include <cstdio>
#include <string>
#include <carmen/tf.h>

using namespace std;
using namespace tf;


int
main()
{
	Transformer transformer(false, Duration(0));
	Transform t(Quaternion(0, 0, 0), Vector3(0, 0, 0));
	StampedTransform t_lookup;

	transformer.setTransform(StampedTransform(t, tf::Time(1), string("/bola"), string("/agua")), "bola_and_agua");
	transformer.setTransform(StampedTransform(t, tf::Time(1), string("/agua"), string("/casa")), "casa_and_agua");
	transformer.setTransform(StampedTransform(t, tf::Time(1), string("/dedo"), string("/bola")), "bola_and_dedo");

	for (int n = 0; n < 100 * 100; n++)
	{
		for (int i = 0; i < 100000; i++)
		{
			transformer.setTransform(StampedTransform(Transform(Quaternion(0, 0, 0), Vector3(n, n, n)),
																								tf::Time(1), string("/bola"), string("/agua")), "bola_and_agua");

			if (i == 0)
			{
				transformer.lookupTransform("/agua", "/bola", tf::Time(0), t_lookup);
				printf("agua_bola.x: %lf agua_bola.y: %lf\n", t_lookup.getOrigin().x(), t_lookup.getOrigin().y());

				transformer.lookupTransform("/casa", "/agua", tf::Time(0), t_lookup);
				printf("casa_agua.x: %lf casa_agua.y: %lf\n", t_lookup.getOrigin().x(), t_lookup.getOrigin().y());

				transformer.lookupTransform("/dedo", "/bola", tf::Time(0), t_lookup);
				printf("bola_dedo.x: %lf bola_dedo.y: %lf\n", t_lookup.getOrigin().x(), t_lookup.getOrigin().y());

				printf("\n");
			}
		}
	}

	printf("Press any key to finish...\n");
	getchar();

	return 0;
}

