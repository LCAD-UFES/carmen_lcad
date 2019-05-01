
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
	StampedTransform s_t_update;

	transformer.setTransform(s_t, "name");

	TransformStorage storage;

	for (int n = 0; n < 100 * 100; n++)
	{
		for (int i = 0; i < 100000; i++)
		{
			//transformer.setTransform(s_t, "name");

			TimeCache *cache = transformer.getFrameCarmen("/agua");
			cache->getData(tf::Time(0), storage);
			storage.translation_ = Vector3(i, i, i);

			transformer.lookupTransform("/agua", "/bola", tf::Time(0), s_t_update);
			printf("s_t_update.x: %lf s_t_update.y: %lf\n", s_t_update.getOrigin().x(), s_t_update.getOrigin().y());
			//s_t_update.setOrigin(Vector3(i, i, i));
		}
	}

	printf("Press any key to finish...\n");
	getchar();

	return 0;
}

