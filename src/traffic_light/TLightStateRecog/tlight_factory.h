
#ifndef TLIGHT_FACTORY_H_
#define TLIGHT_FACTORY_H_

#include <string>
#include <carmen/tlight_state_recog.h>
#include <carmen/tlight_mlp.h>
#include <carmen/tlight_vgram.h>

class TLightStateRecogFactory
{
public:

	/**
	 * Current recognizers:
	 * name = "vgram"
	 * name = "svm"
	 * name = "mlp"
	 */
	static TLightRecogInterface* build(std::string name)
	{
		if (name == "mlp")
			return new TLightMlp();
		else if (name == "vgram")
			return new TLightVgRam();
		else
		{
			printf("Error::TLightRecogInterface::build()::Unknown recognizer '%s'\n", name.c_str());
			return NULL;
		}
	}
};



#endif
