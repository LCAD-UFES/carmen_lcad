#include "Robots/Scorbot/ScorbotInterface.H"
#include "Component/ModelManager.H"
#include <vector>

int main(int argc, char* argv[])
{
  ModelManager mgr("Test Scorbot Interface");

  nub::ref<ScorbotInterface> scorbot(new ScorbotInterface(mgr));
  mgr.addSubComponent(scorbot);

  if(mgr.parseCommandLine(argc, argv, "", 0, 0) == false) return -1;

  mgr.start();

  scorbot->setEnabled(true);

	std::vector<ScorbotInterface::encoderVals_t> positions;
	ScorbotInterface::encoderVals_t encoders;

encoders[ScorbotInterface::Base] = -3180;
encoders[ScorbotInterface::Shoulder] = 4886;
encoders[ScorbotInterface::Elbow] = -1938;
encoders[ScorbotInterface::Wrist1] = 2106;
encoders[ScorbotInterface::Wrist2] = -2187;
encoders[ScorbotInterface::Gripper] = 0;
encoders[ScorbotInterface::Slider] = -42027;
	positions.push_back(encoders);

	
encoders[ScorbotInterface::Base] = 1542;
encoders[ScorbotInterface::Shoulder] = 4178;
encoders[ScorbotInterface::Elbow] = -340;
encoders[ScorbotInterface::Wrist1] = 2327;
encoders[ScorbotInterface::Wrist2] = -2187;
encoders[ScorbotInterface::Gripper] = 0;
encoders[ScorbotInterface::Slider] = -42032;
	positions.push_back(encoders);

encoders[ScorbotInterface::Base] = -5269;
encoders[ScorbotInterface::Shoulder] = 3810;
encoders[ScorbotInterface::Elbow] = 2886;
encoders[ScorbotInterface::Wrist1] = 2187;
encoders[ScorbotInterface::Wrist2] = -2179;
encoders[ScorbotInterface::Gripper] = 0;
encoders[ScorbotInterface::Slider] = -42031;
	positions.push_back(encoders);



	while(1)
	{
		for(size_t posIdx=0; posIdx<positions.size(); posIdx++)
		{
			std::cout << "---------------------------------------" << std::endl;
			std::cout << "Going To Position " << posIdx << std::endl;
			scorbot->setJoints(positions[posIdx], 2000);
			sleep(2);
			std::cout << "Done..." << std::endl;

			ScorbotInterface::encoderVals_t currPos = scorbot->getEncoders();
			std::cout << "Base:     " << currPos[ScorbotInterface::Base]     << " / " << positions[posIdx][ScorbotInterface::Base] << std::endl;
			std::cout << "Shoulder: " << currPos[ScorbotInterface::Shoulder] << " / " << positions[posIdx][ScorbotInterface::Shoulder] << std::endl;
			std::cout << "Elbow:    " << currPos[ScorbotInterface::Elbow]    << " / " << positions[posIdx][ScorbotInterface::Elbow] << std::endl;
			std::cout << "Wrist1:   " << currPos[ScorbotInterface::Wrist1]   << " / " << positions[posIdx][ScorbotInterface::Wrist1] << std::endl;
			std::cout << "Wrist2:   " << currPos[ScorbotInterface::Wrist2]   << " / " << positions[posIdx][ScorbotInterface::Wrist2] << std::endl;
			std::cout << "Gripper:  " << currPos[ScorbotInterface::Gripper]  << " / " << positions[posIdx][ScorbotInterface::Gripper] << std::endl;
			std::cout << "Slider:   " << currPos[ScorbotInterface::Slider]   << " / " << positions[posIdx][ScorbotInterface::Slider] << std::endl;
			sleep(2);
		}
	}
}
