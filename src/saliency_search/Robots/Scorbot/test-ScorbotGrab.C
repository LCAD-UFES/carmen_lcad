#include <Ice/Ice.h>
#include "Raster/Raster.H"
#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "GUI/PrefsWindow.H"
#include "Devices/DeviceOpts.H"
#include "Devices/Serial.H"
#include "Image/DrawOps.H"
#include <vector>
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/PrefsWindow.H"
#include "GUI/DebugWin.H"
#include <pthread.h>
#include <iomanip>

int numPossibleObjects = 30;
int numSlots = 25;
std::vector<std::set<int> > slotObjs;

nub::soft_ref<OutputFrameSeries> ofs;
nub::soft_ref<InputFrameSeries> ifs;

class ScorbotReallySimple: public ModelComponent
{
	public:
		ScorbotReallySimple(OptionManager& mgr, 
				const std::string& descrName = "",
				const std::string& tagName = "") :
			ModelComponent(mgr, descrName, tagName),
			itsSerial(new Serial(mgr)) 
		{
			addSubComponent(itsSerial);
			itsSerial->configure("/dev/ttyUSB0", 115200, "8N1", false, false, 10000);
			//itsSerial->setBlocking(true);
		}

		bool getState()
		{

			int ret;
			//char tmpbuf[1024];
			//int ret = itsSerial->read(tmpbuf, 1024);
			//LINFO("Got %i", ret);

			char retVal = '?';
			for(int i=0; i<10; i++)
			{
				char cmd = 'S';
				itsSerial->write(&cmd, 1);
				usleep(10000);

				ret = itsSerial->read(&retVal, 1);
				LINFO("%i: RetVal %c", ret, retVal);
				if (ret > 0) break;

				itsSerial->enablePort("/dev/ttyUSB0");

			}

			return (retVal == '1');

		}

		void setPos(int pos)
		{
			LINFO("Moving to %i", pos);
			char cmd[2];
			cmd[0] = 'M';
			cmd[1] = pos;
			itsSerial->write(&cmd, 2);

			sleep(5);

			//LINFO("Wait for 0");
			//while(getState()) usleep(100000);
			LINFO("Wait for 1");
			while(!getState()) usleep(10000);
			LINFO("Move Done");
		}

	private:
		nub::ref<Serial> itsSerial;
};

// ######################################################################
int getKey()
{
	const nub::soft_ref<ImageDisplayStream> ids =
		ofs->findFrameDestType<ImageDisplayStream>();

	const rutz::shared_ptr<XWinManaged> uiwin =
		ids.is_valid()
		? ids->getWindow("Output")
		: rutz::shared_ptr<XWinManaged>();
	return uiwin->getLastKeyPress();
}

// ######################################################################
std::map<int, Image<PixRGB<byte> > > loadObjectImgs()
{
	std::map<int, Image<PixRGB<byte> > > imgs;
	for(int i=1; i<=numPossibleObjects; i++)
	{
		char filename[256];
		sprintf(filename, "/home/igpu/Desktop/MicroMachines/cropped/%d.png", i);
		imgs[i] = Raster::ReadRGB(filename);
	}
	return imgs;
}

// ######################################################################
void initPossibleObjs()
{
	slotObjs.clear();

	for(int slotIdx=0; slotIdx<numSlots; slotIdx++)
	{
		slotObjs.push_back(std::set<int>());
		for(int objIdx=1; objIdx<=numPossibleObjects; objIdx++)
		{
			slotObjs[slotIdx].insert(objIdx);
		}
	}
}

// ######################################################################
std::vector<int> randomizeSlots(int slotOffset, int slotsInc)
{
	std::vector<int> slotAssignments(numSlots, -1);

	//Make a copy of slotObjs
	std::vector<std::set<int> > tmpSlotObjs = slotObjs;

	//For each slot, pick a random index from it's list of available objects
	for(int slotIdx = slotOffset; slotIdx<numSlots; slotIdx+=slotsInc)
	{
		//Make sure there are objects left to go in this slot
		if(tmpSlotObjs[slotIdx].size() == 0)
		{
			std::cout << "NO MORE OBJECTS AVAILABLE FOR SLOT " << slotIdx << " -- EXITING" << std::endl;
			exit(0);
		}

		//Randomly pick an object from those available for the slot
		int objIdx = rand() % (int)tmpSlotObjs[slotIdx].size();

		//Linearly roll through the avaible objects to get to the id at our random index
		std::set<int>::iterator objIdIt = tmpSlotObjs[slotIdx].begin();
		for(int objItIdx=0; objItIdx!=objIdx; objItIdx++) objIdIt++;

		//Assign the object to the slot
		int objId = *objIdIt;
		slotAssignments[slotIdx] = objId;

		//Never let this object be assigned to this slot again
		slotObjs[slotIdx].erase(slotObjs[slotIdx].find(objId));

		//Make this object unavailable for other slots in this run
		for(int i=0; i<numSlots; i++)
		{
			std::set<int>::iterator objIt = tmpSlotObjs[i].find(objId);
			if(objIt != tmpSlotObjs[i].end()) tmpSlotObjs[i].erase(objId);
		}
	}

	return slotAssignments;
}


// ######################################################################
int main(int argc, char* argv[])
{
	LINFO("#############################################STARTING##############################################");
	//srand(unsigned(time(NULL)));

	std::vector<Point2D<int> > slotCoords(numSlots);
	slotCoords[0]  = Point2D<int>(782,634);
	slotCoords[1]  = Point2D<int>(812,538);
	slotCoords[2]  = Point2D<int>(844,433);
	slotCoords[3]  = Point2D<int>(866,344);
	slotCoords[4]  = Point2D<int>(890,244);
	slotCoords[5]  = Point2D<int>(915,164);
	slotCoords[6]  = Point2D<int>(945,80);
	slotCoords[7]  = Point2D<int>(540,614);
	slotCoords[8]  = Point2D<int>(530,503);
	slotCoords[9]  = Point2D<int>(515,366);
	slotCoords[10] = Point2D<int>(538,258);
	slotCoords[11] = Point2D<int>(593,162);
	slotCoords[12] = Point2D<int>(643,90);
	slotCoords[13] = Point2D<int>(93,451);
	slotCoords[14] = Point2D<int>(185,411);
	slotCoords[15] = Point2D<int>(273,366);
	slotCoords[16] = Point2D<int>(387,301);
	slotCoords[17] = Point2D<int>(472,209);
	slotCoords[18] = Point2D<int>(492,128);
	slotCoords[19] = Point2D<int>(202,144);
	slotCoords[20] = Point2D<int>(736,210); 
	slotCoords[21] = Point2D<int>(1161,466); 
	slotCoords[22] = Point2D<int>(1028,429); 
	slotCoords[23] = Point2D<int>(1187,297); 
	slotCoords[24] = Point2D<int>(347,47);  

	ModelManager mgr("Test Scorbot Interface");

	nub::ref<ScorbotReallySimple> scorbot(new ScorbotReallySimple(mgr));
	mgr.addSubComponent(scorbot);

	ifs.reset(new InputFrameSeries(mgr));
	mgr.addSubComponent(ifs);

	ofs.reset(new OutputFrameSeries(mgr));
	mgr.addSubComponent(ofs);

	if(mgr.parseCommandLine(argc, argv, "filename", 1, 1) == false) return -1;
	mgr.start();

	std::string fileName = mgr.getExtraArg(0);


  //while(true)
	//{
	//	for(int pos=1; pos<25; pos++)
	//		scorbot->setPos(pos);
	//}

//	while(true)
//		scorbot->getState();

	PrefsWindow pWin("Camera Control", SimpleFont::FIXED(8));
	pWin.setValueNumChars(16);
	pWin.addPrefsForComponent(ifs.get(), true);

	//Load in the object images
	std::map<int, Image<PixRGB<byte> > > objectImgs = loadObjectImgs();

	//Initialize the data structure for our randomization algorithm
	initPossibleObjs();

	ifs->startStream();

	// ######################################################################
	// LOOP
	// ######################################################################
	int sceneId = 0;

	int slotIncrement = 3; // How many slots to skip when placing toys
	int slotOffset    = 0; // On which slot should we start

	while(1)
	{

		//Set the first focus
		char focusString[128];
		sprintf(focusString, "%d", 7);
		mgr.setOptionValString(&OPT_FrameGrabberFocus, focusString);

		//Randomize which toy goes into which slot
		std::vector<int> objAssignments(numSlots, -1);
		if (sceneId > 0)
			objAssignments = randomizeSlots(slotOffset, slotIncrement);

		slotOffset = (slotOffset+1)%slotIncrement;

		for(int ori=0; ori<2; ori++)
		{
			//Go to position the top position
			scorbot->setPos(63);

			int key = -1;
			do 
			{
				pWin.update();

				if(ifs->updateNext() == FRAME_COMPLETE) break;
				GenericFrame input = ifs->readFrame();

				//Display the new frame
				Image<PixRGB<byte> > img = input.asRgb();

				for(size_t slotIdx=0; slotIdx<slotCoords.size(); slotIdx++)
				{
					if(objAssignments[slotIdx] != -1)
					{
						Point2D<int> center = slotCoords[slotIdx] + Point2D<int>(0, -15);
						drawRect(img, Rectangle::centerDims(center, Dims(40,40)), PixRGB<byte>(255, 0, 0));
						drawCircle(img, center, 3, PixRGB<byte>(255, 0, 0), 3);

						int objId = objAssignments[slotIdx];

						Rectangle objImgRect = img.getBounds().getOverlap(objectImgs[objId].getBounds()/2 + center + Point2D<int>(-20, objectImgs[objId].getHeight()/4));

						inplaceEmbed(img, objectImgs[objId], objImgRect, PixRGB<byte>(0,0,0));

						char buffer[128];
						sprintf(buffer, "%d", (int)objId);

						writeText(img, center+Point2D<int>(0, 20), buffer, PixRGB<byte>(0, 0, 0));
					}
				}

				char msg[255];
				sprintf(msg, "Please place items in  ori %i the slots indicated above and then press ENTER", ori);

				writeText(img, Point2D<int>(0, img.getHeight() - 20), msg , PixRGB<byte>(255,0,0));

				ofs->writeRGB(img, "Output", FrameInfo("output", SRC_POS));
				ofs->updateNext();

				key = getKey();
				if(key != -1) LINFO("Pressed: %d", key);

			}while(key != 36 && key != 44);


			// Enumerate the focus values for each position
			std::vector<int> focusVals(numSlots);
			focusVals[0]  = 9;   	
			focusVals[1]  = 9;	  
			focusVals[2]  = 9;	  
			focusVals[3]  = 10;	  
			focusVals[4]  = 10;	  
			focusVals[5]  = 10;	  
			focusVals[6]  = 9;	  
			focusVals[7]  = 9;	  
			focusVals[8]  = 9;	  
			focusVals[9]  = 9;	  
			focusVals[10] = 9;	  
			focusVals[11] = 9;	  
			focusVals[12] = 9;	  
			focusVals[13] = 9;	  
			focusVals[14] = 9;	  
			focusVals[15] = 10;	  
			focusVals[16] = 12;	  
			focusVals[17] = 10;	  
			focusVals[18] = 10;	  
			focusVals[19] = 9;	  
			focusVals[20] = 11;	  
			focusVals[21] = 10;	  
			focusVals[22] = 10;	  
			focusVals[23] = 11;	  
			focusVals[24] = 11;	  

			// Enumerate the target type in each slot as a string 
			std::vector<std::string> targetType(numSlots,"none");
			for(int slotIdx=0; slotIdx<numSlots; slotIdx++)
			{
				std::string objType;

				int objId = objAssignments[slotIdx];
				if(objId == -1)
					objType = "none";
				else if(objId > 0 && objId <= 10)
					objType = "boat";
				else if(objId > 10 && objId <= 20)
					objType = "car";
				else if(objId > 20 && objId <= 30)
					objType = "tank";

				targetType[slotIdx] = objType;
			}

			std::cout << "----------------------------------------" << std::endl;
			std::cout << "----------------------------------------" << std::endl;

			for(size_t i=0; i<objAssignments.size(); i++)
				std::cout << std::setw(5) << objAssignments[i] << " ";
			std::cout << std::endl;

			for(size_t i=0; i<targetType.size(); i++)
				std::cout << std::setw(5) << targetType[i] << " ";
			std::cout << std::endl;


			for(int positionIndex=0; positionIndex<numSlots; positionIndex++)
			{
				if(sceneId > 0 && objAssignments[positionIndex] == -1) continue;

				LINFO("Moving To %d", positionIndex);

				//Tell the box to go to the next position
				scorbot->setPos(positionIndex+1);

				//We've reached the endpoint
				LINFO("Grabbing Images...");

				for(int focus=focusVals[positionIndex]-4; focus < focusVals[positionIndex]+4; focus+=4)
				{ 
					int foc = focus%40;
					if(foc <0) foc = 40+foc;
					//Set the focus
					char focusString[128];
					sprintf(focusString, "%d", foc);
					mgr.setOptionValString(&OPT_FrameGrabberFocus, focusString);

					GenericFrame input;
					for(int i=0; i<30; i++)
					{
						if(ifs->updateNext() == FRAME_COMPLETE) break;
						input = ifs->readFrame();
					}

					//Display the new frame
					Image<PixRGB<byte> > img = input.asRgb();
					ofs->writeRGB(img, "Output", FrameInfo("output", SRC_POS));
					ofs->updateNext();

					LINFO("Final State: %d", scorbot->getState());

					bool inFocus = (foc == focusVals[positionIndex]);
					//Write the frame
					char fileNameBuffer[256];
					sprintf(fileNameBuffer, "%s/%d_%d_%d_%d_%s_%d_%d_%lu.pnm", fileName.c_str(),
							sceneId,
							positionIndex,                      // Position Index
							ori,
							objAssignments[positionIndex],
							targetType[positionIndex].c_str(),  // Target type
							foc,                                // Actual focus value
							inFocus,                            // Whether this value is in focus
							time(NULL));

					Raster::WriteRGB(img, fileNameBuffer);

				} // for(focus)

			} // for(positionIndex)

			if (sceneId == 0) ori = 2; //No orientation for sceneId 0
		}

		LINFO("Done!");
		sceneId++;
	} // while(1) 

} // main

