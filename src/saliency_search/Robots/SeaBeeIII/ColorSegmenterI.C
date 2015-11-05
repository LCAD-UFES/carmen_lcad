#include "Robots/SeaBeeIII/ColorSegmenterI.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "Raster/Raster.H"

#ifndef COLORSEGMENTERI_C
#define COLORSEGMENTERI_C

// ######################################################################
ColorSegmenterI::ColorSegmenterI(OptionManager& mgr,
		const std::string& descrName, const std::string& tagName) :
	VisionBrainComponentI(mgr, descrName, tagName), itsInit(false),
			itsBuoySegmenter(2), itsPipeSegmenter(2)
{

}

// ######################################################################
ColorSegmenterI::~ColorSegmenterI()
{
}

void ColorSegmenterI::registerTopics()
{
	registerPublisher("RetinaMessageTopic");
	registerPublisher("BuoyColorSegmentMessageTopic");
	registerPublisher("PipeColorSegmentMessageTopic");
	//registerSubscription("BuoyColorSegmentConfigMessageTopic");
	//registerSubscription("PipeColorSegmentConfigMessageTopic");
	registerVisionTopics();
}

void ColorSegmenterI::updateFrame(Image<PixRGB<byte> > img, string cameraId)
{
	bool isFwdCamera = false;
	if (cameraId == "FwdCamera") //once the L/RDnCamera identities work, we'll look for orange in Dn and red in Fwd
		isFwdCamera = true;

	updateFrame(img, isFwdCamera);

}

void ColorSegmenterI::updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera)
{
	//LINFO("Image Recieved");

	if (img.initialized())
	{
		/* Initializates itsColorSegmenter with the dimensions
		 of the images we are going to be receiving. Runs only
		 once.*/
		if (!itsInit)
		{
			LINFO("here\n");
			/*
			 SIsetXXX(double center value, double thresh = how much data can deviate from center,
			 double skew(+- indicates side of skew, magnitude is amount to extend bounds on that side))
			 */
			//uncommented values are values that work well for footage
			//itsBuoySegmenter.SIsetHue(320, 85, 0);
			//itsBuoySegmenter.SIsetSat(55, 45, 0);
			//itsBuoySegmenter.SIsetVal(128, 128, 0);
			//^old values for red

			//new values for red
			itsBuoySegmenter.SIsetHue(352, 22, 0);
			itsBuoySegmenter.SIsetSat(55, 42, 0);
			itsBuoySegmenter.SIsetVal(128, 128, 0);

			//itsPipeSegmenter.SIsetHue(150, 50, 0);
			//itsPipeSegmenter.SIsetSat(28, 18, 0);
			////itsPipeSegmenter.SIsetSat(34,24,0);
			//itsPipeSegmenter.SIsetVal(165.75, 38.25, 0);
			//^old values for yellow

			//new values for orange
			itsPipeSegmenter.SIsetHue(22, 10, 0);
			itsPipeSegmenter.SIsetSat(55, 30, 0);
			itsPipeSegmenter.SIsetVal(128, 128, 0);

			//SIsetFrame(x1,y1,x2,y2,realX,realY) limits the consideration into the rectangle from (x1,y1) to (x2,y2)
			itsBuoySegmenter.SIsetFrame(0, 0, img.getWidth(), img.getHeight(),
					img.getWidth(), img.getHeight());
			itsPipeSegmenter.SIsetFrame(0, 0, img.getWidth(), img.getHeight(),
					img.getWidth(), img.getHeight());
			itsInit = true;
		}

		Image<PixRGB<float> > display(img);

		//segmenters do the color segmenting here
		itsBuoySegmenter.SIsegment(display);
		itsPipeSegmenter.SIsegment(display);

		//Candidates are pixels that fit within the accepted range that also fit in the previous frame (noise elimination)
		Image<bool> buoyCand = itsBuoySegmenter.SIreturnCandidates();
		Image<bool> pipeCand = itsPipeSegmenter.SIreturnCandidates();

		///*The code between this ///* and the following //*/ can be commented out when not looking at output to improve performance.
		//work-around to get writeRGB to display something visible;
		Image<PixRGB<byte> > segImgDisp; //Image to display to screen
		segImgDisp.resize(buoyCand.getWidth(), buoyCand.getHeight());

		for (int i = 0; i < buoyCand.getWidth(); ++i)
		{
			for (int j = 0; j < buoyCand.getHeight(); ++j)
			{
				//LINFO("x=%d,y=%d",i,j);
				// if(buoyCand.getVal(i,j) and pipeCand.getVal(i,j)){
				//   segImgDisp.setVal(i,j,PixRGB<byte>(255,128,0));
				//}
				if (buoyCand.getVal(i, j))
				{
					segImgDisp.setVal(i, j, PixRGB<byte> (255, 0, 0));
				}
				else if (pipeCand.getVal(i, j))
				{
					segImgDisp.setVal(i, j, PixRGB<byte> (255, 127, 0));
				}
				else
				{
					segImgDisp.setVal(i, j, PixRGB<byte> (0, 0, 255));
				}
			}
		}

		itsOfs->writeRGB(concatLooseX(img, segImgDisp, PixRGB<byte> (0, 0, 0)),
				"Color Segmenter Image", FrameInfo("Color Segementer", SRC_POS));

		//*/Code block can be commented out when not debugging output

		//sends out Retina Messages with the candidate images
		RobotSimEvents::RetinaMessagePtr msg =
				new RobotSimEvents::RetinaMessage;
		msg->img = Image2Ice(buoyCand);
		msg->cameraID = "BuoyColorSegmenter";

		this->publish("RetinaMessageTopic", msg);

		msg = new RobotSimEvents::RetinaMessage;
		msg->img = Image2Ice(pipeCand);
		msg->cameraID = "PipeColorSegmenter";

		this->publish("RetinaMessageTopic", msg);

		//blob properties calculated here
		itsBuoySegmenter.SIcalcMassCenter();
		itsPipeSegmenter.SIcalcMassCenter();

		//the following code puts the info about the red blobs (buoy) and yellow blobs(pipe) into a sorted array to send out as location messages
		int i = 0;
		vector<pair<float, pair<int, int> > > redBlobs(
				itsBuoySegmenter.SInumberBlobs());
		for (i = 0; i < itsBuoySegmenter.SInumberBlobs(); ++i)
		{
			redBlobs[i] = pair<float, pair<int, int> > (
					itsBuoySegmenter.SIgetMass(i), pair<int, int> (
							itsBuoySegmenter.SIgetCenterX(i),
							itsBuoySegmenter.SIgetCenterY(i)));
			/*LINFO("Red Segmenter Blob %d: center location: (%f, %f) mass: %ld, Xrange: [%d, %d], Yrange: [%d, %d]\n",
					i, itsBuoySegmenter.SIgetCenterX(i),itsBuoySegmenter.SIgetCenterY(i),itsBuoySegmenter.SIgetMass(i),
					itsBuoySegmenter.SIgetXmin(i),itsBuoySegmenter.SIgetXmax(i),itsBuoySegmenter.SIgetYmin(i),itsBuoySegmenter.SIgetYmax(i));
					*/
		}
		vector<pair<float, pair<int, int> > > yellowBlobs(
				itsPipeSegmenter.SInumberBlobs());
		for (i = 0; i < itsPipeSegmenter.SInumberBlobs(); ++i)
		{
			yellowBlobs[i] = pair<float, pair<int, int> > (
					itsPipeSegmenter.SIgetMass(i), pair<int, int> (
							itsPipeSegmenter.SIgetCenterX(i),
							itsPipeSegmenter.SIgetCenterY(i)));
			/*LINFO("Yellow Segmenter Blob %d: center location: (%f, %f) mass: %ld, Xrange: [%d, %d], Yrange: [%d, %d]\n",
					i, itsPipeSegmenter.SIgetCenterX(i),itsPipeSegmenter.SIgetCenterY(i),itsPipeSegmenter.SIgetMass(i),
					itsPipeSegmenter.SIgetXmin(i),itsPipeSegmenter.SIgetXmax(i),itsPipeSegmenter.SIgetYmin(i),itsPipeSegmenter.SIgetYmax(i));*/
		}
		sort(redBlobs.begin(), redBlobs.end());
		sort(yellowBlobs.begin(), yellowBlobs.end());

		//for each blob, sends out a message with center location and size
		RobotSimEvents::BuoyColorSegmentMessagePtr msg2;
		for (i = itsBuoySegmenter.SInumberBlobs() - 1; i >= 0; --i)
		{
			msg2 = new RobotSimEvents::BuoyColorSegmentMessage;
			msg2->x = float(redBlobs[i].second.first) / float(img.getWidth());
			msg2->y = float(redBlobs[i].second.second) / float(img.getHeight());
			msg2->size = redBlobs[i].first;

			this->publish("BuoyColorSegmentMessageTopic", msg2);
		}
		RobotSimEvents::PipeColorSegmentMessagePtr msg3;
		//cout << "yellow blobs:" << endl;
		for (i = itsPipeSegmenter.SInumberBlobs() - 1; i >= 0; --i)
		{
			msg3 = new RobotSimEvents::PipeColorSegmentMessage;
			msg3->x = float(yellowBlobs[i].second.first)
					/ float(img.getWidth());
			msg3->y = float(yellowBlobs[i].second.second)
					/ float(img.getHeight());
			msg3->size = yellowBlobs[i].first;
			/*if(msg3->size > 1000)
			{
				cout << ">" << msg3->size << endl;
			}*/

			this->publish("PipeColorSegmentMessageTopic", msg3);
		}

	}
}
/*    void ColorSegmenterI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
 const Ice::Current&)
 {
 //if message is a Buoy config message, sets buoy segmenter settings
 if(eMsg->ice_isA("::RobotSimEvents::BuoyColorSegmentConfigMessage")){
 RobotSimEvents::BuoyColorSegmentConfigMessagePtr msg = RobotSimEvents::BuoyColorSegmentConfigMessagePtr::dynamicCast(eMsg);
 printf("Got A Buoy ColorSegment Config Message! HUT: %f HLT: %f SUT: %f SLT: %f VUT: %f VLT: %f", msg->HUT, msg->HLT, msg->SUT, msg->SLT, msg->VUT, msg->VLT);
 //SIsetXXX(center,deviation,skew), so this calculates center using avg of bounds and deviation as twice the diff of bounds
 itsBuoySegmenter.SIsetHue((msg->HUT+msg->HLT)/2,(msg->HUT-msg->HLT)/2,0);
 itsBuoySegmenter.SIsetSat((msg->SUT+msg->SLT)/2,(msg->SUT-msg->SLT)/2,0);
 itsBuoySegmenter.SIsetVal((msg->VUT+msg->VLT)/2,(msg->VUT-msg->VLT)/2,0);
 }
 //if message is a Pipe config message, sets pipe segmenter settings
 else if(eMsg->ice_isA("::RobotSimEvents::PipeColorSegmentConfigMessage")){
 RobotSimEvents::PipeColorSegmentConfigMessagePtr msg = RobotSimEvents::PipeColorSegmentConfigMessagePtr::dynamicCast(eMsg);
 printf("Got A Pipe ColorSegment Config Message! HUT: %f HLT: %f SUT: %f SLT: %f VUT: %f VLT: %f", msg->HUT, msg->HLT, msg->SUT, msg->SLT, msg->VUT, msg->VLT);
 itsPipeSegmenter.SIsetHue((msg->HUT+msg->HLT)/2,(msg->HUT-msg->HLT)/2,0);
 itsPipeSegmenter.SIsetSat((msg->SUT+msg->SLT)/2,(msg->SUT-msg->SLT)/2,0);
 itsPipeSegmenter.SIsetVal((msg->VUT+msg->VLT)/2,(msg->VUT-msg->VLT)/2,0);
 }
 }*/

#endif
