/*!@file Devices/OpenNIGrabber.C Interface with a Kinect frame grabber via OpenNI*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Eric Hu <ehu@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/OpenNIGrabber.C $
// $Id: OpenNIGrabber.C 14130 2010-10-13 04:59:07Z itti $
//

#ifdef INVT_HAVE_OPENNI

#include "Devices/OpenNIGrabber.H"
#include "Devices/DeviceOpts.H"

#define MYLOGID itsFd

#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		LERROR("%s failed: %s\n", what, xnGetStatusString(rc));		\
	}


// ######################################################################
OpenNIGrabber::OpenNIGrabber(OptionManager& mgr, const std::string& descrName,
                         const std::string& tagName, const ParamFlag flags) :
  FrameIstream(mgr, descrName, tagName),
  // NOTE that contrary to the common case, we may give (by default
  // value of 'flags') USE_MY_VAL here when we construct the
  // OModelParam objects; that means that we push our values into the
  // ModelManager as the new default values, rather than having our
  // param take its value from the ModelManager's default
  itsDims(&OPT_FrameGrabberDims, this, Dims(GL_WIN_SIZE_X, GL_WIN_SIZE_Y), flags),
  itsListener()
{

}

// ######################################################################
void OpenNIGrabber::start1()
{
  	//initialize vars
  	NI_DMap = NULL;
  	NI_DMapX = 0;
  	NI_DMapY = 0;

	//this is a copy of the code from NiSimpleViewer. 
	XnStatus rc;

	// Initialize OpenNI
	rc = NI_context.InitFromXmlFile(SAMPLE_XML_PATH);
	CHECK_RC(rc, "InitFromXmlFile");		

	//Check the nodes in the XML file
	rc = NI_context.FindExistingNode(XN_NODE_TYPE_DEPTH, NI_depth);
	rc = NI_context.FindExistingNode(XN_NODE_TYPE_IMAGE, NI_image);

	//check specifications (ie dimensions, color type) of the acquired data
	NI_depth.GetMetaData(NI_depthMD);
	NI_image.GetMetaData(NI_imageMD);
	
	// Needed to parse depth data
	NI_DMapX = (((unsigned short)(NI_depthMD.XRes()-1) / 512) + 1) * 512;
	NI_DMapY = (((unsigned short)(NI_depthMD.YRes()-1) / 512) + 1) * 512;
	NI_DMap = (XnUInt16*)malloc(NI_DMapX * NI_DMapY * sizeof(XnUInt16));
}

// ######################################################################
void OpenNIGrabber::stop2()
{
  //Nothing to stop?
  NI_context.Shutdown();
}

// ######################################################################
OpenNIGrabber::~OpenNIGrabber()
{ }

// ######################################################################
void OpenNIGrabber::setListener(rutz::shared_ptr<FrameListener> listener)
{ itsListener = listener; }

// ######################################################################
GenericFrameSpec OpenNIGrabber::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGBD;
  result.videoFormat = VIDFMT_RGB24;
  result.videoByteSwap = false;
  result.dims = itsDims.getVal();
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame OpenNIGrabber::readFrame()
{
	XnStatus rc = XN_STATUS_OK;

	// Read a new frame
	rc = NI_context.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK)
	{
		LFATAL("Read failed: %s\n", xnGetStatusString(rc));
	}

        //Acquire Depth and Image Data
	NI_depth.GetMetaData(NI_depthMD);
	NI_image.GetMetaData(NI_imageMD);

	//Construct depth image
	const XnDepthPixel* pDepth = NI_depthMD.Data();
	XnUInt16* d_Tex = NI_DMap;

	for (XnUInt y = 0; y < NI_depthMD.YRes(); ++y)
	{
		for (XnUInt x = 0; x < NI_depthMD.XRes(); ++x, ++pDepth, ++d_Tex)
		{
			if (*pDepth != 0)
			{
				int nHistValue = (*pDepth)/4; //scaling
				*d_Tex = nHistValue;
			}
		}
	}
	Image<uint16> img(static_cast<uint16*>(NI_DMap), NI_depthMD.XRes(), NI_depthMD.YRes());	 
	{		
		itsDepthImage = img;
	}


	//Constructs camera image
	Image<PixRGB<byte> > img2((PixRGB<byte>*)(NI_imageMD.RGB24Data()), NI_imageMD.XRes(), NI_imageMD.YRes());	 
	{		
		itsColorImage = img2;
	}
    
    //create frame
    GenericFrame NIframe(itsColorImage, itsDepthImage);

    return NIframe;
}
// ######################################################################
xn::Context* OpenNIGrabber::getContext()
{
	return &NI_context;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
