/**
   \file Robots/LoBot/LoLaserMain.C

   \brief Testing the Hokuyo laser range finder.

   This program implements a simple GUI to help visualize the distance
   measurements made by lobot's Hokuyo laser range finder.
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/LoLaserMain.C $
// $Id: LoLaserMain.C 13905 2010-09-09 21:38:44Z mviswana $
//

//--------------------------- LIBRARY CHECK -----------------------------

#ifndef INVT_HAVE_LIBGLUT

#include "Util/log.H"

int main()
{
   LERROR("Sorry, this program needs the OpenGL and GLUT libraries.") ;
   return 1 ;
}

#else

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoLaserWindow.H"

#include "Robots/LoBot/config/LoConfig.H"
#include "Robots/LoBot/config/LoCommonOpts.H"

#include "Robots/LoBot/misc/LoExcept.H"

// INVT model component support
#include "Component/ModelManager.H"
#include "Component/ModelParam.H"

// INVT utilities
#include "Util/log.H"

// OpenGL headers
#include <GL/glut.h>

//------------------------ APPLICATION OBJECT ---------------------------

// The following class wraps around the ModelManager and associated
// objects, providing a neatly encapsulated API for the main program.
namespace lobot {

struct LoApp {
   LoApp(int argc, const char* argv[]) ;
   void run() ;
   ~LoApp() ;

private:
   // We use the INVT model manager mostly for processing command line
   // arguments.
   ModelManager m_model_manager ;

   // Various command line options specific to this program.
   //
   // NOTE: There's only one! The --config-file option allows users to
   // specify the name of a file containing various settings. All
   // customization of this program's behaviour is achieved via this
   // settings file.
   OModelParam<std::string> m_cf_option ; // --config-file

   // Accessing command line arguments and config file settings
   std::string config_file() const {return m_cf_option.getVal() ;}

   // Initialization
   void init_glut(int* argc, char* argv[]) ;
   void parse_command_line(int argc, const char* argv[]) ;
} ;

} // end of namespace encapsulating above class definition

//------------------------------- MAIN ----------------------------------

int main(int argc, const char* argv[])
{
   MYLOGVERB = LOG_ERR ; // minimize INVT's incessant chatter
   try
   {
      lobot::LoApp app(argc, argv) ;
      app.run() ;
   }
   catch (lobot::uhoh& e)
   {
      LERROR("%s", e.what()) ;
      return e.code() ;
   }
   catch (std::exception& e)
   {
      LERROR("%s", e.what()) ;
      return 255 ;
   }
   return 0 ;
}

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//----------------------------- APP INIT --------------------------------

// Setup INVT model manager and GLUT and process command line options
LoApp::LoApp(int argc, const char* argv[])
   : m_model_manager("lolaser"),
     m_cf_option(& OPT_ConfigFile, & m_model_manager)
{
   init_glut(& argc, const_cast<char**>(argv)) ;
   parse_command_line(argc, argv) ;
}

void LoApp::init_glut(int* argc, char* argv[])
{
   glutInit(argc, argv) ;
   glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE) ;
}

void LoApp::parse_command_line(int argc, const char* argv[])
{
   std::string default_config_file = getenv("HOME") ;
   default_config_file += "/.lolaserrc" ;
   m_model_manager.setOptionValString(& OPT_ConfigFile,
                                      default_config_file.c_str()) ;

   if (! m_model_manager.parseCommandLine(argc, argv, "", 0, 0))
      throw customization_error(BAD_OPTION) ;
}

//----------------------------- MAIN LOOP -------------------------------

// Application object's run method (aka main loop)
void LoApp::run()
{
   try
   {
      m_model_manager.start() ;
      Configuration::load(config_file()) ;
      //Configuration::dump() ;
      m_model_manager.stop() ; // GLUT's main loop will prevent proper clean-up
   }
   catch (customization_error& e) // this is not fatal
   {
      LERROR("%s", e.what()) ; // simply report error and move on
   }

   LaserWindow::create("Hokuyo LRF Tester") ;
   glutMainLoop() ;
}

//--------------------------- APP CLEAN-UP ------------------------------

LoApp::~LoApp(){}

//------------------------------ HELPERS --------------------------------

} // namespace lobot encapsulating application object and its helpers

//-----------------------------------------------------------------------

#endif // #ifndef INVT_HAVE_LIBGLUT

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
