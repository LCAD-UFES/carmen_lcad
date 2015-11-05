/*!@file AppPsycho/psycho-keypad.C  this is a simple virtual keypad for psychophysics experiments to collect non-speeded response using mouse*/ 

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
// Primary maintainer for this file: Nader Noori <nnoori@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/PsychoKeypad.C$


#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Psycho/PsychoDisplay.H"
#include "Psycho/EyeTrackerConfigurator.H"
#include "Psycho/EyeTracker.H"
#include "Psycho/PsychoOpts.H"
#include "Component/EventLog.H"
#include "Component/ComponentOpts.H"
#include "Raster/Raster.H"
#include "Util/MathFunctions.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "GameBoard/basic-graphics.H"
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <SDL/SDL_mixer.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <time.h>
#include "Image/DrawOps.H"
#include "GameBoard/resize.h"
#include <iostream>
#include <fstream>
#include <set>
#include <algorithm>
#include <ctime>
//#include "GameBoard/ColorDef.H"

using namespace std;

////////////////////////////////////////////////////////////////
////This is our button factory
////////////////////////////////////////////////////////////////
SDL_Surface* getButtonImage ( nub::soft_ref<PsychoDisplay> d, string label , PixRGB<byte> txtcolor=PixRGB<byte> ( 0,0,0 ) , PixRGB<byte> bgcolor=PixRGB<byte> ( 255,255,255 ) ,Point2D<int> size = Point2D<int> ( 100,100 ) ,PixRGB<byte> bordercolor=PixRGB<byte> ( 0,0,0 ) , int border=3 )
{
  Image<PixRGB<byte> > textIm ( d->getWidth(),d->getHeight(),ZEROS );
  textIm.clear ( bgcolor );
  writeText ( textIm, Point2D<int> ( ( size.i - label.length() *10 ) /2, ( size.j-20 ) /2 ),label.c_str(),txtcolor,bgcolor );
  SDL_Surface *surf = d->makeBlittableSurface ( textIm , true );
  Uint32 bc = d->getUint32color ( bordercolor );
  drawRectangle ( surf,bc,0,0,size.i -1,size.j -1 ,border );
  SDL_Surface* blank =getABlankSurface ( size.i , size.j );
  SDL_Rect clip;
  clip.x = 0 ;
  clip.y = 0 ;
  clip.w = size.i ;
  clip.h = size.j ;
  apply_surface ( 0,0,*surf,*blank,clip );
  dumpSurface ( surf ) ;
  return blank ;
}

////////////////////////////////////////////////////////////////////////
////This is the function for creating the keypad, in fact it generates
////12 buttons and associates the actions to the region for each button
////////////////////////////////////////////////////////////////////////

SDL_Surface* getKeyPad ( nub::soft_ref<PsychoDisplay> d, vector<string>& tokens,map<string , SDL_Rect>& buttmap , Point2D<float> ratios=Point2D<float>(4.0f,3.0f))
{
  SDL_Surface* pad= getABlankSurface ( (int)((float)d->getWidth() /ratios.i),(int)((float)d->getHeight() /ratios.j) );
  SDL_Rect clip;
  clip.x=0;
  clip.y=0;
  int numofrows = tokens.size() /3 +1;
  if ( tokens.size() %3 != 0 ) numofrows++ ;
  int numofcolumns = 3 ;
  clip.w= pad->w / numofcolumns ;
  clip.h = pad->h / numofrows ;

  //keys for 1 to 9
  for ( int i = 0 ; i < numofrows*3 ; i++ )
    {
      SDL_Surface* but ;
      if ( ( uint ) i < tokens.size() )
        {
          but = getButtonImage ( d,tokens.at(i),PixRGB<byte> ( 0,0,0 ),PixRGB<byte> ( 255,255,255 ),Point2D<int> ( pad->w / numofcolumns , pad->h / numofrows ),PixRGB<byte> ( 255, 98 , 25 ),3 );
        }
      else
        {
          but = getButtonImage ( d," ",PixRGB<byte> ( 0,0,0 ),PixRGB<byte> ( 255,255,255 ),Point2D<int> ( pad->w / numofcolumns , pad->h / numofrows ),PixRGB<byte> ( 255, 98 , 25 ),3 );
        }

      SDL_Rect cl ;
      cl.x = ( ( i ) %numofcolumns ) * ( pad->w ) /numofcolumns ; cl.y= ( ( i ) /numofcolumns ) * ( ( pad->h ) /numofrows ) ;
      cl.w = clip.w ;
      cl.h = clip.h ;
      apply_surface ( cl.x , cl.y ,*but,*pad,clip );
      if ( ( uint ) i < tokens.size() ) buttmap[tokens.at(i) ] = cl ;
      dumpSurface ( but );
    }
  SDL_Rect cl1 ;
  cl1.x = 0 ; cl1.y= ( numofrows-1 ) * ( ( pad->h ) /numofrows ) ;
  cl1.w = clip.w ;
  cl1.h = clip.h ;
  buttmap["!"] = cl1 ;
  SDL_Surface* but = getButtonImage ( d,string ( "<-" ),PixRGB<byte> ( 0,0,0 ),PixRGB<byte> ( 255,255,255 ),Point2D<int> ( pad->w / numofcolumns , pad->h / numofrows ),PixRGB<byte> ( 255, 98 , 25 ),3 );
  apply_surface ( 0, ( numofrows-1 ) * ( ( pad->h ) /numofrows ),*but,*pad,clip );
  dumpSurface ( but );
  SDL_Rect cl2 ;
  cl2.x = ( pad->w ) /numofcolumns ; cl2.y= ( numofrows-1 ) * ( ( pad->h ) /numofrows ) ;
  cl2.w = clip.w ;
  cl2.h = clip.h ;
  buttmap[" "] = cl2 ;
  but = getButtonImage ( d,string ( "spc" ),PixRGB<byte> ( 0,0,0 ),PixRGB<byte> ( 255,255,255 ),Point2D<int> ( pad->w / numofcolumns , pad->h / numofrows ),PixRGB<byte> ( 255, 98 , 25 ),3 );
  apply_surface ( ( pad->w ) /numofcolumns, ( numofrows-1 ) * ( ( pad->h ) /numofrows ),*but,*pad,clip );
  dumpSurface ( but );
  SDL_Rect cl3 ;
  cl3.x = 2* ( pad->w ) /numofcolumns ; cl3.y= ( numofrows-1 ) * ( ( pad->h ) /numofrows ) ;
  cl3.w = clip.w ;
  cl3.h = clip.h ;
  buttmap["*"] = cl3 ;
  but = getButtonImage ( d,string ( "Ok" ),PixRGB<byte> ( 0,0,0 ),PixRGB<byte> ( 255,255,255 ),Point2D<int> ( pad->w / numofcolumns , pad->h / numofrows ),PixRGB<byte> ( 255, 98 , 25 ),3 );
  apply_surface ( 2* ( pad->w ) /numofcolumns, ( numofrows-1 ) * ( ( pad->h ) /numofrows ),*but,*pad,clip );
  dumpSurface ( but );
  return pad ;
}




///////////////////////////////////////////////////////////////////////////
/////this function listens to mouse clicks and then finds the region of the screen
/////associated with the action, buttmap is the map of the region, offset is the offset of
/////buttons
///////////////////////////////////////////////////////////////////////////
string getPressedButtonCommand ( nub::soft_ref<PsychoDisplay> d, map<string , SDL_Rect>& buttmap,Point2D<int> offset=Point2D<int> ( 0,0 ) )
{
  int quit = 0 ;
  string s ;
  SDL_Event event ;
  while (SDL_PollEvent(&event)) {}
  while ( quit!=2 )
    {
      while ( SDL_PollEvent ( &event ) )
        {
          if ( event.type == SDL_MOUSEBUTTONUP  && event.button.button == SDL_BUTTON_LEFT )
            {
	      d->pushEvent("virtual keypad button-press");
              for ( map<string , SDL_Rect>::iterator it = buttmap.begin() ; it!=buttmap.end() ; ++it )
                {
                  if ( event.button.x >= ( it->second ).x + offset.i && event.button.x <= ( it->second ).x + ( it->second ).w + offset.i  && event.button.y >= ( it->second ).y+ offset.j && event.button.y <= ( it->second ).y + ( it->second ).h + offset.j )
                    {
                      quit = 2 ;
                      s = it->first ;
		      d->pushEvent("virtural keypad command : " +s);
                      break;
                    }

                }
            }

        }
    }
  return s ;

}


///////////////////////////////////////////////////////////////////////////
/////this function listens to mouse clicks and then finds the region of the screen
/////associated with the action, buttmap is the map of the region, offset is the offset of
/////buttons
///////////////////////////////////////////////////////////////////////////
string getTimeLimitedPressedButtonCommand ( nub::soft_ref<PsychoDisplay> d, map<string , SDL_Rect>& buttmap,Point2D<int> offset=Point2D<int> ( 0,0 ) , long tL=0)
{
  int quit = 0 ;
  string s = "" ;
  SDL_Event event ;
  while (SDL_PollEvent(&event)) {}
  long start = d->getTimerValue();
  long end = start ;
  while ( quit!=2 && (end-start < tL))
    {
      end = d->getTimerValue();
      while ( SDL_PollEvent ( &event ) )
        {
          if ( event.type == SDL_MOUSEBUTTONUP  && event.button.button == SDL_BUTTON_LEFT )
            {
	      d->pushEvent("virtual keypad button-press");
              for ( map<string , SDL_Rect>::iterator it = buttmap.begin() ; it!=buttmap.end() ; ++it )
                {
                  if ( event.button.x >= ( it->second ).x + offset.i && event.button.x <= ( it->second ).x + ( it->second ).w + offset.i  && event.button.y >= ( it->second ).y+ offset.j && event.button.y <= ( it->second ).y + ( it->second ).h + offset.j )
                    {
                      quit = 2 ;
                      s = it->first ;
		      d->pushEvent("virtural keypad command : " +s);
                      break;
                    }

                }
            }

        }
    }
  return s ;

}

////////////////////////////////////////////////////
////This function creates a virtual keypad, creates a map of buttons
////and their representation area and listens to the button press and at
////the end returns the keyed digits
////////////////////////////////////////////////////
vector<string> getKeypadResponse ( nub::soft_ref<PsychoDisplay> d,vector<string>& tokens , uint maxl , uint minl , string separtor,string message)
{
  d->showCursor ( true ) ;
  //let's creat a map to map actions to regions of the screen, each region is represented as an SDL_Rect
  map<string , SDL_Rect>* buttmap = new map<string , SDL_Rect>();
  //now let's get the keypad surface while we get the actions map to regions
  SDL_Surface * keypad = getKeyPad ( d,tokens,*buttmap  );
  //this will be the offset of displaying the keypad on the screen
  SDL_Rect offset ;
  offset.x = ( d->getWidth() - keypad->w ) /2;
  offset.y = ( d-> getHeight() - keypad->h ) /2;
  //now let's display the keypad
  d->displaySDLSurfacePatch ( keypad , &offset,NULL , -2,false, true );
  
  //now let's display the message 
   SDL_Surface* dp = getButtonImage ( d,message ,PixRGB<byte> ( 110,110,110 ) ,PixRGB<byte> ( 255,255,255 ) ,Point2D<int> ( d->getWidth() /6,d->getHeight() /18 ) ,PixRGB<byte> ( 110,110,110 ) , 1 ) ;
   SDL_Rect offs ; offs.x = ( d->getWidth() - dp->w ) /2 ; offs.y = d->getHeight() /4 ;
   d->displaySDLSurfacePatch ( dp , &offs , NULL , -2 , false ,true ) ;
   dp = 0;
  //this will hold the final string keyed be the subject
  vector<string> p ;
  //this is a temporary string holding the last action related to the pressed key
  string tp = string ( "" );
  //now let's record subject's key press
  while ( tp.compare ( "*" ) !=0 || p.size()<minl)
    {
      //this button is actually the display for the current string
      
      string monitorString = "";
      for(uint i = 0 ;  i < p.size() ; i++) monitorString += p.at(i)+separtor;
      SDL_Surface* dp = getButtonImage ( d,monitorString ,PixRGB<byte> ( 195,60,12 ) ,PixRGB<byte> ( 255,255,255 ) ,Point2D<int> ( d->getWidth() /6,d->getHeight() /15 ) ,PixRGB<byte> ( 0,25,180 ) , 4 ) ;
      SDL_Rect offs ; offs.x = ( d->getWidth() - dp->w ) /2 ; offs.y = d->getHeight() /6 ;
      d->displaySDLSurfacePatch ( dp , &offs , NULL , -2 , false ,true ) ;
      //now let's listen to button events
      tp = getPressedButtonCommand (d, *buttmap,Point2D<int> ( offset.x,offset.y ) ) ;
      dumpSurface ( dp ) ;
      if ( tp.compare ( "!" ) ==0 && p.size() >=0 )
        {
          if ( p.size() >0 ) p.pop_back();
        }
      else
        {
          if ( p.size() < maxl && tp.compare ( "*" ) !=0 )
            {
              p.push_back(tp);
            }

        }

    }
  buttmap = 0 ;
  dumpSurface ( keypad ) ;
  d->clearScreen() ;
  d->showCursor ( false ) ;
  return p ;

}
//////////////////////////////////
vector<string> getSquaredKeypadResponse ( nub::soft_ref<PsychoDisplay> d,vector<string>& tokens , uint maxl ,uint minl , string separator, string message){
  
  d->showCursor ( true ) ;
  
  //let's creat a map to map actions to regions of the screen, each region is represented as an SDL_Rect
  map<string , SDL_Rect>* buttmap = new map<string , SDL_Rect>();
  //now let's get the keypad surface while we get the actions map to regions
  SDL_Surface * keypad = getKeyPad ( d,tokens,*buttmap , Point2D<float>(4.0f,4.0f*(float)(d->getHeight())/(float)(d->getWidth()) ));
  //this will be the offset of displaying the keypad on the screen
  SDL_Rect offset ;
  offset.x = ( d->getWidth() - keypad->w ) /2;
  offset.y = ( d-> getHeight() - keypad->h ) /2;
  //now let's display the keypad
  d->displaySDLSurfacePatch ( keypad , &offset,NULL , -2,false, true );
  
  //now let's display the message 
  if(message.size()>0){
   SDL_Surface* dp = getButtonImage ( d,message ,PixRGB<byte> ( 110,110,110 ) ,PixRGB<byte> ( 255,255,255 ) ,Point2D<int> ( d->getWidth() /6,d->getHeight() /18 ) ,PixRGB<byte> ( 110,110,110 ) , 1 ) ;
   SDL_Rect offs ; offs.x = ( d->getWidth() - dp->w ) /2 ; offs.y = d->getHeight() /4 ;
   d->displaySDLSurfacePatch ( dp , &offs , NULL , -2 , false ,true ) ;
   dp = 0;
  }
  //this will hold the final string keyed be the subject
  vector<string> p ;
  //this is a temporary string holding the last action related to the pressed key
  string tp = string ( "" );
  //now let's record subject's key press
  while ( tp.compare ( "*" ) !=0 || p.size()<minl)
    {
      //this button is actually the display for the current string
      
      string monitorString = "";
      for(uint i = 0 ;  i < p.size() ; i++) monitorString = p.at(i)+separator;
      SDL_Surface* dp = getButtonImage ( d,monitorString ,PixRGB<byte> ( 195,60,12 ) ,PixRGB<byte> ( 255,255,255 ) ,Point2D<int> ( d->getWidth() /30,d->getHeight() /15 ) ,PixRGB<byte> ( 0,25,180 ) , 4 ) ;
      SDL_Rect offs ; offs.x = ( d->getWidth() - dp->w ) /2 ; offs.y = d->getHeight() /6 ;
      d->displaySDLSurfacePatch ( dp , &offs , NULL , -2 , false ,true ) ;
      //now let's listen to button events
      tp = getPressedButtonCommand (d, *buttmap,Point2D<int> ( offset.x,offset.y ) ) ;
      dumpSurface ( dp ) ;
      if ( tp.compare ( "!" ) ==0 && p.size() >=0 )
        {
          if ( p.size() >0 ) p.pop_back();
        }
      else
        {
          if ( p.size() < maxl && tp.compare ( "*" ) !=0 )
            {
              p.push_back(tp);
            }

        }

    }
  buttmap = 0 ;
  dumpSurface ( keypad ) ;
  d->clearScreen() ;
  d->showCursor ( false ) ;
  return p ;

}
//////////////////////////////////
vector<string> getSpeededKeypadResponse ( nub::soft_ref<PsychoDisplay> d,vector<string>& tokens , uint maxl , uint minl , string separtor,string message, long waitTime)
{
  d->showCursor ( true ) ;
  //let's creat a map to map actions to regions of the screen, each region is represented as an SDL_Rect
  map<string , SDL_Rect>* buttmap = new map<string , SDL_Rect>();
  //now let's get the keypad surface while we get the actions map to regions
  SDL_Surface * keypad = getKeyPad ( d,tokens,*buttmap  );
  //this will be the offset of displaying the keypad on the screen
  SDL_Rect offset ;
  offset.x = ( d->getWidth() - keypad->w ) /2;
  offset.y = ( d-> getHeight() - keypad->h ) /2;
  //now let's display the keypad
  d->displaySDLSurfacePatch ( keypad , &offset,NULL , -2,false, true );
  
  //now let's display the message 
   SDL_Surface* dp = getButtonImage ( d,message ,PixRGB<byte> ( 110,110,110 ) ,PixRGB<byte> ( 255,255,255 ) ,Point2D<int> ( d->getWidth() /6,d->getHeight() /18 ) ,PixRGB<byte> ( 110,110,110 ) , 1 ) ;
   SDL_Rect offs ; offs.x = ( d->getWidth() - dp->w ) /2 ; offs.y = d->getHeight() /4 ;
   d->displaySDLSurfacePatch ( dp , &offs , NULL , -2 , false ,true ) ;
   dp = 0;
  //this will hold the final string keyed be the subject
  vector<string> p ;
  //this is a temporary string holding the last action related to the pressed key
  string tp = string ( "" );
  
  long start = d->getTimerValue();
  long end = start ;
  //now let's record subject's key press
  while ( (end-start < waitTime) && (tp.compare ( "*" ) !=0 || p.size()<minl))
    {
      //this button is actually the display for the current string
      
      string monitorString = "";
      for(uint i = 0 ;  i < p.size() ; i++) monitorString += p.at(i)+separtor;
      SDL_Surface* dp = getButtonImage ( d,monitorString ,PixRGB<byte> ( 195,60,12 ) ,PixRGB<byte> ( 255,255,255 ) ,Point2D<int> ( d->getWidth() /6,d->getHeight() /15 ) ,PixRGB<byte> ( 0,25,180 ) , 4 ) ;
      SDL_Rect offs ; offs.x = ( d->getWidth() - dp->w ) /2 ; offs.y = d->getHeight() /6 ;
      d->displaySDLSurfacePatch ( dp , &offs , NULL , -2 , false ,true ) ;
      end = d->getTimerValue();
      //now let's listen to button events
      tp = getTimeLimitedPressedButtonCommand (d, *buttmap,Point2D<int> ( offset.x,offset.y ),waitTime - (end - start) ) ;
      dumpSurface ( dp ) ;
      if ( tp.compare ( "!" ) ==0 && p.size() >=0 )
        {
          if ( p.size() >0 ) p.pop_back();
        }
      else
        {
          if ( p.size() < maxl && tp.compare ( "*" ) !=0 && tp.size()>0 )
            {
              p.push_back(tp);
            }

        }

    }
  buttmap = 0 ;
  dumpSurface ( keypad ) ;
  d->clearScreen() ;
  d->showCursor ( false ) ;
  return p ;

}

//////////////////////////////////
vector<string> getSpeededSquaredKeypadResponse ( nub::soft_ref<PsychoDisplay> d,vector<string>& tokens , uint maxl , uint minl , string separtor,string message, long waitTime)
{
  d->showCursor ( true ) ;
  //let's creat a map to map actions to regions of the screen, each region is represented as an SDL_Rect
  map<string , SDL_Rect>* buttmap = new map<string , SDL_Rect>();
  //now let's get the keypad surface while we get the actions map to regions
  SDL_Surface * keypad = getKeyPad ( d,tokens,*buttmap , Point2D<float>(5.0f,5.0f*(float)(d->getHeight())/(float)(d->getWidth()) ));
  //this will be the offset of displaying the keypad on the screen
  SDL_Rect offset ;
  offset.x = ( d->getWidth() - keypad->w ) /2;
  offset.y = ( d-> getHeight() - keypad->h ) /2;
  //now let's display the keypad
  d->displaySDLSurfacePatch ( keypad , &offset,NULL , -2,false, true );
  
  //now let's display the message 
  if(message.size()>0){
   SDL_Surface* dp = getButtonImage ( d,message ,PixRGB<byte> ( 110,110,110 ) ,PixRGB<byte> ( 255,255,255 ) ,Point2D<int> ( d->getWidth() /6,d->getHeight() /18 ) ,PixRGB<byte> ( 110,110,110 ) , 1 ) ;
   SDL_Rect offs ; offs.x = ( d->getWidth() - dp->w ) /2 ; offs.y = d->getHeight() /4 ;
   d->displaySDLSurfacePatch ( dp , &offs , NULL , -2 , false ,true ) ;
   dp = 0;
  }
  //this will hold the final string keyed be the subject
  vector<string> p ;
  //this is a temporary string holding the last action related to the pressed key
  string tp = string ( "" );
  
  long start = d->getTimerValue();
  long end = start ;
  //now let's record subject's key press
  while ( (end-start < waitTime) && (tp.compare ( "*" ) !=0 || p.size()<minl))
    {
      //this button is actually the display for the current string
      
      string monitorString = "";
      for(uint i = 0 ;  i < p.size() ; i++) monitorString = p.at(i)+separtor;
      SDL_Surface* dp = getButtonImage ( d,monitorString ,PixRGB<byte> ( 195,60,12 ) ,PixRGB<byte> ( 255,255,255 ) ,Point2D<int> ( d->getWidth() /30,d->getHeight() /15 ) ,PixRGB<byte> ( 0,25,180 ) , 4 ) ;
      SDL_Rect offs ; offs.x = ( d->getWidth() - dp->w ) /2 ; offs.y = d->getHeight() /6 ;
      d->displaySDLSurfacePatch ( dp , &offs , NULL , -2 , false ,true ) ;
      end = d->getTimerValue();
      //now let's listen to button events
      tp = getTimeLimitedPressedButtonCommand (d, *buttmap,Point2D<int> ( offset.x,offset.y ),waitTime - (end - start) ) ;
      dumpSurface ( dp ) ;
      if ( tp.compare ( "!" ) ==0 && p.size() >=0 )
        {
          if ( p.size() >0 ) p.pop_back();
        }
      else
        {
          if ( p.size() < maxl && tp.compare ( "*" ) !=0 && tp.size()>0 )
            {
              p.push_back(tp);
            }

        }

    }
  buttmap = 0 ;
  dumpSurface ( keypad ) ;
  d->clearScreen() ;
  d->showCursor ( false ) ;
  return p ;

}
