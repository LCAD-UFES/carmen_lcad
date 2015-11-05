/*!@file ObjRec/getLabelMeInfo.C get information for the labelme data set */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/getLabelMeInfo.C $
// $Id: getLabelMeInfo.C 7063 2006-08-29 18:26:55Z rjpeters $
//

#ifdef HAVE_LIBXML
#include <libxml/parser.h>
#include <libxml/tree.h>

#include <dirent.h>
#include <vector>
#include <string>

std::vector<std::string> files;

void readDir(const char *path = NULL);
void getObjects(std::string &filename, std::vector<std::string> &objNames);

#define ROOT_PATH "/lab/ilab15/tmp/objectsDB/mit/labelMe/Annotations"


int main(const int argc, const char **argv)

{


  readDir(NULL);

  for(uint i=0; i<files.size(); i++)
  {
    std::vector<std::string> objNames;
    //Check for  jpg
    if (files[i].find(".xml") != std::string::npos){
      //printf("%s\n", files[i].c_str());
      getObjects(files[i], objNames);
      for(uint obj=0; obj<objNames.size(); obj++)
        printf("%s\n", objNames[obj].c_str());
    }
  }
}


// #######################################################################
void readDir(const char *path)
{

  std::string newPath;
  if (path == NULL)
     newPath = std::string(ROOT_PATH);
  else
     newPath = std::string(ROOT_PATH) + "/" + std::string(path);

  DIR *dp = opendir(newPath.c_str());
  if (dp != NULL)
  {
    dirent *dirp;
    while ((dirp = readdir(dp)) != NULL ) {
      if (dirp->d_name[0] != '.')
      {
        if (dirp->d_type == 4)
        {
          readDir(dirp->d_name);
        } else {
          if (path == NULL)
            files.push_back(std::string(dirp->d_name));
          else
            files.push_back(std::string(path) + "/" + std::string(dirp->d_name));
        }
      }
    }
  }

}

// ######################################################################
void getObjects(std::string &filename, std::vector<std::string> &objNames)
{

  xmlDocPtr doc;

  //Get the xml file for the given scene
  std::string xmlFile = std::string(ROOT_PATH) + "/" + filename;


  doc = xmlReadFile(xmlFile.c_str(), NULL, 0);
  if (doc == NULL)
  {
    printf("Failed to parse %s\n", xmlFile.c_str());
    return;
  }

  //clear the objname and polygons
  objNames.clear();

  //Get the root element node
  xmlNode *root_element = xmlDocGetRootElement(doc);

  //look for object annotations
  xmlNodePtr cur = root_element->xmlChildrenNode; //dont care about toop level
  while (cur != NULL) {
    if ((!xmlStrcmp(cur->name, (const xmlChar *)"object"))) {

      xmlNodePtr object = cur->xmlChildrenNode;
      while(object != NULL)  //read the attributes and polygons
      {
        //Name
        if ((!xmlStrcmp(object->name, (const xmlChar *)"name"))) {
          xmlChar* name = xmlNodeListGetString(doc, object->xmlChildrenNode, 1);
          if (name != NULL)
          {
            objNames.push_back(std::string((const char*)name));
            xmlFree(name);
          }
        }

        object = object->next; //next object
      }

    }

    cur = cur->next;
  }


  xmlFreeDoc(doc);
  xmlCleanupParser();

}


#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
