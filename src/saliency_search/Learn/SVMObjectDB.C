/*!@file Learn/SVMObjectDB.H SVM Object Database module */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://dparks@isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SVMObjectDB.C $
// $Id: SVMObjectDB.C 11135 2009-04-21 20:14:32Z mviswana $
//

#include "Learn/SVMObjectDB.H"

#ifdef HAVE_LIBXML

void SVMObjectDB::getNodeContent(xmlDocPtr doc, xmlNodePtr nodePtr,
                                  const char* nodeName, std::string &result)
{
  xmlChar *tmp = NULL;
  if (!xmlStrcmp(nodePtr->name, (const xmlChar *)nodeName))
    tmp = xmlNodeGetContent(nodePtr);
  //xmlNodeListGetString(doc, nodePtr->xmlChildrenNode, 1);

  if (tmp != NULL) {
    result = std::string((const char*)tmp);
    xmlFree(tmp);
  }
}



void SVMObjectDB::loadObjDB(const std::string& filename)
{
  xmlDocPtr doc;

  // Get the xml file for the given scene:
  LINFO("Reading xml formatted object db file: %s", filename.c_str());

  // check if the file exists:
  doc = xmlReadFile(filename.c_str(), NULL, 0);
  if (doc == NULL) {
    LINFO("Failed to parse %s", filename.c_str());
    return;
  }

  /* Get the root element node */
  xmlNode *root_element = xmlDocGetRootElement(doc);

  // look for object annotations
  xmlNodePtr cur = root_element->xmlChildrenNode; //dont care about top level
  while (cur != NULL) {

    // get the objects from the list:
    if ((!xmlStrcmp(cur->name, (const xmlChar *)"object"))) {
      // get the  object data:
      std::string objId, objName, objDescription;

      xmlNodePtr objectPtr = cur->xmlChildrenNode;

      while(objectPtr != NULL) { // read the attributes
        getNodeContent(doc, objectPtr, "id", objId);
        getNodeContent(doc, objectPtr, "name", objName);
        getNodeContent(doc, objectPtr, "description", objDescription);
        // next object data:
        objectPtr = objectPtr->next;
      }
      SVMObject obj;
      obj.id = atoi(objId.c_str());
      obj.name = objName;
      obj.description = objDescription;
      LINFO("Reading in object %s[%s]",objName.c_str(),objId.c_str());
      objdb.insert(obj);
     }
    cur = cur->next;
  }

  xmlFreeDoc(doc);
  xmlCleanupParser();
}

void SVMObjectDB::writeObjDB(const std::string& filename)
{
  xmlDocPtr doc = xmlNewDoc((xmlChar *)"1.0");

  // Write the xml file for the given scene:
  LINFO("Writing out xml formatted object db file: %s", filename.c_str());

  // check if the file exists:

  /* Get the root element node */
  xmlNode *root_element = xmlNewNode(0, (xmlChar *)"objectDatabase");
  xmlDocSetRootElement(doc,root_element);
  char idbuf[50];
  std::set<SVMObject>::iterator it;
  for(it=objdb.begin();it!=objdb.end();it++) {
    xmlNode *objXML = xmlNewNode(0, (xmlChar *)"object");
    xmlAddChild(root_element,objXML);
    SVMObject objRef = (SVMObject) *it;
    LINFO("Writing out object %d[%s]\n",objRef.id,objRef.name.c_str());
    sprintf(idbuf,"%d",objRef.id);
    xmlNewTextChild(objXML,NULL,(xmlChar *)"id",(xmlChar *) idbuf);
    xmlNewTextChild(objXML,NULL,(xmlChar *)"name",(xmlChar *) objRef.name.c_str());
    xmlNewTextChild(objXML,NULL,(xmlChar *)"description",(xmlChar *) objRef.description.c_str());
    //xmlAttr *attr = xmlNewProp(objXML,"id",idbuf);
    //attr = xmlNewProp(objXML,"name",objRef.name.c_str());
    //attr = xmlNewProp(objXML,"description",objRef.description.c_str());
  }
  // Write out the document and clean up
  xmlSaveFileEnc(filename.c_str(),doc,"UTF-8");
  xmlFreeDoc(doc);
  xmlCleanupParser();

}

SVMObject SVMObjectDB::updateObject(const int id, const std::string& name)
{
  SVMObject h;
  // Id unknown
  if(id == -1) {
    if(name.compare("")==0)
      LFATAL("When updating an object, the name must be defined");
    h=getObject(name);
  }
  else {
    h=getObject(id);
    if(!h.initialized() && name.compare("")==0)
      LFATAL("New id is given, with no name");
  }
  if(h.initialized())
    return h;
  else
    return newObject(id,name);
}

SVMObject SVMObjectDB::newObject(const int id, const std::string& name)
{
  SVMObject h;
  int maxId=0;
  h.name = name;
  std::set<SVMObject>::iterator it;
  if(id == -1){
    SVMObject tmp;
    for(it=objdb.begin();it!=objdb.end();it++){
      tmp = (SVMObject) *it;
      if(tmp.id > maxId)
        maxId = tmp.id;
    }
    h.id = maxId+1;
  }
  else {
    h.id = id;
  }
  objdb.insert(h);
  return h;
}

SVMObject SVMObjectDB::getObject(const std::string& name)
{
  std::set<SVMObject>::iterator it;
  SVMObject h;
  SVMObject tmp;
  for(it=objdb.begin();it!=objdb.end();it++){
    tmp = (SVMObject) *it;
    if(tmp.name.compare(name)==0)
      return tmp;
  }
  return h;
}

SVMObject SVMObjectDB::getObject(const int id)
{
  std::set<SVMObject>::iterator it;
  SVMObject h;
  SVMObject tmp;
  for(it=objdb.begin();it!=objdb.end();it++){
    tmp = (SVMObject) *it;
    if(tmp.id==id)
      return tmp;
  }
  return h;
}

#else


void SVMObjectDB::getNodeContent(xmlDocPtr doc, xmlNodePtr nodePtr,
                                  const char* nodeName, std::string &result){
  LFATAL("SVM Object DB Error: Binary was not compiled with XML Support");
}

void SVMObjectDB::loadObjDB(const std::string& filename){
  LFATAL("SVM Object DB Error: Binary was not compiled with XML Support");
}

void SVMObjectDB::writeObjDB(const std::string& filename) {
  LFATAL("SVM Object DB Error: Binary was not compiled with XML Support");
}


SVMObject SVMObjectDB::updateObject(const int id, const std::string& name) {
  LFATAL("SVM Object DB Error: Binary was not compiled with XML Support");
  return SVMObject() ; // just to keep the compiler happy
}

SVMObject SVMObjectDB::getObject(const std::string& name) {
  LFATAL("SVM Object DB Error: Binary was not compiled with XML Support");
  return SVMObject() ; // just to keep the compiler happy
}

SVMObject SVMObjectDB::getObject(const int id) {
  LFATAL("SVM Object DB Error: Binary was not compiled with XML Support");
  return SVMObject() ; // just to keep the compiler happy
}


#endif
