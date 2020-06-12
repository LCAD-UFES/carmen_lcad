/*********************************************************************

  EVG-THIN v1.1: A Thinning Approximation to the Extended Voronoi Graph
  
  Copyright (C) 2006 - Patrick Beeson  (pbeeson@cs.utexas.edu)


  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301
  USA

*********************************************************************/


#ifndef ImageHelper_h 
#define ImageHelper_h 1

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

using namespace std;

class ImageHelper {
private:

  int _width, _height, _bpp, _size;
  unsigned char *_buffer;
  
  void setup(int width, int height, int bpp, unsigned char *buffer){  
    _width=width;
    _height=height;
    _bpp=bpp;
    _size=width*height*bpp;
    _buffer= new unsigned char[_size];
    if (buffer)
      memcpy(_buffer,buffer,_size);
    else 
      for (int i=0;i<_size;i++) _buffer[i]=0;
  }

  void scan_comment(FILE *f) {
    char s[2];
    fscanf( f,"%c",s);
    fscanf( f,"%c",s);
    while (s[0] == '#') {
      do {
	fscanf( f,"%c",s);
      } while (s[0] != '\n'); 
      fscanf( f,"%c",s);
    }
    ungetc(s[0],f);
  }

  int readpnm(const char *filename, int &width, int& height, int& bpp, unsigned char* & buffer)
  {
    FILE *f=fopen(filename,"r");
    if (!f) 
      {
	cerr << "Cannot read file " << filename <<endl;
	exit(-1);
      }
    char s[10];
    fscanf (f,"%s",s);
    if (s[0]!='P' || !(s[1] == '6' || s[1] == '5')  ) 
      {
	cerr << "Bad file format: "<<filename<<"\nWant raw PNM file."<<endl;
	exit(-1);
      }
    
    if (s[1]== '6') 
      bpp=3;
    else 
      bpp=1;
    scan_comment(f);
    if (2!= fscanf(f,"%d %d",&width, &height) ) 
      {
	cerr << "Failed to read in image dimensions." <<endl;
	exit(-1);
      }
    scan_comment(f);
    int max_color;
    fscanf(f,"%d%c",&max_color,s);
    int size=width*height*bpp;
    buffer=new unsigned char[size];
    fread(buffer, 1 , size, f);    
    fclose(f);
    
    if (bpp==1){
      unsigned char* buffer2=new unsigned char[width*height*3];
      bpp=3;
      int j=0;
      for (int i=0;i<size;i++) {
	char c=buffer[i];
	buffer2[j++]=c;
	buffer2[j++]=c;
	buffer2[j++]=c;
      }
      delete [] buffer;
      buffer=buffer2;
    }
    
    return 1;
  }
  

public:
  ~ImageHelper(){    
    _width=0;_height=0;_bpp=0;delete[] _buffer; _buffer=0;
    //    cerr << "Destroyed ImageHelper"<<endl;
  }
  
  ImageHelper(const ImageHelper &a){
    setup(a.width(),a.height(),a.bpp(),a.buffer());
  }

  ImageHelper(int width, int height, int bpp, unsigned char *buffer=NULL) {
    setup(width,height,bpp,buffer);
  }
  
  ImageHelper(const char *name) { 
    if (!readpnm(name, _width, _height, _bpp, _buffer)) {
      cerr << "Failed to construct ImageHelper." <<endl;
      _size=_width=_height=0;
      exit(-1);
    }
    _size=_width*_height*_bpp;
  }

  int save(const char *name) {
    FILE *f=fopen(name,"w");
    if (!f) return 0;
    fprintf(f,"P%d\n%d %d\n255\n", _bpp==3 ? 6 : 5, _width, _height);
    fwrite(_buffer, 1 , _size, f);
    fclose(f);
    return 1;
  }

  int bpp() const {return _bpp;}
  int height () const  { return _height;}
  int width() const {return _width;}

  unsigned  char *buffer (int x=0, int y=0) const
  {
      if (x<0 || y<0 || x>=_width || y >=_height) {
	cerr << "Image access: Bad call: "<< x <<" " << _width << " " << y << " " <<_height <<endl;
	return 0;
      }
      return _buffer+ _bpp*(y*_width + x);
    }


  void put(int x, int y, unsigned char* data) {
    unsigned char *p=buffer(x,y);
    for (int c=0;c<_bpp;c++){
      *p++=*data++;
    }
  }
  
  void greyscale(){ //convert image to gray 
    for (int y=0;y<_height;y++)
      for (int x=0;x<_width;x++){
	int maxx=0;
	for (int b=0;b<_bpp;b++) 
	  maxx=max(maxx,int(*(buffer(x,y)+b)));
	unsigned char maxxx[3]={(unsigned char)maxx,(unsigned char)maxx,(unsigned char)maxx};
	this->put(x,y,maxxx);
      }
  }

};

#endif

