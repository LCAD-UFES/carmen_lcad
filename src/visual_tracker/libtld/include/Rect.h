/*
 * Rect.h
 *
 *  Created on: 30/11/2011
 *      Author: avelino
 */

#ifndef RECT_H_
#define RECT_H_

class TLDRect {
private:
	int rx;
	int ry;
	int rwidth;
	int rheight;
public:
	TLDRect();
	~TLDRect();
	TLDRect(int x, int y, int width, int height);
public:
        int x() {return rx;}
        int y() {return ry;}
        int width() {return rwidth;}
        int height() {return rheight;}

        void setX(int x){rx = x;}
        void setY(int y){ry = y;}
        void setWidth(int width){rwidth = width;}
        void setHeight(int height){rheight = height;}

public:
        bool intersects(const TLDRect &rect) const;

        bool contains(const TLDRect &rect) const;

        bool containsCoords(int px, int py) const;

        TLDRect intersection(const TLDRect &rect) const;

        bool isEmpty() const;

        int area() const;

        void clear();

        void copy(const TLDRect &rect);
} ;



#endif /* RECT_H_ */
