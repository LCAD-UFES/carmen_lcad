/*
 * Rect.cpp
 *
 *  Created on: 30/11/2011
 *      Author: avelino
 */

#include "Rect.h"

TLDRect::TLDRect()
{
	clear();
}

TLDRect::TLDRect(int x, int y, int width, int height)
{
	rx = x;
	ry = y;

	//take absolute value to make sure width and height are positive
	rwidth = (width < 0 ? width * -1 : width);
	rheight = (height < 0 ? height * -1 : height);

}

bool TLDRect::intersects ( const TLDRect &rect ) const
{
    //for axis aligned rectangles
    //keep in mind some intersections are at a single point or a line
    //since this class is primarily designed for graphics, a point = a pixel which has area
    //this is really simple, I have no idea why Adobe’s algorithm is so slow
    /*
    A____B       A’___________B’
    |    |        |           |
    |    |       C’___________D’
    C____D

    */

    //line AB or CD (horizontal lines of ABCD) intersecting with A’C’ or B’D’ (vertical lines of A’B'C’D')
    //line AB is y = ry, CD is y = ry + rheight (in flash coordinates, not cartesian)
    //and the x range of AB == the x range of CD (y = # lines are parallel to x axis)

    //line A’C’ is x = rect.rx, B’D’ is x = rect.rx + rect.rwidth
    //and the y range of A’C’ == the y range of B’D’ (x = # lines are parallel to y axis)

    //A’.x == C’.x, is either in the x range of AB (which is the same as the x range of CD)
    //OR B’.x == D’.x, is either in the x range of AB
    if(    (rx <= rect.rx && rect.rx <= (rx + rwidth))//A’C’ check
        || (rx <= (rect.rx + rect.rwidth) && (rect.rx + rect.rwidth) <= (rx + rwidth))){ //B’D’ check

        //to check line AB intersecting with A’C', B’D’
        //AND A.y == B.y, is A.y in the y range of A’C’ AND y range A’C’ == B’D’
        if(rect.ry <= ry && ry <= (rect.ry + rect.rheight)){
            //there is an intersection point
            //at (rect.rx, ry)               aka (A’.x, A.y)
            //or (rect.rx + rect.rwidth, ry) aka (B’.x , A.y) - respectively (to initial if statement)
            return true;
        }
        //to check line CD intersecting with A’C', B’D’
        //AND C.y == D.y, is C.y in the y range of A’C’ AND y range A’C’ == B’D’
        if(rect.ry <= (ry + rheight) && (ry + rheight) <= (rect.ry + rect.rheight)){
            //there is an intersection point
            //at (rect.rx, ry + rheight)               aka (A’.x, C.y)
            //or (rect.rx + rect.rwidth, ry + rheight) aka (B’.x, C.y) - respectively (to initial if statement)
            return true;
        }
    }
    /*
    by now we have captured all varieties of:
       ___
     _|___|_____
    | |   |     |
    |_|___|_____|
      |   |
      |___|

    and all of the varieties of:
     ____
    |  __|_
    |_|__| |
      |____|

    and some varieties of:
         ___
     ___|___|__
    |   |___|  |
    |__________|

    */
    //so, to get the last few cases, we need to check
    //line AC or BD (vertical lines of ABCD) intersecting with A’B’ or C’D’ (horizontal lines of A’B'C’D')
    //for which, we just need to swap our x and y references

    //line AC is x = rx, BD is x = rx + rwidth (in flash coordinates, not cartesian)
    //and the y range of AC == the y range of BD (x = # lines are parallel to y axis)
    //A’.y == B’.y, is either in the y range of AC (which is the same as the y range of BD)
    //OR C’.y == D’.y, is either in the y range of AC
    if(    (ry <= rect.ry && rect.ry <= (ry + rheight))  //A’B’ check
        || (ry <= (rect.ry + rect.rheight) && (rect.ry + rect.rheight) <= (ry + rheight)) ){

        //check line AC intersecting with A’B', C’D’
        //AND A.x == C.x, is A.x in the x range of A’B’ AND x range A’B’ == C’D’
        if(rect.rx <= rx && rx <= (rect.rx + rect.rwidth)){
            //there is an intersection point
            //at (rx, rect.ry)                aka (A.x, A’.y)
            //or (rx, rect.ry + rect.rheight) aka (A.x , B’.y) - respectively
            return true;
        }
        //check line BD intersecting with A’B', C’D’
        //AND B.x == D.x, is B.x in the x range of A’B’ AND x range A’B’ == C’D’
        if(rect.rx <= (rx + rwidth) && (rx + rwidth) <= (rect.rx + rect.rwidth)){
            //there is an intersection point
            //at (rx + rwidth, rect.ry)                aka (B.x, A’.y)
            //or (rx + rwidth, rect.ry + rect.rheight) aka (B.x , B’.y) - respectively
            return true;
        }
    }

    /*
    The only remaining cases deal with containment
     _____
    |  _  |
    | |_| |
    |_____|

    */
    //lastly we check for either rectange completely containing the other
    //you need to check rect containing rect’ and rect’ containing rect
    if(contains(rect) || rect.contains(*this)){
        return true;
    }
    //if we havent already returned true…
    return false;
}

bool TLDRect::contains(const TLDRect &rect) const
{
	//does ‘this’ contain the rectangle ‘rect’
	//This is really simple since our rectangles are described
	//by a point (A) and positive width and height.
	//we only need to determine if points A’ and D’ are inside ABCD
	//but we don’t even have to do full point inside rect checks
	//just ensure that A <= A’  and D’ <= D
	//NOTE: contains does not check for the rectangle being empty
	//however, if the rectangle has no width or height, it equates to the point in rectangle test
	/*

            A _________B
             |A’____B’ |
             | |    |  |
             | |____|  |
             |C’    D’ |
             |_________|
            C          D
	 */

	if(   (rx <= rect.rx && ry <= rect.ry) //A <= A’
			&&((rect.rx + rect.rwidth) <= (rx + rwidth)) //D’.x <= D.x
			&&((rect.ry + rect.rheight) <= (ry + rheight))){//D’.y <= D.y
		return true;
	}

	return false;
}

bool TLDRect::containsCoords(int px, int py) const
{
	//determine if this set of coordinates is on the border or
	//inside of this rectangle
	//just like the contains function,
	//if our coordinates are greater than or equal too A
	//and our coordinates are less than or equal too D
	//it is contained since our rectangles are axis aligned

	if(   (rx <= px && ry <= py) //A <= point
			&&(px <= (rx + rwidth) && py <= (ry + rheight))){//point <= D
		return true;
	}

	return false;

}

TLDRect TLDRect::intersection(const TLDRect &rect) const
{
	//returns the rectangular overlap of two rectangles
	//if there is no overlap, the returned value is null
	//this function can return "empty" rectangles, points and lines are valid results
	//since it is used for graphics a point can be a single pixel and a line a row / column of pixels
	//this is why there is a check for 0 width and 0 height - which would be numerically correct for
	//rectangles over the real numbers, but for pixels and lines width and height are always atleast 1
	//since this class is primarily designed for graphics, a point = a pixel which has area
	/*
            A____B       A’___________B’
            |    |        |           |
            |    |       C’___________D’
            C____D

	 */

	//even if one rectangle contains the other we dont want to return
	//one of our parameters because that could cause nasty object sharing bugs.
	//while containment is perhaps the most unlikely case, it is also the least
	//computationally expensive - and greatly simplfies further calculations
	TLDRect newRect(0,0,0,0);

	if(contains(rect)){
		//return a copy of rect
		newRect.copy(rect);
		return newRect;
	}
	if(rect.contains(*this)){
		//return a copy of this
		newRect.copy(*this);
		return newRect;
	}

	//ok now the tricky part, we need to find and store the intersection points in some kind of order
	//without adding any overhead of a point class, and trying to avoid the slow Flash array class
	//there is a very finite list of points that would potentially compose an interection rectangle
	//any of the 4 points of ABCD, any of the 4 points of A’B'C’D',
	//and any of the 8 potential line segment intersection points examined in ‘intersects()’
	//However, there is no need to find all 4 corners of the newly formed rectangle
	//we just need to find its top left point, and bottom right point - only 4 potential point candidates for each.

	//the top left most point must be A, A’, the intersection of AB with A’C',
	//or the intersection of AC with A’B’ (the pairs of top and left lines)
	//and we must check in that order because the intersections can only be the top
	//left point if A or A’ isnt.
	//If all of those cases fail
	//there is not an intersection (assuming your "point in rectangle" test includes borders)

	//rect.containsCoords(rx,ry)
	if(rect.containsCoords(rx,ry)){
		newRect.rx = rx;
		newRect.ry = ry;
	}
	//containsCoords(rect.rx, rect.ry)
	else if(containsCoords(rect.rx, rect.ry)){
		newRect.rx = rect.rx;
		newRect.ry = rect.ry;
	}
	//intersection of AB with A’C’ - if this is confusing take a look at ‘intersects()’
	else if(   (rx <= rect.rx && rect.rx <= (rx + rwidth))
			&&(rect.ry <= ry && ry <= (rect.ry + rect.rheight))){
		newRect.rx = rect.rx;
		newRect.ry = ry;
	}
	//intersection of AC with A’B’
	else if(   (ry <= rect.ry && rect.ry <= (ry + rheight))
			&&(rect.rx <= rx && rx <= (rect.rx + rect.rwidth))){
		newRect.rx = rx;
		newRect.ry = rect.ry;
	}
	else{
		//intersection does not exist
		return newRect;
	}


	//the bottom right point must be D, D’, intersection of CD with B’D',
	//intersection of C’D’ with BD (the pairs of bottom and right lines)
	//and we must check in that order because the intersections can only be the bottom
	//right point if D or D’ isnt.

	//rect.containsCoords(rx + rwidth, ry + rheight)
	if(rect.containsCoords(rx + rwidth, ry + rheight)){
		newRect.rwidth = (rx + rwidth) - newRect.rx;
		newRect.rheight = (ry + rheight) - newRect.ry;
		newRect.rwidth = (newRect.rwidth > 0 ? newRect.rwidth : 1);
		newRect.rheight = (newRect.rheight > 0 ? newRect.rheight : 1);
		return newRect;
	}
	//containsCoords(rect.rx + rect.rwidth, rect.ry + rect.rheight)
	else if(containsCoords(rect.rx + rect.rwidth, rect.ry + rect.rheight)){
		newRect.rwidth = (rect.rx + rect.rwidth) - newRect.rx;
		newRect.rheight = (rect.ry + rect.rheight) - newRect.ry;
		newRect.rwidth = (newRect.rwidth > 0 ? newRect.rwidth : 1);
		newRect.rheight = (newRect.rheight > 0 ? newRect.rheight : 1);
		return newRect;
	}
	//CD intersecting with B’D’
	else if(   (rx <= (rect.rx + rect.rwidth) && (rect.rx + rect.rwidth) <= (rx + rwidth))
			&&(rect.ry <= (ry + rheight) && (ry + rheight) <= (rect.ry + rect.rheight))){
		//then there is an intersection at (rect.rx + rect.rwidth, ry + rheight)
		newRect.rwidth = (rect.rx + rect.rwidth) - newRect.rx;
		newRect.rheight = (ry + rheight) - newRect.ry;
		newRect.rwidth = (newRect.rwidth > 0 ? newRect.rwidth : 1);
		newRect.rheight = (newRect.rheight > 0 ? newRect.rheight : 1);
		return newRect;
	}
	//C’D’ intersecting with BD
	//I’m fairly confident that this can be an else case
	//but in the unlikely case that I missed something, or my future self edits this
	//and forgets about this little statement, lets just do the whole check
	else if(   (ry <= (rect.ry + rect.rheight) && (rect.ry + rect.rheight) <= (ry + rheight))
			&&(rect.rx <= (rx + rwidth) && (rx + rwidth) <= (rect.rx + rect.rwidth))){
		//then there is an intersection at (rx + rwidth, rect.ry + rect.rheight)
		newRect.rwidth = (rx + rwidth) - newRect.rx;
		newRect.rheight = (rect.ry + rect.rheight) - newRect.ry;
		newRect.rwidth = (newRect.rwidth > 0 ? newRect.rwidth : 1);
		newRect.rheight = (newRect.rheight > 0 ? newRect.rheight : 1);
		return newRect;
	}

	//I dont think I should ever hit this - you should have already returned something
	//but if you did hit it, the result should be null
	return newRect;
}

bool TLDRect::isEmpty() const
{
	//are all values 0 - isEmpty doesnt really suit this class well
	if(rx == 0 && ry == 0 && rwidth == 0 && rheight == 0){
		return true;
	}
	return false;
}

int TLDRect::area() const
{
	return rwidth * rheight;
}

void TLDRect::clear()
{
	rx = ry = 0;
	rwidth = rheight = 0;
}

void TLDRect::copy(const TLDRect &rect)
{
	rx = rect.rx;
	ry = rect.ry;
	rwidth = rect.rwidth;
	rheight = rect.rheight;
}

TLDRect::~TLDRect()
{
	clear();
}
