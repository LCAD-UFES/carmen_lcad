
#include "GUI/SimpleMeter.H"
#include "Image/PixelsTypes.H"

SimpleMeter::SimpleMeter(int width, int height, double min, double max, double lowThresh, double medThresh)
{

        itsWidth                  = width;
        itsHeight          = height;
        itsMin                          = min;
        itsMax                          = max;
        itsLowThresh = itsHeight - (lowThresh - itsMin)*itsHeight/(itsMax - itsMin);
        itsMedThresh = itsHeight - (medThresh - itsMin)*itsHeight/(itsMax - itsMin);
}

SimpleMeter::SimpleMeter(int width, int height, double min, double max)
{

        itsWidth           = width;
        itsHeight          = height;
        itsMin                          = min;
        itsMax                          = max;

        int lowThresh = (int)((itsMax - itsMin)/2.0 + itsMin);
        int medThresh = (int)((itsMax - itsMin)*2.0/3.0 + itsMin);

        itsLowThresh = itsHeight - (lowThresh - itsMin)*itsHeight/(itsMax - itsMin);
        itsMedThresh = itsHeight - (medThresh - itsMin)*itsHeight/(itsMax - itsMin);
}

Image<PixRGB<byte> > SimpleMeter::render(double val,bool vertical)
{
	Image<PixRGB<byte> > meter(itsWidth, itsHeight, ZEROS);
	if(vertical){
        int valY;
        if(val < itsMin)
                valY = 0;
        else if(val > itsMax)
                valY = itsHeight;
        else
        {
                valY = (int)((val - itsMin)*(itsHeight)/(itsMax - itsMin));
        }

        for(int y=itsHeight-1; y>1; y -= 3)
        {
                PixRGB<byte> Color;
                if(y > itsLowThresh)
                        Color = PixRGB<byte>(0,255,0);
                else if(y < itsLowThresh && y > itsMedThresh)
                        Color = PixRGB<byte>(255,255,0);
                else
                        Color = PixRGB<byte>(255,0,0);

                float alpha = .4;
                if(y > itsHeight-valY)
                        alpha = 1;
                drawLine(meter, Point2D<int>(0, y), Point2D<int>(itsWidth-1,y),PixRGB<byte>(Color*alpha),1);

        }
	}else{
		Image<PixRGB<byte> > meterH(itsHeight, itsWidth, ZEROS);
        int valX;
        if(val < itsMin)
                valX = 0;
        else if(val > itsMax)
                valX = itsHeight;
        else
        {
                valX = (int)((val - itsMin)*(itsHeight)/(itsMax - itsMin));
        }

        for(int x=itsHeight-1; x>1; x -= 3)
        {
                PixRGB<byte> Color;
                if(x > itsLowThresh)
                        Color = PixRGB<byte>(0,255,0);
                else if(x < itsLowThresh && x > itsMedThresh)
                        Color = PixRGB<byte>(255,255,0);
                else
                        Color = PixRGB<byte>(255,0,0);

                float alpha = .4;
                if(x > itsHeight-valX)
                        alpha = 1;
                drawLine(meter, Point2D<int>(x, 0), Point2D<int>(x,itsWidth-1),PixRGB<byte>(Color*alpha),1);

        }
		
			 meter = meterH;	
	}
        return meter;
}

