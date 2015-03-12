//
//  Mousecallbackfunc.cpp
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 8/9/13.
//  Copyright 2013 __MyCompanyName__. All rights reserved.
//

#include "Mousecallbackfunc.h"

static int j=0;
static int i=0;
void draw_box( IplImage* img, CvRect rect )
{
    /** 
     * User Interface -- ROI selection.
     * Input : image Out rect coordinate .
     */    
    cvRectangle( img, cvPoint(rect.x, rect.y), cvPoint(rect.x+rect.width,rect.y+rect.height),
                cvScalar(0xff,0x00,0x00) );
    
}

void draw_line  (IplImage* img,  CvPoint &pt1, CvPoint &pt2)
{
    /** 
     * User Interface -- ROI selection.
     * Input : image Out: plot line around 4 corners .
     */      
    cvLine(img, pt1, pt2, cvScalar(0x00,0xff,0x00));
    
}


char *convert(const string & s)
{
    char *pc = new char[s.size()+1];
    strcpy(pc, s.c_str());
    return pc; 
}


void my_mouse_callback( int event, int x, int y, int flags, void* param )
{
 
    /** 
     * User Interface -- mouse callback function selection.
     * Input : image Out: plot line around 4 corners .
     */      
    
    Globalvariable* callback_variable= (Globalvariable*) param;
    int num_plot; 
    callback_variable->CP= BOX;
    switch(callback_variable->CP)
    { 
        case BOX:{num_plot=1; break;}
        case LINE: {num_plot=5; break;}
    }
    switch( event )
    {
        case CV_EVENT_MOUSEMOVE: 
            if( callback_variable->drawing_box )
            {
                
                callback_variable->box.width  =  x-callback_variable->box.x;
                callback_variable->box.height =  y-callback_variable->box.y;
            }
            
            break;
            
        case CV_EVENT_LBUTTONDOWN:
            
            if (callback_variable->CP==BOX)
            { callback_variable->drawing_box = true;}
            if (callback_variable->CP==LINE)
            { callback_variable->drawing_line = true;}
            
            callback_variable->box = cvRect( x, y, 0, 0 );
            
            if (callback_variable->CP==LINE)
            {
                callback_variable->Pt_location[0].x= x;
                callback_variable->Pt_location[0].y= y;
            }
            
            break;
            
        case CV_EVENT_LBUTTONUP:
            
            if (callback_variable->CP==LINE)
            {
                if( callback_variable->drawing_line )
                {
                    callback_variable->Pt_location[1].x= x;
                    callback_variable->Pt_location[1].y= y;
                    i++;
                    
                }
                if (i==1)
                {
                    callback_variable->linePt1.x=callback_variable->Pt_location[1].x;
                    callback_variable->linePt1.y=callback_variable->Pt_location[1].y;
                    callback_variable->linePt2.x=callback_variable->Pt_location[0].x;
                    callback_variable->linePt2.y=callback_variable->Pt_location[0].y;
                    i=0;
                }
            }
            
                    callback_variable->drawing_box  = false;
                    callback_variable->drawing_line = false;
            
            
            if( callback_variable->box.width < 0 )
            {
                    callback_variable->box.x += callback_variable->box.width;
                    callback_variable->box.width *= -1;
            }
            if( callback_variable->box.height < 0 )
            {
                    callback_variable->box.y += callback_variable->box.height;
                    callback_variable->box.height *= -1;
            }
            
                    callback_variable->location[0].x= callback_variable->box.x;
                    callback_variable->location[0].y= callback_variable->box.y;
            
                    callback_variable->location[1].x= callback_variable->box.width+callback_variable->box.x;
                    callback_variable->location[1].y= callback_variable->box.y;
            
                    callback_variable->location[2].x= callback_variable->box.x;
                    callback_variable->location[2].y= callback_variable->box.y+callback_variable->box.height;
            
                    callback_variable->location[3].x= callback_variable->box.width+callback_variable->box.x;
                    callback_variable->location[3].y= callback_variable->box.y+callback_variable->box.height;
            
                j++;
                break;
    }
    if (j==num_plot)
    {
        
        callback_variable->stpFlag= false;
        
    }
    
}
