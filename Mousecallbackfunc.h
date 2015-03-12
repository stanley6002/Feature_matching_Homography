//
//  Mousecallbackfunc.h
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 8/9/13.
//  Copyright 2013 __MyCompanyName__. All rights reserved.
//
 
#include "DataType_.h"


void draw_box( IplImage* img, CvRect rect );
void draw_line  (IplImage* img,  CvPoint &pt1, CvPoint &pt2);
char *convert(const string & s);
void my_mouse_callback( int event, int x, int y, int flags, void* param );