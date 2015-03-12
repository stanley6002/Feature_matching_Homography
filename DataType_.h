//
//  DataType_.h
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 8/8/13.
//  Copyright 2013 __MyCompanyName__. All rights reserved.
//

#include <stdio.h>
#include <string.h>
#include "Opencvheader.h"

enum MyEnumType { BOX, LINE};

class Globalvariable 
{
public:
    Globalvariable()
   
    {
        location    =   new CvPoint[4];
        Pt_location =   new CvPoint[4];
        drawing_box =   FALSE;
        drawing_line =  FALSE;
        stpFlag = true;
        box = cvRect(-1,-1,0,0);  
        //image = img;
    }
    MyEnumType  CP;
    CvRect box;
    CvPoint  *location ;
    CvPoint  *Pt_location ;
    CvPoint linePt1 ;
    CvPoint linePt2 ;
    bool drawing_box;
    bool drawing_line;
    bool stpFlag;
    void inline location_export (double* location_export )
    {
        location_export[0]= (double) location[0].x;
        location_export[1]= (double) location[0].y;
        
        location_export[2]= (double) location[1].x;
        location_export[3]= (double) location[1].y;
        
        
        location_export[4]=(double) location[2].x;
        location_export[5]=(double) location[2].y;
        
        location_export[6]=(double) location[3].x;
        location_export[7]=(double) location[3].y;
        
    }
    //Globalvariable & operator = (const Globalvariable &scr);
    ~ Globalvariable();
};


//Globalvariable &  Globalvariable :: operator = (const Globalvariable & scr)
//{
//    return *this; 
//}

