//
//  Homography_Transformation .h
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 8/14/13.
//  Copyright 2013 __MyCompanyName__. All rights reserved.
//


# include "feature_matche.h"

//# define CvMat *Matrix =  cvCreateMat(3,3,CV_64FC1);


extern IplImage **img;
struct  matrix {
    double element[9];
};

struct rectangle
{
    CvPoint LU;
    CvPoint RU;
    CvPoint LD;
    CvPoint RD;
};
class Homography_T 
{
public:
    Homography_T (int i)
    
    {
        H_matrix = new matrix[i];
        Rectangle = new rectangle[i]; 
    }
    matrix * H_matrix;
    rectangle * Rectangle;
    void inline Save_Hmatrix (int i, CvMat* DataInput)
    {
        H_matrix[i].element[0]=cvmGet(DataInput,0,0);
        H_matrix[i].element[1]=cvmGet(DataInput,0,1); 
        H_matrix[i].element[2]=cvmGet(DataInput,0,2);
        H_matrix[i].element[3]=cvmGet(DataInput,1,0);
        H_matrix[i].element[4]=cvmGet(DataInput,1,1); 
        H_matrix[i].element[5]=cvmGet(DataInput,1,2);
        H_matrix[i].element[6]=cvmGet(DataInput,2,0);
        H_matrix[i].element[7]=cvmGet(DataInput,2,1); 
        H_matrix[i].element[8]=cvmGet(DataInput,2,2);
        
    }
    void inline Read_Hmatrix(int i, CvMat* DataInput)
    {
        cvmSet(DataInput,0,0,H_matrix[i].element[0]);
        cvmSet(DataInput,0,1,H_matrix[i].element[1]);
        cvmSet(DataInput,0,2,H_matrix[i].element[2]);
        cvmSet(DataInput,1,0,H_matrix[i].element[3]);
        cvmSet(DataInput,1,1,H_matrix[i].element[4]);
        cvmSet(DataInput,1,2,H_matrix[i].element[5]);
        cvmSet(DataInput,2,0,H_matrix[i].element[6]);
        cvmSet(DataInput,2,1,H_matrix[i].element[7]);
        cvmSet(DataInput,2,2,H_matrix[i].element[8]);
    
    }
    void inline Save_rectangle (int i , CvPoint* DataInput)
    {
        Rectangle[i].LU = DataInput[0];
        Rectangle[i].RU = DataInput[1];
        Rectangle[i].LD = DataInput[2];
        Rectangle[i].RD = DataInput[3];
    }
    void   Homography_2D (int Img_Index, IplImage *IGray_r,IplImage *IGray_l, double *location);
    //Globalvariable & operator = (const Globalvariable &scr);
    ~ Homography_T ();
};
