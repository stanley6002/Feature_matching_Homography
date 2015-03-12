//
//  Homography_Transformation .cpp
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 8/14/13.
//  Copyright 2013 __MyCompanyName__. All rights reserved.
//

#include "Homography_Transformation .h"
void  Homography_T::Homography_2D(int Img_index , IplImage *IGray_r, IplImage *IGray_l, double* location)
{
    
    // Homography_T * Homography_Trans = new Homography_T(1);

     double HomographyMatrix1 [9];
     
     vector<matched_pts> _pts_left;
     vector<matched_pts> _pts_right;

     featurematching_process(IGray_r,IGray_l,HomographyMatrix1,&_pts_left,&_pts_right);
     
    //Save_Hmatrix(1, HomographyMatrix1);
    vector<matched_pts> _pts_left_selected;
    vector<matched_pts> _pts_right_selected;
    
    //double *location = new double[8];
     double HomographyMatrix[9];
    
    select_points( &_pts_left, &_pts_right, location , & _pts_left_selected, & _pts_right_selected, HomographyMatrix);
    
    cout<<"Homography selected size : " <<_pts_left.size()<<endl;
    
  
    int indx= (int)  _pts_left_selected.size();
    
    CvPoint *corners1 = new CvPoint [indx];
    CvPoint *corners2 = new CvPoint [indx];
    
    for (int i=0;i< indx;i++)
    {
        corners1[i].x= _pts_left_selected[i].x;
        corners1[i].y= _pts_left_selected[i].y;
        corners2[i].x= _pts_right_selected[i].x;
        corners2[i].y= _pts_right_selected[i].y;
    }
    
    CvMat *H_matrix =  cvCreateMat(3,3,CV_64FC1);
    
    cvmSet(H_matrix,0,0,HomographyMatrix1[0]); cvmSet(H_matrix,0,1,HomographyMatrix1[1]); cvmSet(H_matrix,0,2,HomographyMatrix1[2]); 
    cvmSet(H_matrix,1,0,HomographyMatrix1[3]); cvmSet(H_matrix,1,1,HomographyMatrix1[4]); cvmSet(H_matrix,1,2,HomographyMatrix1[5]); 
    cvmSet(H_matrix,2,0,HomographyMatrix1[6]); cvmSet(H_matrix,2,1,HomographyMatrix1[7]); cvmSet(H_matrix,2,2,HomographyMatrix1[8]); 
    
    CvMat *pt = cvCreateMat(3,1,CV_64FC1);
    CvMat *x =  cvCreateMat(3,1,CV_64FC1);
    
    CvMat *invH =  cvCreateMat(3,3,CV_64FC1);
    cvInvert(H_matrix, invH, CV_LU); 
    Save_Hmatrix(Img_index, invH);
   
    /* 
    CvPoint *corners11 = new CvPoint [indx];
    for (int i=0;i< indx ;i++)
    { 
        cvmSet(x,0,0,corners2[i].x);
        cvmSet(x,1,0,corners2[i].y);
        cvmSet(x,2,0,1);    
        cvMatMul(invH,x, pt);
        
        corners11[i].x= (cvmGet(pt, 0, 0) / cvmGet(pt, 2, 0));
        corners11[i].y= (cvmGet(pt, 1, 0) / cvmGet(pt, 2, 0));
        
        //cvCircle(IGray_r, corners1[i], 5, CV_RGB(255,0,0));
        //cvCircle(IGray_r, corners11[i],3, CV_RGB(0,0,0));
        
    }
    
    //cout<< HomographyMatirx[0]<<" "<<HomographyMatirx[1]<<" "<<HomographyMatirx[2]<<endl;
    //cout<< HomographyMatirx[3]<<" "<<HomographyMatirx[4]<<" "<<HomographyMatirx[5]<<endl;
    //cout<< HomographyMatirx[6]<<" "<<HomographyMatirx[7]<<" "<<HomographyMatirx[8]<<endl;
    
    //double location_export [8];
    //ptr->location_export (location_export);
    
    CvPoint *s1 = new CvPoint [4];
    CvPoint *s2 = new CvPoint [4];
    
    s2[0].x = location[0];
    s2[0].y = location[1];
    
    s2[1].x = location[2];
    s2[1].y = location[3];
    
    s2[2].x = location[4];
    s2[2].y = location[5];
    
    s2[3].x = location[6];
    s2[3].y = location[7];
    
    //cvInvert(H_matrix, invH, CV_LU); 
    
    for (int i=0;i<4;i++)
    {
        cvmSet(x,0,0,s2[i].x);
        cvmSet(x,1,0,s2[i].y);
        cvmSet(x,2,0,1);  
        cvMatMul(invH,x, pt);
        s1[i].x = (cvmGet(pt, 0, 0) / cvmGet(pt, 2, 0));
        s1[i].y = (cvmGet(pt, 1, 0) / cvmGet(pt, 2, 0));
        cvCircle(img[1], s1[i], 5, CV_RGB(255,0,0));
        cvCircle(img[0], s2[i], 5, CV_RGB(255,0,0));
    }
    
    cvLine(img[1], 
           cvPoint(s1[0].x, s1[0].y ), 
           cvPoint(s1[1].x, s1[1].y ), 
           CV_RGB(255,255,255)
           );
    cvLine(img[1], 
           cvPoint(s1[1].x, s1[1].y ), 
           cvPoint(s1[3].x, s1[3].y ), 
           CV_RGB(255,255,255)
           );
    cvLine(img[1], 
           cvPoint(s1[2].x, s1[2].y ), 
           cvPoint(s1[3].x, s1[3].y ), 
           CV_RGB(255,255,255)
           );
    cvLine(img[1], 
           cvPoint(s1[0].x, s1[0].y ), 
           cvPoint(s1[2].x, s1[2].y ), 
           CV_RGB(255,255,255)
           );
    
    cvLine(img[0], 
           cvPoint(s2[0].x, s2[0].y ), 
           cvPoint(s2[1].x, s2[1].y ), 
           CV_RGB(255,255,255)
           );
    cvLine(img[0], 
           cvPoint(s2[1].x, s2[1].y ), 
           cvPoint(s2[3].x, s2[3].y ), 
           CV_RGB(255,255,255)
           );
    cvLine(img[0], 
           cvPoint(s2[2].x, s2[2].y ), 
           cvPoint(s2[3].x, s2[3].y ), 
           CV_RGB(255,255,255)
           );
    cvLine(img[0], 
           cvPoint(s2[0].x, s2[0].y ), 
           cvPoint(s2[2].x, s2[2].y ), 
           CV_RGB(255,255,255)
           );

    */
       
//    IplImage* I_show;
//    I_show=plot_two_imagesf(IGray_r, IGray_l , corners2, corners1, indx, s2, s1); 
//    cvShowImage("trts",I_show);
 
    //cvShowImage("T1",img[0]);
    //cvShowImage("T2",img[1]);
    //return(HomographyMatirx);
    //cvWaitKey(0);
    

}