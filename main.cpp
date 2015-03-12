
//
//  main.cpp
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 8/6/13.
//  Copyright 2013 __MyCompanyName__. All rights reserved.
//

#include <assert.h>
//#include "DataType_.h"
#include "Mousecallbackfunc.h"
#include <fstream> 
//#include "feature_matche.h"
# include "Homography_Transformation .h"
# include "videoprocessing.h"
# include "matchingandtracking.h"

void points2keypoints(const vector<cv::Point2f>& in, vector<cv::KeyPoint>& out);
void keypoints2points(const vector<cv::KeyPoint>& in, vector<cv::Point2f>& out);
void warpKeypoints(const cv::Mat& H, const vector<cv::KeyPoint>& in, vector<cv::KeyPoint>& out);
IplImage* skipNFrames(CvCapture* capture, int n)
{
    for(int i = 0; i < n; ++i)
    {
        if(cvQueryFrame(capture) == NULL)
        {
            return NULL;
        }
    }
    
    return cvQueryFrame(capture);
}
CvCapture *camCapture;
bool _1stframe=true;

int  LK_features;
int  FAST_features;
bool feature_matching;
bool Homography;

enum Function
{
    LK_Features,
    Fast_Features,
    Homography_Match,
};

void Function_selection(Function eColor)
{
    using namespace std;
    if (eColor == LK_Features)
    { 
       LK_features=1;
       FAST_features=0;
       feature_matching=true;
       Homography=FALSE; 
    }
    else if (eColor == Fast_Features)
    {
       LK_features=0;
       FAST_features=1;
       feature_matching=true;
       Homography=FALSE; 
    }
    else if (eColor == Homography_Match)
    {
        LK_features=0;
        FAST_features=0;
        feature_matching=false;
        Homography=TRUE; 
    
    }
}

int main(int argc, char** argv)
{
   
    Function_selection(Fast_Features);
    
    int  Img_width=  320;
    int  Img_height= 240;
    
    if(feature_matching)
    {
        
        IplImage *imgA= cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
        IplImage *imgB= cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
        IplImage *imgC= cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
        
        if (!(camCapture = cvCaptureFromCAM(CV_CAP_ANY))) 
        {
            cout << "Failed to capture from camera" << endl;
            // goto exitCameraOpenFailed;
        }
        
        cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_WIDTH,  320); 
        cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_HEIGHT, 240); 
        
        cout << "Camera opened successfully" << endl;
        IplImage *cameraFrame;
        cameraFrame = cvCreateImage(cvSize (Img_width,Img_height), IPL_DEPTH_8U,3);
        
        IplImage* imgGrayA  = cvCreateImage(cvGetSize(imgA), IPL_DEPTH_8U, 1);
        IplImage* imgGrayB  = cvCreateImage(cvGetSize(imgB), IPL_DEPTH_8U, 1);
        
        IplImage* frame ;
        
        VideoProcessing VideoProcessing(Img_width, Img_height);    
  
        do 
            
            if ((cameraFrame = cvQueryFrame(camCapture))) 
            {
                
                if (_1stframe)
                {
                    frame = cvQueryFrame(camCapture);
                    frame = skipNFrames(camCapture,6);
                    imgB  = cvCloneImage(frame); 
                    
                }
                
                if( ! _1stframe)
                {
                    bool CaptureFrames=0; 
                    frame = skipNFrames(camCapture,2);
                    frame= VideoProcessing.CaptureInitialFrame( camCapture);
                    
                    if (! CaptureFrames)
                    {                                          
                        if (VideoProcessing.captureNextFrame)
                        {
                              CaptureFrames=1;
                        }
                    }
                    
                    if(CaptureFrames==1)
                    {              
                        
                        imgA= frame;  // new frame //
                        imgC= cvCloneImage(imgB);  // previous frame //
                        
                        cvCvtColor(imgA,imgGrayA, CV_BGR2GRAY);  
                        cvCvtColor(imgC,imgGrayB, CV_BGR2GRAY);
                        
                        if (LK_features==1)
                        {
                           LKFeatures LKFeatures (imgGrayB,imgGrayA);
                            
                           std::vector<CvPoint2D32f> match_query; 
                           std::vector<CvPoint2D32f> match_train;
                            
                           LKFeatures.FeaturesMatched (match_query, match_train);
                            
                           int size_match= (int) match_query.size();
                           CvPoint *pt_new_query = new CvPoint [size_match];
                           CvPoint *pt_new_train= new CvPoint [size_match];
                            
                            for(int i=0 ; i<size_match;i++)
                            {
                                pt_new_query[i].x = match_query[i].x;
                                pt_new_query[i].y = match_query[i].y;
                                
                                pt_new_train[i].x = match_train[i].x;
                                pt_new_train[i].y = match_train[i].y;
                            }
                            
                            IplImage* Two_image= plot_two_imagesf(imgA,imgC, pt_new_train, pt_new_query,size_match );
                            
                            cvShowImage("test", Two_image);
                            
                            //cvShowImage("frame1",imgC);
                            imgB= cvCloneImage(frame);
                        }
                  
                  if (FAST_features==1)
                      
                  {
                        
                      FAST_ FAST_ (30, imgGrayA, imgGrayB);
                      std::vector<CvPoint2D32f> match_query;
                      std::vector<CvPoint2D32f> match_train;
                      FAST_.FAST_tracking(match_query, match_train);
                      
                      int size_match= (int) match_query.size();
                      CvPoint *pt_new_query = new CvPoint [size_match];
                      CvPoint *pt_new_train= new CvPoint [size_match];
                      
                      for(int i=0 ; i< size_match;i++) 
                      {
                          pt_new_query[i].x = match_query[i].x;
                          pt_new_query[i].y = match_query[i].y;
                          
                          pt_new_train[i].x = match_train[i].x;
                          pt_new_train[i].y = match_train[i].y;
                          
                      }
                      
                      IplImage* Two_image= plot_two_imagesf(imgC,imgA, pt_new_query, pt_new_train,size_match );
                      cvShowImage("test", Two_image);
                      
                      //cvShowImage("frame1",imgC);
                      imgB= cvCloneImage(frame);
                      
                     }
                  }
               }    
                
                _1stframe=false;   // deactivate first frame
          }
    
            while (true) ;
            cvReleaseImage(&frame);
            cvReleaseImage(&cameraFrame);
            cvReleaseImage(&imgA);
            cvReleaseImage(&imgB);
            cvReleaseImage(&imgC);
                

}
if(Homography)
{
        vector<string>  vs;
        ifstream ifs("/data_1.txt");
        string temp;
        while(getline( ifs, temp ))
            vs.push_back(temp);
        vector<char*>  vc;
        transform(vs.begin(), vs.end(), back_inserter(vc), convert);   
        uchar **imgdata;
        IplImage **img;
        img= new IplImage*[(int)vs.size()];
        int index = 0;
        img = (IplImage **)malloc((int)vs.size() * sizeof(IplImage *)); // Allocates memory to store just an IplImage pointer for each image loaded
        imgdata = (uchar **)malloc((int)vs.size() * sizeof(uchar *));
        
        
        int Img_Index = 0;  /// Number of Homography matrix;
        
        for (int i=0;i<(int)vs.size();i++)
            
        {
            img[i] = cvLoadImage(vc[i], 1);
            if (!img[i]->imageData)
            {
                cout<<"can't open file"<<endl;
            }
            imgdata[i]=(uchar *)img[i]->imageData; 
        }
        
        cout<<"Total numbers of Images"<< (int)vs.size()<<endl;
        
        const char* name = "Box Example";
        
        IplImage* tempimage = cvCloneImage( img[0]);
        
        cvNamedWindow( name );
        
        Globalvariable *ptr = new Globalvariable();
        
        cvSetMouseCallback( name, my_mouse_callback, ptr);
        
        while( ptr->stpFlag )
        {
            cvCopyImage( img[0], tempimage );
            if( /*CP==BOX*/  ptr->drawing_box ) 
                draw_box( tempimage , ptr->box );
            if (ptr->CP== LINE)
                draw_line(img[0],ptr->linePt1, ptr->linePt2);
            cvShowImage( name, tempimage );
            if( cvWaitKey(15)==27 ) 
                break;
        }
        
        IplImage* IGray_r  = cvCreateImage(cvGetSize(img[0]), IPL_DEPTH_8U, 1);
        IplImage* IGray_l =  cvCreateImage(cvGetSize(img[1]), IPL_DEPTH_8U, 1); 
        
        cvCvtColor( img[0],  IGray_r, CV_RGB2GRAY);
        cvCvtColor( img[1],  IGray_l, CV_RGB2GRAY);

        //double HomographyMatrix1 [9];
        
        vector<matched_pts> _pts_left;
        vector<matched_pts> _pts_right;
         
        double *location = new double[8];
        //CvMat  *HomographyMatrix= new CvMat[Img_Index];
        CvMat *H =  cvCreateMat(3,3,CV_64FC1);
        double HomographyMatrix1[9];
        
        ptr->location_export(location);
        
        Homography_T * Homography_Trans = new Homography_T(1);
      
        Homography_Trans->Homography_2D(Img_Index,IGray_r, IGray_l, location);
        
        Homography_Trans->Read_Hmatrix(Img_Index, H);

            
        featurematching_process(IGray_r,IGray_l,HomographyMatrix1,&_pts_left,&_pts_right);
        
        vector<matched_pts> _pts_left_selected;
        vector<matched_pts> _pts_right_selected;
    
        double HomographyMatirx[9];
        
        select_points( &_pts_left, &_pts_right, location , & _pts_left_selected, & _pts_right_selected, HomographyMatirx);

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
    
    CvPoint *corners11 = new CvPoint [indx];
    
    for (int i=0;i< indx ;i++)
    { 
        cvmSet(x,0,0,corners2[i].x);
        cvmSet(x,1,0,corners2[i].y);
        cvmSet(x,2,0,1);    
        cvMatMul(invH,x, pt);
        
        corners11[i].x= (cvmGet(pt, 0, 0) / cvmGet(pt, 2, 0));
        corners11[i].y= (cvmGet(pt, 1, 0) / cvmGet(pt, 2, 0));
    
        cvCircle(img[1], corners1[i], 5, CV_RGB(255,0,0));
        cvCircle(img[1], corners11[i],3, CV_RGB(0,0,0));
        
    }
    
    cout<< HomographyMatirx[0]<<" "<<HomographyMatirx[1]<<" "<<HomographyMatirx[2]<<endl;
    cout<< HomographyMatirx[3]<<" "<<HomographyMatirx[4]<<" "<<HomographyMatirx[5]<<endl;
    cout<< HomographyMatirx[6]<<" "<<HomographyMatirx[7]<<" "<<HomographyMatirx[8]<<endl;
    
    double location_export [8];
    ptr->location_export (location_export);
    
    CvPoint *s1 = new CvPoint [4];
    CvPoint *s2 = new CvPoint [4];
   
    s2[0].x = ptr->location[0].x;
    s2[0].y = ptr->location[0].y;
    
    s2[1].x = ptr->location[1].x;
    s2[1].y = ptr->location[1].y;
   
    s2[2].x = ptr->location[2].x;
    s2[2].y = ptr->location[2].y;
    
    s2[3].x = ptr->location[3].x;
    s2[3].y = ptr->location[3].y;
    
    cvInvert(H_matrix, invH, CV_LU); 
    
    for (int i=0;i<4;i++)
    {
        cvmSet(x,0,0,s2[i].x);
        cvmSet(x,1,0,s2[i].y);
        cvmSet(x,2,0,1);  
        cvMatMul(invH,x, pt);
        s1[i].x = (cvmGet(pt, 0, 0) / cvmGet(pt, 2, 0));
        s1[i].y = (cvmGet(pt, 1, 0) / cvmGet(pt, 2, 0));
    
    }
    
    
    //indx=4;
    CvPoint* RECT;
    IplImage* I_show;
    I_show=plot_two_imagesf(img[0], img[1] , corners2, corners1, indx);    
    //I_show=plot_two_imagesf(img[0], img[1] , s1, indx, RECT);
    cvShowImage("I_show", I_show);
     cvSaveImage("/I_show.jpg", I_show);
   
    cvWaitKey(0);
    cvReleaseImage( &img[0] );
    cvReleaseImage( &tempimage );
    cvDestroyWindow( name );
    
    return 0;
   }
}
void warpKeypoints(const cv::Mat& H, const vector<cv::KeyPoint>& in, vector<cv::KeyPoint>& out)
{
    vector<cv::Point2f> pts;
    keypoints2points(in, pts);
    vector<cv::Point2f> pts_w(pts.size());
    cv::Mat m_pts_w(pts_w);
   
       
    perspectiveTransform(cv::Mat(pts), m_pts_w, H);
    
        
    points2keypoints(pts_w, out);
}
void keypoints2points(const vector<cv::KeyPoint>& in, vector<cv::Point2f>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(in[i].pt);
    }
}
void points2keypoints(const vector<cv::Point2f>& in, vector<cv::KeyPoint>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
       
        out.push_back(cv::KeyPoint(in[i], 1));
    }
}
