//
//  matchingandtracking.h
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 3/9/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//
#include "Opencvheader.h"


class FAST_
{
    
public:
    
    int Level ;
    
    enum Function
    {
        SURF_descriptor,
        BRIEF_descriptor,
    };
    
    cv::Mat FAST_outputImg;
    cv::Mat FAST_H_prev;
    cv::Ptr<cv::FeatureDetector> FAST_detector;
    std::vector<cv::KeyPoint> FAST_train_kpts, FAST_query_kpts;
    std::vector<cv::Point2f> FAST_train_pts, FAST_query_pts;
    
    cv::Ptr<cv::DescriptorExtractor> FAST_descriptor;
    cv::Mat FAST_train_desc, FAST_query_desc;
    
    cv::Ptr<cv::DescriptorMatcher> FAST_matcher;
    std::vector<cv::DMatch> FAST_matche;
    std::vector<unsigned char> FAST_match_mask;
    
    //std::vector<CvPoint2D32f> match_query;
    //std::vector<CvPoint2D32f> match_train;
    
    FAST_(int Level , IplImage* imgGrayA, IplImage * imgGrayB, Function descriptor);
   
    void warpKeypoints(const cv::Mat& H, const vector<cv::KeyPoint>& in, vector<cv::KeyPoint>& out);
    
    void FAST_tracking(std::vector<CvPoint2D32f>& match_query, std::vector<CvPoint2D32f>& match_train);
    
    ~ FAST_ ();
    void keypoints2points(const vector<cv::KeyPoint>& in, vector<cv::Point2f>& out);
    void points2keypoints(const vector<cv::Point2f>& in, vector<cv::KeyPoint>& out);

private:
    
    IplImage* ImageGray1;
    IplImage* ImageGray2;
    bool Surf_activate ;
    //bool countFrameRest;
};


class LKFeatures
  {

   public:
      
    enum Function
    {
          Optical_flow,
          BRIEF_descriptor,
    }; 
      
    vector<cv::Point2f> corners, nextPts, trackedPts;
    vector<uchar> status;
    vector<float> err;
    vector<cv::Point2f> conersNew;
     
    cv::Mat LK_H_prev;
    cv::Ptr<cv::FeatureDetector> LK_detector;
    std::vector<cv::Point2f> LK_train_kpts, LK_query_kpts;
    std::vector<cv::Point2f> LK_train_pts, LK_query_pts;
    
    cv::Ptr<cv::DescriptorExtractor> LK_descriptor;
    cv::Mat LK_train_desc, LK_query_desc;
    
    cv::Ptr<cv::DescriptorMatcher> LK_matcher;
    std::vector<cv::DMatch> LK_matche;
    std::vector<unsigned char> LK_match_mask;
    
      LKFeatures(IplImage* imgGrayA, IplImage* imgGrayB, Function input);
    ~ LKFeatures( );
      void ParametersInitialized( int Numberofcorner , int threshold1, int W_size , Function input);
     void LKFeaturesTracking ();
     void FeaturesMatched  (std::vector<CvPoint2D32f> &match_query, std::vector<CvPoint2D32f> &match_train);
    
    private:
      
     bool UseOptical_flow;
     int NumberCorn;
     int Threshold;
     int W_size;
     int Windowsize;
     IplImage* ImageGray1;
     IplImage* ImageGray2;
};