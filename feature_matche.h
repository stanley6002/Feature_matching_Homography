//
//  feature_matche.h
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 8/9/13.
//  Copyright 2013 __MyCompanyName__. All rights reserved.
//


#include "Opencvheader.h"


typedef struct 
{
    double x;
    double y;
 
} matched_pts;

// # define ShowFeaturedetection
# define  T_DIST 5
//# define  plot_feature_matching


void SURF_feature_matching (IplImage *result,IplImage *result1, double*H, vector<matched_pts>*_pts_left, vector<matched_pts> *_pts_right);
//void SURF_feature_matching (IplImage *result,IplImage *result1, double H);
void flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs );
void featurematching_process(IplImage *IGray_r,IplImage *IGray_l, double *H, vector<matched_pts>* _pts_left,vector<matched_pts>* _pts_right);

bool isColinear(int num, CvPoint2D64f *p);
void ComputeH(int n, CvPoint2D64f *p1, CvPoint2D64f *p2, CvMat *H);
void Normalization(int num, CvPoint2D64f *p, CvMat *T);
int RANSAC_homography(int num, CvPoint2D64f *m1, CvPoint2D64f *m2, CvMat *H, CvMat *inlier_mask);
bool isColinear(int num, CvPoint2D64f *p);
int  ComputeInliers(int num, CvPoint2D64f *p1, CvPoint2D64f *p2, CvMat *H, CvMat *inlier_mask, double *dist_std);
//IplImage* plot_two_imagesf(IplImage *IGray,IplImage *IGray1,CvPoint *corners1,CvPoint *corners3, int &vector_index, CvPoint *rect1, CvPoint*rect2);
IplImage* plot_two_imagesf(IplImage *IGray,IplImage *IGray1,CvPoint *corners1,CvPoint *corners3, int &vector_index);
void select_points(vector<matched_pts>* _pts_left,vector<matched_pts>* _pts_right, double* position, vector<matched_pts>* refinded_pt_left, vector<matched_pts>* refinded_pt_right, double * H_matrix);
