//
//  feature_matche.cpp
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 8/9/13.
//  Copyright 2013 __MyCompanyName__. All rights reserved.
//

#include "feature_matche.h"

void SURF_feature_matching (IplImage *result,IplImage *result1, double*H, vector<matched_pts>*_pts_left, vector<matched_pts> *_pts_right)
{
    
    CvSeq  *imageDescriptors = 0;
    CvSeq  *imageKeypoints =0;
    CvSeq  *objectDescriptors =0;
    CvSeq  *objectKeypoint =0;
    CvSURFParams params = cvSURFParams(100);
    CvMemStorage* storage = cvCreateMemStorage(0);
    cvExtractSURF(result, NULL, &imageKeypoints,  &imageDescriptors,storage,params);
    cvExtractSURF(result1, NULL, &objectKeypoint, &objectDescriptors,storage,params);
    
    
    CvSeq  *descriptors=0;
    descriptors = cvCreateSeq( 0, sizeof(CvSeq),20*CV_ELEM_SIZE(CV_32F), storage );
    
        
    vector <int> ptpair;
    flannFindPairs (objectKeypoint,objectDescriptors,imageKeypoints,imageDescriptors,ptpair);
      
    CvPoint2D64f center1[(int) ptpair.size()/2];
    CvPoint2D64f center2[(int) ptpair.size()/2];
    
    std::cout<<"Feature Points After_matching__ "<<(int) ptpair.size()/2<<"\n";
    
    for (int i=0;i<(int) ptpair.size();i+=2)
    {
        
        CvSURFPoint* rr= (CvSURFPoint*)cvGetSeqElem(objectKeypoint, ptpair[i]);
        CvSURFPoint* rl= (CvSURFPoint*)cvGetSeqElem(imageKeypoints, ptpair[i+1]);
        center1[i/2].x= rr->pt.x;
        center1[i/2].y= rr->pt.y;
        center2[i/2].x= rl->pt.x;
        center2[i/2].y= rl->pt.y;
    }
    int num_matched=(int) ptpair.size()/2;
    //CvMat *points1=cvCreateMat(num_matched,2,CV_32FC1);
    //CvMat *points2=cvCreateMat(num_matched,2,CV_32FC1);

#  ifdef ShowFeaturedetection
    
            CvPoint  putative_point1[num_matched];
            CvPoint  putative_point2[num_matched];
    
    
            for (int i=0; i<(int)num_matched;i++)
            {
                putative_point1[i].x= center1[i].x;
                putative_point1[i].y= center1[i].y;
                putative_point2[i].x= center2[i].x;
                putative_point2[i].y= center2[i].y; 
                cvCircle(result1,putative_point1[i], 5, CV_RGB(255,255,255));
                cvCircle(result, putative_point2[i], 5, CV_RGB(255,255,255));
            }
   
            cvShowImage("right_image", result1);
            cvWaitKey(0);
    
#endif  
    

    CvMat *H_matrix = cvCreateMat(3,3,CV_64FC1);
    CvMat *inlier_mask = cvCreateMat(num_matched,1,CV_64FC1); 
    
    int Homography_inliers;
    
    Homography_inliers=RANSAC_homography(num_matched, center1, center2, H_matrix, inlier_mask);

    H[0]= cvmGet(H_matrix, 0, 0); H[1]=cvmGet(H_matrix, 0, 1);H[2]=cvmGet(H_matrix, 0, 2);
    H[3]= cvmGet(H_matrix, 1, 0); H[4]=cvmGet(H_matrix, 1, 1);H[5]=cvmGet(H_matrix, 1, 2);
    H[6]= cvmGet(H_matrix, 2, 0); H[7]=cvmGet(H_matrix, 2, 1);H[8]=cvmGet(H_matrix, 2, 2);
           
    //cout<<cvmGet(H, 0, 0)<<" "<<cvmGet(H, 0, 1)<<" "<<cvmGet(H, 0, 2)<<endl;
    //cout<<cvmGet(H, 1, 0)<<" "<<cvmGet(H, 1, 1)<<" "<<cvmGet(H, 1, 2)<<endl;
    //cout<<cvmGet(H, 2, 0)<<" "<<cvmGet(H, 2, 1)<<" "<<cvmGet(H, 2, 2)<<endl;
    
    
    
    //# define DEBUG_RANSAC

# ifdef  DEBUG_RANSAC
    int num_inlier = 0;
    for(int i=0; i<num_matched; i++)
    {
        if(cvmGet(inlier_mask,i,0) == 1)
        {
            
            num_inlier++;
        }
    }
    CvPoint  new_center1[num_inlier];
    CvPoint  new_center2[num_inlier];
    int new_center_index=0;
    for (int i=0; i<num_inlier; i++)
    {
        if(cvmGet(inlier_mask,i,0) == 1)
        {
            new_center1[new_center_index].x= (int)center1[i].x;
            new_center1[new_center_index].y= (int)center1[i].y;
            new_center2[new_center_index].x= (int)center2[i].x;
            new_center2[new_center_index].y= (int)center2[i].y; 
            new_center_index++;  
        }
    }
#  endif
    
    //vector <matched_pts> _pts_left;
    //vector <matched_pts> _pts_right;
    
    for(int i=0; i<num_matched; i++)
    {
        matched_pts temp_L,temp_R;
        if (cvmGet(inlier_mask, i, 0)==1)
       {
           temp_L.x= center1[i].x;
           temp_L.y= center1[i].y;
           temp_R.x= center2[i].x;
           temp_R.y= center2[i].y;

           _pts_left->push_back(temp_L);
           _pts_right->push_back(temp_R);
       
       }
           }
    //cout<< _pts_left.size()<<" "<<_pts_right.size()<<endl;
    
    
    //vector<int> putative_point;
    //putative_point=determine_uniquness(new_center1,new_center2, result,num_matched);
    //cout<<"After_Ransac__ "<<(int)putative_point.size()/4<<endl;
    
   //    FILE *f1 = fopen("/Users/chih-hsiangchang/Desktop/Archive/featurePt1.txt", "w");
   //    FILE *f2 = fopen("/Users/chih-hsiangchang/Desktop/Archive/featurePt2.txt", "w");
   //#define DEBUG1_TEST
#ifdef DEBUG1_TEST
    
    CvMat *points1_=cvCreateMat( (int)putative_point.size()/4,2,CV_32FC1);
    CvMat *points2_=cvCreateMat( (int)putative_point.size()/4,2,CV_32FC1);
    for (int i=0; i<(int)putative_point.size();i+=4)
    {
        putative_point1[i/4].x= putative_point[i];
        putative_point1[i/4].y= putative_point[i+1];
        putative_point2[i/4].x= putative_point[i+2];
        putative_point2[i/4].y= putative_point[i+3];
        
        ////  
        float location_x1= putative_point[i];
        float location_y1= putative_point[i+1];
        float location_x2= putative_point[i+2];
        float location_y2= putative_point[i+3];
        
        
        
        CvScalar s1=cvGet2D(result,location_y1,location_x1);
        CvScalar s2=cvGet2D(result1,location_y2,location_x2);
        
        int s1_in= (int) s1.val[0];
        int s2_in= (int) s2.val[0];
        fprintf(f1, "%d %d %d\n", putative_point[i],putative_point[i+1],s1_in);
        fprintf(f2, "%d %d %d\n", putative_point[i+2],putative_point[i+3],s2_in);
        
        //////
        cvCircle(result1,putative_point1[i/4], 5, CV_RGB(255,255,255));
        cvCircle(result,putative_point2[i/4], 5, CV_RGB(255,255,255));
    }
    fclose(f1);
    fclose(f2); 
    
    //    new_center_index=(int)putative_point.size()/4;
    //    for (int i=0; i<=1  ;i++)
    //    {
    //        for (int j=0;j< new_center_index;j++)
    //        {
    //            if (i==0)
    //            {
    //                cvmSet(points1,j,i,putative_point1[j].x);
    //                cvmSet(points2,j,i,putative_point2[j].x);     
    //            }
    //            else
    //            {                
    //                cvmSet(points1,j,i,putative_point2[j].y);
    //                cvmSet(points2,j,i,putative_point2[j].y);
    //            }
    //        }
    //    }
    //    new_center_index=(int)putative_point.size()/4;
    //cout<<"putative_point"<<"\n";
    
    cvShowImage("right_image", result);
    cvShowImage("left_image",result1);
    cvSaveImage("/img.jpg", result);
    cvSaveImage("/img1.jpg", result1);
    cvWaitKey(0);
#endif
    cvReleaseMat(&H_matrix);
    cvClearMemStorage(storage);
    cvReleaseMemStorage(&storage);
    
    
   // return( putative_point);
}
void flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
                    const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
	int length = (int)(objectDescriptors->elem_size/sizeof(float));
    
    cv::Mat m_object(objectDescriptors->total, length, CV_32F);
	cv::Mat m_image(imageDescriptors->total, length, CV_32F);
    
    
	// copy descriptors
    CvSeqReader obj_reader;
	float* obj_ptr = m_object.ptr<float>(0);
    cvStartReadSeq( objectDescriptors, &obj_reader );
    for(int i = 0; i < objectDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)obj_reader.ptr;
        CV_NEXT_SEQ_ELEM( obj_reader.seq->elem_size, obj_reader );
        memcpy(obj_ptr, descriptor, length*sizeof(float));
        obj_ptr += length;
    }
    CvSeqReader img_reader;
	float* img_ptr = m_image.ptr<float>(0);
    cvStartReadSeq( imageDescriptors, &img_reader );
    for(int i = 0; i < imageDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)img_reader.ptr;
        CV_NEXT_SEQ_ELEM( img_reader.seq->elem_size, img_reader );
        memcpy(img_ptr, descriptor, length*sizeof(float));
        img_ptr += length;
    }
    cv::Mat m_indices(objectDescriptors->total, 2, CV_32S);
    cv::Mat m_dists(objectDescriptors->total, 2, CV_32F);
    cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(8));  // using 4 randomized kdtrees
    //  flann_index.radiusSearch(m_object, m_indices, m_dists, 16, cv::flann::SearchParams(128));
    flann_index.knnSearch(m_object, m_indices, m_dists,2, cv::flann::SearchParams(32)); // maximum number of leafs checked
    
    int* indices_ptr = m_indices.ptr<int>(0);
    float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) { 
    	if (dists_ptr[2*i]<0.4*dists_ptr[2*i+1]) {
    		ptpairs.push_back(i);
    		ptpairs.push_back(indices_ptr[2*i]);
    	}
    }
}

void featurematching_process(IplImage *IGray_r,IplImage *IGray_l, double *H, vector<matched_pts>* _pts_left,vector<matched_pts>* _pts_right)
{
    
# define SURF
# ifdef SURF
    vector<int> featurematching_vector;
    //vector<matched_pts> _pts_left;
    //vector<matched_pts> _pts_right;
    //double *H_matrix = new double [9];
    SURF_feature_matching (IGray_r,IGray_l, H, _pts_left, _pts_right);
    
   
    //cout<<H[0]<<" "<<H[1]<<" "<<H[2]<<endl;
    //cout<<H[3]<<" "<<H[4]<<" "<<H[5]<<endl;
    //cout<<H[6]<<" "<<H[7]<<" "<<H[8]<<endl;

#ifdef plot_feature_matching
    
    CvPoint  putative_point1[(int)_pts_left->size()];
    CvPoint  putative_point2[(int)_pts_right->size()];
    
    
     
    for (int i=0; i<(int)_pts_left->size();i++)
    {
          putative_point1[i].x =(int) (*_pts_left)[i].x;
          putative_point1[i].y =(int) (*_pts_left)[i].y;
          putative_point2[i].x =(int) (*_pts_right)[i].x;
          putative_point2[i].y =(int) (*_pts_right)[i].y; 
         // cout<<putative_point2[i].x<<" "<<putative_point2[i].y<<" "<<putative_point1[i].x<<" "<<putative_point1[i].y<<endl;
    }
    
  //*/
    //
     int size= (int) _pts_left->size();
    IplImage* I_show;
    I_show=plot_two_imagesf(IGray_l,IGray_r,putative_point1,putative_point2,size);
    cvShowImage("test", I_show);
    cvWaitKey(0);
# endif
#endif    
    /*
    CvPoint  putative_point1[(int)featurematching_vector.size()/4];
    CvPoint  putative_point2[(int)featurematching_vector.size()/4];
    for (int i=0; i<(int)featurematching_vector.size();i+=4)
    {
        putative_point1[i/4].x= featurematching_vector[i];
        putative_point1[i/4].y= featurematching_vector[i+1];
        putative_point2[i/4].x= featurematching_vector[i+2];
        putative_point2[i/4].y= featurematching_vector[i+3]; 
    }
    int size=(int)featurematching_vector.size()/4; 
    int num_pts= size;
    
    //v3_t *l1_pt= new v3_t[num_pts];
    //v3_t *r1_pt= new v3_t[num_pts];
    
    //cvpoint2vector(num_pts, putative_point1, l1_pt);
    //cvpoint2vector(num_pts, putative_point2, r1_pt);
    //for (int i=0; i<num_pts; i++)
    //{
    //    pts[0].R_pts.push_back(r1_pt[i]);
    //    pts[0].L_pts.push_back(l1_pt[i]);
    //}
    
#endif   
    //#define SIFT
# ifdef SIFT    
    vector<float> featurematching_vector_sift;
    // SIFT_feature_matching_real (IGray_r,IGray_l);
    //featurematching_vector_sift= SIFT_feature_matching_real (IGray_r,IGray_l);
    featurematching_vector_sift=SIFT_feature_matching(IGray_r,IGray_l);
    Point2f  *putative_pt1 =new Point2f[(int)featurematching_vector_sift.size()/4];
    Point2f  *putative_pt2= new Point2f[(int)featurematching_vector_sift.size()/4];
    for (int i=0; i<(int)featurematching_vector_sift.size();i+=4)
    {
        putative_pt2[i/4].x= featurematching_vector_sift[i];
        putative_pt2[i/4].y= featurematching_vector_sift[i+1];
        putative_pt1[i/4].x= featurematching_vector_sift[i+2];
        putative_pt1[i/4].y= featurematching_vector_sift[i+3]; 
    }
    int size=(int)featurematching_vector_sift.size()/4; 
    int num_pts= size;
    
    v3_t *l1_pt= new v3_t[num_pts];
    v3_t *r1_pt= new v3_t[num_pts];
    
    float_cvpoint2vector(num_pts, putative_pt1, l1_pt);
    float_cvpoint2vector(num_pts, putative_pt2, r1_pt);
    for (int i=0; i<num_pts; i++)
    {
        pts[0].R_pts.push_back(r1_pt[i]);
        pts[0].L_pts.push_back(l1_pt[i]);
    }
    
    //IplImage* I_show;
    I_show=plot_two_imagesf(IGray_l,IGray_r,putative_pt1,putative_pt2, size);
    //cvShowImage("test", I_show);
    //cout<<"test"<<endl;
    //cvWaitKey(0);
    delete[] putative_pt1;
    delete[] putative_pt2;
    //  # endif
#endif    
    
    
    //#define DEBUG1
# ifdef DEBUG1
    IplImage* I_show;
    I_show=plot_two_images(IGray_l,IGray_r,putative_point1,putative_point2, size);
    cvShowImage("test", I_show);
    //cout<<"test"<<endl;
    cvWaitKey(0);
# endif
   */ 
}

int  RANSAC_homography(int num, CvPoint2D64f *m1, CvPoint2D64f *m2, CvMat *H, CvMat *inlier_mask)
{
    
    int i,j; 
    int N = 1, s = 4, sample_cnt = 0.9; 
    double e, p = 0.8; 
    int numinlier, MAX_num; 
    double curr_dist_std, dist_std; 
    bool iscolinear; 
    CvPoint2D64f *curr_m1 = new CvPoint2D64f[s]; 
    CvPoint2D64f *curr_m2 = new CvPoint2D64f[s]; 
    int *curr_idx = new int[s];
    CvMat *curr_inlier_mask = cvCreateMat(num,1,CV_64FC1);
    CvMat *curr_H = cvCreateMat(3,3,CV_64FC1);
    CvMat *T1 = cvCreateMat(3,3,CV_64FC1);
    CvMat *T2 = cvCreateMat(3,3,CV_64FC1);
    CvMat *invT2 = cvCreateMat(3,3,CV_64FC1); 
    CvMat *tmp_pt = cvCreateMat(3,1,CV_64FC1);
    // RANSAC algorithm (reject outliers and obtain the best H)
    //srand(time(0)); 
    MAX_num = -1;
    for (int test=0;test<200;test++)
       {
    //while(N > sample_cnt)
    //     {
        // for a randomly chosen non-colinear correspondances
        iscolinear = true;
        while(iscolinear == true)
        {
            iscolinear = false;
            for(i=0; i<s; i++)
            {
                curr_idx[i] = rand()%num; 
                for(j=0; j<i; j++)
                {
                    if(curr_idx[i] == curr_idx[j])
                    {
                        iscolinear = true; 
                        break;
                    }
                }
                
                if(iscolinear == true)
                    break; 
                curr_m1[i].x = m1[curr_idx[i]].x; 
                curr_m1[i].y = m1[curr_idx[i]].y;
                curr_m2[i].x = m2[curr_idx[i]].x; 
                curr_m2[i].y = m2[curr_idx[i]].y;
            }
            // Check whether these points are colinear
            if(iscolinear == false) 
                iscolinear = isColinear(s, curr_m1);
         }
        // Nomalized DLT 
        Normalization(s, curr_m1, T1); //curr_m1 <- T1 * curr_m1 
        Normalization(s, curr_m2, T2); //curr_m2 <- T2 * curr_m2
        // Compute the homography matrix H = invT2 * curr_H * T1
        ComputeH(s, curr_m1, curr_m2, curr_H); 
        cvInvert(T2, invT2);
        cvMatMul(invT2, curr_H, curr_H); // curr_H <- invT2 * curr_H 
        cvMatMul(curr_H, T1, curr_H);	// curr_H <- curr_H * T1
        // Calculate the distance for each putative correspondence // and compute the number of inliers 
        numinlier = ComputeInliers(num,m1,m2,curr_H,curr_inlier_mask,&curr_dist_std);
        //cout<<numinlier<<endl;
        // Update a better H
        
        // if(numinlier > MAX_num || (numinlier == MAX_num && curr_dist_std < dist_std))
        if(numinlier > MAX_num) 
        {
            MAX_num = numinlier; 
            cvCopy(curr_H, H); 
            cvCopy(curr_inlier_mask, inlier_mask); 
            dist_std = curr_dist_std;
        }
        //     
        //     e = 1 - (double) numinlier / (double)num;
          
        //     N = (int)(log(1-p)/log(1-pow(1-e,s))); 
             
             //sample_cnt++;
        //     cout<<"test "<<e<<endl;
        //   if (e<0.05)
        //       break;
    }
    delete curr_m1, curr_m2, curr_idx; 
    cout<<MAX_num<<endl;
    cvReleaseMat(&curr_H); 
    cvReleaseMat(&T1); 
    cvReleaseMat(&T2); 
    cvReleaseMat(&invT2); 
    cvReleaseMat(&tmp_pt);
    cvReleaseMat(&curr_inlier_mask);
    return(MAX_num);
    
}
void Normalization(int num, CvPoint2D64f *p, CvMat *T)
{
    double scale, tx, ty;
    double meanx, meany;
    double value;
    int i; 
    CvMat *x = cvCreateMat(3,1,CV_64FC1); 
    CvMat *xp = cvCreateMat(3,1,CV_64FC1);
    meanx = 0; 
    meany = 0;
    for(i=0; i<num; i++)
    {
        meanx += p[i].x; 
        meany += p[i].y;
    } 
    meanx /= (double)num; 
    meany /= (double)num;
    value= 0;
    for(i=0; i<num; i++)
    { value += sqrt(pow(p[i].x-meanx, 2.0) + pow(p[i].y-meany, 2.0));}
    
    value /= (double)num;
    scale = sqrt(2.0)/value; 
    tx = -scale * meanx;
    ty = -scale * meany;
    cvZero(T); 
    cvmSet(T,0,0,scale);
    cvmSet(T,0,2,tx); 
    cvmSet(T,1,1,scale); 
    cvmSet(T,1,2,ty); 
    cvmSet(T,2,2,1.0);
    for(i=0; i<num; i++)
    { 
        cvmSet(x,0,0,p[i].x);
        cvmSet(x,1,0,p[i].y); 
        cvmSet(x,2,0,1.0); 
        cvMatMul(T,x,xp); 
        p[i].x = cvmGet(xp,0,0)/cvmGet(xp,2,0);
        p[i].y = cvmGet(xp,1,0)/cvmGet(xp,2,0);
    }
    cvReleaseMat(&x); 
    cvReleaseMat(&xp);
}
void ComputeH(int n, CvPoint2D64f *p1, CvPoint2D64f *p2, CvMat *H)
{
    int i; 
    CvMat *A = cvCreateMat(2*n, 9, CV_64FC1); 
    CvMat *U = cvCreateMat(2*n, 2*n, CV_64FC1); 
    CvMat *D = cvCreateMat(2*n, 9, CV_64FC1); 
    CvMat *V = cvCreateMat(9, 9, CV_64FC1);
    cvZero(A); 
    for(i=0; i<n;i++)
    {
        cvmSet(A,2*i,3,-p1[i].x);
        cvmSet(A,2*i,4,-p1[i].y); 
        cvmSet(A,2*i,5,-1); 
        cvmSet(A,2*i,6,p2[i].y*p1[i].x); 
        cvmSet(A,2*i,7,p2[i].y*p1[i].y);
        cvmSet(A,2*i,8,p2[i].y);
        
        // 2*i+1 row
        cvmSet(A,2*i+1,0,p1[i].x); 
        cvmSet(A,2*i+1,1,p1[i].y); 
        cvmSet(A,2*i+1,2,1); 
        cvmSet(A,2*i+1,6,-p2[i].x*p1[i].x); 
        cvmSet(A,2*i+1,7,-p2[i].x*p1[i].y); 
        cvmSet(A,2*i+1,8,-p2[i].x);
    }
    cvSVD(A, D, U, V, CV_SVD_U_T|CV_SVD_V_T);
    for(i=0; i<9; i++)
    {
        cvmSet(H, i/3, i%3, cvmGet(V, 8, i));
    }
    cvReleaseMat(&A);
    cvReleaseMat(&U); 
    cvReleaseMat(&D);
    cvReleaseMat(&V);
}
bool isColinear(int num, CvPoint2D64f *p)
{
    //    Check colinearity of a set of pts input: p (pts to be checked)
    //    num (ttl number of pts) return true if some pts are coliner
    //        false if not
    int i,j,k; 
    bool iscolinear;
    double value; 
    CvMat *pt1 = cvCreateMat(3, 1, CV_64FC1);
    CvMat *pt2 = cvCreateMat(3, 1, CV_64FC1);
    CvMat *pt3 = cvCreateMat(3, 1, CV_64FC1);
    CvMat *line = cvCreateMat(3, 1, CV_64FC1);
    iscolinear = false;
    for(i=0; i<num-2; i++)
    {
        cvmSet(pt1,0,0,p[i].x); 
        cvmSet(pt1,1,0,p[i].y); 
        cvmSet(pt1,2,0,1);
        for(j=i+1; j<num-1; j++)
        {
            cvmSet(pt2,0,0,p[j].x); 
            cvmSet(pt2,1,0,p[j].y); 
            cvmSet(pt2,2,0,1); // compute the line connecting pt1 & pt2 cvCrossProduct(pt1, pt2, line);
            cvCrossProduct(pt1, pt2, line);
            for(k=j+1; k<num; k++)
            {
                cvmSet(pt3,0,0,p[k].x); 
                cvmSet(pt3,1,0,p[k].y);
                cvmSet(pt3,2,0,1); // check whether pt3 on the line value = cvDotProduct(pt3, line);
                value = cvDotProduct(pt3, line);
                if(abs(value) < 10e-2)
                {
                    iscolinear = true; 
                    break;
                }
            }
            if(iscolinear == true) 
                break;
        }
        if(iscolinear == true) 
            break;
    }
    cvReleaseMat(&pt1); 
    cvReleaseMat(&pt2); 
    cvReleaseMat(&pt3);
    cvReleaseMat(&line); 
    return iscolinear;
}

int ComputeInliers(int num, CvPoint2D64f *p1, CvPoint2D64f *p2, CvMat *H, CvMat *inlier_mask, double *dist_std)
{
    int i, num_inlier; double curr_dist,curr_dist1,curr_dist2 , sum_dist, mean_dist;
    CvPoint2D64f tmp_pt;
    CvMat *dist = cvCreateMat(num, 1, CV_64FC1); 
    CvMat *x = cvCreateMat(3,1,CV_64FC1);
    CvMat *xp = cvCreateMat(3,1,CV_64FC1); 
    CvMat *pt = cvCreateMat(3,1,CV_64FC1); 
    CvMat *invH = cvCreateMat(3,3,CV_64FC1);
    cvInvert(H, invH);
    sum_dist = 0; 
    num_inlier = 0; 
    cvZero(inlier_mask); 
    for(i=0; i<num; i++){
        
        cvmSet(x,0,0,p1[i].x); 
        cvmSet(x,1,0,p1[i].y); 
        cvmSet(x,2,0,1); // initial point x'
        
        cvmSet(xp,0,0,p2[i].x); 
        cvmSet(xp,1,0,p2[i].y); 
        cvmSet(xp,2,0,1);
        cvMatMul(H, x, pt);
        tmp_pt.x = (int)(cvmGet(pt,0,0)/cvmGet(pt,2,0));
        tmp_pt.y = (int)(cvmGet(pt,1,0)/cvmGet(pt,2,0)); 
        curr_dist1 = pow(tmp_pt.x-p2[i].x, 2.0) + pow(tmp_pt.y-p2[i].y, 2.0); // d(x, invH x') 
        cvMatMul(invH, xp, pt);
        tmp_pt.x = (int)(cvmGet(pt,0,0)/cvmGet(pt,2,0)); 
        tmp_pt.y = (int)(cvmGet(pt,1,0)/cvmGet(pt,2,0)); 
        curr_dist2 = pow(tmp_pt.x-p1[i].x, 2.0) + pow(tmp_pt.y-p1[i].y, 2.0);
        curr_dist=sqrt(curr_dist1+curr_dist2);
        // cout<<curr_dist<<endl;
        if(curr_dist < T_DIST)
        { // an inlier
            num_inlier++; 
            cvmSet(inlier_mask,i,0,1); 
            cvmSet(dist,i,0,curr_dist); 
            sum_dist += curr_dist;
        }
    }
    
    mean_dist = sum_dist/(double)num_inlier; 
    *dist_std = 0; 
    for(i=0; i<num; i++)
    {
        if(cvmGet(inlier_mask,i,0) == 1) 
            *dist_std += pow(cvmGet(dist,i,0)-mean_dist,2.0);
    } 
    *dist_std /= (double) (num_inlier -1);
    
    cvReleaseMat(&dist);
    cvReleaseMat(&x);
    cvReleaseMat(&xp); 
    cvReleaseMat(&pt);
    cvReleaseMat(&invH);
    return num_inlier;
}
IplImage* plot_two_imagesf(IplImage *IGray, IplImage *IGray1, CvPoint *corners1, CvPoint *corners3, int &vector_index)
{
    CvPoint newmatched;       
    int  ttw  =  IGray->width+ IGray->width;
    int  ttl  =  IGray->height;
    IplImage* Imagedisplay  = cvCreateImage(cvSize(2*IGray->width,IGray->height), IGray->depth, IGray->nChannels );
    for (int i=0; i<ttl;i++)
    {
        for (int j=0; j<ttw; j++)
        {
            if (j< IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray,i,j));
            }
            if (j>IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray1,i,j-(IGray1->width)));
            }
        }
    }
    for (int i=0;i<vector_index; i++)
    {           
        newmatched.x= (int)(corners3[i].x+(IGray->width));
        newmatched.y= (int)(corners3[i].y);
        cvLine(Imagedisplay, 
               cvPoint( corners1[i].x-1, corners1[i].y ), 
               cvPoint( newmatched.x, newmatched.y ), 
               CV_RGB(256,256,256)
               );
        cvCircle(Imagedisplay, newmatched, 3, CV_RGB(0,255,0));
        cvCircle(Imagedisplay, corners1[i], 3, CV_RGB(0,255,0));
        
    }   
    //double rect2_width, rect2_height;
    //double rect1_width, rect1_height;
    
    //rect2_width = rect2[3].x - rect2[0].x;
    //rect2_height= rect2[3].y - rect2[0].y;
    
    //rect1_width = rect1[3].x - rect1[0].x;
    //rect1_height= rect1[3].y - rect1[0].y;
    
    
    //cvRectangle(Imagedisplay, cvPoint(rect2[0].x, rect2[0].y), cvPoint(rect2[0].x+rect2_width,rect2[0].y+rect2_height),
    //            cvScalar(0xff,0xff,0xff) );
    
        //for (int i=1;i<3;i++)
    //{
    
    //    cvLine(Imagedisplay, 
    //           cvPoint(rect1[0].x+ (IGray->width), rect1[0].y ), 
    //           cvPoint(rect1[1].x+ (IGray->width), rect1[1].y ), 
    //           CV_RGB(0,0,255)
    //           );
    //    cvLine(Imagedisplay, 
    //           cvPoint(rect1[1].x+ (IGray->width), rect1[1].y ), 
    //           cvPoint(rect1[3].x+ (IGray->width), rect1[3].y ), 
    //           CV_RGB(0,0,255)
    //           );
    //   cvLine(Imagedisplay, 
    //           cvPoint(rect1[2].x+ (IGray->width), rect1[2].y ), 
    //           cvPoint(rect1[3].x+ (IGray->width), rect1[3].y ), 
    //           CV_RGB(0,0,255)
    //           );
    //   cvLine(Imagedisplay, 
    //           cvPoint(rect1[0].x+ (IGray->width), rect1[0].y ), 
    //           cvPoint(rect1[2].x+ (IGray->width), rect1[2].y ), 
    //           CV_RGB(0,0,255)
    //           );

    //}
    
    
    return (Imagedisplay);
}

void select_points(vector<matched_pts>* _pts_left,vector<matched_pts>* _pts_right, double* position, vector<matched_pts>* refinded_pt_left, vector<matched_pts>* refinded_pt_right, double *H)
{
    
   
    //    pt_1  --- pt_2
    //      |        |
    //    pt_3  --- pt_4
    
    matched_pts* pt_1 = new matched_pts [1]; matched_pts* pt_2 = new matched_pts [1]; matched_pts* pt_3 = new matched_pts [1];
    matched_pts* pt_4 = new matched_pts [1];
    
    pt_1[0].x = position[0]; pt_1[0].y = position[1]; pt_2[0].x = position[2];  pt_2[0].y = position[3]; 
    pt_3[0].x = position[4]; pt_3[0].y = position[5]; pt_4[0].x = position[6];  pt_4[0].y = position[7]; 


    for (int i=0;i< _pts_right->size();i++)
    {   
        if  (((*_pts_right)[i].x > pt_1[0].x) && ((*_pts_right)[i].x) < pt_4[0].x)
        {
            if (((*_pts_right)[i].y > pt_1[0].y) && ((*_pts_right)[i].y) < pt_4[0].y)
            {
                refinded_pt_left->push_back((*_pts_left)[i]);
                refinded_pt_right->push_back((*_pts_right)[i]);
            }
        }
    }
    
    
    
       /*
        for (int i=0;i< _pts_left->size();i++)
        {   
            if  (((*_pts_left)[i].x > pt_1[0].x) && ((*_pts_left)[i].x) < pt_4[0].x)
            {
            if (((*_pts_left)[i].y > pt_1[0].y) && ((*_pts_left)[i].y) < pt_4[0].y)
            {
             refinded_pt_left->push_back((*_pts_left)[i]);
             refinded_pt_right->push_back((*_pts_right)[i]);
            }
         }
        }
      */    
    int size = (int) refinded_pt_left->size();
    
    CvPoint2D64f *m1 = new CvPoint2D64f[size];
    CvPoint2D64f *m2 = new CvPoint2D64f[size];
    
    for (int i=0;i<size;i++)
    {
        m1[i].x = (*refinded_pt_left)[i].x;
        m1[i].y = (*refinded_pt_left)[i].y;
        m2[i].x = (*refinded_pt_right)[i].x;
        m2[i].y = (*refinded_pt_right)[i].y;
        //cout<< m1[i].x<<" "<< m1[i].y<<endl;
        //cout<< m2[i].x<<" "<< m2[i].y<<endl;
    }
    
    assert(size > 4 && (" points inside ROI < 4"));
    
    CvMat *H_matrix = cvCreateMat(3,3,CV_64FC1);
    CvMat *inlier_mask = cvCreateMat(size,1,CV_64FC1); 
    RANSAC_homography(size,m1, m2, H_matrix, inlier_mask);
    
    
    ///  error_evaluation //
    
    CvMat *pt = cvCreateMat(3,1,CV_64FC1);
    CvMat *x =  cvCreateMat(3,1,CV_64FC1);
    double error;
    double *_2Dpt = new double[2];
    for (int i=0;i< size;i++)
    {
   
    cvmSet(x,0,0,m1[i].x);
    cvmSet(x,1,0,m1[i].y);
    cvmSet(x,2,0,1);  
        
    cvMatMul(H_matrix,x, pt);
    _2Dpt[0]= (cvmGet(pt, 0, 0) / cvmGet(pt, 2, 0));
    _2Dpt[1]= (cvmGet(pt, 1, 0) / cvmGet(pt, 2, 0));
     error += sqrt(((_2Dpt[0]- m2[i].x)*(_2Dpt[0]- m2[i].x))+((_2Dpt[1]- m2[i].y)*(_2Dpt[1]- m2[i].y)));
        
    }
    
    cout<<"error output "<<error/size<<endl;
    H[0]= cvmGet(H_matrix, 0, 0); H[1]=cvmGet(H_matrix, 0, 1);H[2]=cvmGet(H_matrix, 0, 2);
    H[3]= cvmGet(H_matrix, 1, 0); H[4]=cvmGet(H_matrix, 1, 1);H[5]=cvmGet(H_matrix, 1, 2);
    H[6]= cvmGet(H_matrix, 2, 0); H[7]=cvmGet(H_matrix, 2, 1);H[8]=cvmGet(H_matrix, 2, 2);
    
    delete  _2Dpt;
    cvReleaseMat(&H_matrix);
    cvReleaseMat(&inlier_mask);
    cvReleaseMat(&x);
    cvReleaseMat(&pt);
}

