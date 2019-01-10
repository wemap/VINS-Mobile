//
//  feature_tracker.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "feature_tracker.hpp"

int FeatureTracker::n_id = 0;
FeatureTracker::FeatureTracker()
:update_finished{false},img_cnt{0},current_time{-1.0},use_pnp{false}
{
    printf("init ok\n");
}
/*********************************************************tools function for feature tracker start*****************************************************/
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

template <typename T>
void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


/*********************************************************tools function for feature tracker ending*****************************************************/

bool FeatureTracker::solveVinsPnP(double header, Vector3d &P, Matrix3d &R, bool vins_normal)
{
    if(!vins_normal)
        return false;
    /*
     if(solved_features.size() < 2)
     {
     printf("pnp not enough features\n");
     return false;
     }
     */
    vector<IMG_MSG_LOCAL> feature_msg;
    int i = 0;
    for (auto &it : solved_features)
    {
        while(ids[i] < it.id)
        {
            i++;
        }
        if(ids[i] == it.id)
        {
            IMG_MSG_LOCAL tmp;
            tmp = it;
            tmp.observation = (Vector2d((forw_pts[i].x - PX)/FOCUS_LENGTH_X, (forw_pts[i].y - PY)/FOCUS_LENGTH_Y));
            feature_msg.push_back(tmp);
        }
    }
    /*
     if(feature_msg.size() < 2 )
     {
     printf("pnp Not enough solved feature!\n");
     return false;
     }
     */
    vins_pnp.setInit(solved_vins);
    printf("pnp imu header: ");
    for(auto &it : imu_msgs)
    {
        double t = it.header;
        if (current_time < 0)
            current_time = t;
        double dt = (t - current_time);
        current_time = t;
        printf("%lf ",t);
        vins_pnp.processIMU(dt, it.acc, it.gyr);
    }
    printf("image %lf\n", header);
    vins_pnp.processImage(feature_msg, header, use_pnp);
    
    P = vins_pnp.Ps[PNP_SIZE - 1];
    R = vins_pnp.Rs[PNP_SIZE - 1];
    Vector3d R_ypr = Utility::R2ypr(R);
    return true;
}

void FeatureTracker::readImage(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<Point2f> &good_pts, vector<double> &track_len, double header, Vector3d &P, Matrix3d &R, bool vins_normal)
{
    
    // ********
    // Thibaud: This method is called BEFORE and AFTER INITIALIZATION
    //
    //          track_len is empty here, it will become a vector of the size of good_pts (often 70), each value is between 0 and 1,
    //              1 is a good parallax.
    //
    //          good_pts and track_len are here just for drawing, good_pts is empty here
    // ********
    
    printf("TIME: FeatureTracker::readImage: %.3ld\n", std::time(nullptr));


    // ********
    // Thibaud: *_img are empty first time
    // ********
    result = _img;
    if(forw_img.empty())
        pre_img = cur_img = forw_img = _img;
    else
        forw_img = _img;

    
    forw_pts.clear();
    
    //track
    {
        // ********
        // Thibaud: Here, cur_pts are forw_pts from previous frame.
        //          So, following code is not called if no points are tracked at previous frame. (a black frame for eg.)
        // ********
        if(cur_pts.size()>0)
        {
            vector<uchar> status;
            vector<float> err;
            
            //TS(time_track);
            

            // ********
            // Thibaud: cur_pts has a size of N and N is often 70
            //          forw_pts input is empty
            //          forw_pts output is size of N
            //          status is size of N
            // ********
            printf("TIME: FeatureTracker::readImage - calcOpticalFlowPyrLK (before): %ld\n", cur_pts.size());
            calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);



            // ********
            // Thibaud: Remove pts which are on image border
            // ********
            //TE(time_track);
            for (int i = 0; i < int(forw_pts.size()); i++)
                if (status[i] && !inBorder(forw_pts[i]))
                    status[i] = 0;
            
            // ********
            // Thibaud: Remove indexes from status in following vectors
            // ********
            reduceVector(pre_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(forw_pts, status);
            reduceVector(ids, status);
            reduceVector(parallax_cnt, status);
            printf("TIME: FeatureTracker::readImage - calcOpticalFlowPyrLK (after): %ld\n", cur_pts.size());

            // ********
            // Thibaud: Call ransac with cur_pts and forw_pts and reduce following vectors:
            //          pre_pts, cur_pts, forw_pts, ids, parallax_cnt
            //          Call it only if there is more than 8 forw_pts
            // ********
            if (forw_pts.size() >= 8)
            {
                vector<uchar> status;
                
                cv::findFundamentalMat(cur_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
//                printf("TIME: FeatureTracker::readImage - findFundamentalMat: %ld\n", cur_pts.size());
                reduceVector(cur_pts, status);
                reduceVector(pre_pts, status);
                reduceVector(forw_pts, status);
                reduceVector(ids, status);
                reduceVector(parallax_cnt, status);
            }
            
            solveVinsPnP(header, P, R, vins_normal);
            
            // ********
            // Thibaud: This section is called 2 frames out of 3 (when detection is not called
            //           I think this is just for drawing
            // ********
            if(img_cnt!=0)
            {
                for (int i = 0; i< forw_pts.size(); i++)
                {
                    //cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
                    good_pts.push_back(forw_pts[i]);
                    if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                    {
                        parallax_cnt[i].min = forw_pts[i];
                    }
                    else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                    {
                        parallax_cnt[i].max = forw_pts[i];
                    }
                    double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                    track_len.push_back(std::min(1.0, 1.0 * parallax/30));
                }
            }
        }
    }
    
    // ********
    // Thibaud: Every 3 frames (when img_cnt == 0), good_pts is empty here
    // ********

    // detect
    // ********
    // Thibaud: This part is called every 3 frames (when img_cnt == 0)
    //          good_pts is empty here
    //
    //          Firstly, ransac is called if forw_pts >= 8
    //              -> pre_pts, cur_pts, forw_pts, ids, parallax_cnt are modified with ransac result
    //              -> remaining fwd_pts are added to good_pts vector
    //          Then, parallax_cnt is modified regarding to good_pts found with ransac. (min/max for each point)
    //
    // ********
    {
        // ********
        // Thibaud: img_cnt is an iterative counter with a modulus 3
        // ********
        if(img_cnt==0)
        {
            // ********
            // Thibaud: Following is called every 3 frames
            // ********
            
            
            
            
            // ********
            // Thibaud: Call ransac with pre_pts and forw_pts and reduce following vectors:
            //          pre_pts, cur_pts, forw_pts, ids, parallax_cnt
            //          Call it only if there is more than 8 forw_pts
            // ********
            if (forw_pts.size() >= 8)
            {
                vector<uchar> status;
                cv::findFundamentalMat(pre_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
                reduceVector(pre_pts, status);
                reduceVector(cur_pts, status);
                reduceVector(forw_pts, status);
                reduceVector(ids, status);
                reduceVector(parallax_cnt, status);
            }
            
            
            // ********
            // Thibaud: good_pts are forw_pts after ransac
            // ********
            for (int i = 0; i< forw_pts.size(); i++)
            {
                good_pts.push_back(forw_pts[i]);
            }
            
            // ********
            // Thibaud: Check parallax with forw_pts
            // ********
            for (int i = 0; i< forw_pts.size(); i++)
            {
                if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
                {
                    parallax_cnt[i].min = forw_pts[i];
                }
                else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
                {
                    parallax_cnt[i].max = forw_pts[i];
                }
                double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
                
                // ********
                // Thibaud: Value between 0 (no translation) and 1 (good translation)
                // ********
                track_len.push_back(std::min(1.0, 1.0 * parallax/50));
            }
            
            

            
            // ********
            // Thibaud: MAX_CNT is 70, n_max_cnt is often less than 70
            // ********
            int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
            
            if(n_max_cnt>0)
            {
                n_pts.clear();
                TS(time_goodfeature);
                //goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, MIN_DIST, mask, 3, false, 0.04);
                // ********
                // Thibaud: Retrieve n_max_cnt features from forw_img. Result is in n_pts
                // ********
                goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, MIN_DIST);
                printf("TIME: FeatureTracker::readImage - goodFeaturesToTrack\n");
                TE(time_goodfeature);
            }
            else
            {
                // ********
                // Thibaud: It seems to work without this line
                // ********
                n_pts.clear();
            }

            // ********
            // Thibaud: Add new points to forw_pts and parallax_cnt. Create a -1 id
            // ********
            for (auto &p : n_pts) {
                forw_pts.push_back(p);
                ids.push_back(-1);
                
                // ********
                // Thibaud: Init max-min parallax with first value (p)
                // ********
                max_min_pts tmp;
                tmp.min = p;
                tmp.max = p;
                parallax_cnt.push_back(tmp);
            }
            
            
            // ********
            // Thibaud: forw img and pts became pre img and pts
            // ********
            pre_img = forw_img;
            pre_pts = forw_pts;

            // ********
            // Thibaud: Add new points to good_pts (good_pts are points after ransac, so I don't know why here)
            // ********
            //draw
            for (int i = 0; i < n_pts.size(); i++)
            {
                good_pts.push_back(n_pts[i]);
                track_len.push_back(0);
            }
            
            
//            printf("TIME: FeatureTracker::readImage - good_pts: %d\n", good_pts == forw_pts);
//            if(good_pts == forw_pts) {
//            }
            
//            good_pts = forw_pts;
            
        }
        cur_img = forw_img;
        cur_pts = forw_pts;
    }
    
    
    if(img_cnt == 0)
    {
        //update id and msg
        image_msg.clear();
        int num_new = 0;
        
        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            completed |= updateID(i);
            if (!completed)
                break;
        }
        for(int i = 0; i<ids.size(); i++)
        {
            double x = (cur_pts[i].x - PX)/FOCUS_LENGTH_X;
            double y = (cur_pts[i].y - PY)/FOCUS_LENGTH_Y;
            double z = 1.0;
            image_msg[(ids[i])] = (Vector3d(x, y, z));
        }
    }
    //finished and tell solver the data is ok
    update_finished = true;
}
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}
