/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "../../../include/Frame.h"


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void GrabGNSS(const geometry_msgs::PoseStamped& gnss_pose);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    ros::Publisher trans_pub, pose_pub;
    ros::Subscriber gnss_sub;
    tf2::Transform initial_pose_position;
    tf2::Transform initial_pose_orientation;


};

tf2::Transform FramePose (ORB_SLAM2::Frame *cframe)
{
	cv::Mat Rwc = cframe->mTcw.rowRange(0,3).colRange(0,3).t();
	cv::Mat twc = -Rwc * cframe->mTcw.rowRange(0,3).col(3);
	tf2::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
					Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
					Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
	tf2::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

	return tf2::Transform(M, V);
}

void print(const tf2::Transform& tr){
    double roll, pitch, yaw;
    tf2::Matrix3x3(tr.getRotation()).getRPY(roll, pitch, yaw);
    cout<<"x: "<<tr.getOrigin().x()<<", y: "<<tr.getOrigin().y()<<", z: "<<tr.getOrigin().z()<<", roll: "<<(roll * 180 / M_PI)<<", pitch: "<<(pitch * 180 / M_PI)<<", yaw: "<<(yaw * 180 / M_PI)<<endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh;

    if(argc < 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify left_image_topic right_image_topic" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);
    igb.trans_pub = nh.advertise<geometry_msgs::Transform>("/orb_transform", 5);
    igb.pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/orb_pose", 5);
    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    // char* left_image_topic_name = "/kitti/camera_color_left/image_raw";
    // char* right_image_topic_name = "/kitti/camera_color_right/image_raw";
    char* left_image_topic_name = "/carla/ego_vehicle/rgb_front_left/image";
    char* right_image_topic_name = "/carla/ego_vehicle/rgb_front_right/image";
    if(argc > 4 && strlen(argv[4]) > 0) left_image_topic_name = argv[4];
    if(argc > 5 && strlen(argv[5]) > 0) right_image_topic_name = argv[5];
    cout<< "left and right topic names: "<< left_image_topic_name<<", "<< right_image_topic_name <<endl;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, left_image_topic_name, 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, right_image_topic_name, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    igb.gnss_sub = nh.subscribe("/gnss_pose_throttle", 1, &ImageGrabber::GrabGNSS,&igb);
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabGNSS(const geometry_msgs::PoseStamped& gnss_pose) {
    mpSLAM->Reset();
    initial_pose_position.setOrigin(tf2::Vector3(gnss_pose.pose.position.x, gnss_pose.pose.position.y, gnss_pose.pose.position.z));
    initial_pose_orientation.setRotation(tf2::Quaternion(gnss_pose.pose.orientation.x, gnss_pose.pose.orientation.y, gnss_pose.pose.orientation.z, gnss_pose.pose.orientation.w).normalize());
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat ret_mat;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        ret_mat = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        ret_mat = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    cout<<ret_mat<<endl;

    // tf2::Transform init_orientation;
    // init_orientation.setRotation(tf2::Quaternion(0.0599455097402, -0.0614342804398, 0.688561734451, 0.720079928132).normalize());
    // // init_orientation.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0).normalize());
    
    // tf2::Matrix3x3 init_pose_rot(
    //     0.9999971, -0.0000008, -0.0023856,
    //     0.0000011,  1.0000000,  0.0001028,
    //     0.0023856, -0.0001028,  0.9999971
    // );

    // tf2::Vector3 init_pose_translation(85.0, 35.0, 2.5);
    // tf2::Vector3 init_pose_translation(-56.998928, 0.000476144 , -0.005998306);
    // tf2::Transform tf2_transform(init_pose_rot, init_pose_translation);
	
    ORB_SLAM2::Frame &cframe = mpSLAM->getTracker()->mCurrentFrame;
    tf2::Transform currentOrbPose = FramePose(&cframe);
    cout<<"CurrentOrbPose"<<endl;
    print(currentOrbPose);
    
    tf2::Transform flip;
    flip.setOrigin(tf2::Vector3(0,0,0));
    flip.setRotation(tf2::Quaternion(0.5, -0.5, 0.5, 0.5).normalize());
    flip = (initial_pose_orientation * flip.inverse()).inverse();
    tf2::Transform new_tf =  flip.inverse() * currentOrbPose;
    cout<<"New_tf"<<endl;
    print(new_tf);

    

    // tf2::Transform tf2_transform(init_pose_rot, init_pose_translation);
    // tf2::Transform tf2_transform1(tf2::Matrix3x3(flip.getRotation())*init_pose_rot, init_pose_translation);
    // // tf2_transform1 = flip * tf2_transform1;
    // tf2::Transform pose = new_tf*tf2_transform1;
    // cout<<"Pose"<<endl;
    // print(pose);
    tf2::Transform final_test_pose = new_tf * flip;
    // tf::Transform tf2_transform(tf2_rot*init_pose_rot, translation+init_pose_translation);
    geometry_msgs::Pose pose_msg;
    geometry_msgs::PoseStamped pose_stamped;
    tf2::Transform final_test_pose_orientation = final_test_pose * initial_pose_orientation;
    pose_stamped.header.frame_id = "map";
	pose_stamped.header.stamp = ros::Time(cv_ptrLeft->header.stamp.toSec());
    pose_stamped.pose.position.x = final_test_pose.getOrigin().x() + initial_pose_position.getOrigin().x();
    pose_stamped.pose.position.y = final_test_pose.getOrigin().y() + initial_pose_position.getOrigin().y();
    pose_stamped.pose.position.z = final_test_pose.getOrigin().z() + initial_pose_position.getOrigin().z();
    pose_stamped.pose.orientation.x = final_test_pose_orientation.getRotation().x();
    pose_stamped.pose.orientation.y = final_test_pose_orientation.getRotation().y();
    pose_stamped.pose.orientation.z = final_test_pose_orientation.getRotation().z();
    pose_stamped.pose.orientation.w = final_test_pose_orientation.getRotation().w();
    cout<< pose_stamped<<endl;

    pose_pub.publish(pose_stamped);
}


