/*********************************************************************************************//**
* @file viewpoint_estimation_lib.h
*
* ViewPoint estimator header
*
* Copyright (c)
* Frantisek Durovsky
* Department of Robotics
* Technical University Kosice
* March 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/

#ifndef VIEWPOINT_ESTIMATOR_LIB_H
#define VIEWPOINT_ESTIMATOR_LIB_H

//Standard ROS headers
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <fstream>
#include <visualization_msgs/Marker.h>

//Aruco headers
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/arucofidmarkers.h>

//OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class ViewPoint_Estimator
{
public:

    explicit ViewPoint_Estimator(ros::NodeHandle *nh);
    ~ViewPoint_Estimator();
    bool load_calibration_file(std::string filename);
    void image_callback(const sensor_msgs::ImageConstPtr &original_image);

    bool chessboard_find_pattern(cv::Mat input_image, cv::Mat output_image);
    bool markers_find_pattern(cv::Mat input_image, cv::Mat output_image);
  
    void publish_marker(geometry_msgs::Pose marker_pose,const int marker_id);


    tf::Transform arucoMarker2Tf(const aruco::Marker &marker);
    
    
private:

    cv::Mat I;                                       //OpeCV image

    //Calibration parameters
    std::string filename;                            //calibration file path
    cv::Mat *intrinsics;                             //camera intrinsics
    cv::Mat *distortion_coeff;                       //camera distortion coeffs
    cv::Size *image_size;                            //image_size

    //ROS messaging
    ros::Publisher marker_pub;                       //marker visualization

    //Chessboard detection variables
    cv::Size board_size;                             //widht and height of calibration grid
    float square_size;                               //size of grid square

    std::vector <cv::Point3f> chessboard3D_points;   //chessboard 3D points in own reference frame
    std::vector<cv::Point2f>  chessboard2D_points;   //chessboard 2D points in image

    std::vector <cv::Point3d> ref_frame_points;      //reference xyz frame points
    std::vector<cv::Point2d>  image_frame_points;     //reference xyz frame points in image

    //Marker parameters
    double marker_size;                               //marker geometry
    aruco::CameraParameters aruco_calib_params;      //Camera parameters for aruco lib

    //TF frame
    std::string camera_frame;                        //parrent camera frame name
   
};

#endif //VIEWPOINT_ESTIMATOR_LIB_H
