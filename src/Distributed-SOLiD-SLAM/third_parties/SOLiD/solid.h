//
// Created by yewei on 2/27/20.

// This is an unofficial c++ implementation of Scan Context:
// @ARTICLE{ gkim-2019-ral,
//     author = {G. {Kim} and B. {Park} and A. {Kim}},
//     journal = {IEEE Robotics and Automation Letters},
//     title = {1-Day Learning, 1-Year Localization: Long-Term LiDAR Localization Using Scan Context Image},
//     year = {2019},
//     volume = {4},
//     number = {2},
//     pages = {1948-1955},
//     month = {April}
// }
// For more information please visit: https://github.com/irapkaist/scancontext

#ifndef SRC_SOLID_H
#define SRC_SOLID_H
//#include "utility.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

#pragma once
#include <Eigen/Core>


typedef pcl::PointXYZI  PointType;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef PointXYZIRPYT  PointTypePose;


struct SOLiDBin
{
    std::string robotname;

    double time;

    PointTypePose pose;

    pcl::PointCloud<PointType>::Ptr cloud;

    Eigen::VectorXf asolid;
    Eigen::VectorXf rsolid;
};

class SOLiD {
public:
    SOLiD(int max_range, int num_rings, int num_sectors);
    SOLiDBin ptcloud2bin(pcl::PointCloud<PointType>::Ptr pt_cloud,
                         int NUM_RANGE,
                         int NUM_ANGLE,
                         int NUM_HEIGHT,
                         float FOV_u,
                         float FOV_d,
                         int MAX_DISTANCE); 

private:
    int _max_range;
    int _num_rings;
    int _num_sectors;

    float _gap;
    float _angle_one_sector;

    //pcl::VoxelGrid<PointType> downSizeFilterInput;

    float xy2Theta(float x, float y);
    float rad2deg(float rad);
        
    std::pair<Eigen::VectorXf, Eigen::VectorXf> ptCloud2SOLiD(pcl::PointCloud<PointType>::Ptr pt_cloud,
                                                              int NUM_RANGE,
                                                              int NUM_ANGLE,
                                                              int NUM_HEIGHT,
                                                              float FOV_u,
                                                              float FOV_d,
                                                              int MAX_DISTANCE); 
};

#endif //SRC_SOLID_H
