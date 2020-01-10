// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

#define DISTORTION 1


int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLastun(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLastun(new pcl::PointCloud<PointType>());


pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
//pcl::PointCloud<PointType>::Ptr laserCloudFullResekf(new pcl::PointCloud<PointType>());
//pcl::PointCloud<PointType>::Ptr laserCloudFullResekf2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullResnormal(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr pubtest2(new pcl::PointCloud<PointType>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};
double para_q_old[4] = {0, 0, 0, 1};
double para_t_old[3] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);
Eigen::Map<Eigen::Quaterniond> q_last_curr_old(para_q_old);
Eigen::Map<Eigen::Vector3d> t_last_curr_old(para_t_old);

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex mBuf;
std::vector<nav_msgs::Odometry> ekfposelog;
//std::vector<nav_msgs::Odometry> realekfposelog;

nav_msgs::Path ekfposepath;
double lastscantime=0;
bool MODEEKF=false;
Eigen::Quaterniond q_scan_start({1, 0, 0, 0});
Eigen::Vector3d t_scan_start({0, 0, 0});
Eigen::Quaterniond q_scan_end({1, 0, 0, 0});
Eigen::Vector3d t_scan_end({0, 0, 0});
//Eigen::Vector3d t_ekf_start({0, 0, 0});
//Eigen::Vector3d t_ekf_end({0, 0, 0});

// undistort lidar point
void TransformToStart(PointType const *const pi, PointType* const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    
    
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}
void TransformToStartun(PointType const *const pi, PointType* const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    s=1.0;
    
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}


// transform all lidar points to the start of the next frame

void TransformToEnd(PointType const *const pi, PointType* const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);
    //Eigen::Vector3d point_end = q_last_curr.conjugate() * (un_point - t_last_curr);
    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}
void TransformToEndun(PointType const *const pi, PointType* const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStartun(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);
    //Eigen::Vector3d point_end = q_last_curr.conjugate() * (un_point - t_last_curr);
    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}

double startt=0;

void TransformToStartekf(PointType const *const pi, PointType* const po)
{	mBuf.lock();
	if (!ekfposelog.empty()) { 

	double t =pi->intensity - int(pi->intensity);
	
	double T=lastscantime+t; // time of point


	int N =ekfposelog.size();
	//Eigen::Quaterniond q_scan_start;
	//Eigen::Vector3d t_scan_start;
	Eigen::Quaterniond q_point_last;
	Eigen::Vector3d t_point_last;
	//should be made into function
	//findqtpose(q_scan_start,t_scan_start,lastscantime);



	for (int i=0;i<N-1;i++){
		nav_msgs::Odometry pose2=ekfposelog.at(N-i-1); //newest pose
		nav_msgs::Odometry pose1=ekfposelog.at(N-i-2); //oldest pose
		double time1=pose1.header.stamp.toSec(); //time of pose
		double time2=pose2.header.stamp.toSec(); //time of pose




		if((T<time2 && T>=time1)||(T>time2 && T>time1 && time1!=time2)){ //in between these two poses



			Eigen::Quaterniond q1(pose1.pose.pose.orientation.w, pose1.pose.pose.orientation.x, pose1.pose.pose.orientation.y, pose1.pose.pose.orientation.z);
			Eigen::Vector3d t1(pose1.pose.pose.position.x, pose1.pose.pose.position.y, pose1.pose.pose.position.z);
			Eigen::Quaterniond q2(pose2.pose.pose.orientation.w, pose2.pose.pose.orientation.x, pose2.pose.pose.orientation.y, pose2.pose.pose.orientation.z);
			Eigen::Vector3d t2(pose2.pose.pose.position.x, pose2.pose.pose.position.y, pose2.pose.pose.position.z);



			double s= (T-time1)/(time2-time1); //interpolation factor for slerp
		
			q_point_last = q1.slerp(s, q2);
			t_point_last = s*(t2-t1)+t1; //korrekt so far

			//std::cout<<"TWO FOUND"<<std::endl;

			break;
		}


	}
	mBuf.unlock();


	q_point_last=q_point_last * q_scan_start.inverse();

	double S = t / SCAN_PERIOD; //use normal translation estimate
	t_point_last=S * t_last_curr;
	//t_point_last=S*(t_ekf_end-t_ekf_start);
	Eigen::Vector3d point(pi->x, pi->y, pi->z);
	Eigen::Vector3d un_point = q_point_last * point + t_point_last;
	if(startt==0){
		startt=T;
	}
	po->x = un_point.x();//+0.1*(T-startt);
	po->y = un_point.y();
	po->z = un_point.z();//+0.1*(T-startt);
	po->intensity = pi->intensity;




}else{
	po->x = pi->x;
	po->y = pi->y;
	po->z = pi->z;
	po->intensity = pi->intensity;
}
	
}


void findqtpose(Eigen::Quaterniond& q_scan, Eigen::Vector3d& t_scan,double scantime){
	int N =ekfposelog.size();
	for (int i=0;i<N-1;i++){ //find last scan pose /should only be done once for each scan
		nav_msgs::Odometry pose2=ekfposelog.at(N-i-1); //newest pose
		nav_msgs::Odometry pose1=ekfposelog.at(N-i-2); //oldest pose
		double time1=pose1.header.stamp.toSec(); //time of pose
		double time2=pose2.header.stamp.toSec(); //time of pose


		if((scantime<time2 && scantime>=time1)||(scantime>time2 && scantime>=time1 && time1!=time2)){ //in between these two poses
			Eigen::Quaterniond q1(pose1.pose.pose.orientation.w, pose1.pose.pose.orientation.x, pose1.pose.pose.orientation.y, pose1.pose.pose.orientation.z);
			Eigen::Vector3d t1(pose1.pose.pose.position.x, pose1.pose.pose.position.y, pose1.pose.pose.position.z);
			Eigen::Quaterniond q2(pose2.pose.pose.orientation.w, pose2.pose.pose.orientation.x, pose2.pose.pose.orientation.y, pose2.pose.pose.orientation.z);
			Eigen::Vector3d t2(pose2.pose.pose.position.x, pose2.pose.pose.position.y, pose2.pose.pose.position.z);



			double s= (scantime-time1)/(time2-time1); //interpolation factor for slerp
			
			q_scan = q1.slerp(s, q2);
			t_scan = s*(t2-t1)+t1;
			std::cout<<"ONE FOUND"<<std::endl;
			break;

		}
	}

}
/*
void findekfpose( Eigen::Vector3d& t_scan,double scantime){
	int N =realekfposelog.size();
	for (int i=0;i<N-1;i++){ //find last scan pose /should only be done once for each scan
		nav_msgs::Odometry pose2=realekfposelog.at(N-i-1); //newest pose
		nav_msgs::Odometry pose1=realekfposelog.at(N-i-2); //oldest pose
		double time1=pose1.header.stamp.toSec(); //time of pose
		double time2=pose2.header.stamp.toSec(); //time of pose


		if((scantime<time2 && scantime>=time1)||(scantime>time2 && scantime>=time1 && time1!=time2)){ //in between these two poses
			
			Eigen::Vector3d t1(pose1.pose.pose.position.x, pose1.pose.pose.position.y, pose1.pose.pose.position.z);
			
			Eigen::Vector3d t2(pose2.pose.pose.position.x, pose2.pose.pose.position.y, pose2.pose.pose.position.z);



			double s= (scantime-time1)/(time2-time1); //interpolation factor for slerp
			
			
			t_scan = s*(t2-t1)+t1;
			//std::cout<<"ONE FOUND"<<std::endl;
			break;

		}
	}

}
*/
void TransformToEndekf(PointType const *const pi, PointType* const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStartekf(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Quaterniond q_start_end=q_scan_end*q_scan_start.inverse();
	Eigen::Vector3d t_start_end=q_scan_start.inverse()*(t_scan_end-t_scan_start);

    Eigen::Vector3d point_end = q_start_end.inverse() * (un_point - t_start_end);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}
/*
void ekfposehandler(const nav_msgs::Odometry::ConstPtr & ekfPose)
{       mBuf.lock();
		  
			nav_msgs::Odometry yo;
			yo.header = ekfPose->header;
			yo.pose = ekfPose->pose;
			yo.twist=ekfPose->twist;
			yo.child_frame_id=ekfPose->child_frame_id;
			//ekfposelog.push_back(yo);
			mBuf.unlock();
}
*/
double oldt =0;
double angle =0;
Eigen::Quaterniond qo_f2=Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0.78539816339, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-1.57079632679, Eigen::Vector3d::UnitZ());
double imucounter=0;
void imuposehandler(const sensor_msgs::Imu::ConstPtr & msg)
{       
			double imudownsamplefactor =1;
			imucounter++;
			if( imucounter==imudownsamplefactor){
			imucounter=0;
            double t=msg->header.stamp.sec + 0.000000001*msg->header.stamp.nsec;
            
            if (oldt==0){
                oldt=t;
            }
            double dt =t-oldt;
            //std::cout<<dt<<std::endl;
            oldt=t;
            angle+=msg->angular_velocity.y*dt;
   
            Eigen::Quaterniond q_f1 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
            
            Eigen::Quaterniond out=qo_f2*q_f1*qo_f2.inverse();

            nav_msgs::Odometry odomAftMapped;
            odomAftMapped.header.frame_id = "camera_init";
            odomAftMapped.child_frame_id = "aft_mapped";
            odomAftMapped.header.stamp = msg->header.stamp;
            odomAftMapped.pose.pose.orientation.x = out.x();
            odomAftMapped.pose.pose.orientation.y = out.y();
            odomAftMapped.pose.pose.orientation.z = out.z();
            odomAftMapped.pose.pose.orientation.w = out.w();
            odomAftMapped.pose.pose.position.x = 0;
            odomAftMapped.pose.pose.position.y = 0;
            odomAftMapped.pose.pose.position.z = 0;

            mBuf.lock();
			ekfposelog.push_back(odomAftMapped);
			mBuf.unlock();
		}

}
/*
void realekfposehandler(const nav_msgs::Odometry::ConstPtr & ekfPose)
{       mBuf.lock();
		  
			nav_msgs::Odometry yo;
			yo.header = ekfPose->header;
			yo.pose = ekfPose->pose;
			yo.twist=ekfPose->twist;
			yo.child_frame_id=ekfPose->child_frame_id;



			realekfposelog.push_back(yo);
			mBuf.unlock();
}
*/


void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);

    mBuf.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);

    printf("Mapping %d Hz \n", 10 / skipFrameNum);

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);

    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);

    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);

    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);

    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
    //ros::Publisher pubLaserCloudFullResekf = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_ekf", 100);
    //ros::Publisher pubLaserCloudFullResekf2 = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_ekf2", 100);
    ros::Publisher pubLaserCloudFullResnormal = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_normal", 100);
    ros::Publisher pubLaserCloudFullResnormal2 = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_normal2", 100);
    ros::Publisher pubtest = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud", 100);
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    ros::Publisher pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry>("/laser_test", 100);

    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
    //ros::Subscriber subekfpose = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered/imu", 100, ekfposehandler);
    //ros::Subscriber subrealekfpose = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 100, realekfposehandler);
   	ros::Subscriber subimupose = nh.subscribe<sensor_msgs::Imu>("/imu0", 100, imuposehandler);
    nav_msgs::Path laserPath;

    int frameCount = 0;
    ros::Rate rate(100);

    while (ros::ok())
    {
        ros::spinOnce();

        
        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        {
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                printf("unsync messeage!");
                ROS_BREAK();
            }

            mBuf.lock();
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullRes->clear();
            pubtest2->clear();
            //laserCloudFullResekf->clear();
            //laserCloudFullResekf2->clear();
            laserCloudFullResnormal->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            pcl::fromROSMsg(*fullPointsBuf.front(), *pubtest2);
            //pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullResekf);
            //pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullResekf2);
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullResnormal);
            lastscantime=fullPointsBuf.front()->header.stamp.toSec()-0.1; //minus 0.1 because the time is at the end of the scan of 0.1 sec, maybe change so its the actually old one
            fullPointsBuf.pop();
            mBuf.unlock();

            if (ekfposelog.size()>2) { 
				MODEEKF=true;

				
			}else{
				std::cout<<ekfposelog.size()<<std::endl;
				//continue;
			}
			MODEEKF=true;

			if (MODEEKF){

                
	                findqtpose(q_scan_start,t_scan_start,lastscantime);
					findqtpose(q_scan_end,t_scan_end,lastscantime+0.1);
					//findekfpose(t_ekf_start,lastscantime);
					//findekfpose(t_ekf_end,lastscantime);
					q_last_curr=q_scan_end * q_scan_start.inverse(); //used for ceres initial guess //was used before dec5
					std::cout<<"guess: "<<t_last_curr<<std::endl;
					//int laserCloudFullResNum = laserCloudFullRes->points.size();
	                //for (int i = 0; i < laserCloudFullResNum; i++)
	                //{   
	                	//printf("%d",i);
	                	//TransformToStartekf(&laserCloudFullResekf->points[i], &laserCloudFullResekf->points[i]);
	                	//TransformToStart(&laserCloudFullResnormal->points[i], &laserCloudFullResnormal->points[i]);

	                //}


			}


			
            TicToc t_whole;
            // initializing
            if (!systemInited)
            {
                systemInited = true;
                std::cout << "Initialization finished \n";
            }
            else
            {
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                TicToc t_opt;
                for (size_t opti_counter = 0; opti_counter <1; ++opti_counter)
                {   //std::cout<<opti_counter<<std::endl;
                    corner_correspondence = 0;
                    plane_correspondence = 0;

                    //ceres::LossFunction *loss_function = NULL;
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::LocalParameterization *q_parameterization =
                        new ceres::EigenQuaternionParameterization();
                    ceres::Problem::Options problem_options;

                    ceres::Problem problem(problem_options);
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    problem.AddParameterBlock(para_t, 3);

                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

                    TicToc t_data;
                    // find correspondence for corner features


                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    {   
                    	if (MODEEKF) { 
							TransformToStartekf(&(cornerPointsSharp->points[i]), &pointSel);

						}else{
							if (opti_counter==0){

							TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);

							}
							else if(opti_counter==1){
								TransformToStartun(&(cornerPointsSharp->points[i]), &pointSel);
							}
						}

                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }
                        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                        {  	if (MODEEKF){
                            TransformToEndekf(&cornerPointsSharp->points[i], &cornerPointsSharp->points[i]);

                        	}
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);

                            double s;
                            if (MODEEKF){

								s=1;
								ceres::CostFunction *cost_function = LidarEdgeFactorekf::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;
							}
							else{
								if (DISTORTION)
								s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
								else
								s = 1.0;
							if (opti_counter==1){s=1;}

							ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;

							}
                            
                        }
                    }

                    // find correspondence for plane features
                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    {   if (MODEEKF) {
							TransformToStartekf(&(surfPointsFlat->points[i]), &pointSel);



						}else{
						if (opti_counter==0){
						
				
						TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
					}else if(opti_counter==1){

						TransformToStartun(&(surfPointsFlat->points[i]), &pointSel);
					}

						} 
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;
                                
                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            if (minPointInd2 >= 0 && minPointInd3 >= 0)
                            {	if (MODEEKF){
                                TransformToEndekf(&surfPointsFlat->points[i], &surfPointsFlat->points[i]);
                                
                            	}
                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);

                                double s;
                                if (MODEEKF){

								s=1;
								ceres::CostFunction *cost_function = LidarPlaneFactorekf::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;
							}else{
								if (DISTORTION)
								s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
							else
								s = 1.0;
							if (opti_counter==1){s=1;}
							ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;

							}
                                
                            }
                        }
                    }

                    printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());

                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        printf("less correspondence! *************************************************\n");
                    }

                    TicToc t_solver;
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.max_num_iterations = 10;
                    options.minimizer_progress_to_stdout = false;
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);
                    std::cout<<"solved: "<<t_last_curr<<std::endl;
                    printf("solver time %f ms \n", t_solver.toc());
                   //printf("Antal correspondence: %d",corner_correspondence + plane_correspondence);
                }
                printf("optimization twice time %f \n", t_opt.toc());
               
                

                std::cout<<"before: "<<t_w_curr<<std::endl;

                t_w_curr = t_w_curr + q_scan_start * t_last_curr;
                q_w_curr = q_scan_start * q_last_curr;
                std::cout<<"After: "<<t_w_curr<<std::endl;
				//std::cout << "Debug: "<< q_scan_start.vec() << std::endl;
                //t_w_curr = t_w_curr + q_w_curr * t_last_curr;
				//q_w_curr = q_w_curr * q_last_curr;
                std::cout<<"t_w_curr: "<<t_w_curr<<std::endl;

                std::cout<<"q_w_curr: "<<q_w_curr.vec()<<std::endl;
                std::cout<<"q_scan_end: "<<q_scan_end.vec()<<std::endl;
               
                t_last_curr_old=t_last_curr;
                q_last_curr_old=q_last_curr;



            }

            TicToc t_pub;

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "/camera_init";
            laserOdometry.child_frame_id = "/laser_odom";
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);


            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "/camera_init";
            pubLaserPath.publish(laserPath);

            // transform corner features and plane features to the scan end point

           // laserCloudSurfLast=surfPointsLessFlat;
           // laserCloudCornerLast=cornerPointsLessSharp;
            if(MODEEKF){

            	t_scan_end=t_w_curr;//q_scan_start*t_last_curr+t_scan_start;
				q_scan_end=q_w_curr;//q_scan_start*q_last_curr;



	            if (1)
	            {
	                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
	                for (int i = 0; i < cornerPointsLessSharpNum; i++)
	                {
	                    TransformToEndekf(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
	                }

	                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
	                for (int i = 0; i < surfPointsLessFlatNum; i++)
	                {
	                    TransformToEndekf(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
	                }

	                int laserCloudFullResNum = laserCloudFullRes->points.size();
	                
	                for (int i = 0; i < laserCloudFullResNum; i++)
	                {   TransformToEndekf(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
	                	//TransformToStartekf(&laserCloudFullRes->points[i], &laserCloudFullResekf2->points[i]);
	                }
	            }
	        }else{
	        	if (1)
	            {
	                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
	                for (int i = 0; i < cornerPointsLessSharpNum; i++)
	                {
	                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
	                }

	                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
	                for (int i = 0; i < surfPointsLessFlatNum; i++)
	                {
	                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
	                }

	                int laserCloudFullResNum = laserCloudFullRes->points.size();
	                
	                for (int i = 0; i < laserCloudFullResNum; i++)
	                {   TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);

	                }
	            }
	        }

            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

            if (frameCount % skipFrameNum == 0)
            {
                frameCount = 0;

                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera_init";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);

                sensor_msgs::PointCloud2 pubtest3;
                pcl::toROSMsg(*pubtest2, pubtest3);
                pubtest3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                pubtest3.header.frame_id = "/camera_init";
                pubtest.publish(pubtest3);
                
                /*
                sensor_msgs::PointCloud2 pubtestekf;
                pcl::toROSMsg(*laserCloudFullResekf, pubtestekf);
                pubtestekf.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                pubtestekf.header.frame_id = "/camera_init";
                pubLaserCloudFullResekf.publish(pubtestekf);


                sensor_msgs::PointCloud2 pubtestekf2;
                pcl::toROSMsg(*laserCloudFullResekf2, pubtestekf2);
                pubtestekf2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                pubtestekf2.header.frame_id = "/camera_init";
                pubLaserCloudFullResekf2.publish(pubtestekf2);
				*/
                sensor_msgs::PointCloud2 pubtestnormal;
                pcl::toROSMsg(*laserCloudFullResnormal, pubtestnormal);
                pubtestnormal.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                pubtestnormal.header.frame_id = "/camera_init";
                pubLaserCloudFullResnormal.publish(pubtestnormal);

	                // publish odometry
	            nav_msgs::Odometry laserOdometry2;
	            laserOdometry2.header.frame_id = "/camera_init";
	            laserOdometry2.child_frame_id = "/laser_odom";
	            laserOdometry2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
	            laserOdometry2.pose.pose.orientation.x = q_last_curr.x();
	            laserOdometry2.pose.pose.orientation.y = q_last_curr.y();
	            laserOdometry2.pose.pose.orientation.z = q_last_curr.z();
	            laserOdometry2.pose.pose.orientation.w = q_last_curr.w();
	            laserOdometry2.pose.pose.position.x = t_last_curr.x();
	            laserOdometry2.pose.pose.position.y = t_last_curr.y();
	            laserOdometry2.pose.pose.position.z = t_last_curr.z();
	            pubLaserOdometry2.publish(laserOdometry2);




            }
            printf("publication time %f ms \n", t_pub.toc());
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");

            frameCount++;
        }
        rate.sleep();
    }
    return 0;
}