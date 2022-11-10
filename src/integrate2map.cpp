#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <MapRingBuffer.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;

const double PI = 3.1415926;

string afterMapTopic = "aft";
string integrateTopic="integrate_to_init";
int key=0;
int lastKey=0;
MapRingBuffer<nav_msgs::Odometry> afterMapOdomBuff;
MapRingBuffer<nav_msgs::Odometry> imuOdomBuff;

nav_msgs::Odometry odomNowCal;
Eigen::Quaterniond qAfterMap;
Eigen::Vector3d pAfterMap;
Eigen::Vector3d rpyAfterMap;

Eigen::Quaterniond dQImu2Map;
Eigen::Vector3d dRPYImu2Map;
Eigen::Vector3d dPImu2Map;
double runtime;
double lastMapTime;
double initTime;
bool init=false;

void afterMapCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    double time = odom->header.stamp.toSec();
    // cout<<"afterMapCallback"<<endl;
    nav_msgs::Odometry odomIn = *odom;
    afterMapOdomBuff.addMeas(odomIn,time);
    if(!init){
        init = true;
        initTime = time;
    }
    key++;
}

void integrateCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    double time = odom->header.stamp.toSec();
    // cout<<"integrateCallback"<<endl;
    nav_msgs::Odometry odomIn = *odom;
    imuOdomBuff.addMeas(odomIn,time);
}

void push_vector3(Eigen::Vector3d & data,std_msgs::Float32MultiArray& msg){
    msg.data.emplace_back(data[0]);
    msg.data.emplace_back(data[1]);
    msg.data.emplace_back(data[2]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "OdomIntegrate2MapTools");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("afterMapTopic", afterMapTopic);
    nhPrivate.getParam("integrateTopic", integrateTopic);
    afterMapOdomBuff.allocate(50000);
    imuOdomBuff.allocate(50000);

    ros::Subscriber subOdometryMap = nh.subscribe<nav_msgs::Odometry> (afterMapTopic, 5, afterMapCallback);
    ros::Subscriber subOdometryImu = nh.subscribe<nav_msgs::Odometry> (integrateTopic, 500, integrateCallback);
    ros::Publisher pubData = nh.advertise<std_msgs::Float32MultiArray>("data_show",5);
    ros::Rate rate(500);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        if(key>lastKey){
            // ROS_INFO_STREAM("key: "<<key<<" last key:"<<lastKey);
            while(!afterMapOdomBuff.empty()){
                afterMapOdomBuff.getFirstTime(lastMapTime);
                afterMapOdomBuff.getFirstMeas(odomNowCal);
                qAfterMap = Eigen::Quaterniond(odomNowCal.pose.pose.orientation.w,odomNowCal.pose.pose.orientation.x,
                                                                                    odomNowCal.pose.pose.orientation.y,odomNowCal.pose.pose.orientation.z);
                pAfterMap = Eigen::Vector3d(odomNowCal.pose.pose.position.x,odomNowCal.pose.pose.position.y,odomNowCal.pose.pose.position.z);
                rpyAfterMap = qAfterMap.toRotationMatrix().eulerAngles(2,1,0);
                // ROS_INFO_STREAM("lastMapTime: "<<lastMapTime-initTime);
                // ROS_INFO_STREAM("imu size:"<<imuOdomBuff.getSize());
                while(!imuOdomBuff.empty()){
                    nav_msgs::Odometry imuOdom;
                    imuOdomBuff.getFirstMeas(imuOdom);
                    imuOdomBuff.getFirstTime(runtime);
                    if(runtime>lastMapTime+1./5){
                        lastKey++;
                        // 清除对应的data
                        afterMapOdomBuff.clean(lastMapTime);
                        break;
                    }else{
                        // 计算差值
                        // ROS_INFO_STREAM("runtime: "<<runtime-lastMapTime);
                        Eigen::Quaterniond qImu = Eigen::Quaterniond(imuOdom.pose.pose.orientation.w,imuOdom.pose.pose.orientation.x,
                                                                                            imuOdom.pose.pose.orientation.y,imuOdom.pose.pose.orientation.z);
                        Eigen::Vector3d pImu = Eigen::Vector3d(imuOdom.pose.pose.position.x,imuOdom.pose.pose.position.y,imuOdom.pose.pose.position.z);
                        Eigen::Vector3d rpyImu = qImu.toRotationMatrix().eulerAngles(2,1,0);
                        double dAngle = qImu.angularDistance(qAfterMap);
                        dAngle = dAngle * 180/M_PI;
                        dRPYImu2Map = rpyImu - rpyAfterMap;
                        dPImu2Map = pImu - pAfterMap;
                        dRPYImu2Map =  dRPYImu2Map * 180/M_PI;
                        // publish
                        // ROS_INFO_STREAM("dPImu2Map: "<<dPImu2Map);
                        std_msgs::Float32MultiArray publishData;
                        publishData.data.emplace_back(runtime-initTime);
                        push_vector3(dRPYImu2Map, publishData);
                        push_vector3(dPImu2Map, publishData);
                        publishData.data.emplace_back(dAngle);
                        publishData.data.emplace_back(lastMapTime-initTime);
                        pubData.publish(publishData);
                        // 清除对应的data
                        imuOdomBuff.clean(runtime);
                        ros::Duration(0.001).sleep();
                    }
                }
                if(afterMapOdomBuff.getSize()<=1){
                    break;
                }
            }
        }
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
