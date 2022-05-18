/**
 * @file tf_from_odom.cpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief Helper node that republishes an Odometry (header.frame_id->child_frame) as a
 * TransformStamped message under topic /tf.
 * @version 0.1
 * @date 2022-05-17
 * 
 * @copyright Copyright (c) 2022 Haoguang Yang
 * 
 */

#include "omniveyor_common/tf_from_odom.h"

TFfromOdom_node::TFfromOdom_node(ros::NodeHandle *node): _nh(*node){
    _nh.param<std::string>("odom_topic", _odomTopic, "odom");
    _nh.param<std::string>("odom_topic_repub", _odomRepub, "");
    _nh.param<double>("default_linear_pose_covariance", _defaultCovPosLin, 0.05);
    _nh.param<double>("default_angular_pose_covariance", _defaultCovPosAng, 0.05);
    _nh.param<double>("default_linear_vel_covariance", _defaultCovVelLin, 0.1);
    _nh.param<double>("default_angular_vel_covariance", _defaultCovVelAng, 0.1);
    
    if (!_odomRepub.empty()){
        _odomPub = _nh.advertise<nav_msgs::Odometry>(_odomRepub, 2);
    }

    ros::Subscriber sub = _nh.subscribe(_odomTopic, 10, &TFfromOdom_node::odomSubsCb, this);
}

void TFfromOdom_node::odomSubsCb(const nav_msgs::Odometry::ConstPtr& msg){
    _tfMsg.header = msg->header;
    _tfMsg.child_frame_id = msg->child_frame_id;
    _tfMsg.transform.translation.x = msg->pose.pose.position.x;
    _tfMsg.transform.translation.y = msg->pose.pose.position.y;
    _tfMsg.transform.translation.z = msg->pose.pose.position.z;
    _tfMsg.transform.rotation.x = msg->pose.pose.orientation.x;
    _tfMsg.transform.rotation.y = msg->pose.pose.orientation.y;
    _tfMsg.transform.rotation.z = msg->pose.pose.orientation.z;
    _tfMsg.transform.rotation.w = msg->pose.pose.orientation.w;

    _br.sendTransform(_tfMsg);
    if (_odomRepub.empty())
        return;

    if (msg->pose.covariance[0] > 0. && msg->twist.covariance[0] > 0.)
        _odomPub.publish(msg);
    else
    {
        // Need to rectify the covariances
        _odomMsg = *msg;
        if (_odomMsg.pose.covariance[0] <= 0.)
            _odomMsg.pose.covariance[0] = _defaultCovPosLin;
        if (_odomMsg.pose.covariance[7] <= 0.)
            _odomMsg.pose.covariance[7] = _defaultCovPosLin;
        if (_odomMsg.pose.covariance[14] <= 0.)
            _odomMsg.pose.covariance[14] = _defaultCovPosLin;
        if (_odomMsg.pose.covariance[21] <= 0.)
            _odomMsg.pose.covariance[21] = _defaultCovPosAng;
        if (_odomMsg.pose.covariance[28] <= 0.)
            _odomMsg.pose.covariance[28] = _defaultCovPosAng;
        if (_odomMsg.pose.covariance[35] <= 0.)
            _odomMsg.pose.covariance[35] = _defaultCovPosAng;
        if (_odomMsg.twist.covariance[0] <= 0.)
            _odomMsg.twist.covariance[0] = _defaultCovVelLin;
        if (_odomMsg.twist.covariance[7] <= 0.)
            _odomMsg.twist.covariance[7] = _defaultCovVelLin;
        if (_odomMsg.twist.covariance[14] <= 0.)
            _odomMsg.twist.covariance[14] = _defaultCovVelLin;
        if (_odomMsg.twist.covariance[21] <= 0.)
            _odomMsg.twist.covariance[21] = _defaultCovVelAng;
        if (_odomMsg.twist.covariance[28] <= 0.)
            _odomMsg.twist.covariance[28] = _defaultCovVelAng;
        if (_odomMsg.twist.covariance[35] <= 0.)
            _odomMsg.twist.covariance[35] = _defaultCovVelAng;
        _odomPub.publish(_odomMsg);
    }
}

void TFfromOdom_node::run(){
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_from_odom");

    ros::NodeHandle node;
    TFfromOdom_node repub(&node);
    //std::cout << "initialized" << std::endl;
    repub.run();
} 
