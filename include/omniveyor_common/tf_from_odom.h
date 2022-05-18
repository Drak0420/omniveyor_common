/**
 * @file tf_from_odom.h
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief Helper node that republishes an Odometry (header.frame_id->child_frame) as a
 * TransformStamped message under topic /tf.
 * @version 0.1
 * @date 2022-05-17
 * 
 * @copyright Copyright (c) 2022 Haoguang Yang
 * 
 */
#ifndef _TF_FROM_ODOM_H_
#define _TF_FROM_ODOM_H_
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

class TFfromOdom_node{
    public:

        /**
         * @brief Construct a new TFfromOdom_node node object. This object republishes an Odometry
         * (header.frame_id->child_frame) as a TransformStamped message under topic /tf. If a repub
         * topic (param: odom_topic_repub) is specified, the node republishes the incoming Odometry
         * message at the specified topic as a relay.
         * 
         * @param node The ROS node handle pointer.
         */
        TFfromOdom_node(ros::NodeHandle *node);

        /**
         * @brief Destroy the TFfromOdom_node node object
         * 
         */
        ~TFfromOdom_node() {};

        /**
         * @brief Callback function of the odometry subscriber. Perfroms republishing and TF
         * publishing in-place.
         * 
         * @param msg 
         */
        void odomSubsCb(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * @brief Main (blocking) execution of the republisher loop.
         * 
         */
        void run();

    protected:
        ros::NodeHandle _nh;
        ros::Publisher _odomPub;
        inline static tf2_ros::TransformBroadcaster _br;
        geometry_msgs::TransformStamped _tfMsg;
        nav_msgs::Odometry _odomMsg;
        std::string _odomTopic, _odomRepub;
        double _defaultCovPosLin, _defaultCovPosAng, _defaultCovVelLin, _defaultCovVelAng;
};

#endif