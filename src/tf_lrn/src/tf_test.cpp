#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <cmath>
#include <string>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>


enum State{
    s1,
    s2,
    s3,
    s4
};

using namespace std;

class Bocha
{
    int m_count;
    int m_state;
    double m_y;
    double m_th;
    double m_dv;
    double m_dvth;
    double m_dt;

    bool m_path_flag;
    double m_min_y;
    double m_max_y;
    string m_frame_id;
    string m_father_frame_id;
    string m_father_father_frame_id;
    tf::TransformBroadcaster m_br;


public:
    Bocha(int num, ros::NodeHandle& nh)
    {
        std::stringstream ssTemp;
        ssTemp << num;
        std::string str = ssTemp.str();

        m_count = 1;
        m_state = s2;
        m_y = -0.5;
        m_th = -M_PI / 6;
        m_dv = M_PI/300;
        m_dvth = M_PI/300;
        m_dt = 1;

        m_path_flag = true;
        m_update_flag = false;
        m_min_y = -2.5;
        m_max_y = -0.5;

        m_frame_id = "l_bocha_" + str;
        m_father_frame_id = "l_bocha_help_" + str;
        m_father_father_frame_id = "odom";

        m_path_pub = nh.advertise<nav_msgs::Path>("l_path_" + str,  1, true);
    }

    Bocha(int num, ros::NodeHandle& nh, bool right)
    {
        std::stringstream ssTemp;
        ssTemp << num;
        std::string str = ssTemp.str();

        m_count = 2;
        m_state = s4;
        m_y = 0.5;
        m_th = -(M_PI * 2) + M_PI / 6;
        m_dv = M_PI/300;
        m_dvth = M_PI/300;
        m_dt = 1;

        m_path_flag = true;
        m_update_flag = false;
        m_min_y = 0.5;
        m_max_y = 2.5;

        m_frame_id = "r_bocha_" + str;;
        m_father_frame_id = "r_bocha_help_" + str;
        m_father_father_frame_id = "odom";

        m_path_pub = nh.advertise<nav_msgs::Path>("r_path_" + str,  1, true);
    }

    void update();

    bool m_update_flag;
    nav_msgs::Path m_path;
    ros::Publisher m_path_pub;
};

void Bocha::update()
{
    if(m_state == s1)
    {
        m_th = m_count * (-M_PI);
        m_y += m_dv * m_dt;
        if(fabs(m_y - m_max_y) < 0.01)
        {
            m_count++;
            m_state = s2;
            ROS_INFO_STREAM( "state from s1 to s2" );
        }
    }
    else if(m_state == s2)
    {
        m_th -= m_dvth * m_dt;
        if(fabs((m_th / M_PI) + m_count) < 0.001 )
        {
            m_state = s3;
            ROS_INFO_STREAM( "state from s2 to s3" );
        }
    }
    else if(m_state == s3)
    {
        m_th = m_count * (-M_PI);
        m_y -= m_dv * m_dt;
        if(fabs(m_y - m_min_y)  < 0.01)
        {
            m_count++;
            m_state = s4;
            ROS_INFO_STREAM( "state from s3 to s4" );
        }
    }
    else if(m_state == s4)
    {
        m_th -= m_dvth * m_dt;
        if(fabs(m_th / M_PI + m_count) < 0.001)
        {
            m_state = s1;
            ROS_INFO_STREAM( "state from s4 to s1" );
        }
    }

    //cout << m_y << "   " << m_th << "   "  << m_count << endl;

    Eigen::Quaterniond q;
    tf::Transform transform, transform2;
    geometry_msgs::TransformStamped tf_msgs, tf_msgs2;

    Eigen::AngleAxisd rotationV(0 ,Eigen::Vector3d(1, 0, 0));               // bocha 相对于bocha_help 的tf
    q = Eigen::Quaterniond(rotationV);
    transform2.setOrigin(tf::Vector3(0, 0, 1));
    transform2.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

    Eigen::AngleAxisd rotationV2(m_th ,Eigen::Vector3d(1, 0, 0));     // bocha_help 相对于odom 的tf
    q = Eigen::Quaterniond(rotationV2);
    transform.setOrigin(tf::Vector3(0.0, m_y, 0.0));
    transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

    tf::StampedTransform tf_odom_base(transform, ros::Time::now(), m_father_father_frame_id, m_father_frame_id);
    tf::transformStampedTFToMsg(tf_odom_base, tf_msgs);
    m_br.sendTransform(tf_msgs);

    tf::StampedTransform tf_base_bocha(transform2, ros::Time::now(), m_father_frame_id, m_frame_id);
    tf::transformStampedTFToMsg(tf_base_bocha, tf_msgs2);
    m_br.sendTransform(tf_msgs2);

    if(m_path_flag)
    {
        m_path.header.stamp = ros::Time::now();
        m_path.header.frame_id = m_father_father_frame_id;
        geometry_msgs::PoseStamped this_pose_stamped;

        this_pose_stamped.pose.position.y = m_y + sin(fabs(m_th));
        this_pose_stamped.pose.position.z = cos(m_th);

        this_pose_stamped.pose.orientation.x = tf_msgs.transform.rotation.x;
        this_pose_stamped.pose.orientation.y = tf_msgs.transform.rotation.y;
        this_pose_stamped.pose.orientation.z = tf_msgs.transform.rotation.z;
        this_pose_stamped.pose.orientation.w = tf_msgs.transform.rotation.w;

        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.header.frame_id = m_father_father_frame_id;

        m_path.poses.push_back(this_pose_stamped);

    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_test");
    ros::NodeHandle node;

    int num = 0;
    int count = 100;
    bool init_flag = false;

    vector<Bocha> l_vBochas;
    for(int i = 1; i <= 5; i++)
    {
        Bocha bocha(i, node);
        l_vBochas.push_back(bocha);
    }

    vector<Bocha> r_vBochas;
    for(int i = 1; i <= 5; i++)
    {
        Bocha bocha(i, node, true);
        r_vBochas.push_back(bocha);
    }

    ros::Rate rate(10);

    while (node.ok())
    {

        if(!init_flag)
        {
            if(count == 100)
            {
                count = -1;
                if(num == 4)
                {
                    init_flag = true;
                }
                l_vBochas.at(num).m_update_flag = true;
                r_vBochas.at(num).m_update_flag = true;
                num++;
            }
        }

        for(size_t i = 0; i < 5; i++)
        {
            if(l_vBochas.at(i).m_update_flag)
            {
                l_vBochas.at(i).update();
                l_vBochas.at(i).m_path_pub.publish(l_vBochas.at(i).m_path);
            }
            if(r_vBochas.at(i).m_update_flag)
            {
                r_vBochas.at(i).update();
                r_vBochas.at(i).m_path_pub.publish(r_vBochas.at(i).m_path);
            }
        }
        rate.sleep();
        count++;
    }

    return 0;
}

