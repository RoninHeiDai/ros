#include <ros/ros.h>
#include<string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <fstream>
#include <sys/time.h>
#include<sensor_msgs/image_encodings.h>
#include<std_msgs/Header.h>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h> 
#include<iostream>


using namespace std;
using namespace cv;

string result_pic_add_File = "/home/heidai/ws/src/yolo_ros_pic/yolo/result_pic_add";
int count_num = 0;
vector<string> result_pic_add;

/*
// 接收到订阅的消息后，会进入消息回调函数
//turtlesim::Pose::ConstPtr&是/turtle1/pose topic的message类型，此处是一个常指针
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Turtle pose: x:%0.6f, y:%0.6f", msg->x, msg->y);
}*/

void picCallback(const sensor_msgs::ImageConstPtr &img)
{
	ROS_INFO("ok\n");
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, 			  		sensor_msgs::image_encodings::BGR8);
	Mat msg = cv_ptr -> image;	
	imshow("image", msg);
	waitKey(100);
    	//destroyAllWindows();
    	imwrite(result_pic_add[count_num],msg);
	++count_num;
}

int main(int argc, char **argv)
{
	//加载结果图片的地址
	
	ifstream ifs(result_pic_add_File.c_str());
	string line;
	while (getline(ifs, line)) result_pic_add.push_back(line);
    	// 初始化ROS节点，创建ros节点pose_subscriber
    	ros::init(argc, argv, "pic_sub");
    	// 创建节点句柄n
    	ros::NodeHandle n;
    	// 创建一个Subscriber，订阅名为/turtle1/pose的topic，10为队列长度， 注册回调函数poseCallback  
    	// 回调函数是当和topic相关的message输入时，返回回调函数进行处理
    	ros::Subscriber pic_sub = n.subscribe("/ros_yolo_pic/result_pic", 10, picCallback);
    	// 循环等待回调函数，会不断查看队列，如果有就会调用callback，否则一直循环
    	ros::spin();

    return 0;
}
