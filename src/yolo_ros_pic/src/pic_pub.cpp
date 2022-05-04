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
#include<iostream>
#include<image_transport/image_transport.h> 
#include "../include/yolo_v2_class.hpp"

using namespace std;
using namespace cv;

string test_pic_add_File = "/home/heidai/ws/src/yolo_ros_pic/yolo/test_pic_add";
string classesFile = "/home/heidai/ws/src/yolo_ros_pic/yolo/coco.names";
string modelConfig = "/home/heidai/ws/src/yolo_ros_pic/yolo/yolov4.cfg";
string modelWeights = "/home/heidai/ws/src/yolo_ros_pic/yolo/yolov4.weights";

//画出检测框和相关信息
void DrawBoxes(Mat &frame, vector<string> classes, int classId, float conf, int left, int top, int right, int bottom)
{
	//画检测框
	rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);
	//该检测框对应的类别和置信度
	string label = format("%.2f", conf);
	if (!classes.empty())
	{
		CV_Assert(classId < (int)classes.size());
		label = classes[classId] + ":" + label;
	}
	//将标签显示在检测框顶部
	int baseLine;
	Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, labelSize.height);
	rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
	putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 1);
}


//画出检测结果
void Drawer(Mat &frame, vector<bbox_t> outs, vector<string> classes)
{
	//获取所有最佳检测框信息
	for (int i = 0; i < outs.size(); i++)
	{
		DrawBoxes(frame, classes, outs[i].obj_id, outs[i].prob, outs[i].x, outs[i].y,
			outs[i].x + outs[i].w, outs[i].y + outs[i].h);
	}
}


int main(int argc, char **argv)
{
	// ROS节点初始化,最后一个参数为节点名称，不可以重复
	ros::init(argc, argv, "pic_pub");
	//加载测试图片的地址
	vector<string> test_pic_add;
	ifstream ifs1(test_pic_add_File.c_str());
	string line1;
	while (getline(ifs1, line1)) test_pic_add.push_back(line1);
	cout<<test_pic_add[0]<<endl;
	//加载类别名
	vector<string> classes;
	ifstream ifs2(classesFile.c_str());
	string line2;
	while (getline(ifs2, line2)) classes.push_back(line2);
	//加载网络模型
	Detector detector(modelConfig, modelWeights, 0);

    	// 创建节点句柄变量 n
    	ros::NodeHandle n;

    	// 创建一个Publisher变量为turtle_vel_pub，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10，
    	//sensor_msgs::ImagePtr msg;
	image_transport::ImageTransport it(n);
	image_transport::Publisher pub = it.advertise("/ros_yolo_pic/result_pic", 10);
	sensor_msgs::ImagePtr msg;

    	// ros::Rate设置发布频率，loop_rate()是设置循环的频率
	//ros::Rate loop_rate(5);
	
    	int count_num = 0;
	int size = test_pic_add.size();
    while (ros::ok())
    {
        // 初始化geometry_msgs::Twist类型的消息，定义消息变量 vel_msg
        Mat img = imread(test_pic_add[count_num],IMREAD_COLOR);
	shared_ptr<image_t> detImg = detector.mat_to_image_resize(img);
	vector<bbox_t> outs = detector.detect_resized(*detImg, img.cols, img.rows, 0.25);
	Drawer(img, outs, classes);
	//imwrite(result_pic_add[count_num], img);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	pub.publish(msg);
        // 发布消息,需要通过前面创建的publisher变量中的publish函数来进行发布
        //turtle_vel_pub.publish(vel_msg);
        // 类似于printf，讲所执行的内容打印出来
	string temp = test_pic_add[count_num];
        ROS_INFO("%s after yolov4 has been published!",temp.c_str());
	++count_num;
	if(count_num>=size)break;
        // 按照所设置的循环频率延时，前面设置为10，也就是1/10秒
        //loop_rate.sleep();
    }

    return 0;
}
