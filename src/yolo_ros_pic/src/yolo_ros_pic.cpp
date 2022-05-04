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

#include "../include/yolo_v2_class.hpp"

using namespace std;
using namespace cv;

bool e=true;
static ros::Subscriber image_sub;
static ros::Publisher detect_pub;

string classesFile = "/home/heidai/ws/src/yolo_ros_pic/cfg/coco.names";
string modelConfig = "/home/heidai/ws/src/yolo_ros_pic/cfg/yolo-obj.cfg";
string modelWeights = "/home/heidai/ws/src/yolo_ros_pic/cfg/yolov4.weights";
//加载网络模型，0是指定第一块GPU
Detector detector(modelConfig, modelWeights, 0);


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


void image_callback(const sensor_msgs::ImageConstPtr &in_image)
{
	if(e==true)
	{
		e=false;
		struct timeval tv1,tv2;
		long long T;
	
		//image -->mat
		cv_bridge::CvImagePtr frame;
		frame=cv_bridge::toCvCopy(in_image,sensor_msgs::image_encodings::BGR8);
		gettimeofday(&tv1,NULL);
		/* //Mat图像转为yolo输入格式
		shared_ptr<image_t> detImg = detector.mat_to_image_resize(frame->image);
		//前向预测
		vector<bbox_t> outs = detector.detect_resized(*detImg, frame->image.cols, frame->image.rows, 0.25);
		// 以上{}内的代码可以用 
		*/
		vector<bbox_t> outs = detector.detect(frame->image, 0.25);
		
		//加载类别名
		vector<string> classes;
		ifstream ifs(classesFile.c_str());
		string line;
		while (getline(ifs, line)) classes.push_back(line);
		
		//画图
		Drawer(frame->image, outs, classes);
		gettimeofday(&tv2,NULL);
	
		T=(tv2.tv_sec - tv1.tv_sec)*1000 + (tv2.tv_usec -tv1.tv_usec)/1000;
		cout<<T<<"ms"<<endl;

		detect_pub.publish(frame->toImageMsg());
	}
	else
	{
		cout<<"error is true"<<endl;	
	}
}



int main(int argc,char**argv)
{
	ros::init(argc,argv,"yolo_ros_pic");
	ros::NodeHandle nh;

	detect_pub=nh.advertise<sensor_msgs::Image>("detect_img",1);
	image_sub=nh.subscribe("/src_image",1,image_callback);

	ros::spin();
	return 0;
}


