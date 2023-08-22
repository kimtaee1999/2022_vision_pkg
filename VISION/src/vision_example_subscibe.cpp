#include "opencv2/opencv.hpp"
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv::ml;
using namespace cv;

Mat sub_image1;
bool callback = false;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	printf("이미지 수신중\n");
	Mat image1 = cv_bridge::toCvShare(msg)->image;
	sub_image1 = image1.clone();
	cv::waitKey(1);
	callback = true;
	// imshow("image1", image1);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_subsciber");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub_image = it.subscribe("/yolov7/yolov7/visualization", 1, imageCallback);

	ros::Rate loop_rate(50);
	printf("Waiting for ---/yolov7/yolov7/visualization---\n");

	while (ros::ok())
	{
		if (callback == true)
		{
			Mat img;
			cv::resize(sub_image1, img, cv::Size(640, 480), 0, 0);
			imshow("subscribe_image",img);

		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
