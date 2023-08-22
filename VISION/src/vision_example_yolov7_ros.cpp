#include "opencv2/opencv.hpp"
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Int16.h"
#include "vision_msgs/Detection2DArray.h"
#include "vision_msgs/Detection2D.h"
#include "vision_msgs/ObjectHypothesisWithPose.h"

using namespace std;
using namespace cv::ml;
using namespace cv;

Mat sub_image1;
bool callback = false;
int detect_length = 0;

vision_msgs::BoundingBox2D boundingBox2d;
vision_msgs::Detection2D aDetection;
vision_msgs::ObjectHypothesisWithPose result;
geometry_msgs::Pose2D center;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

void detectionlength_callback(const std_msgs::Int16::ConstPtr &msg)
{
	detect_length = msg->data;
}

void detections_callback(const vision_msgs::Detection2DArray::ConstPtr &detections2dArray)
{
    int isDone = false;
    int detectionsCount = 0;

   	ROS_INFO("---- ");
    ROS_INFO("Object detection message");

    try
    {
		for(int i = 0; i < detect_length; i++)
		{
			ROS_INFO("sssssss");

			

			aDetection = detections2dArray->detections[i];

			
			ROS_INFO(" ");

			// if(aDetection == 0) 

			// Id and confidence
			result = aDetection.results[0];
			int id = result.id;
			float score = result.score;
			ROS_INFO("   id    %i", id);
			ROS_INFO("   score %f", score);

			// Bounding boxes
			boundingBox2d = aDetection.bbox;
			center = boundingBox2d.center;
			ROS_INFO("   position (%f, %f)", center.x, center.y);
			ROS_INFO("   size %f x %f", boundingBox2d.size_x, boundingBox2d.size_y);

			
		}
    }

    // Exception handling for array out of bounds behavior is undefined. The behavior in
    // this example may not be the same in other C++ environments. Behavior was susceptible to
    //   https://stackoverflow.com/questions/1239938/accessing-an-array-out-of-bounds-gives-no-error-why
    // Behavior is consistent enough in my environment to explore message parsing.
    catch (exception& e)
    {
      ROS_INFO("Exception %s", e.what());
      isDone = true;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	// printf("이미지 수신중\n");
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
	ros::Subscriber detection_sub = nh.subscribe("/yolov7/yolov7", 10, detections_callback);
	ros::Subscriber detect_length_sub = nh.subscribe("/yolov7/flag_pub", 10, detectionlength_callback);


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
