#include "StereoVision.h"
#include "StereoVision.cpp"

// ------------CAMERA NUMBER 입력--------------
#define LEFT_CAM 6
#define RIGHT_CAM 8


using namespace cv;
using namespace std;

void parking_cb_1(const std_msgs::Bool::ConstPtr &msgs);
void mission_cb(const std_msgs::Int16::ConstPtr& msgs);

int array_count = 0;

float baseline = 23; //카메라 사이 거리
float focal_pixels = 800; // size(1280, 720) 일때 focal_pixels
float alpha_angle = 23.9;	//alpha = 카메라 머리 숙인 각도
float beta_angle = 45.5999; //beta = erp 헤딩으로부터 카메라 각도
float gps_for_camera_x = 30; //cm
float gps_for_camera_z = -50; //cm

int target_x = 135;
int target_z = 135;
int mission_flag = 0;


Mat img_color;
Mat img_color_2;
int H, S, V;
void mouse_callback(int event, int x, int y, int flags, void *param);
void on_mouse(int event, int x, int y, int flags, void *);

bool Lidar_Stop = false;
bool finish_park = false;


bool impulse = false;

float sum_array[6] ={0,0,0,0,0,0};

ros::Subscriber Parking_level1_sub;
ros::Subscriber mission_sub;

Point ptOld1;

float camera_values();

int main(int argc, char **argv) 
{   
	ros::init(argc, argv, "parking_publisher");
 
	// string left_cam_index = "/dev/video" + to_string(LEFT_CAM);
	// string right_cam_index = "/dev/video" + to_string(RIGHT_CAM);


    // VideoCapture capLeft(left_cam_index);
    // VideoCapture capRight(right_cam_index);  
 
	VideoCapture capLeft("/home/kroad/catkin_ws/src/record_video/kcity_0923_left.mp4");
    VideoCapture capRight("/home/kroad/catkin_ws/src/record_video/kcity_0923_right.mp4");

	if (!capLeft.isOpened()) {
        cout << "Cannot Open Left Camera" << endl;
    }
    if (!capRight.isOpened()) {
        cout << "Cannot Open Right Camera" << endl;  
    }

	// Korus ver. topic
	ros::NodeHandle nh;
	ros::Publisher center_XZ_pub = nh.advertise<std_msgs::Float64MultiArray>("Vision/diag_points", 10);
	Parking_level1_sub = nh.subscribe("/LiDAR/Park_Stop", 10, parking_cb_1); //lidar parkingstop
    mission_sub = nh.subscribe("/Planning/mission", 10, mission_cb);

    capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capRight.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
 
    StereoVision stereovision(baseline, focal_pixels);

	Mat leftFrame, rightFrame;
    Mat leftMask, rightMask;
	
    Point leftCircle, rightCircle; // using yellow ball
    Point2d top_XZ, mid_XZ, bottom_XZ; // using parking

    while (ros::ok()) 
    {
		capLeft.read(leftFrame);
		capRight.read(rightFrame);

		resize(leftFrame, leftFrame, Size(1280, 720));
		resize(rightFrame, rightFrame, Size(1280, 720));

		imshow("leftFrame", leftFrame);
		imshow("rightFrame", rightFrame);

		// rectangle(leftFrame, Rect(0, 0, 1280, 100), Scalar(0, 0, 0), -1); //상단 for koreatech
		// rectangle(rightFrame, Rect(0, 0, 1280, 100), Scalar(0, 0, 0), -1); //상단 for koreatech

		rectangle(leftFrame, Rect(Point(0, 0) ,Point(1280, 70)), Scalar(0, 0, 0), -1); //for k-citys
		rectangle(rightFrame, Rect(Point(0, 0) ,Point(1280, 70)), Scalar(0, 0, 0), -1); //for k-citys
		rectangle(leftFrame, Rect(Point(leftFrame.cols-150, 0), Point(leftFrame.cols, leftFrame.rows)), Scalar(0, 0, 0), -1);
		rectangle(rightFrame, Rect(Point(leftFrame.cols-250, 0), Point(leftFrame.cols, leftFrame.rows)), Scalar(0, 0, 0),-1);

		if((mission_flag == 10)||(mission_flag == 6))
		{
			// system("clear");
			// cout << "---------------------------------------------" << endl;
			// cout << "Lidar_Stop : " << Lidar_Stop << "  finish_park : " << finish_park << endl;	

			// imshow("rightFrame", rightFrame);  
			// imshow("leftFrame", leftFrame);

			leftMask = stereovision.find_edge(leftFrame,0);
			rightMask = stereovision.find_edge(rightFrame,1);
	
			imshow("Left_mask", leftMask);
			imshow("Right_mask", rightMask);
			// ==================[ using mouse_callback ]=====================
 
			// img_color = leftFrame.clone();
			// img_color_2 = rightFrame.clone();
			// setMouseCallback("Left Frame", mouse_callback);
			// setMouseCallback("Right Frame", mouse_callback_2);

			// setMouseCallback("Left Frame",on_mouse);
			// setMouseCallback("Right Frame",on_mouse);

			if((Lidar_Stop == false) && (finish_park == false))
			{ 
				cout << " Wating Lidar Stop" << endl;
				// imshow("Left Frame", leftFrame);   
				// imshow("Right Frame", rightFrame);
			}
			else if((Lidar_Stop == true) && (finish_park == false))
			{	
				if(impulse == false)
				{
					cout << "impulse !!" << endl;
					ros::Duration(1.5).sleep();
					impulse = true;
				}


				double left_array[6] = {0,0,0,0,0,0};
				double right_array[6] = {0,0,0,0,0,0};
				double array[6] = {0,0,0,0,0,0};
				double pub_array[6] = {0,0,0,0,0,0};

				double *ptr_left = stereovision.find_center(leftMask, left_array, 0);
				double *ptr_right = stereovision.find_center(rightMask, right_array, 1);

				// If no ball is detected in one of the cameras - show the text "tracking lost"
				if (left_array[0] && right_array[0])
				{
					bottom_XZ = stereovision.find_XZ({left_array[0],left_array[1]}, {right_array[0],right_array[1]}, leftFrame, rightFrame, alpha_angle, beta_angle);
					// mid_XZ = stereovision.`({left_array[2],left_array[3]}, {right_array[2],right_array[3]}, leftFrame, rightFrame, alpha_angle, beta_angle);
					top_XZ = stereovision.find_XZ({left_array[4],left_array[5]}, {right_array[4],right_array[5]}, leftFrame, rightFrame, alpha_angle, beta_angle);
					mid_XZ.x = (bottom_XZ.x + top_XZ.x)/2.0;
					mid_XZ.y = (bottom_XZ.y + top_XZ.y)/2.0;
 
					// cout << "bottom_XZ : " << bottom_XZ << endl;
					// cout << "mid_XZ : " << mid_XZ << endl;
					// cout << "top_XZ : " << top_XZ << endl;
				
					array[0]= (bottom_XZ.x + gps_for_camera_z)/100.00;
					array[1]= -(bottom_XZ.y + gps_for_camera_x)/100.00;
					array[2]= (mid_XZ.x + gps_for_camera_z)/100.00;
					array[3]= -(mid_XZ.y + gps_for_camera_x)/100.00;
					array[4]= (top_XZ.x + gps_for_camera_z)/100.00;
					array[5]= -(top_XZ.y + gps_for_camera_x)/100.00;
	

					for(int i=0; i<6; i++) 
					{
						cout << " array : " << array[i] << endl;
					}
					cout << endl;
					for(int i=0; i<6; i++)
					{
						sum_array[i] = sum_array[i] + array[i];
					}				

					// array_count++;
					cout << "array_count : " << array_count << endl;
					if(array_count == 20)
					{
						cout << " Finish !!!! " << endl;
						std_msgs::Float64MultiArray center_XZ_msg;
						center_XZ_msg.data.clear();

						for(int i=0; i<6; i++)
						{
							pub_array[i] = sum_array[i]/(double)array_count;
							center_XZ_msg.data.push_back(pub_array[i]);
							printf("pub_array[%d] : %f\n", i, pub_array[i]);
						}
						center_XZ_pub.publish(center_XZ_msg);
						finish_park = true;
					}
				}
			}
			else if((Lidar_Stop == true) && (finish_park == true))
			{
				// cout << " Finish !!!! " << endl;
			}
		}
	
		if (waitKey() == 27)
		{
			break;
			printf("end");
		}

		
        // capLeft.release();
        // capRight.release();
        ros::spinOnce();
	    // loop_rate.sleep();
    }
    return 0;
}

/**
 * @brief 마우스 왼쪽 클릭을 하면 마우스가 있는 지점의 HSV 색상을 알려줌
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param param 
 */

void mouse_callback(int event, int x, int y, int flags, void *param)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		Vec3b color_pixel = img_color.at<Vec3b>(y, x);

		Mat hsv_color = Mat(1, 1, CV_8UC3, color_pixel);


		H = hsv_color.at<Vec3b>(0, 0)[0];
		S = hsv_color.at<Vec3b>(0, 0)[1];
		V = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "left H= " << H << endl;
		cout << "left S= " << S << endl;
		cout << "left V = " << V << "\n"
			 << endl;

	}
}

// float camera_values()
// {
//     cv::Mat K(3,3,CV_64F);

//     K = (Mat_<_Float64>(3, 3) << 538.39128993937,   0.,   308.049009633327, 0.,  539.777910345317,  248.065244763797, 0., 0., 1.);

//     cout << "K : " << K << endl;


//     cv::Size imageSize(640,480);
//     double apertureWidth = 0;
//     double apertureHeight = 0;
//     double fieldOfViewX;
//     double fieldOfViewY;
//     double focalLength2;
//     cv::Point2d principalPoint;
//     double aspectRatio;
//     cv::calibrationMatrixValues(K, imageSize, apertureWidth, apertureHeight, fieldOfViewX, fieldOfViewY, focalLength2, principalPoint, aspectRatio);

    
//     cout << fieldOfViewX << endl;

//     return (float)focalLength2;
// }

/**
 * @brief 마우스 왼쪽 버튼을 누르고 땔 때마다 그 위치의 좌표를 알려줌
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 */

void on_mouse(int event, int x, int y, int flags, void *)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		ptOld1 = Point(x, y);
		cout << "EVENT_LBUTTONDOWN: " << x << ", " << y << endl;
		break;
	case EVENT_LBUTTONUP:
		cout << "EVENT_LBUTTONUP: " << x << ", " << y << endl;
		break;
	}
}

/**
 * @brief 라이더로부터 플래그를 받음 Lidar_Stop
 * Lidar_Stop = true 인 경우 level2로 넘어감
 * 
 * @param msgs 
 */
void parking_cb_1(const std_msgs::Bool::ConstPtr &msgs)
{
	Lidar_Stop = msgs->data;
}

/**
 * @brief 플래닝으로부터 미션 번호를 받음
 * 10 = 주차 예비
 * 6 = 주차
 * 
 * @param msgs 
 */
void mission_cb(const std_msgs::Int16::ConstPtr& msgs)
{
	mission_flag = msgs->data;
}
