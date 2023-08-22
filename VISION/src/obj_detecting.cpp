#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"

#define CAM_NUM 2
 
using namespace cv;
using namespace std;

void BoundingBox_callback(darknet_ros_msgs::BoundingBoxes boundingBox);
void ObjectCount_callback(darknet_ros_msgs::ObjectCount ObjectCount);

int obj_count;

std_msgs::Int32MultiArray sign_msg;
std_msgs::Int32 sign_test_msg;
std_msgs::Bool A_sign_msg;
string Class_name_test;
ros::Subscriber detect_obj;
ros::Subscriber obj_num;
ros::Publisher final_pub;
ros::Publisher Stop_final_pub;
ros::Publisher traffic_sign_pub;
ros::Publisher traffic_sign_test_pub;
ros::Subscriber mission_sub;
ros::Publisher A_sign_pub;
ros::Publisher Speed_pub;



float Max_Area = 0;
int mission_A = 1;
int mission_B = 0;
int count_mission = 0;
bool mission_stop = false;
int count_A1 = 0;
int count_A2 = 0;
int count_A3 = 0;
int sign[3]={0,0,0};
int red = 0;
int green = 0;
int greenleft = 0;
int light_test=0;
int mission_flag = 0;
bool A_sign = false;
bool speed_down = false;


void mission_cb(const std_msgs::Int16::ConstPtr& msgs)
{
	mission_flag = msgs->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obj_detecting");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/vision/image_raw", 1);
    detect_obj = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1000, BoundingBox_callback);
    obj_num = nh.subscribe<darknet_ros_msgs::ObjectCount>("/darknet_ros/found_object", 1000, ObjectCount_callback);

    // original
    // traffic_sign_pub = nh.advertise<std_msgs::Int32MultiArray>("traffic_sign", 100);
    // traffic_sign_test_pub = nh.advertise<std_msgs::Int32>("traffic_sign_test", 100);
    // Stop_final_pub = nh.advertise<std_msgs::Bool>("/Mission_Stop", 1000);
    // A_sign_pub = nh.advertise<std_msgs::Bool>("A_sign", 1000);
    // mission_sub = nh.subscribe("mission", 10, mission_cb);

    // Korus ver. topic
    traffic_sign_pub = nh.advertise<std_msgs::Int32MultiArray>("Vision/traffic_sign", 100);
    traffic_sign_test_pub = nh.advertise<std_msgs::Int32>("Vision/traffic_sign_test", 100);
    Stop_final_pub = nh.advertise<std_msgs::Bool>("Vision/B_sign", 10);
    A_sign_pub = nh.advertise<std_msgs::Bool>("Vision/A_sign", 10);
    Speed_pub = nh.advertise<std_msgs::Bool>("Vision/Speed", 10);
    mission_sub = nh.subscribe("/Planning/mission", 10, mission_cb);
    


    ros::Rate loop_rate(1000);
    sensor_msgs::ImagePtr msg;

    string cam_index = "/dev/video" + to_string(CAM_NUM);

    VideoCapture cap(cam_index);
    // VideoCapture cap("/home/kroad/catkin_ws/src/VISION/src/video/0923_kcity/kcity_0923_03.mp4");

    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    
    if (!cap.isOpened())
    {
        cerr << "video load fail!!\n"
             << endl;
    }

    Mat src;

    while (ros::ok())
    {
        cap >> src;
        imshow("src", src);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();

        pub.publish(msg);
        // Mat src_resize;
        //resize(src, src_resize, Size(1280, 720));

        if (src.empty())
        {
            printf("finish!!\n");
            break;
        }

        if (waitKey(10) == 27)
        {
            break;
            printf("강제 종료\n");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void ObjectCount_callback(darknet_ros_msgs::ObjectCount ObjectCount)
{
    // printf("발견한 객체 수(YOLO) = %d\n", ObjectCount.count);
    obj_count = ObjectCount.count;
        red = 0, green = 0, greenleft = 0;
            sign_msg.data = {red,green,greenleft};

        if(obj_count==0)
        {
         traffic_sign_pub.publish(sign_msg);
        }
        // printf("test02\n");

}

void BoundingBox_callback(darknet_ros_msgs::BoundingBoxes boundingBox)
{
    // printf("test01\n");
    sign_test_msg.data=light_test;
    A_sign_msg.data = A_sign;
    A_sign_pub.publish(A_sign_msg);
    
    red = 0, green = 0, greenleft = 0;
    if (obj_count != 0)
    {
        std::string Class_name;
        Max_Area = 0;
        light_test=0;
        // mission_flag=0;
        
        
        std_msgs::Bool mission_stop_msg;
        std_msgs::Bool mission_speed_down;
        
        for (int i = 0; i < obj_count; i++)
        {
            Class_name = boundingBox.bounding_boxes[i].Class;
            // A_sign=false;
            float xmin = boundingBox.bounding_boxes[i].xmin;
            float ymin = boundingBox.bounding_boxes[i].ymin;
            float xmax = boundingBox.bounding_boxes[i].xmax;
            float ymax = boundingBox.bounding_boxes[i].ymax;
            float Area = ((xmax - xmin) * (ymax - ymin));

            cout << "Class_name [ " << i << "] : " << Class_name << " Area: " << Area << endl;

            if (Class_name == "A1" && Area>1150 && mission_flag==7)   //1500
            {
                count_A1 += 1;
                Class_name_test="A1";
                A_sign=true;
                cout<<"Area: "<<Area<<endl;
            }
            else if (Class_name == "A2" && Area>1150 && mission_flag==7)
            {
                count_A2 += 1;
                A_sign=true;
                cout<<"Area: "<<Area<<endl;
            }
            else if (Class_name == "A3" && Area>1150 && mission_flag==7)
            {
                count_A3 += 1;
                A_sign=true;
                cout<<"Area: "<<Area<<endl;
            }
            else if (Class_name == "green")
            {
                green = 1;
                light_test=2;
            }
            else if (Class_name == "red")
            {
                red = 1;
                light_test=1;
            }
            else if (Class_name == "greenleft")
            {
                greenleft = 1;
                light_test=3;
            }
            else if (Class_name == "B1" || Class_name == "B2" || Class_name == "B3")
            {
                if((mission_flag== 8)&&(Area > 1000))
                {
                    speed_down = true;
                }
                
                cout << "Class_name [ " << i << "] : " << Class_name << " Area: " << Area << endl;
                if ((Max_Area < Area && Area>2300)&&( (ymax - ymin)/(xmax - xmin)<=1.5) && (mission_flag==8))   //2700
                {
                    Max_Area = Area;
                    speed_down = true;
                    
                    if (Class_name == "B1" && mission_flag==8)
                    {
                        mission_B = 1;
                        speed_down = true;
                    }

                    else if (Class_name == "B2" && mission_flag==8)
                    {
                        mission_B = 2;
                        speed_down = true;
                    }

                    else if (Class_name == "B3" && mission_flag==8)
                    {
                        mission_B = 3;
                        speed_down = true;
                    }
                }
            }
        }
        
        cout<<"A_sign: "<<A_sign<<endl;
        // mission_A = 1;
        if (count_A1 < count_A2 && count_A3 < count_A2)
        {
            mission_A = 2;
        }
        if (count_A1 < count_A3 && count_A2 < count_A3)
        {
            mission_A = 3;
        }
        // if (count_A1 < count_A2)
        // {
        //     mission_A = 2;
        //     // cout<<"Class_name: "<<Class_name<<"!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            
        //     if (count_A2 < count_A3)
        //     {
        //         mission_A = 3;
        //         // cout<<"Class_name: "<<Class_name<<"!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        //     }
        // }
        // else if (count_A1 < count_A3)
        // {
        //     mission_A = 3;
        //     // cout<<"Class_name: "<<Class_name<<"!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        // }

        cout<<"count_A == [ "<<count_A1<< ", " << count_A2<<", "<< count_A3<< " ]"<<endl;

        cout << "mission_A: " << mission_A << endl;
        cout << "mission_B: " << mission_B << endl;

        if (mission_A == mission_B)
        {
            mission_stop = true;
        }
        else if (mission_A != mission_B)
        {
            mission_stop = false;
        }
        cout << "mission_stop: " << mission_stop << endl;
        cout << "speed_down:" << speed_down << endl;

        mission_speed_down.data = speed_down;
        Speed_pub.publish(mission_speed_down);

        mission_stop_msg.data = mission_stop;
        Stop_final_pub.publish(mission_stop_msg);
        sign_msg.data = {red,green,greenleft};

        // cout<<"red : "<<sign_msg.data[0] << "\t\tgreen : "<<sign_msg.data[1]<<"\tgreenleft : "<<sign_msg.data[2]<<endl;
        traffic_sign_pub.publish(sign_msg);

        cout<<"light_test: "<<light_test<<endl;
        traffic_sign_test_pub.publish(sign_test_msg);
    }
    cout<<"A_sign: "<<A_sign<<endl;
    cout<<"red : "<<sign_msg.data[0] << "\t\tgreen : "<<sign_msg.data[1]<<"\tgreenleft : "<<sign_msg.data[2]<<endl;
    
}
