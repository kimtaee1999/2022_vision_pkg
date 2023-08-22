#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;


class ControlCam {

protected:
	int CamNum = 0, flag = 0;
	VideoCapture* cap = new VideoCapture[15];
	Mat* image = new Mat[15];
	string Name[15] = { "camera 0","camera 1","camera 2", "camera 3", "camera 4", "camera 5","camera 6","camera 7","camera 8","camera 9","camera 10","camera 11","camera 12","camera 13","camera 14" };
public:

	void CheckCamPort() {// 카메라 열렸는지 확인하여 VideoCapture객체 만들어주는 메서드
		int startflag = 0;
		for (int i = 0; i < 15; i++) {
			string cam_num = "/dev/video" + to_string(i);
			cap[i] = VideoCapture(cam_num);
			if (cap[i].isOpened()) {
				cout << "Opened Cam Port is : " << i << '\n';
				CamNum++; // 연결된 카메라의 갯수 카운팅
				startflag = 1;
			}
		}
		if (startflag == 0) { // 연결 카메라가 없을경우
			cout << "No camera connecetd" << '\n';
			return;
		}
	}

	void RunCam() {
		for (int i = 0; i < 15; i++) {
			if (cap[i].isOpened()) {
				cap[i].read(image[i]);
				imshow(Name[i], image[i]);
			}
		}
		waitKey(1);
	}

	// 생성자
	ControlCam() {
		CheckCamPort();
		cout << "=== 생성자 호출 : 연결된 카메라 대수 : " << CamNum << "===\n";
	}

	// 소멸자 -> 동적할당 해제용
	~ControlCam() {
		delete cap;
	}
};


int main() {

	ControlCam ctrl;

	while (1) {
		ctrl.RunCam();
	}
	return 0;
}
