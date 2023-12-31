#include "stopline_func.h"

using namespace cv;
using namespace std;
// WeAreVision test;

// -----------FUNCTIONS--------------------------------------------------------------


Point ptOld2;
void on_mouse1(int event, int x, int y, int flags, void *)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		ptOld2 = Point(x, y);
		cout << "EVENT_LBUTTONDOWN: " << x << ", " << y << endl;
		break;
	case EVENT_LBUTTONUP:
		cout << "EVENT_LBUTTONUP: " << x << ", " << y << endl;
		break;
	}
}

/**
 * @brief 0값만을 가지는 영상을 소스 영상과 동일한 사이즈로 만듦
 * 
 * @param img 소스영상
 * @return Mat 
 */
Mat WeAreVision::make_zeros(Mat img)
{
    return Mat::zeros(img.rows, img.cols, img.type());
}

/**
 * @brief 영상에서 그려져있는 사각형 내의 평균값을 냄
 * 
 * @param img 입력영상
 * @param X 좌상단 X점
 * @param width 가로길이
 * @param Y 좌상단 Y점
 * @param height 높이
 * @return Scalar 
 */
Scalar WeAreVision::IMG_mean(Mat img, int X, int width, int Y, int height)
{
	Mat img_roi = img(Rect(Point(Y, X), Point(Y + height, X + width)));
	// imshow("img_roi",img_roi);
	Scalar average = mean(img_roi);
	// std::cout << average << std::endl;
	return average;
}

/**
 * @brief 대각선 길이가 1인 영상을 입력 영상과 사이즈를 동일하게 만듦
 * 
 * @param img 입력영상
 * @return Mat 
 */
Mat WeAreVision::make_ones(Mat img)
{
    return Mat::ones(img.rows, img.cols, img.type());
}

/**
 * @brief 이미지를 이진화하는 함수, 우리는 여기서 HLS색공간에서 L값만 조정해서 영상을 이진화함
 * 
 * @param imgUnwarp 이진화하고자 하는 영상 입력
 * @param toColorChannel 어떤 색공간으로 변환할지 코드(0 = HLS, 1 = LAB)
 * @param mode 어떤 것을 통해서 이진화할 것인가(0 = H, 1 = L, 2 = S)
 * @return Mat 이진화된 영상
 */
Mat WeAreVision::filterImg(Mat imgUnwarp, int toColorChannel, int mode)
{
    /*
    channel mode definition.
        0 : Hue
        1 : Lightness
        2 : Saturation
        
        hue max : 179, l and s max : 255
    */
    Mat imgConverted;
    // Mat imgOUT = Mat(720, 1280, CV_8UC3);
    // Mat imgOUT = Mat::zeros(720, 1280, CV_8UC3);
    Mat imgOUT = Mat::zeros(480, 640, CV_8UC1);

    /* 1. convert color channel from BGR to HLS or LAB. */
    if (toColorChannel == 0)
    {
        cvtColor(imgUnwarp, imgConverted, COLOR_BGR2HLS);
        // imshow("Hls", imgConverted);
    }
    else if (toColorChannel == 1)
    {
        cvtColor(imgUnwarp, imgConverted, COLOR_BGR2Lab);
        // imshow("Lab", imgConverted);
    }

    /* 2. pixel pointer variable setting.
    help -->  https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
    */
    uint8_t *pixelPtr = (uint8_t *)imgConverted.data;
    int cn = imgConverted.channels();

    switch (mode)
    {
    case 0:
        // set H space Only  // set L space Only
        for (int i = 0; i < imgConverted.rows; i++)
        {
            for (int j = 0; j < imgConverted.cols; j++)
            {
                imgOUT.at<uint8_t>(i, j) = pixelPtr[i * imgConverted.cols * cn + j * cn + 0];
            }
        }
        break;
    case 1:
        // set L space Only  // set A space Only
        for (int i = 0; i < imgConverted.rows; i++)
        {
            for (int j = 0; j < imgConverted.cols; j++)
            {
                imgOUT.at<uint8_t>(i, j) = pixelPtr[i * imgConverted.cols * cn + j * cn + 1];
            }
        }
        break;

    case 2:
        // set S space Only  // set B space Only
        for (int i = 0; i < imgConverted.rows; i++)
        {
            for (int j = 0; j < imgConverted.cols; j++)
            {
                imgOUT.at<uint8_t>(i, j) = pixelPtr[i * imgConverted.cols * cn + j * cn + 2];
            }
        }
        break;

    default:
        break;
    }

    /* 
    COLORMAP for grayscale.     
    ref : https://learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
     applyColorMap(imgOUT, imgOUT, COLORMAP_BONE);
     컬러맵을 쓰니깐 채널이 1개에서 3개가 되버렸다. 이 메쏘드는 안쓰는게 맞는듯.
     컬러맵은 그레이 스케일에 대해서, 색상을 역으로 부여하는 꼴이 되기 때문에 채널이 3개로 늘어난 듯.
    */

    return imgOUT;
}

/**
 * @brief 두 개의 색공간 영상을 합치는 함수
 * 
 * @param hls HLS로 이진화된 영상
 * @param lab LAB로 이진화된 영상
 * @return Mat 결과
 */
Mat WeAreVision::combine_both_img(Mat hls, Mat lab)
{
    Mat imgOut;

    imgOut = make_zeros(hls);
    for (int i = 0; i < imgOut.rows; i++)
    {
        for (int j = 0; j < imgOut.cols; j++)
        {
            if (hls.at<uint8_t>(i, j) == 1 || lab.at<uint8_t>(i, j) == 1)
            {
                imgOut.at<uint8_t>(i, j) = 1;
            }
        }
    }
    return imgOut;
}

/**
 * @brief HLS 색공간에서 화면을 이진화함
 * 
 * @param unWarp 워프변환한 영상
 * @return Mat 
 */
Mat WeAreVision::normalize_HLS_L(Mat unWarp)
{
    /* normalizing L color channel pixel from hls img. */
    Mat imgHLS_L, imgNormal;
    double minVal, maxVal;
    Point minLoc, maxLoc;

    Scalar mean_color1 = IMG_mean(unWarp, unWarp.rows * 3 / 5, unWarp.rows * 1/ 5, unWarp.cols * 1 / 7, unWarp.cols / 7);
	Scalar mean_color2 = IMG_mean(unWarp, unWarp.rows * 3 / 5, unWarp.rows * 1/ 5, unWarp.cols * 4 / 7, unWarp.cols * 1 / 7);
    // Scalar mean_color3 = IMG_mean(unWarp, unWarp.rows * 1 / 7 ,unWarp.rows * 1/ 7 , unWarp.cols * 1/ 7, unWarp.cols * 1/7);

    // int mean_thres = (mean_color1[1] + mean_color2[1]+ mean_color3[1]) /2 +20;
	int mean_thres = (mean_color1[1] + mean_color2[1]) /2 +20;
    
    int lowThres; // origin : 200

    if (mean_thres < 159){
        lowThres = 170; 
    }
    else{
        lowThres = mean_thres;
    }

    // printf("%d",lowThres);
    
    // get a single channel img(filtered one.)
    imgHLS_L = filterImg(unWarp, 0, 1);
    // imshow("filterimg",imgHLS_L);
    // get max, min value of the matrix.
    minMaxLoc(imgHLS_L, &minVal, &maxVal, &minLoc, &maxLoc);

    // make normalized img.
    imgNormal = (255 / maxVal)* imgHLS_L;
    
    // imshow("normalimg",imgNormal);
    // apply threshold for L channel.
    Mat imgOut = make_zeros(imgNormal);
    // Mat imgOut2 = make_zeros(imgNormal);

    threshold(imgNormal, imgOut, lowThres, 255, THRESH_BINARY);
    // threshold(imgNormal, imgOut2,0,255,THRESH_BINARY|THRESH_TRIANGLE);
    // 적응형 이진화
    
    return imgOut;
}

/**
 * @brief LAB색공간의 영상을 평균화하는 함수, 우리는 안씀
 * 
 * @param unWarp 입력영상
 * @return Mat 
 */
Mat WeAreVision::normalize_LAB_B(Mat unWarp)
{
    /* normalizing B color channel pixel from LAB img. */
    Mat imgLAB_B, imgNormal;
    double minVal, maxVal;
    Point minLoc, maxLoc;
    int yellowCrit = 190;
    int lowThres = 10; // origin: 190

    // get a single channel img(filtered one.)
    imgLAB_B = filterImg(unWarp, 1, 2);

    // get max, min value of the matrix.
    minMaxLoc(imgLAB_B, &minVal, &maxVal, &minLoc, &maxLoc);

    // (conditional) make normalized img.
    // B channel means a range from blue(0) to yellow(255).
    // So, the bigger values, it becomes close to yellow color.(yellow lane)
    if (maxVal > yellowCrit)
    {
        imgNormal = (255 / maxVal) * imgLAB_B;
    }
    else
    {
        imgNormal = imgLAB_B;
    }
    // imshow("imgNormal", imgNormal);

    // apply threshold for L channel.
    Mat imgOut = Mat::zeros(imgNormal.rows, imgNormal.cols, imgNormal.type());
    threshold(imgNormal, imgOut, 200, 255, THRESH_BINARY);

    return imgOut;
}

/**
 * @brief 항공뷰로 바꾸는 함수(어쩌면 stopline 코드에서 가장 중요한 부분이라고 할 수 있음)
 * 
 * @param img 항공뷰로 변환하고자 하는 영상
 * @return Mat 
 */
Mat WeAreVision::bird_eyes_view(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];
	// -----------------------------------------------------------------------------------------------

	// //원본의 좌표(좌하단, 우하단, 좌상단, 우상단) <ORIGIN>
	warp_src_point[0].x = 5;
	warp_src_point[0].y = height;
	warp_src_point[1].x = width - warp_src_point[0].x;
	warp_src_point[1].y = warp_src_point[0].y;
	warp_src_point[2].x = 260;
	warp_src_point[2].y = 140;
	warp_src_point[3].x = width - warp_src_point[2].x;
	warp_src_point[3].y = warp_src_point[2].y;

	//원본의 좌표(좌하단, 우하단, 좌상단, 우상단) _ need to be retouched. <EXP>
	// warp_src_point[0].x = 8; // fix
	// warp_src_point[0].y = height; // fix
	// warp_src_point[1].x = width - warp_src_point[0].x; // fix
	// warp_src_point[1].y = warp_src_point[0].y; // fix
	// warp_src_point[2].x = 274; 
	// warp_src_point[2].y = 279;
	// warp_src_point[3].x = width - warp_src_point[2].x;
	// warp_src_point[3].y = warp_src_point[2].y;


    // -----------------------------------------------------------------------------------------------
	//목표이미지의 좌표(좌하단, 우하단, 좌상단, 우상단) _ modified
	// warp_dst_point[0].x = 10;
	// warp_dst_point[0].y = height;
	// warp_dst_point[1].x = width - warp_dst_point[0].x;
	// warp_dst_point[1].y = height;
	// warp_dst_point[2].x = 150;
	// warp_dst_point[2].y = 0;
	// warp_dst_point[3].x = width - warp_dst_point[2].x;
	// warp_dst_point[3].y = 0;

	// //목표이미지의 좌표(좌하단, 우하단, 좌상단, 우상단) _ origin
	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width - warp_dst_point[0].x;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - warp_dst_point[2].x;
	warp_dst_point[3].y = 0;
	
	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
	invert(warp_matrix, warp_matrix_inv);

	//------------------------copying ROI : local to global---------------------------------------
	warp_SRC_ROI[0].x = warp_src_point[0].x;
	warp_SRC_ROI[0].y = warp_src_point[0].y;
	warp_SRC_ROI[1].x = warp_src_point[1].x;
	warp_SRC_ROI[1].y = warp_src_point[1].y;
	warp_SRC_ROI[2].x = warp_src_point[2].x;
	warp_SRC_ROI[2].y = warp_src_point[2].y;
	warp_SRC_ROI[3].x = warp_src_point[3].x;
	warp_SRC_ROI[3].y = warp_src_point[3].y;
	
	warp_DST_ROI[0].x = warp_dst_point[0].x;
	warp_DST_ROI[0].y = warp_dst_point[0].y;
	warp_DST_ROI[1].x = warp_dst_point[1].x;
	warp_DST_ROI[1].y = warp_dst_point[1].y;
	warp_DST_ROI[2].x = warp_dst_point[2].x;
	warp_DST_ROI[2].y = warp_dst_point[2].y;
	warp_DST_ROI[3].x = warp_dst_point[3].x;
	warp_DST_ROI[3].y = warp_dst_point[3].y;
	//-------------------------------------------------------------------------------------------------

	Mat dst;
	cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
	// imshow("dst", dst);  //top view
	// setMouseCallback("dst",on_mouse1);

	return dst;
}

/**
 * @brief 정해진 영역 내에서 마스크를 훑으면서 마스크 내에 흰색 영역이 많으면 마스크 중심을 흰색으로 정의하고 오른쪽 열로 넘어감
 * 더욱더 자세히 알고싶으면 코드를 뜯어보셈
 * 이 함수 내에서 화면 y값의 따른 길이를 조정함
 * 
 * @param img 정지선을 찾고자 하는 영상(HLS로 이진화한 영상)
 * @param _mask_w 마스크의 가로길이
 * @param _mask_h 마스크의 세로길이
 * @param thresh 이 길이 이상으로 흰 색이 연속적이면 정지선이라고 인식함
 * @return Mat 
 */
Mat WeAreVision::mask_filter(Mat img, int _mask_w, int _mask_h, int thresh)
{
	//2020 1st member made.
	int height = img.rows;
	int width = img.cols;
	Mat img_maskfilter;
	img_maskfilter = Mat::zeros(height, width, CV_8UC1); // zerolize filter
	Mat img_stop;
	img_stop = Mat::zeros(height, width, CV_8UC3); // zerolize stop_img
	float mask[3];
	int sx = 0;
	isStop = 100;

	uint *image = (uint *)img.data;
	uchar *score_data = (uchar *)img_maskfilter.data;
	int mask_w = _mask_w, mask_h = _mask_h;

	int sy = 0;

	int roi_w = 150; // 80         check!!!!!!!!!!!!!!! 꼭 체크!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@
	int histo = 0;

	for (int y = height - 15; y > 20; y--)
	{
		histo = 0;
		for (int x = int(width / 2) - roi_w; x <= int(width / 2) + roi_w; x++)
		{
			for (int i = 0; i < 3; i++)
			{
				sy = y + (2 * mask_h + 1) * (i - 1);
				int dx, cx, bx, ax;
				int dy, cy, by, ay;
				dy = sy + mask_h;
				dx = x + mask_w;
				cy = sy - mask_h - 1;
				cx = x + mask_w;
				by = sy + mask_h;
				bx = x - mask_w - 1;
				ay = sy - mask_h - 1;
				ax = x - mask_w - 1;
				mask[i] = image[(dy)*width + dx] - image[(cy)*width + cx] - image[(by)*width + bx] + image[(ay)*width + ax];
			}
			float sum = ((mask[1] - mask[0]) + (mask[1] - mask[2])) / 2;
			// cout<<"sum"<<sum<<endl;
			if (sum > 6000) // 10000  6000(5*5)  ---------------if(mask_h*mask_w == 40) -> sum > 100000----------------
			{
				score_data[width * y + x] = 255;
				histo++;
			}
		}
		line(img_stop, Point(int(width / 2) + roi_w, 20), Point(int(width / 2) + roi_w, height), Scalar(255, 255, 0), 5);
		line(img_stop, Point(int(width / 2) - roi_w, 20), Point(int(width / 2) - roi_w, height), Scalar(255, 255, 0), 5);

		// imshow("mask_filter", img_maskfilter);

		if (histo > thresh)
		{
			line(img_stop, Point(int(width / 2) - roi_w, y), Point(int(width / 2) + roi_w, y), Scalar(255, 0, 0), 30);
			// cout << "histo : " << histo << endl;
			// cout<<"y"<<y<<endl;
			// if (y < 110) //255
			// {
			// 	cout << "stop line distance : 10M\n"
			// 		 << endl;
			// 	isStop = 10;
			// }
			// else if (y < 130)//245
			// {
			// 	cout << "stop line distance : 9M\n"
			// 		 << endl;
			// 	isStop = 9;
			// }
			// else if (y < 160) //275
			// {
			// 	cout << "stop line distance : 8M\n"
			// 		 << endl;
			// 	isStop = 8;
			// }
			if (y < 33) //305
			{
				cout << "stop line distance : 7M\n"
					 << endl;
				isStop = 7;
			}
			else if (y < 94) //335
			{
				cout << "stop line distance : 6M\n"
					 << endl;
				isStop = 6;
			}
			else if (y < 152) //370
			{
				cout << "stop line distance : 5M\n" ///12m
					 << endl;
				isStop = 5;
			}
			else if (y < 216) //410
			{
				cout << "stop line distance : 4M\n"
					 << endl;
				isStop = 4;
			}
			else if (y < 288) //450
			{
				cout << "stop line distance : 3M\n"
					 << endl;
				isStop = 3;
			}
			else if (y < 367) //500
			{
				cout << "stop line distance : 2M\n"
					 << endl;
				isStop = 2;
			}
			else
			{
				cout << "GO\n"
					 << endl;
				isStop = 100;
			}
			
			break;
		}
	}
	// imshow("img_stop", img_stop);
	return img_stop;
}


/**
 * @brief stopline내의 어지간한 함수를 사용하면서 영상을 HLS 색공간 기반으로 이진화하고 그 안에서 mask_filter함수를 사용해 정지선인지 아닌지를 판단함
 * 
 * @param img 원본영상 입력
 * @param img_warp 항공 뷰 영상 입력
 * @return Mat 최종 결과 영상
 */
Mat WeAreVision::STOP_preprocessing(Mat img, Mat img_warp){
	// cerr << "1-----------------"<<endl;
	Mat img_warp_clone,img_warp_clone_hls,img_show;

	Mat white = Mat::zeros(480, 640, img_warp.type());
    rectangle(white, Rect(0, 450, 30, 30), Scalar(250, 250, 250), -1);
	// imshow("11", white);
	img_warp_clone = img_warp.clone();
    img_show = img_warp.clone();
	img_warp_clone_hls = img_warp_clone.clone();
	
	Mat img_warp_clone_hls_white= img_warp_clone_hls + white;
	// imshow("11", img_warp_clone_hls_white);
		
	Mat img_OUT_HLS;
	img_OUT_HLS= normalize_HLS_L(img_warp_clone_hls_white);
	// imshow("img_out_HLS",img_OUT_HLS);
	
	Mat BLURRED;
	medianBlur(img_OUT_HLS, BLURRED,3);
	// imshow("bluur~~",BLURRED);
	
	Mat OPEN_CLOSE;	
	cv::morphologyEx(BLURRED,OPEN_CLOSE,MORPH_CLOSE,Mat());
	cv::morphologyEx(OPEN_CLOSE,OPEN_CLOSE,MORPH_OPEN,Mat());
	// imshow("OPEN~~",OPEN_CLOSE);

	// rectangle(img, Rect(Point(img_warp.cols * 1 / 5, img_warp.rows * 3 / 5), Point(img_warp.cols * 2 / 5, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0); // why one?
	// rectangle(img, Rect(Point(img_warp.cols * 3 / 5, img_warp.rows * 3 / 5), Point(img_warp.cols * 4 / 5, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0);
	rectangle(img_show, Rect(Point(img_warp.cols * 1 / 7, img_warp.rows * 3 / 5), Point(img_warp.cols * 2 / 7, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0); // why one?
	rectangle(img_show, Rect(Point(img_warp.cols * 4 / 7, img_warp.rows * 3 / 5), Point(img_warp.cols * 5 / 7, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0);
    rectangle(img_show, Rect(Point(img_warp.cols * 1 / 2- 30 , img_warp.rows * 1 / 7), Point(img_warp.cols * 1/2 +30 , img_warp.rows * 2/7)), Scalar(0, 0, 255), 1, 8, 0);
	// unWarp.rows * 1 / 7 ,unWarp.rows * 1/ 7 , unWarp.cols * 1/ 2, unWarp.cols * 1/5
    
    // imshow("img_warp_color_mean", img_show); // 색 평균 영역 설정

	// // ----------------------------------------------------INTEGRATE_EXP----------------------------------------------------
	// Scalar mean_color1 = IMG_mean(img_warp, img_warp.rows * 3 / 5, img_warp.rows * 1/ 5, img_warp.cols * 1 / 7, img_warp.cols / 7);
	// Scalar mean_color2 = IMG_mean(img_warp, img_warp.rows * 3 / 5, img_warp.rows * 1/ 5, img_warp.cols * 4 / 7, img_warp.cols * 1 / 7);
	// cout << "<BGR_ mean1 color> " << mean_color1 << "  <BGR_ mean2 color>" << mean_color2 << endl;

	Mat img_binary;
	// cv::inRange(img_warp, (mean_color1 + mean_color2) / 2 + cv::Scalar(10, 10, 10), cv::Scalar(255, 255, 255), img_binary);
	img_binary = OPEN_CLOSE;
	// imshow("img_binary", img_binary);

	Mat img_integral;
	cv::integral(img_binary, img_integral);
	// imshow("img_integral",img_integral);

	Mat img_mask;
	img_mask = mask_filter(img_integral, 5, 5, 200); // (5,5,85)   (5,8,120)      check!!!!!!!!!!!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	// imshow("img_mask", img_mask);   //mask filter 이거 체크!!!!!!!!!!!!!

	Mat warp_inv;
	cv::warpPerspective(img_mask, warp_inv, warp_matrix_inv, cv::Size());
	// imshow("warp_inv",warp_inv);

	return warp_inv;
}

/**
 * @brief 화면에다가 현재 몇 미터 남았는지를 출력하는 함수
 * 
 * @param final 미터수를 출력하고자 하는 영상
 * @param count 
 * @return int 정지선까지의 거리가 몇 미터인지를 출력
 */
int WeAreVision::DISPLAY_meter(Mat final, int count){
	
	int data;
		// if (isStop == 12)
		// {
		// 	count = 12;
		// 	data = count;
		// 	putText(final, "12M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		// 	return data;
		// }
		// else if (isStop == 10)
		// {
		// 	count = 10;
		// 	data = count;
		// 	putText(final, "10M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		// 	return data;
		// }
		// else if (isStop == 9)
		// {
		// 	count = 9;
		// 	data = count;
		// 	putText(final, "9M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		// 	return data;
		// }
		// else if (isStop == 8)
		// {
		// 	count = 8;
		// 	data = count;
		// 	putText(final, "8M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		// 	// isStop=100;
		// 	return data;
		// }
		if (isStop == 7)
		{
			count = 7;
			data = count;
			putText(final, "7M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
			return data;
		}

		else if (isStop == 6)
		{
			count = 6;
			data = count;
			putText(final, "6M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
			return data;
		}
		else if (isStop == 5)
		{
			count = 5;
			data = count;
			putText(final, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
			return data;
		}
		else if (isStop == 4)
		{
			count = 4;
			data = count;
			putText(final, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop = 100;
			return data;
		}
		else if (isStop == 3)
		{
			count = 3;
			data = count;
			putText(final, "3M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop = 100;
			return data;	
		}
		else if (isStop == 2)
		{
			count = 2;
			data = count;
			putText(final, "2M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			isStop = 100;
			return data;
		}

		else if (isStop == 100)
		{
			count = 100;
			data = count;
			putText(final, "Go", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			return data;
		}

	
}

/**
 * @brief 마우스 왼쪽 버튼을 누르면 화면에서 마우스 커서의 위치가 어디인지 알려줌
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 */
void WeAreVision::on_mouse(int event, int x, int y, int flags, void *)
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
 * @brief 마우스 왼쪽 버튼을 누르면 화면에서 마우스 커서의 위치에서의 HSV값을 알려줌
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param param 
 */
void WeAreVision::mouse_callback(int event, int x, int y, int flags, void *param)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		Vec3b color_pixel = img_color.at<Vec3b>(y, x);

		Mat bgr_color = Mat(1, 1, CV_8UC3, color_pixel);

		Mat hsv_color;
		cvtColor(bgr_color, hsv_color, COLOR_BGR2HSV);

		H = hsv_color.at<Vec3b>(0, 0)[0];
		S = hsv_color.at<Vec3b>(0, 0)[1];
		V = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "H= " << H << endl;
		cout << "S= " << S << endl;
		cout << "V = " << V << "\n"
			 << endl;

		H = H - 200;
		S = S - 50;
		V = V - 50;

		if (H < 0)
			H = 0;

		if (S < 0)
			S = 0;

		if (V < 0)
			V = 0;
	}
}
