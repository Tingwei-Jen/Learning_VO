#include <iostream>
#include <opencv2/core/core.hpp>

#include "pinhole_camera.h"
#include "visual_odometry.h"

using namespace std;
using namespace cv;

int main()
{

	PinholeCamera *pinhole_camera = new PinholeCamera(640, 480, 618.854, 618.854, 320, 240, 0.1, 0.2, 0.3, 0.4, 0.5);
	Visual_Odometry VO(pinhole_camera);

    cv::Mat img1 = cv::imread("/home/server01/LearningVO/data/1.jpg",0);
    cv::Mat img2 = cv::imread("/home/server01/LearningVO/data/2.jpg",0);
    cv::Mat img3 = cv::imread("/home/server01/LearningVO/data/3.jpg",0);

	VO.addImage(img1, 1);
	VO.addImage(img2, 2);
	VO.addImage(img3, 3);

	cv::Mat R = VO.getCurrentR();
	std::cout<<"R:"<<R<<std::endl;
	return 0;
}