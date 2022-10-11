#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1000, 600);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

	Vec3d t = (0, 0, 0);
	createHT(t, t);

  	///////////////////////////////////////////////
	// uArm setup

	//uarm.init_com("COM4");
	//uarm.init_robot();
}

CRobot::~CRobot()
{
}

// Create Homogeneous Transformation Matrix
Mat CRobot::createHT(Vec3d t, Vec3d r)
{
	Mat rot = Mat::zeros(3, 3, CV_32S);
	Mat trn = Mat::zeros(3, 1, CV_32S);
	Mat T = Mat::zeros(4, 4, CV_32S);
	T.at<int>(3, 3) = 1;

	trn = t;	// Assigns t to translation matrix

	int g = r[0];
	int b = r[1];
	int a = r[2];
	
	rot.at<int>(0, 0) = cos(a)*cos(b);	//cacB
	rot.at<int>(0, 1) = cos(a) * sin(b) * sin(g) - sin(a) * cos(g); //casBsg - sacg
 	rot.at<int>(0, 2) = cos(a) * sin(b) * cos(g) + sin(a) * sin(g); //casBcg + sasg

	rot.at<int>(1, 0) = sin(a) * cos(b);//sacB
	rot.at<int>(1, 1) = sin(a) * sin(b) * sin(g) + cos(a) * cos(g);//sasBsg + cacg
	rot.at<int>(1, 2) = sin(a) * sin(b);//sasBcg - casg
	
	rot.at<int>(2, 0) = -sin(b);//-sB
	rot.at<int>(2, 1) = cos(b)*sin(g);//cBsg
	rot.at<int>(2, 2) = cos(b)*cos(g);//cBcg

	

	for (int i = 0; i < 4; i++) T.at<int>(i, 3) = trn.at<int>(i);

	return T;
}

std::vector<Mat> CRobot::createBox(float w, float h, float d)
{
	std::vector <Mat> box;

	// The 8 vertexes, origin at the center of the box
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	return box;
}

std::vector<Mat> CRobot::createCoord()
{
	std::vector <Mat> coord;

	float axis_length = 0.05;

	coord.push_back((Mat1f(4, 1) << 0, 0, 0, 1)); // O
	coord.push_back((Mat1f(4, 1) << axis_length, 0, 0, 1)); // X
	coord.push_back((Mat1f(4, 1) << 0, axis_length, 0, 1)); // Y
	coord.push_back((Mat1f(4, 1) << 0, 0, axis_length, 1)); // Z

	return coord;
}

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d)
{
	Point2f O, X, Y, Z;

	_virtualcam.transform_to_image(coord3d.at(0), O);
	_virtualcam.transform_to_image(coord3d.at(1), X);
	_virtualcam.transform_to_image(coord3d.at(2), Y);
	_virtualcam.transform_to_image(coord3d.at(3), Z);

	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
}

void CRobot::transformPoints(std::vector<Mat>& points, Mat T)
{
	for (int i = 0; i < points.size(); i++)
	{
		points.at(i) = T * points.at(i);
	}
}

void CRobot::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	_virtualcam.transform_to_image(box3d, box2d);

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1);
	}
}

void CRobot::create_simple_robot()
{

}

void CRobot::draw_simple_robot()
{
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	std::vector<Mat> O = createCoord();
	drawCoord(_canvas,O);

	_virtualcam.update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
}
