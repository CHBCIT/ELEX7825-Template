#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include "cvui.h"

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1000, 600);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

	init();
}

CRobot::~CRobot()
{
}

void CRobot::init()
{
	// reset variables
	_do_animate = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB3
////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Create Homogeneous Transformation Matrix
Mat CRobot::createHT(Vec3d t, Vec3d r)
{
	return (Mat1f(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
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

	// If Virtual Camera
	_virtualcam.transform_to_image(box3d, box2d);
	// If Real Camera
	//_realcam.transform_to_image();

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1);
	}
}

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d)
{
	Point2f O, X, Y, Z;

	// If Virtual Camera
	_virtualcam.transform_to_image(coord3d.at(0), O);
	_virtualcam.transform_to_image(coord3d.at(1), X);
	_virtualcam.transform_to_image(coord3d.at(2), Y);
	_virtualcam.transform_to_image(coord3d.at(3), Z);

	// If Real Camera
	//_realcam.transform_to_image();

	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
}

void CRobot::create_simple_robot()
{

}

void CRobot::draw_simple_robot()
{
	Mat im;

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	std::vector<Mat> O = createCoord();

	//_realcam.get_image(im);
	//im.copyTo(_canvas);

	_virtualcam.update_settings(_canvas);
	//_realcam.update_settings(_canvas);
	update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB4
////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB5
////////////////////////////////////////////////////////////////////////////////////////////////////////

void CRobot::update_settings(Mat& im)
{
	Point _setting_window;

	_setting_window.x = im.size().width - 200;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 20;

	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "Animate"))
	{
		init();
		_do_animate = 1;
	}

	if (_do_animate != 0)
	{
		int step_size = 5;
		if (_do_animate == 1)
		{
			// state 1
			if (1) { _do_animate = 2; }
		}
		else if (_do_animate == 2)
		{
			// state 2
			if (1) { _do_animate = 3; }
		}
		else if (_do_animate == 3) {
			if (1) { _do_animate = 0; init(); }
		}
	}

	cvui::update();
}

void CRobot::draw()
{
	_virtualcam.update_settings(_canvas);
	update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
}
