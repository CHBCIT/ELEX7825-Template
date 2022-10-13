#pragma once

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "uArm.h"

using namespace std;
using namespace cv;
using namespace dnn;

class CRobot
{
public:
	CRobot();
	~CRobot();

private:
	Size _image_size;
	Mat _canvas;

	////////////////////////////////////
  // LAB 3
	
	vector<vector<Mat>> _simple_robot;

	CCamera _virtualcam;

	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);
	
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void drawCoord(Mat& im, std::vector<Mat> coord3d);

	////////////////////////////////////
	// LAB 4

	////////////////////////////////////
	// LAB X

	//CuArm uarm;

public:
	////////////////////////////////////
	// Lab 3

	Mat createHT(Vec3d t, Vec3d r);
	void create_simple_robot();
	void draw_simple_robot();

	////////////////////////////////////
	// Lab 4

	////////////////////////////////////
	// Lab 5

};

