#ifndef DETECTOR_H
#define DETECTOR_H

// #include "stdafx.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;
using std::vector;

class FindCorners
{
public:
	FindCorners();
	FindCorners(Mat img);

	~FindCorners();

public:
	void detectCorners(Mat &Src, vector<Point> &resultCornors,float scoreThreshold);

private:
	//normal distribution
	float normpdf(float dist, float mu, float sigma);
	//get the minimum value
	void getMin(Mat src1, Mat src2, Mat &dst);
	//get the maximum value
	void getMax(Mat src1, Mat src2, Mat &dst);
	//Get gradient angles and weights
	void getImageAngleAndWeight(Mat img, Mat &imgDu, Mat &imgDv, Mat &imgAngle, Mat &imgWeight);
	//estimate edge orientations
	void edgeOrientations(Mat imgAngle, Mat imgWeight,int index);
	//find modes of smoothed histogram
	void findModesMeanShift(vector<float> hist, vector<float> &hist_smoothed, vector<pair<float, int>> &modes, float sigma);
	//score corners
	void scoreCorners(Mat img, Mat imgAngle, Mat imgWeight, vector<Point> &cornors, vector<int> radius, vector<float> &score);
	//compute corner statistics
	void cornerCorrelationScore(Mat img, Mat imgWeight, vector<Point2f> cornersEdge, float &score);
	//Find corners with sub-pixel accuracy
	void refineCorners(vector<Point> &cornors, Mat imgDu, Mat imgDv, Mat imgAngle, Mat imgWeight, float radius);
	//Generate kernel
	void createkernel(float angle1, float angle2, int kernelSize, Mat &kernelA, Mat &kernelB, Mat &kernelC, Mat &kernelD);
	//non-maximum suppression
	void nonMaximumSuppression(Mat& inputCorners, vector<Point>& outputCorners, float threshold, int margin, int patchSize);

private:
	vector<Point2f> templateProps;
	vector<int> radius;
	vector<Point> cornerPoints;
	std::vector<std::vector<float> > cornersEdge1;
	std::vector<std::vector<float> > cornersEdge2;
	std::vector<cv::Point* > cornerPointsRefined;

};

#endif // DETECTOR_H
