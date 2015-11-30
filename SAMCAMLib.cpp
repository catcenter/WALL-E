/*
 * SAMCAMLib.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: mahmoud
 */

#include "SAMCAMLib.h"

string intToString(float number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}

vector<Point> findcenters(Mat threshold)
{
	Mat temp;
	threshold.copyTo(temp);
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> centers;
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
	if (hierarchy.size()>0)
	{
		int numObjects = hierarchy.size();
		if(numObjects<=15)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				if(area>100)
				{
					centers.push_back(Point(moment.m10/area,moment.m01/area));
				}
			}
		}
	}
	return centers;
}
void BThreshMinTrackBarCallBack(int BThresh,void*)
{

}
void BThreshMaxTrackBarCallBack(int BThresh,void*)
{

}

void RThreshMinTrackBarCallBack(int RThresh,void*)
{

}
void RThreshMaxTrackBarCallBack(int RThresh,void*)
{

}
vector<double> trianglate(double z1,double z2,double R)
{
	vector<double> position;
	double phi=acos((z1*z1+R*R-z2*z2)/(2*z1*R));
	position.push_back(z1*cos(phi)-R/2);
	position.push_back(z1*sin(phi));
	return position;
}
void morphOps(Mat &thresh)
{

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(15,15));
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}
