/*
 * SAMCAMLib.h
 *
 *  Created on: Nov 18, 2015
 *      Author: mahmoud
 */

#pragma once
#ifndef SAMCAMLIB_H_
#define SAMCAMLIB_H_
#ifndef VARIABLEDEFINITION_H_
#include "VariableDefinition.h"
#endif
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>

using namespace cv;
using namespace std;

void createTrackBarsAndButtons(int SGBMorBM);
string intToString(float number);
void morphOps(Mat &thresh);
void SADWindowSizePosTrackBarCallBack(int SADWindowSizePos,void*);
void NumberOfDisparitiesPosTrackBarCallBack(int NumberOfDisparitiesPos,void*);
void preFilterCapPosTrackBarCallBack(int preFilterCapPos,void*);
void preFilterSizePosTrackBarCallBack(int preFilterSizePos,void*);
void minDisparityPosTrackBarCallBack(int minDisparityPos,void*);
void disp12MaxDiffPosTrackBarCallBack(int disp12MaxDiffPos,void*);
void SmoothnessParametersPosTrackBarCallBack(int firstSmoothnessParameterPos,void*);
void uniquenessRatioPosTrackBarCallBack(int uniquenessRatioPos,void*);
void speckleWindowSizePosTrackBarCallBack(int speckleWindowSizePos,void*);
void speckleRangePosTrackBarCallBack(int speckleRangePos,void*);
vector<double> trianglate(double z1,double z2,double R);
double getMedian(Mat hist);
vector<Point> findcenters(Mat threshold);
void BThreshMinTrackBarCallBack(int BThresh,void*);
void BThreshMaxTrackBarCallBack(int BThresh,void*);
void RThreshMinTrackBarCallBack(int RThresh,void*);
void RThreshMaxTrackBarCallBack(int RThresh,void*);

#endif /* SAMCAMLIB_H_ */
