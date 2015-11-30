#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <string>

using namespace cv;
using namespace std;

int main ()
{
	Size ImageSize(320,240);
	Mat CamMat_left,CamMat_right,ProjMat_left,ProjMat_right,RotMat_left,RotMat_right,Q,EssentMat,FundMatrix,R,T,DistCoeffs_left,DistCoeffs_right;
	Mat Map1_Left,Map2_Left, Map1_Right, Map2_Right;
	FileStorage trial("trial.yml",FileStorage::READ);
	trial["Q"] >> Q;
	trial["P1"] >> ProjMat_left;
	trial["P2"] >> ProjMat_right;
	trial["CM1"] >> CamMat_left;
	trial["CM2"] >> CamMat_right;
	trial["R1"] >> RotMat_left;
	trial["R2"] >> RotMat_right;
	trial["E"] >> EssentMat;
	trial["F"] >> FundMatrix;
	trial["R"] >> R;
	trial["T"] >> T;
	trial["D1"] >> DistCoeffs_left;
	trial["D2"] >> DistCoeffs_right;
	trial.release();
	FileStorage wallE("wallE.yml",FileStorage::WRITE);
	initUndistortRectifyMap(CamMat_left, DistCoeffs_left, RotMat_left, ProjMat_left, ImageSize, CV_16SC2, Map1_Left,Map2_Left);
	initUndistortRectifyMap(CamMat_right, DistCoeffs_right, RotMat_right, ProjMat_right, ImageSize, CV_16SC2, Map1_Right,Map2_Right);
	wallE << "Q" << Q;
	wallE <<"P1" << ProjMat_left;
	wallE << "P2"  << ProjMat_right;
	wallE << "CM1"  << CamMat_left;
	wallE << "CM2"  << CamMat_right;
	wallE << "R1"  << RotMat_left;
	wallE << "R2"  << RotMat_right;
	wallE << "E"  << EssentMat;
	wallE << "F"  << FundMatrix;
	wallE << "R"  << R;
	wallE << "T"  << T;
	wallE << "D1"  << DistCoeffs_left;
	wallE << "D2"  << DistCoeffs_right;
	wallE << "LMap1" << Map1_Left;
	wallE << "LMap2" << Map2_Left;
	wallE << "RMap1" << Map1_Right;
	wallE << "RMap2" << Map2_Right;
	wallE.release();
}