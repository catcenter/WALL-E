#include "SAMCAMLib.h"
int basss = 1;
int* baseLinez = &basss;
bool use_fullDP = true;

int SADWindowSizePos;
int SADWindowSize = 5;
int NumberOfDisparitiesPos;
int NumberOfDisparities = 16*13;
int preFilterCapPos;
int preFilterCap = 16;
int preFilterSizePos;
int preFilterSize = 5;
int minDisparityPos;
int minDisparity = 0;
int uniquenessRatioPos = 0;
int speckleRangePos = 1;
int speckleWindowSizePos = 0;
int disp12MaxDiffPos;
int disp12MaxDiff = 100;
int firstSmoothnessParameterPos = 172*4;
int secondSmoothnessParameter = 172*4*4;

int main()
{
	int IMAGE_HEIGHT = 240;
	int IMAGE_WIDTH  = 320;
	int BThreshMin=0,BThreshMax=255,RThreshMin=0,RThreshMax=255;
	//namedWindow(MainWindowName,WINDOW_AUTOSIZE);
	//namedWindow(TrackBarsWindow,WINDOW_NORMAL);
	system("v4l2-ctl -d /dev/video1 -c focus_auto=0");
	system("v4l2-ctl -d /dev/video2 -c focus_auto=0");
	system("v4l2-ctl -d /dev/video1 -c focus_absolute=0");
	system("v4l2-ctl -d /dev/video2 -c focus_absolute=0");
	system("v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0");
	system("v4l2-ctl -d /dev/video2 -c white_balance_temperature_auto=0");
	system("v4l2-ctl -d /dev/video1 -c sharpness=50");
	system("v4l2-ctl -d /dev/video2 -c sharpness=50");
	Mat img1, img2, g1, g2, disparity,disparity8;
	Mat Rectified(IMAGE_HEIGHT, 2*IMAGE_WIDTH, CV_8U);
	Mat Q,Map1_Left,Map2_Left,Map1_Right,Map2_Right;
	Mat_<float> depth(IMAGE_HEIGHT,IMAGE_WIDTH);
	vector<Mat> BGR;
	vector<Point> centers_f1;
	vector<Point> centers_f2;
	FileStorage wallE_Configuration("wallE.yml",FileStorage::READ);
	wallE_Configuration["Q"] >> Q;
	wallE_Configuration["LMap1"] >> Map1_Left;
	wallE_Configuration["LMap2"] >> Map2_Left;
	wallE_Configuration["RMap1"] >> Map1_Right;
	wallE_Configuration["RMap2"] >>  Map2_Right;
	VideoCapture cam1(1);
	VideoCapture cam2(2);
	cam1.set( CV_CAP_PROP_FRAME_WIDTH,IMAGE_WIDTH);
	cam1.set( CV_CAP_PROP_FRAME_HEIGHT,IMAGE_HEIGHT);
	cam2.set( CV_CAP_PROP_FRAME_WIDTH,IMAGE_WIDTH);
	cam2.set( CV_CAP_PROP_FRAME_HEIGHT,IMAGE_HEIGHT);
	Mat IBThresh,IRThresh;
	while((!(waitKey(10)==1048603)) && (cam1.isOpened() && cam2.isOpened()))
	{
		namedWindow("Calibration Parameters",WINDOW_NORMAL);
		createTrackbar("Red Minimum","Calibration Parameters",&RThreshMin,255,&RThreshMinTrackBarCallBack);
		createTrackbar("Red Maximum","Calibration Parameters",&RThreshMax,255,&RThreshMaxTrackBarCallBack);
		createTrackbar("Blue Minimum","Calibration Parameters",&BThreshMin,255,&BThreshMinTrackBarCallBack);
		createTrackbar("Blue Maximum","Calibration Parameters",&BThreshMax,255,&BThreshMaxTrackBarCallBack);
		cam1 >> img1;
		cam2 >> img2;
		cvtColor(img1, g1, CV_BGR2GRAY);
		cvtColor(img2, g2, CV_BGR2GRAY);
		remap(g1, g1, Map1_Left, Map2_Left, INTER_LINEAR);
		remap(g2, g2, Map1_Right, Map2_Right, INTER_LINEAR);
		g1.copyTo(Rectified(Range::all(),Range(0,IMAGE_WIDTH)));
		g2.copyTo(Rectified(Range::all(),Range(IMAGE_WIDTH,IMAGE_WIDTH*2)));
		split(img1,BGR);
		GaussianBlur(BGR[1]-0.3*g1,BGR[1],Size(31,31),0.1);
		GaussianBlur(BGR[2]-g1,BGR[2],Size(31,31),0.1);
		inRange(BGR[1],Scalar(BThreshMin),Scalar(BThreshMax),IBThresh);
		inRange(BGR[2],Scalar(RThreshMin),Scalar(RThreshMax),IRThresh);
		for(int y = 0; y < Rectified.rows; y += 20) line(Rectified, Point(0, y), Point(Rectified.cols, y), Scalar(0, 0, 255));
		if (false)
	    	{
			StereoBM sbm;
			sbm.state->SADWindowSize = SADWindowSize;
			sbm.state->numberOfDisparities = NumberOfDisparities;
			sbm.state->preFilterSize = preFilterSize;
			sbm.state->preFilterCap = preFilterCap;
			sbm.state->minDisparity = minDisparity;
			sbm.state->textureThreshold = 507;
			sbm.state->uniquenessRatio = uniquenessRatioPos;
			sbm.state->speckleWindowSize = speckleWindowSizePos;
			sbm.state->speckleRange = speckleRangePos;
			sbm.state->disp12MaxDiff = disp12MaxDiff;
			sbm(g1, g2, disparity);
    		}
	    	else if (true)
		{
			StereoSGBM sbm;
			sbm.SADWindowSize = SADWindowSize;
			sbm.numberOfDisparities = NumberOfDisparities;
			sbm.preFilterCap = preFilterCap;
			sbm.minDisparity = minDisparity;
			sbm.uniquenessRatio = uniquenessRatioPos;
			sbm.speckleWindowSize = speckleWindowSizePos;
			sbm.speckleRange = speckleRangePos;
			sbm.disp12MaxDiff = disp12MaxDiff;
			sbm.fullDP = use_fullDP;
			sbm.P1 = firstSmoothnessParameterPos;
			sbm.P2 = secondSmoothnessParameter;
			sbm(g1, g2, disparity);
		}
		disparity = disparity/16.0;
		disparity.convertTo(disparity,CV_32F);
		morphOps(IBThresh);
		morphOps(IRThresh);
		centers_f1=findcenters(IBThresh);
		centers_f2=findcenters(IRThresh);
		for(int y=0; y<disparity.rows; ++y)
		{
				for(int x=0; x<disparity.cols; ++x)
				{
						depth(y,x)=15.0*2.25/abs(disparity.at<float>(y,x));
				}
		}
		if (centers_f1.size()>0)
		{
			for (unsigned int k=0;k<centers_f1.size();k++)
			{
				putText(img1,intToString(depth(centers_f1[k])),centers_f1[k],FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));
				Size TextSize1 =  getTextSize(intToString(depth(centers_f1[k])),FONT_HERSHEY_SIMPLEX,0.5,0.5,baseLinez);
				//rectangle(img1,Rect(centers_f1[k],TextSize1),Scalar(0,255,255),1,CV_FILLED);
			}
		}
		if (centers_f2.size()>0)
		{
			for (unsigned int l=0;l<centers_f2.size();l++)
			{
				putText(img1,intToString(depth(centers_f2[l])),centers_f2[l],FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));
				Size TextSize2 =  getTextSize(intToString(depth(centers_f2[l])),FONT_HERSHEY_SIMPLEX,0.5,0.5,baseLinez);
				//rectangle(img1,Rect(centers_f2[l],TextSize2),Scalar(0,255,255),1,CV_FILLED);
			}
		}
		vector<double> position=trianglate(depth(centers_f2[0]),depth(centers_f1[0]),0.4);
		printf("Position is %f , %f \n",position[0],position[1]);
		disparity.convertTo(disparity,CV_32F);
		GaussianBlur(disparity,disparity,Size(31,31),0.1);
		normalize(disparity,disparity8,0,255,CV_MINMAX,CV_8U);
		inpaint(disparity8,(disparity8==0),disparity8,10,INPAINT_NS);
		//imshow(MainWindowName,disparity8);
		imshow("Rectified Pair",Rectified);
		imshow("Tracked Marker", img1);
		imshow("Threshed Blue",IBThresh);
		imshow("Threshed Red",IRThresh);
		centers_f1.erase(centers_f1.begin(),centers_f1.end());
		centers_f2.erase(centers_f2.begin(),centers_f2.end());
		waitKey(1);
		position.clear();
	}
}
