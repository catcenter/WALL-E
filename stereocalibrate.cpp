#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    system("v4l2-ctl -d /dev/video1 -c focus_auto=0");
	system("v4l2-ctl -d /dev/video2 -c focus_auto=0");
    system("v4l2-ctl -d /dev/video1 -c focus_absolute=0");
    system("v4l2-ctl -d /dev/video2 -c focus_absolute=-0.35");
	system("v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0");
	system("v4l2-ctl -d /dev/video2 -c white_balance_temperature_auto=0");
	system("v4l2-ctl -d /dev/video1 -c sharpness=255");
    system("v4l2-ctl -d /dev/video2 -c sharpness=255");
    
    int numBoards = 15; //Number of different poses that will be used for calibration
    int board_w = 10; //Number of horizontal corners for the pattern used
    int board_h = 7; //Number of vertical corners for the pattern used
    int IMAGE_WIDTH = 320;
    int IMAGE_HEIGHT = 240;
    Size board_sz = Size(board_w, board_h);
    int board_n = board_w*board_h; //Number of squares inside the pattern

    vector<vector<Point3f> > object_points; // These are the 3D object points
    vector<vector<Point2f> > imagePoints1, imagePoints2; //The corresponding image points captured by both cameras
    vector<Point2f> corners1, corners2; //Detected corners in the stereo-pair

    // Here we define the object points assuming that the pattern is fixed and placed at the origin, so Z=0
    vector<Point3f> obj;
    for (int j=0; j<board_n; j++)
    {
        obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
    }

    Mat img1, img2, gray1, gray2;
	Mat stereoPair(IMAGE_HEIGHT, 2*IMAGE_WIDTH, CV_8UC3);
    VideoCapture cap1 = VideoCapture(1);
    VideoCapture cap2 = VideoCapture(2);
    cap1.set(CV_CAP_PROP_FRAME_WIDTH,IMAGE_WIDTH);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,IMAGE_HEIGHT);
    cap2.set(CV_CAP_PROP_FRAME_WIDTH,IMAGE_WIDTH);
    cap2.set(CV_CAP_PROP_FRAME_HEIGHT,IMAGE_HEIGHT);
    int success = 0, k = 0;
    bool found1 = false, found2 = false;

    while (success < numBoards)
    {
        cap1.grab();
        cap2.grab();
        cap1.retrieve(img1);
        cap2.retrieve(img2);
        //resize(img1, img1, Size(320, 280));
        //resize(img2, img2, Size(320, 280));
        cvtColor(img1, gray1, CV_BGR2GRAY);
        cvtColor(img2, gray2, CV_BGR2GRAY);

        found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
        found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

        if (found1)
        {
            cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray1, board_sz, corners1, found1);
        }

        if (found2)
        {
            cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray2, board_sz, corners2, found2);
        }
        
		gray1.copyTo(stereoPair(Range::all(),Range(0,IMAGE_WIDTH)));
		gray2.copyTo(stereoPair(Range::all(),Range(IMAGE_WIDTH,IMAGE_WIDTH*2)));
        imshow("Stereo Pair", stereoPair);

        if (found1 && found2)
        {
            k = waitKey(0);
            cout << "The value of k is: " << k << endl;
        }
        if (k == 1048603)
        {
            break;
        }
        if (k == 1048608 && found1 !=0 && found2 != 0)
        {
            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            object_points.push_back(obj);
            cout << "Corners stored\n" << endl;
            success++;

            if (success >= numBoards)
            {
                break;
            }
        }
        waitKey(1);
    }

    destroyAllWindows();
    printf("Starting Calibration\n");
    Mat CM1 = Mat(3, 3, CV_64FC1);
    Mat CM2 = Mat(3, 3, CV_64FC1);
    Mat D1, D2;
    Mat R, T, E, F;

    stereoCalibrate(object_points, imagePoints1, imagePoints2, 
                    CM1, D1, CM2, D2, img1.size(), R, T, E, F,
                    cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), 
                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

    FileStorage fs1("trial.yml", FileStorage::WRITE);
    fs1 << "CM1" << CM1;
    fs1 << "CM2" << CM2;
    fs1 << "D1" << D1;
    fs1 << "D2" << D2;
    fs1 << "R" << R;
    fs1 << "T" << T;
    fs1 << "E" << E;
    fs1 << "F" << F;

    printf("Done Calibration\n");

    printf("Starting Rectification\n");

    Mat R1, R2, P1, P2, Q;
    stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
    fs1 << "R1" << R1;
    fs1 << "R2" << R2;
    fs1 << "P1" << P1;
    fs1 << "P2" << P2;
    fs1 << "Q" << Q;

    printf("Done Rectification\n");

    printf("Applying Undistort\n");

    Mat map1x, map1y, map2x, map2y;
    Mat imgU1, imgU2;

    initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CV_32FC1, map2x, map2y);

    printf("Undistort complete\n");
    
    cap1.set(CV_CAP_PROP_FRAME_WIDTH,IMAGE_WIDTH);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,IMAGE_HEIGHT);
    cap2.set(CV_CAP_PROP_FRAME_WIDTH,IMAGE_WIDTH);
    cap2.set(CV_CAP_PROP_FRAME_HEIGHT,IMAGE_HEIGHT);
    while(1)
    {    
        cap1 >> img1;
        cap2 >> img2;

        remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
        remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		imgU1.copyTo(stereoPair(Range::all(),Range(0,IMAGE_WIDTH)));
		imgU2.copyTo(stereoPair(Range::all(),Range(IMAGE_WIDTH,IMAGE_WIDTH*2)));
		for(int y = 0; y < stereoPair.rows; y += 20)
			line(stereoPair, Point(0, y), Point(stereoPair.cols, y), Scalar(0, 0, 255));
        k = waitKey(1);

        if(k==1048603)
        {
            break;
        }
    }

    cap1.release();
    cap2.release();

    return(0);
}
