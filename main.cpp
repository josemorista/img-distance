#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include "opencv2/imgcodecs.hpp"
#include <math.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	
	VideoCapture cap(0);

	if (!cap.isOpened())
	{
		cout << "Cannot open your camera" << endl;
		return -1;
	}

	// Create a Control Window for trackbars
	namedWindow("Controlers for HSV");

	int iLowH = 102;
	int iHighH = 130;

	int iLowS = 76;
	int iHighS = 255;

	int iLowV = 35;
	int iHighV = 255;

	int fLowH = 25;
	int fHighH = 67;

	int fLowS = 52;
	int fHighS = 255;

	int fLowV = 72;
	int fHighV = 255;

	// Create Trackbars
	createTrackbar("LowH", "Controlers for HSV", &fLowH, 179);		
	createTrackbar("HighH", "Controlers for HSV", &fHighH, 179);

	createTrackbar("LowS", "Controlers for HSV", &fLowS, 255);		
	createTrackbar("HighS", "Controlers for HSV", &fHighS, 255);

	createTrackbar("LowV", "Controlers for HSV", &fLowV, 255);		
	createTrackbar("HighV", "Controlers for HSV", &fHighV, 255);


	while (true)
	{
		int fR = 0, iR = 0, iposX, iposY, fposX, fposY;
		Mat imgOriginal;
		cap.read(imgOriginal);

		Mat imgHSV;
		// Convert the captured frame from BGR to HSV
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		// Create the Thresholded Image
		Mat iThresholded, fThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), iThresholded);
		inRange(imgHSV, Scalar(fLowH, fLowS, fLowV), Scalar(fHighH, fHighS, fHighV), fThresholded);

		// Erode to remove noise
		erode(iThresholded, iThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)));
		erode(fThresholded, fThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)));

		// Calculate the Moments of the Thresholded Image
		Moments iMoments = moments(iThresholded);
		Moments fMoments = moments(fThresholded);

		// Calculate the Center of the Object
		double dM01 = iMoments.m01;
		double dM10 = iMoments.m10;
		double dArea = iMoments.m00;

		if (dArea > 50000) {	
			iposX = dM10 / dArea;
			iposY = dM01 / dArea;
			iR = sqrt((dArea / 255) / 3.14);
    			
			if (iposX >= 0 && iposY >= 0)
			{
				circle(imgOriginal, Point(iposX, iposY), iR, Scalar(0,0,255), 2);
			}
		}

		dM01 = fMoments.m01;
		dM10 = fMoments.m10;
		dArea = fMoments.m00;
    

		if (dArea > 50000) {
			fposX = dM10 / dArea;
			fposY = dM01 / dArea;
			fR = sqrt((dArea / 255) / 3.14);
			
			if (fposX >= 0 && fposY >= 0)
			{
				circle(imgOriginal, Point(fposX, fposY), fR, Scalar(0,255,0), 2);
			}
		}

		// Calculate the Distance between the Object and the camera
		/*int realR = 4;
		int f = -10;
		int fpx = 750;
		int d = (realR*fpx) / R + f;
		cout << "Distance = " << d << endl;
		cout << endl;*/

		// Calculate distance between the two circles
		if (fR != 0 && iR != 0) {
			int distance = sqrt(((fposX - iposX) * (fposX - iposX)) + ((fposY - iposY) * (fposY - iposY)));
			cout << "Distance = " << distance << "px" << endl;
		}

		imshow("Tracking", imgOriginal);

		// Wait for key is pressed then break loop
		if (waitKey(5) == 27)			//ESC 27, ENTER 13, SPACE 32
		{
			break;
		}
	}

	return 0;
}