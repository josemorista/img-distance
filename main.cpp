#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <opencv2/imgcodecs.hpp>
#include <math.h>

// Consts
#define realRadious 1.5 // This value represents our objects real radious
#define focal 450 // This value represents our camera focus
#define minimumArea 1000 // this represents the minimum area to be considered an object

using namespace cv;
using namespace std;

// Here comes our classes
class TrackingObject
{
	public:

		int lowH, lowS, lowV, highH, highS, highV;
		int x, y;
		double distanceFromCamera, realR, R, minArea;

		TrackingObject (double realR = realRadious, double minArea = minimumArea) {
			this->x = 0;
			this->y = 0;
			this->R = 0;
			this->realR = realR;
			this->minArea = minArea;
		}

		void setHsvFilter (int lowH, int lowS, int lowV, int highH, int highS, int highV) {
			this->lowH=lowH;
			this->lowS=lowS;
			this->lowV=lowV;

			this->highH=highH;
			this->highS=highS;
			this->highV=highV;
		}

		void calculateCoordinatesWithMoments (Moments m) {
			
			double dM01 = m.m01;
			double dM10 = m.m10;
			double dArea = m.m00;

			if (dArea < this->minArea) return;

			this->x = dM10 / dArea;
			this->y = dM01 / dArea;
			
			this->R = sqrt((dArea / 255) / 3.14);

			// Calculate the Distance between the Object and the camera
			this->distanceFromCamera = (focal * this->realR) / this->R;
		}

		void drawCircle (cv::Mat& img, Scalar color = Scalar(0, 255, 0)) {
			if (this->x >= 0 && this->y >= 0) {
				circle(img, Point(this->x, this->y), this->R, color, 2);
			}
		}

		void drawLineBetweenObject (cv::Mat& img, TrackingObject obj, Scalar color = Scalar(255, 0, 0)) {
			line(img, Point(this->x, this->y), Point(obj.x, obj.y), color, 2);
		}

		double calculateDistanceBetweenObjct (TrackingObject obj) {
			if (this->R != 0 && obj.R != 0)
			{
				double distanceBetweenPx = sqrt(pow((obj.x - this->x), 2) + pow((obj.y - this->y), 2));
				double distanceBetweenCm = distanceBetweenPx * this->distanceFromCamera / focal;
				return distanceBetweenCm;
			}
			return 0;
		}

};

// Finally our main arrives!
int main (int argc, char **argv)
{
	// Video capture variable
	VideoCapture cap(0);

	if (!cap.isOpened())
	{
		cout << "Sorry, cannot open your camera :(" << endl;
		return -1;
	}

	
	// Our objects
	TrackingObject initialObj, finalObj;

	// Creates windows of trackbars to help the user to calibrate the HSV filters
	namedWindow("Controlers for initialHSVfilter");
	namedWindow("Controlers for finalHSVfilter");

	initialObj.setHsvFilter(100, 76, 0, 130, 255, 255);
	finalObj.setHsvFilter(28, 69, 0, 35, 255, 255);

	// Create Trackbars
	createTrackbar("LowH", "Controlers for initialHSVfilter", &initialObj.lowH, 179);
	createTrackbar("HighH", "Controlers for initialHSVfilter", &initialObj.highH, 179);
	createTrackbar("LowS", "Controlers for initialHSVfilter", &initialObj.lowS, 255);
	createTrackbar("HighS", "Controlers for initialHSVfilter", &initialObj.highS, 255);

	createTrackbar("LowH", "Controlers for finalHSVfilter", &finalObj.lowH, 179);
	createTrackbar("HighH", "Controlers for finalHSVfilter", &finalObj.highH, 179);
	createTrackbar("LowS", "Controlers for finalHSVfilter", &finalObj.lowS, 255);
	createTrackbar("HighS", "Controlers for finalHSVfilter", &finalObj.highS, 255);

	while (true)
	{

		Mat imgOriginal;
		// Capture image from camera
		cap.read(imgOriginal);

		Mat imgHSV;
		// Convert the captured frame from BGR to HSV
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		// Create and filter the thresholdeds Images
		Mat iThresholded, fThresholded;

		inRange(imgHSV, Scalar(initialObj.lowH, initialObj.lowS, initialObj.lowV), Scalar(initialObj.highH, initialObj.highS, initialObj.highV), iThresholded);
		inRange(imgHSV, Scalar(finalObj.lowH, finalObj.lowS, finalObj.lowV), Scalar(finalObj.highH, finalObj.highS, finalObj.highV), fThresholded);

		// Erode to remove noise
		erode(iThresholded, iThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)));
		erode(fThresholded, fThresholded, getStructuringElement(MORPH_RECT, Size(5, 5)));

		// Calculate the Moments of the Thresholdeds Images
		Moments iMoments = moments(iThresholded);
		Moments fMoments = moments(fThresholded);

		// Calculate what we need with the moments
		initialObj.calculateCoordinatesWithMoments(iMoments);
		finalObj.calculateCoordinatesWithMoments(fMoments);
		
		// Draw the two circles
		initialObj.drawCircle(imgOriginal);
		finalObj.drawCircle(imgOriginal);

		initialObj.drawLineBetweenObject(imgOriginal, finalObj);

		double distance = initialObj.calculateDistanceBetweenObjct(finalObj);
		cout << "Distance between = " << distance << "cm" << endl;
		
		// Show result to user
		imshow("Result!", imgOriginal);

		// Wait for key is pressed then break loop
		if (waitKey(5) == 27) //ESC == 27
		{
			break;
		}
	}

	return 0;
}