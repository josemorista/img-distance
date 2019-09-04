#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <opencv2/imgcodecs.hpp>
#include <math.h>
#include <string>

// Consts
#define realRadious 1.25 // This value represents our objects real radious
#define focal 1					 // This value represents our camera focus
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

	TrackingObject(double realR = realRadious, double minArea = minimumArea)
	{
		this->x = 0;
		this->y = 0;
		this->R = 0;
		this->realR = realR;
		this->minArea = minArea;
	}

	void setHsvFilter(int lowH, int lowS, int lowV, int highH, int highS, int highV)
	{
		this->lowH = lowH;
		this->lowS = lowS;
		this->lowV = lowV;

		this->highH = highH;
		this->highS = highS;
		this->highV = highV;
	}

	void calculateCoordinatesWithMoments(Moments m)
	{

		double dM01 = m.m01;
		double dM10 = m.m10;
		double dArea = m.m00;

		// Check the min area
		if (dArea < this->minArea)
			return;

		// Calculate the centroid of the object
		this->x = dM10 / dArea;
		this->y = dM01 / dArea;

		// Calculate our object radious
		this->R = sqrt((dArea / 255) / 3.14);

		// Calculate the distance between the Object and the camera
		this->distanceFromCamera = (focal * this->realR) / this->R;
	}

	void drawCircle(cv::Mat &img, Scalar color = Scalar(0, 255, 0))
	{
		if (this->x >= 0 && this->y >= 0)
		{
			circle(img, Point(this->x, this->y), this->R, color, 2);
		}
	}

	void drawLineBetweenObject(cv::Mat &img, TrackingObject obj, Scalar color = Scalar(255, 0, 0))
	{
		line(img, Point(this->x, this->y), Point(obj.x, obj.y), color, 2);
	}

	double calculateDistanceBetweenObjct(TrackingObject obj)
	{
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
int main(int argc, char **argv)
{

	String path;
	String readType;

	if (argc > 1)
	{
		path = argv[1];
		readType = argv[2];
	}
	else
	{
		char resp;
		cout << "Do you want to process a static file?(y/n)" << endl;
		cin >> resp;
		if (resp == 'y')
		{
			cout << "What's the path?" << endl;
			cin >> path;
			cout << "Is it a video?(y/n)" << endl;
			cin >> resp;
			if (resp == 'y')
			{
				readType = "video";
			}
			else
			{
				readType = "img";
			}
		}
	}

	VideoCapture myCap(path);

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

	// Here we choose the interval to get the objects by their HSV color, see the hsvMap to get your desired values!
	initialObj.setHsvFilter(160, 122, 0, 180, 255, 255);
	finalObj.setHsvFilter(2, 110, 0, 33, 255, 255);

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

		if (path.length() > 0)
		{
			if (readType == "img")
				imgOriginal = imread(path);
			if (readType == "video")
			{
				if (!myCap.isOpened())
				{
					exit(0);
				}
				myCap.read(imgOriginal);
			}
		}
		else
		{
			cap.read(imgOriginal);
		}

		Mat imgHSV;
		// Convert the captured frame from BGR to HSV
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		// Create and filter the thresholdeds Images
		Mat iThresholded, fThresholded;

		inRange(imgHSV, Scalar(initialObj.lowH, initialObj.lowS, initialObj.lowV), Scalar(initialObj.highH, initialObj.highS, initialObj.highV), iThresholded);
		inRange(imgHSV, Scalar(finalObj.lowH, finalObj.lowS, finalObj.lowV), Scalar(finalObj.highH, finalObj.highS, finalObj.highV), fThresholded);

		// Erode the images to remove some noise
		erode(iThresholded, iThresholded, getStructuringElement(MORPH_RECT, Size(8, 8)));
		dilate(iThresholded, iThresholded, getStructuringElement(MORPH_RECT, Size(8, 8)));
		erode(fThresholded, fThresholded, getStructuringElement(MORPH_RECT, Size(8, 8)));
		dilate(fThresholded, fThresholded, getStructuringElement(MORPH_RECT, Size(8, 8)));

		// Calculate the Moments of the Thresholdeds Images
		Moments iMoments = moments(iThresholded);
		Moments fMoments = moments(fThresholded);

		// Calculate what we need with the moments
		initialObj.calculateCoordinatesWithMoments(iMoments);
		finalObj.calculateCoordinatesWithMoments(fMoments);

		// Draw the two circles
		initialObj.drawCircle(imgOriginal);
		finalObj.drawCircle(imgOriginal);

		// Draw a line between the objects
		initialObj.drawLineBetweenObject(imgOriginal, finalObj);

		// Calcultate the distance between the objects
		double distance = finalObj.calculateDistanceBetweenObjct(initialObj);
		if (distance > 0)
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