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
#define JAN_OFFSET 0
using namespace cv;
using namespace std;

class Window
{
	char *m_name;

public:
	Window(char *name, int tam_ja, int x, int y)
	{
		m_name = name;
		namedWindow(m_name, WINDOW_NORMAL & CV_GUI_NORMAL);
		moveWindow(m_name, tam_ja * x + JAN_OFFSET, tam_ja * y + JAN_OFFSET);
		resizeWindow(m_name, tam_ja, tam_ja);
	}
	void imshow(Mat img)
	{
		cv::imshow(m_name, img);
	}
	void createTrackbar(char *trackName, int *var, int max_val)
	{
		cv::createTrackbar(trackName, m_name, var, max_val);
	}
};
// Here comes our classes
class TrackingObject
{
public:
	int lowH, lowS, lowV, highH, highS, highV;
	int x, y;
	int abertura, fechamento;
	double distanceFromCamera, realR, R, minArea;

	TrackingObject(double realR = realRadious, double minArea = minimumArea)
	{
		this->abertura = 8;
		this->fechamento = 35;
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

	if (argc > 2)
	{
		path = argv[1];
		readType = argv[2];
	}
	else
	{
		char resp;
		cout << "Do you want to process a static file?(y/n)" << endl;
		//cin >> resp;
		if (resp == 'y')
		{
			cout << "What's the path?" << endl;
			//cin >> path;
			cout << "Is it a video?(y/n)" << endl;
			//cin >> resp;
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
	Window controllerInitialHSVfilter = Window("Controlers for initialHSVfilter", 450, 0, 0);

	// Here we choose the interval to get the objects by their HSV color, see the hsvMap to get your desired values!
	initialObj.setHsvFilter(160, 122, 0, 200, 213, 255);
	finalObj.setHsvFilter(57, 93, 0, 77, 221, 255);

	// Create Trackbars
	controllerInitialHSVfilter.createTrackbar("I_LowH", &initialObj.lowH, 255);
	controllerInitialHSVfilter.createTrackbar("I_HighH", &initialObj.highH, 255);
	controllerInitialHSVfilter.createTrackbar("I_LowS", &initialObj.lowS, 255);
	controllerInitialHSVfilter.createTrackbar("I_HighS", &initialObj.highS, 255);

	controllerInitialHSVfilter.createTrackbar("F_LowH", &finalObj.lowH, 255);
	controllerInitialHSVfilter.createTrackbar("F_HighH", &finalObj.highH, 255);
	controllerInitialHSVfilter.createTrackbar("F_LowS", &finalObj.lowS, 255);
	controllerInitialHSVfilter.createTrackbar("F_HighS", &finalObj.highS, 255);

	controllerInitialHSVfilter.createTrackbar("Abertura_Tam", &finalObj.abertura, 120);
	controllerInitialHSVfilter.createTrackbar("Fechamento_Tam", &finalObj.fechamento, 120);

	Window result("result", 400, 0, 1);
	Window thresh_laranja("thresh_laranja", 400, 1, 0);
	Window thresh_rosa("thresh_rosa", 400, 2, 0);
	Window tratada_thresh_laranja("Tratada_thresh_laranja", 400, 1, 1);
	Window tratada_thresh_rosa("Tratada_thresh_rosa", 400, 2, 1);

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

		thresh_laranja.imshow(iThresholded);
		thresh_rosa.imshow(fThresholded);

		// Erode the images to remove some noise
		if (finalObj.abertura > 0)
		{
			erode(iThresholded, iThresholded, getStructuringElement(MORPH_RECT, Size(finalObj.abertura, finalObj.abertura)));
			dilate(iThresholded, iThresholded, getStructuringElement(MORPH_RECT, Size(finalObj.abertura, finalObj.abertura)));
		}
		if (finalObj.fechamento > 0)
		{
			dilate(iThresholded, iThresholded, getStructuringElement(MORPH_RECT, Size(finalObj.fechamento, finalObj.fechamento)));
			erode(iThresholded, iThresholded, getStructuringElement(MORPH_RECT, Size(finalObj.fechamento, finalObj.fechamento)));
		}

		if (finalObj.abertura > 0)
		{
			erode(fThresholded, fThresholded, getStructuringElement(MORPH_RECT, Size(finalObj.abertura, finalObj.abertura)));
			dilate(fThresholded, fThresholded, getStructuringElement(MORPH_RECT, Size(finalObj.abertura, finalObj.abertura)));
		}
		if (finalObj.fechamento > 0)
		{
			dilate(fThresholded, fThresholded, getStructuringElement(MORPH_RECT, Size(finalObj.fechamento, finalObj.fechamento)));
			erode(fThresholded, fThresholded, getStructuringElement(MORPH_RECT, Size(finalObj.fechamento, finalObj.fechamento)));
		}

		tratada_thresh_laranja.imshow(iThresholded);
		tratada_thresh_rosa.imshow(fThresholded);

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
		Mat distance_txt= Mat::zeros(30,340,CV_8UC3);
		if (distance > 0)
			{
			char txt[35];
			sprintf(txt,"Distance between = %f cm",distance);
			putText(distance_txt, txt, cvPoint(5,15),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);
			}
		// Show result to user
		result.imshow(imgOriginal);
		controllerInitialHSVfilter.imshow(distance_txt);

		// Wait for key is pressed then break loop
		if (waitKey(5) == 27) //ESC == 27
		{
			break;
		}
	}

	return 0;
}