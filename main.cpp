/*
COMPILE -->  g++ main.cpp -o main.exe `pkg-config --cflags --libs opencv` -w
EXECUTE --> ./main.exe
*/
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
#define JAN_OFFSET 20
using namespace cv;
using namespace std;

	void callbackButton(int state, void* userdata)
	{
		*(bool*)userdata = !state;
	}
vector<int> channelToHist(Mat img)
{	

	vector<int> res(255, 0);
	for(int i=0;i<img.rows;i++)
		for(int j=0;j<img.cols;j++)
			res[img.at<uchar>(i,j)]++;
	return res;
}
Mat histToImage(vector<int> hist)
{
	int bins=hist.size();
	int maxy=0;
	for(int i=0;i<bins;i++)
		if(hist[i]>maxy)
		maxy=hist[i];
	for(int i=0;i<bins;i++)
		hist[i] = (hist[i]*bins)/maxy;
	Mat res = Mat::zeros(bins,bins,CV_8UC3);
	for(int i=0;i<bins;i++)
		for(int j=0;j<hist[i];j++)
			res.at<Vec3b>(bins-j-1,i) = Vec3b(0,0,255);
return res;
}
class Window
{
	char *m_name;
	int tam_ja;
	int x,y;
	
	bool created;

public:
bool closed;
	Window(char *name, int tam_ja, int x, int y)
	{
		this->tam_ja=tam_ja;
		this->x=x;
		this->y=y;
		
		closed=true;
		created=true;
		m_name = name;
		if(!strcmp(name,"result"))
		{
		namedWindow(m_name,CV_WINDOW_NORMAL);
		moveWindow(m_name, tam_ja * x + JAN_OFFSET, tam_ja * y + JAN_OFFSET);
		resizeWindow(m_name, tam_ja, tam_ja);
		}
		if(strcmp(name,"result"))
			createButton(name,callbackButton,&closed,QT_CHECKBOX,0);
	}
	void imshow(Mat img)
	{
		
		if(closed && created)
			close();
		if(!closed && !created)
			show();

		if(created)
		cv::imshow(m_name, img);
	}
	void createTrackbar(char *trackName, int *var, int max_val)
	{
		cvCreateTrackbar(trackName, m_name, var, max_val);
	}
	void close(){
		destroyWindow(m_name);
		created=false;
	}
	void show(){
		created=true;
		namedWindow(m_name,CV_WINDOW_NORMAL | CV_GUI_NORMAL);
		moveWindow(m_name, tam_ja * x + JAN_OFFSET, tam_ja * y + JAN_OFFSET);
		resizeWindow(m_name, tam_ja, tam_ja);
	}
	void displayText(char * txt)
	{
		displayStatusBar(m_name,txt, 0 );
	}

	
};
// Here comes our classes
class TrackingObject
{
public:
	int lowH, lowS, lowV, highH, highS, highV;
	int x, y;
	int abertura, fechamento,tam_rect;
	double distanceFromCamera, realR, R, minArea;

	TrackingObject(double realR = realRadious, double minArea = minimumArea)
	{
		this->abertura = 8;
		this->fechamento = 35;
		this->tam_rect = 50;
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
	Mat RectWithTam(cv::Mat &img)
	{
		//tam_rect = 1.2*R;
		 Point temp = Point(tam_rect, tam_rect);
		 Point esq = Point(this->x, this->y)-temp;
		 Point dir = Point(this->x, this->y)+temp;
		 if(esq.x<0 || esq.y<0 || esq.x >= img.cols || esq.y>=img.rows
		 	|| dir.x<0 || dir.y<0 || dir.x >= img.cols || dir.y>=img.rows)
			 return Mat::zeros(10,10,CV_8UC1);
		return img(Rect(esq,dir));
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

	VideoCapture cap(1);

	if (!cap.isOpened())
	{
		cout << "Sorry, cannot open your camera :(" << endl;
		return -1;
	}

	// Our objects
	TrackingObject initialObj, finalObj;
	
	// Here we choose the interval to get the objects by their HSV color, see the hsvMap to get your desired values!
	initialObj.setHsvFilter(160, 100, 0, 200, 213, 255);
	finalObj.setHsvFilter(70, 100, 0, 77, 221, 255);

	// Create Trackbars





	bool otsu_not_used=true;
	Window result("result", 400, 0, 0);
	result.closed=0;
	createButton("Cortar e segmentar",callbackButton,&otsu_not_used,QT_CHECKBOX,0);
	Window Final_Cortada("Final Cortada", 400, 1, 0);
	Window Final_Cortada_Histograma("Final Cortada Histograma", 400, 1, 1);
	cvCreateTrackbar("Tamanho rect final",NULL, &finalObj.tam_rect, 400);
	Window Inicial_Cortada("Inicial Cortada", 400, 1, 0);
	Window Inicial_Cortada_Histograma("Inicial Cortada Histograma", 400, 1, 1);
	cvCreateTrackbar("Tamanho rect inicial",NULL, &initialObj.tam_rect, 400);
	Window thresh_rosa("Final", 400, 1, 0);
	cvCreateTrackbar("F_LowH",NULL, &finalObj.lowH, 255);
	cvCreateTrackbar("F_HighH",NULL, &finalObj.highH, 255);
	cvCreateTrackbar("F_LowS",NULL, &finalObj.lowS, 255);
	cvCreateTrackbar("F_HighS",NULL, &finalObj.highS, 255);
	Window thresh_laranja("Inicial", 400, 1, 0);
	cvCreateTrackbar("I_LowH",NULL, &initialObj.lowH, 255);
	cvCreateTrackbar("I_HighH",NULL, &initialObj.highH, 255);
	cvCreateTrackbar("I_LowS",NULL, &initialObj.lowS, 255);
	cvCreateTrackbar("I_HighS",NULL, &initialObj.highS, 255);
	Window H("H", 400, 1, 0);
	Window S("S", 400, 2, 0);
	Window V("V", 400, 3, 0);
	Window H_hist("H_hist", 400, 1, 1);
	Window S_hist("S_hist", 400, 2, 1);
	Window V_hist("V_hist", 400, 3, 1);
	cvCreateTrackbar("Abertura_Tam",NULL, &finalObj.abertura, 120);
	cvCreateTrackbar("Fechamento_Tam",NULL, &finalObj.fechamento, 120);
	Window tratada_thresh_rosa("Tratada_thresh_rosa", 400, 2, 0);
	Window tratada_thresh_laranja("Tratada_thresh_laranja", 400, 2, 0);


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
		vector<Mat> channels;
		split(imgHSV,channels);

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


		if(!otsu_not_used)
		{	
			Mat temp = finalObj.RectWithTam(channels[0]);
			Final_Cortada_Histograma.imshow(histToImage(channelToHist(temp)));
			threshold(temp, temp, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
			Final_Cortada.imshow(temp);

			temp = initialObj.RectWithTam(channels[0]);
			Inicial_Cortada_Histograma.imshow(histToImage(channelToHist(temp)));
			threshold(temp, temp, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
			Inicial_Cortada.imshow(temp);
		}
		


		// Draw the two circles
		initialObj.drawCircle(imgOriginal);
		finalObj.drawCircle(imgOriginal);

		// Draw a line between the objects
		initialObj.drawLineBetweenObject(imgOriginal, finalObj);

		// Calcultate the distance between the objects
		double distance = finalObj.calculateDistanceBetweenObjct(initialObj);
		if (distance > 0)
			{
			char txt[35];
			sprintf(txt,"Distance between = %f cm",distance);
			result.displayText(txt);
			//putText(distance_txt, txt, cvPoint(5,15),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);
			}
		// Show result to user
		result.imshow(imgOriginal);
		H.imshow(channels[0]);
		S.imshow(channels[1]);
		V.imshow(channels[2]);
		H_hist.imshow(histToImage(channelToHist(channels[0])));
		S_hist.imshow(histToImage(channelToHist(channels[1])));
		V_hist.imshow(histToImage(channelToHist(channels[2])));

		// Wait for key is pressed then break loop
		if (waitKey(500) == 27) //ESC == 27
		{
			break;
		}
	}

	return 0;
}