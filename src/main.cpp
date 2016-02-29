/*

Copyright (C) 2014  Kevin Harrilal, Control Engineer, Aluminum Falcon Robotics Inc.
kevin@team2168.org

Dec 31, 2014

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

*/
#define _USE_MATH_DEFINES
#define MIN_WIDTH 120
#define Y_IMAGE_RES 240
#define VIEW_ANGLE 34.8665269
#define AUTO_STEADY_STATE 1.9 //seconds

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <pthread.h>
#include <networktables/NetworkTable.h>
using namespace cv;
using namespace std;

//struct to define program execution variables passed in from the command line
struct ProgParams
{
	string ROBOT_IP;
	string ROBOT_PORT;
	string CAMERA_IP;
	string IMAGE_FILE;

	bool From_Camera;
	bool From_File;
	bool Visualize;
	bool Timer;
	bool Debug;
	bool Process;
	bool USB_Cam;
	bool Real_Robot;
};

//function declarations
//TODO: add pre- and post- comments for each function
void parseCommandInputs(int argc, const char* argv[], ProgParams &params);
void printCommandLineUsage();
void initializeParams(ProgParams& params);
double diffClock(timespec start, timespec end);
Mat ThresholdImage(Mat img);
void error(const char *msg);

//Threaded Video Capture Function
void *VideoCap(void *args);

//GLOBAL CONSTANTS
const double PI = 3.141592653589793;
//Some common colors to draw with
const Scalar RED = Scalar(0, 0, 255),
			BLUE = Scalar(255, 0, 0),
			GREEN = Scalar(0, 255, 0),
			ORANGE = Scalar(0, 128, 255),
			YELLOW = Scalar(0, 255, 255),
			PINK = Scalar(255, 0,255),
			WHITE = Scalar(255, 255, 255);

//GLOBAL MUTEX LOCK VARIABLES
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t matchStartMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;


//Thread Variables
pthread_t MJPEG;
pthread_t AutoCounter;


//Store targets in global variable
Mat frame;

//Global Timestamps for auto
struct timespec autoStart, autoEnd;


//Control process thread exectution
bool progRun;
std::shared_ptr<NetworkTable> table;
int main(int argc, const char* argv[])
{

	//Read command line inputs to determine how the program will execute
	ProgParams params;
	parseCommandInputs(argc, argv, params);

	// setup Network tables for this client to talk to the java code (same place this code is running!)
	NetworkTable::SetClientMode();

	string networktables_ip;

	if (params.Real_Robot) {
		string networktables_ip = "10.36.18.79";
	} else {
		string networktables_ip = "10.36.18.22";
	}

	NetworkTable::SetIPAddress(networktables_ip); // where is the robot?
	 table = NetworkTable::GetTable("SmartDashboard"); // what table will we interface with?

	cout << "Got through the network tables\n";

	//start mjpeg stream thread
    table.GetNumber("Low Hue", 50);
    table.GetNumber("High Hue", 100);
    table.GetNumber("Low Saturation", 80);
    table.GetNumber("High Saturation", 255);
    table.GetNumber("Low Value", 60);
    table.GetNumber("High Value", 255);
	//Create Local Processing Image Variables
	Mat img, thresholded, output;

	//initialize variables so processing loop is false;
	progRun = false;

	struct timespec start, end;

	//run loop forever
	while (true)
	{
		//check if program is allowed to run
		//this bool, is enabled by the mjpeg thread
		//once it is up to 10fps

		if (params.Process && progRun)
		{
			//start clock to determine our processing time;
			clock_gettime(CLOCK_REALTIME, &start);

			pthread_mutex_lock(&frameMutex);
			if (!frame.empty()){
				frame.copyTo(img);
				pthread_mutex_unlock(&frameMutex);

				thresholded = ThresholdImage(img);

				imwrite("/var/local/natinst/www/capture-src.png", img);

				vector < vector<Point> > contours;
				vector<Vec4i> hierarchy;


				findContours(thresholded, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));


				if (contours.size() > 0) {

					cout << contours.size() << " contours" << endl;

					vector <Point> targetContour = contours[0];

					for (unsigned int i = 1; i < contours.size(); i++) {
						if (contourArea(contours[i]) > contourArea(targetContour)) {
							targetContour = contours[i];
						}
					}

					Mat drawing = Mat::zeros( thresholded.size(), CV_8UC3 );
					Scalar color = Scalar( 255, 255, 255);

					vector <Point> hull;

					convexHull(targetContour, hull, false);

					vector <vector <Point> > tmpHull(1, hull);


					cout << "hull size " << hull.size() << endl;


					vector <Point> corners(4, Point(0,0));
					vector <int>  cornerSq(4);
					int TOP_LEFT = 0;
					int TOP_RIGHT = 1;
					int BOTTOM_RIGHT = 2;
					int BOTTOM_LEFT = 3;

					for (unsigned int i = 0; i < corners.size(); i++) {
						corners[i] = hull[0];
						if (i < 2)
						  cornerSq[i] = hull[0].x * hull[0].y;
						else
						  cornerSq[i] = (img.cols - hull[0].x) * hull[0].y;
					}

					for (unsigned int i = 1; i < hull.size(); i++) {

						cout << "hull " << i << ": (" << hull[i].x << ", " << hull[i].y << ") ";
						int thisSq = hull[i].x * hull[i].y;

						if (thisSq < cornerSq[TOP_LEFT]) {
							corners[TOP_LEFT] = hull[i];
							cornerSq[TOP_LEFT] = thisSq;
						}
						if (thisSq > cornerSq[BOTTOM_RIGHT]) {
							corners[BOTTOM_RIGHT] = hull[i];
							cornerSq[BOTTOM_RIGHT] = thisSq;
						}

						thisSq = (img.cols - hull[i].x) * hull[i].y;

						if (thisSq < cornerSq[TOP_RIGHT]) {
							corners[TOP_RIGHT] = hull[i];
							cornerSq[TOP_RIGHT] = thisSq;
						}
						if (thisSq > cornerSq[BOTTOM_LEFT]) {
							corners[BOTTOM_LEFT] = hull[i];
							cornerSq[BOTTOM_LEFT] = thisSq;
						}
					}

					cout << endl;


					for (unsigned int i = 0; i < corners.size(); i++) {
						circle(drawing, corners[i], 4, Scalar(0, 0, 255), 1, 8, 0);
					}

					//Easier equations that will average out the points
					double cenX1 = fabs(((double)(corners[TOP_RIGHT].x - corners[TOP_LEFT].x)) / 2);
					double cenX2 = fabs(((double)(corners[BOTTOM_RIGHT].x - corners[BOTTOM_LEFT].x)) / 2);
					double cenX = ((double) (cenX1 + cenX2) / 2) + corners[TOP_LEFT].x;

					double cenY1 = fabs(((double)(corners[TOP_RIGHT].y - corners[BOTTOM_RIGHT].y)) / 2);
					double cenY2 = fabs(((double)(corners[TOP_LEFT].y - corners[BOTTOM_LEFT].y)) / 2);
					double cenY = ((double) (cenY1 + cenY2) / 2) + corners[TOP_LEFT].y;

					//Code to calculate area, and eliminate the smallest contours

					double wx1 = corners[BOTTOM_RIGHT].x - corners[BOTTOM_LEFT].x;
					double wy1 = corners[BOTTOM_RIGHT].y - corners[BOTTOM_LEFT].y;

					double hx1 = corners[TOP_RIGHT].x - corners[BOTTOM_RIGHT].x;
					double hy1 = corners[TOP_RIGHT].y - corners[BOTTOM_RIGHT].y;

					double wx2 = pow(wx1, 2);
					double wy2 = pow(wy1, 2);

					double hx2 = pow(hx1, 2);
					double hy2 = pow(hy1, 2);

					double dist1 = sqrt(wx2 + wy2);
					double dist2 = sqrt(hx2 + hy2);

					double area = dist1 * dist2;

					cout << "Area: " << area << endl;

					if (area < 99) {
						// Do something here
					}

					table->PutNumber("Goal Width", dist1);

					table->PutNumber("Center X", cenX);
					table->PutNumber("Center Y", cenY);

					drawContours(drawing, tmpHull, 0, color, 1, 8, hierarchy, 0, Point(0, 0) );
					circle(drawing, Point(((int) round(cenX)), ((int) round(cenY))), 4, Scalar(0, 255, 0), 1, 8, 0);
					line(drawing, corners[TOP_LEFT], corners[BOTTOM_RIGHT], Scalar(255, 0, 0), 1, 8, 0);
					line(drawing, corners[TOP_RIGHT], corners[BOTTOM_LEFT], Scalar(255, 0, 0), 1, 8, 0);

					drawContours(img, tmpHull, 0, color, 1, 8, hierarchy, 0, Point(0, 0) );
					circle(img, Point(((int) round(cenX)), ((int) round(cenY))), 4, Scalar(0, 255, 0), 1, 8, 0);
					line(img, corners[TOP_LEFT], corners[BOTTOM_RIGHT], Scalar(255, 0, 0), 1, 8, 0);
					line(img, corners[TOP_RIGHT], corners[BOTTOM_LEFT], Scalar(255, 0, 0), 1, 8, 0);
					for (unsigned int i = 0; i < 4; i++) {
						circle(img, Point(((int) round(cenX)), ((int) round(cenY) - 75 - (25 * i))), 4, Scalar(0, 0, 255), 1, 8, 0);
					}

					imwrite("/var/local/natinst/www/capture.png", drawing);
				} else {
					cout << "can't find contours" << endl;
					int NO_CONTOURS = -1;
					table->PutNumber("Center X", NO_CONTOURS);
					table->PutNumber("Center Y", NO_CONTOURS);
				}

				imwrite("/var/local/natinst/www/capture-test.png", img);


				pthread_mutex_lock(&targetMutex);
				pthread_mutex_unlock(&targetMutex);

				clock_gettime(CLOCK_REALTIME, &end);

				if(params.Timer)
					cout << "It took " << diffClock(start,end) << " seconds to process frame \n";


			}

			pthread_mutex_unlock(&frameMutex);

			if(params.Visualize)
				waitKey(5);

		}

		usleep(1000); //20000 sleep for 5ms); // run 40 times a second
	}

	//if we end the process code, wait for threads to end
	pthread_join(MJPEG, NULL);

	//done
	return 0;

}


Mat ThresholdImage(Mat original)
{

	Mat imgThresholded, imgHSV;

    cvtColor(original, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    //morphological closing (fill small holes in the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//return image
	return imgThresholded;

}
void initializeParams(ProgParams& params)
{
	params.Debug = false;
	params.From_Camera = false;
	params.From_File = false;
	params.Timer = false;
	params.Visualize = false;
	params.Process = true;
	params.USB_Cam = false;
	params.Real_Robot = false;
}

/**
 * This function parses the command line inputs and determines
 * the runtime parameters the program should use as specified
 * by the user.
 */
void parseCommandInputs(int argc, const char* argv[], ProgParams& params)
{
	//todo: define all input flags
	if (argc < 2)
	{ // Check the value of argc. If not enough parameters have been passed, inform user and exit.
		printCommandLineUsage();
		exit(0);
	}
	else
	{ // if we got enough parameters...

		initializeParams(params);

		for (int i = 1; i < argc; i++)
		{ /* We will iterate over argv[] to get the parameters stored inside.
		 * Note that we're starting on 1 because we don't need to know the
		 * path of the program, which is stored in argv[0] */

			if ((string(argv[i]) == "-f") && (i + 1 < argc)) //read from file
			{
				// We know the next argument *should* be the filename:
				params.IMAGE_FILE = string(argv[i + 1]);
				params.From_Camera = false;
				params.From_File = true;
				i++;
			}
			else if ((string(argv[i]) == "-c") && (i + 1 < argc)) //camera IP
			{
				params.CAMERA_IP = string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				params.USB_Cam = false;
				i++;
			}
			else if (string(argv[i]) == "-u") //use USB Camera
			{
				//params.CAMERA_IP = string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				params.USB_Cam = true;
			}
			else if ((string(argv[i]) == "-s") && (i + 1 < argc)) //robot TCP SERVER IP
			{
				params.ROBOT_IP = string(argv[i + 1]);
				i++;
			}
			else if ((string(argv[i]) == "-p") && (i + 1 < argc)) //robot TCP SERVER PORT
			{
				params.ROBOT_PORT = string(argv[i + 1]);
				i++;
			}
			else if (string(argv[i]) == "-t") //Enable Timing
			{
				params.Timer = true;
			}
			else if (string(argv[i]) == "-np") //no processing
			{
				params.Process = false;
			}
			else if (string(argv[i]) == "-r") // Using competition robot
			{
				params.Real_Robot = true;
			}
			else if (string(argv[i]) == "-v") //Enable Visual output
			{
				params.Visualize = true;
			}
			else if (string(argv[i]) == "-debug") //Enable debug output
			{
				params.Debug = true;
			}
			else if (string(argv[i]) == "-d") //Default Params
			{
				params.ROBOT_PORT = string(argv[i + 1]);
				return;
			}
			else if (string(argv[i]) == "-help") //help
			{
				//todo: cout help on commands
				printCommandLineUsage();
				exit(0);
			}
			else
			{
				std::cout
						<< "Not enough or invalid arguments, please try again.\n";
				printCommandLineUsage();
				exit(0);
			}

		}

	}
}


/**
 * This function uses FFMPEG codec apart of openCV to open a
 * MJPEG stream and buffer it. This function should be ran
 * in its own thread so it can run as fast as possibe and store frames.
 *
 * A mutable lock should be used in another thread to copy the latest frame
 *
 * Note: Opening the stream blocks execution. Also
 * Based on my own tests it appears the beaglebone can capture
 * frames at 30fps with 320 x 240 resolution, however
 * the framerate needs to be reduced to allow for processing time.
 *
 * Only run the camera as 10FPS, with a 10kbs limit per frame
 */
void *VideoCap(void *args)
{
	//copy passed in variable to programStruct
	ProgParams *struct_ptr = (ProgParams *) args;

	if (struct_ptr->From_File)
	{
		cout<<"Loading Image from file"<<endl;

		//read img and store it in global variable
		pthread_mutex_lock(&frameMutex);
		frame = imread(struct_ptr->IMAGE_FILE);
		pthread_mutex_unlock(&frameMutex);

		if (!frame.empty())
		{
			cout<<"File Loaded: Starting Processing Thread"<<endl;
			progRun = true;
		}
		else
		{
			cout<<"Error Loading File"<<endl;
			exit(0);
		}


	}

	else if(struct_ptr->From_Camera)
	{
		//create timer variables
		struct timespec start, end, bufferStart, bufferEnd;

		//seconds to wait for buffer to clear before we start main process thread
		int waitForBufferToClear = 12;

		//start timer to time how long it takes to open stream
		clock_gettime(CLOCK_REALTIME, &start);

		cv::VideoCapture vcap;


		// For IP cam this works on a AXIS M1013
		// For USB cam this works on Microsoft HD 3000


		std::string videoStreamAddress;
		if (struct_ptr->USB_Cam)
		{

			int videoStreamAddress = 0; //represents /dev/video0

			std::cout<<"Trying to connect to Camera stream... at: "<<videoStreamAddress<<std::endl;

			int count =1;

			//open the video stream and make sure it's opened
			//We specify desired frame size and fps in constructor
			//Camera must be able to support specified framesize and frames per second
			//or this will set camera to defaults
			while (!vcap.open(videoStreamAddress, 320,240,7.5))
			{
				std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
				count++;
				usleep(1000000);
			}

			//After Opening Camera we need to configure the returned image setting
			//all opencv v4l2 camera controls scale from 0.0 - 1.0

			//vcap.set(CV_CAP_PROP_EXPOSURE_AUTO, 1);
			vcap.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0.1);
			vcap.set(CV_CAP_PROP_BRIGHTNESS, 1);
			vcap.set(CV_CAP_PROP_CONTRAST, 0);
			vcap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
			vcap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

			cout<<vcap.get(CV_CAP_PROP_FRAME_WIDTH)<<endl;
			cout<<vcap.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;

		}
		else //connect to IP Cam
		{
			std::string videoStreamAddress = "http://" + struct_ptr->CAMERA_IP +"/mjpg/video.mjpg";

			std::cout<<"Trying to connect to Camera stream... at: "<<videoStreamAddress<<std::endl;

			int count = 1;

			//open the video stream and make sure it's opened
			//image settings, resolution and fps are set via axis camera webpage
			while (!vcap.open(videoStreamAddress))
			{

				std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
				count++;
				usleep(1000000);
			}

		}



		//Stream started
		cout << "Successfully connected to Camera Stream" << std::endl;

		//set true boolean
		pthread_mutex_lock(&targetMutex);
		pthread_mutex_unlock(&targetMutex);

		//end clock to determine time to setup stream
		clock_gettime(CLOCK_REALTIME, &end);

		cout << "It took " << diffClock(start,end) << " seconds to set up stream " << endl;

		clock_gettime(CLOCK_REALTIME, &bufferStart);


		cout<<"Waiting for stream buffer to clear..."<<endl;


		//run in continuous loop
		while (true)
		{
			//start timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &start);

			//read frame and store it in global variable
			pthread_mutex_lock(&frameMutex);
			vcap.read(frame);
			pthread_mutex_unlock(&frameMutex);

			//end timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &end);


			if(struct_ptr->Timer)
				cout << "It took FFMPEG " << diffClock(start,end) << " seconds to grab stream \n";


			//end timer to get time since stream started
			clock_gettime(CLOCK_REALTIME, &bufferEnd);
			double bufferDifference = diffClock(bufferStart, bufferEnd);

			//The stream takes a while to start up, and because of it, images from the camera
			//buffer. We don't have a way to jump to the end of the stream to get the latest image, so we
			//run this loop as fast as we can and throw away all the old images. This wait, waits some number of seconds
			//before we are at the end of the stream, and can allow processing to begin.
			if ((bufferDifference >= waitForBufferToClear) && !progRun)
			{
				cout<<"Buffer Cleared: Starting Processing Thread"<<endl;
				progRun = true;

			}
			usleep(1000); //sleep for 5ms
		}

	}

	return NULL;
}

/*
 * This function prints the command line usage of this
 * program to the std output
 */
void printCommandLineUsage()
{
	cout<<"Usage: 2168_Vision  [Input]  [Options] \n\n";

	cout<<setw(10)<<left<<"Inputs:  Choose Only 1"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-f <file location>";
	cout<<"Process image at <file location>"<<endl;
	cout<<setw(30)<<""<<"ex: -f /home/image.jpg"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-c <ip address>";
	cout<<"Use IP camera at <ip address>"<<endl;
	cout<<setw(30)<<""<<"ex: -c 10.21.68.2"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-u";
	cout<<"Use USB cam at /dev/video0"<<endl;

	cout<<endl<<endl;
	cout<<setw(10)<<left<<"Options:  Choose Any Combination"<<endl;


	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-t";
	cout<<"Enable Timing Print Outs"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-v";
	cout<<"Enable Visual Output"<<endl;
	cout<<setw(30)<<""<<"Uses X11 forwarding to show processed image"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-np";
	cout<<"No Processing: Disable Processing Thread"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-debug";
	cout<<"Enable Debug Print Outs"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-help";
	cout<<"Prints this menu"<<endl;


}

/*
 * Error Functions
 * - Not Used -
 */
void error(const char *msg)
{
	perror(msg);
	exit(0);
}

/*
 * Calculate real clock difference
 */
double diffClock(timespec start, timespec end)
{
 return	(end.tv_sec - start.tv_sec) + (double) (end.tv_nsec - start.tv_nsec)/ 1000000000.0f;
}



