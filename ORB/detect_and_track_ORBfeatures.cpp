// This code detects ORB keypoints in images and tracks them
// chair semantic keypoint annotation based on a flag
// Stores closestORBPoints to file closestORBPoints.txt. These points are the points which are closest to the manually annotated keypoints on the starting frame
// Tracks these keypoints and stores them in trackedORBkeypints.txt

#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include<bits/stdc++.h>
#include <sstream>
#include <string>
#include <vector>
#include <cmath> 
#include <unistd.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
 
const int MAX_FEATURES = 2000;
const float GOOD_MATCH_PERCENT = 0.15f;
const int NO_OF_ANNOTATION_POINTS = 10;
vector<KeyPoint> closestORBPoints;
Mat closestORBPointsDescriptors;


void split(vector<string> &toks, const string &s, const string &delims)
{
    toks.clear();

    string::const_iterator segment_begin = s.begin();
    string::const_iterator current = s.begin();
    string::const_iterator string_end = s.end();

    while (true)
    {
        if (current == string_end || delims.find(*current) != string::npos || *current == '\r')
        {
            if (segment_begin != current)
                toks.push_back(string(segment_begin, current));
            if (current == string_end || *current == '\r')
                break;
            segment_begin = current + 1;
        }
        current++;
    }
}

// callback function similar to imtool in matlab, for getting pixel info
void CallBackFunc(int event, int x, int y, int flags, void* pptr)
{
	vector<KeyPoint> *v = (vector<KeyPoint>*) pptr;

    if  ( event == EVENT_LBUTTONDOWN )
    {
	   cout << "Left button clicked - position (" << x << ", " << y << ")" << endl;
	   v->push_back(KeyPoint(x,y,NULL));
    }

}

void SaveORBPointsClosestToFile()
{	
	FileStorage fs("closestORBPoints.xml", FileStorage::WRITE);
	fs << "ORBkeypoints" << closestORBPoints;
	fs << "ORBdescriptors" << closestORBPointsDescriptors;

	for(int i=0;i<closestORBPoints.size();i++)
		cout << closestORBPoints[i].pt.x << " " << closestORBPoints[i].pt.y << endl;
}

void loadclosestORBpointsfile()
{
	FileStorage fs("closestORBPoints.xml", FileStorage::READ);
	fs["ORBkeypoints"] >> closestORBPoints;
	fs["ORBdescriptors"] >> closestORBPointsDescriptors;
}

double computeDistance(KeyPoint a, KeyPoint b){
	double dist = pow(a.pt.x-b.pt.x, 2) + pow(a.pt.y-b.pt.y, 2);
}

void findORBPointsClosestToAnnotatedPoints(vector<KeyPoint> v,stringstream* ss, string dir)
{
	Mat img = imread(dir+"/rgb/"+ ss->str());
	Mat imgGray;
	cvtColor(img, imgGray, CV_BGR2GRAY);
	vector<KeyPoint> keypoints;
  	Mat descriptors;

  	Ptr<Feature2D> orb = ORB::create(MAX_FEATURES,1.2,8,31,0,2,ORB::FAST_SCORE);
  	orb->detectAndCompute(imgGray, Mat(), keypoints, descriptors);


  	namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  	drawKeypoints(img,keypoints,imgGray);
  	imshow( "Display window", imgGray);
  	waitKey(0); 
	
  	
  	for(int i=0;i<v.size();i++){
  		double minDistance = 100;
  		int minDistanceIndex;
  		for(int j=0;j<keypoints.size();j++){
  			double dist = computeDistance(v[i],keypoints[j]);
  			if(dist < minDistance){
  				minDistance = dist;
  				minDistanceIndex = j;
  			}
  		}
  		closestORBPoints.push_back(keypoints[minDistanceIndex]);
		closestORBPointsDescriptors.push_back(descriptors.row(minDistanceIndex));
  	}

}


void track(int sframe,int eframe, string dir)
{

	Mat img;
	Mat imgGray;
	vector<KeyPoint> keypoints;
  	Mat descriptors;
  	stringstream ss;

  	ofstream outfile;
  	outfile.open("trackedORBkeypoints.txt");

  	// output the orb point locations to file
  	int framecount = 0;
  	outfile << eframe - sframe+1 << endl;
  	outfile << framecount << " ";
  	for(size_t i=0;i<closestORBPoints.size();i++)
  		outfile << closestORBPoints[i].pt.x << "," << closestORBPoints[i].pt.y << ",";
  	outfile << endl;

	framecount++;  	
	for(int currentframe=sframe;currentframe<eframe;currentframe++)
	{
		// detect orb features
		ss.str("");
		ss << setfill('0') << setw(4) << currentframe << ".jpg";
		img = imread(dir+"/rgb/"+ss.str());
		cvtColor(img, imgGray, CV_BGR2GRAY);
		Ptr<Feature2D> orb = ORB::create(MAX_FEATURES,1.2,8,31,0,2,ORB::FAST_SCORE);
		orb->detectAndCompute(imgGray, Mat(), keypoints, descriptors);

		// match features
		std::vector<DMatch> matches;
		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
		matcher->match(closestORBPointsDescriptors, descriptors, matches, Mat());
		
		// Sort matches by score
  		std::sort(matches.begin(), matches.end());


  		// output the location of the matches to a file
  		vector<KeyPoint> points;
  		outfile << framecount << " ";
  		for( size_t i = 0; i < matches.size(); i++ ){
  			points.push_back(keypoints[ matches[i].trainIdx ]);
			outfile << points[i].pt.x << "," << points[i].pt.y << ",";
  		}
  		outfile << endl;

		// display the tracked keypoints
		namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
		cv::Mat img1 = img.clone();
		drawKeypoints(img,points,img1);
		putText(img1, ss.str(), Point(20,20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0,50,50), 2);
		imshow( "Display window", img1);
		waitKey(30);

		framecount++;

	}
  		
}


int main(int argc, char **argv)
{
	if (argc < 2)
		cout << "Usage: ./detect_and_track   directory_path   starting_frame_no   ending_frame_no   isAnnotationRequiredFlag\n";

	string dir = argv[1]; // dataset directory path

	string out_string;

	istringstream startss(argv[2]); // starting frame number
	istringstream endss(argv[3]); // ending frame number
	istringstream isAnnotationRequiredFlagss(argv[4]); // isAnnotationRequiredFlag: should be either 1 or 0
	

	int sframe, eframe, isAnnotationRequiredFlag;
	startss >> sframe; cout << sframe << endl;
	endss >> eframe;
	isAnnotationRequiredFlagss >> isAnnotationRequiredFlag;

	
	if(isAnnotationRequiredFlag != 1 && isAnnotationRequiredFlag != 0){
		cout << "invalid isAnnotationRequiredFlag input\n";
		return 0;
	}

	stringstream ss;
	ss << setfill('0') << setw(4) << sframe << ".jpg";

	Mat img = imread(dir+"/rgb/"+ss.str());
	if(! img.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image: " + dir+"rgb/"+ss.str() << std::endl ;
        return -1;
    }

    if(isAnnotationRequiredFlag == 1)
    {
	    vector<KeyPoint> vecOfKeypoints;

	// Display window to annotate the keypoints
		namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
		setMouseCallback("Display window", CallBackFunc, &vecOfKeypoints); // pass the img address to callback
		imshow( "Display window", img);
		waitKey(0); 


	// find the closest orb points to the annotated points
		findORBPointsClosestToAnnotatedPoints(vecOfKeypoints,&ss,dir);

		SaveORBPointsClosestToFile();
	}
	else
		loadclosestORBpointsfile();



// display the the orb points closest to the annotated keypoints
	namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
	cv::Mat img1 = img.clone();
	drawKeypoints(img,closestORBPoints,img1);
	putText(img1, ss.str(), Point(20,20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0,50,50), 2);
	imshow( "Display window", img1);
	waitKey(30); 

	sframe++; // start tracking from the next frame
	// track the keypoints and save to file
	track(sframe,eframe,dir);

return 0;
	
}