//General
#include "stdafx.h"
#include <Windows.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <stdexcept>
#include <conio.h>
//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "cv.h"
#include "highgui.h"
#include <opencv2/legacy/legacy.hpp>
//Kinect
#include "NuiApi.h"
#include <NuiImageCamera.h>
#include <NuiSkeleton.h>
#include <NuiSensor.h>



#define KB_UP 72
#define KB_DOWN 80
#define KB_ESCAPE 27

//to check current directory
#include <direct.h>
#define GetCurrentDir _getcwd



using namespace cv;
using namespace std;

HRESULT hr = S_OK;

//Define Variables

#define CHANNEL 3
#define COLOR_WIDTH 640
#define COLOR_HIGHT 480
#define DEPTH_WIDTH 320
#define DEPTH_HIGHT 240

BYTE buf[320*240*CHANNEL];


//variables for object recognition
const NUI_IMAGE_FRAME * pImageFrame = NULL;

/*
IplImage* img1 = cvLoadImage("detectMeGSA.jpg");
cv::Mat objectMat1(img1);


IplImage* img2 = cvLoadImage("detectNoteBook.jpg");
cv::Mat objectMat2(img2);

IplImage* img3 = cvLoadImage("detectCupe.jpg");
cv::Mat objectMat3(img3);

IplImage* img4 = cvLoadImage("detectMag.jpg");
cv::Mat objectMat4(img4);

IplImage* img;
cv::Mat objectMat(img);

IplImage* scn;
cv::Mat sceneMat(scn);

*/
Mat objectMat1= imread( "detectMeGSA.jpg", CV_LOAD_IMAGE_GRAYSCALE );
Mat objectMat2= imread( "detectNoteBook.jpg", CV_LOAD_IMAGE_GRAYSCALE );
Mat objectMat3= imread( "detectCupe.jpg", CV_LOAD_IMAGE_GRAYSCALE );
Mat objectMat4= imread( "detectMag.jpg", CV_LOAD_IMAGE_GRAYSCALE );
Mat objectMat;
Mat sceneMat;

//incrasing hessianValue will result in lesser matches
int hessianValue=50;
bool objectFound = false;
//increasing this value increases number of good_matches
float nndrRatio = 0.6f;
//vector of keypoints
vector< cv::KeyPoint > keypointsO;
vector< cv::KeyPoint > keypointsO1;
vector< cv::KeyPoint > keypointsO2;
vector< cv::KeyPoint > keypointsO3;
vector< cv::KeyPoint > keypointsO4;
vector< cv::KeyPoint > keypointsS;
Mat descriptors_object, descriptors_scene;
std::vector< vector< DMatch > > matches;
std::vector< vector< DMatch > > matches1;
std::vector< vector< DMatch > > matches2;
std::vector< vector< DMatch > > matches3;
std::vector< vector< DMatch > > matches4;
vector< DMatch > good_matches;


//IplImage* img_matches_ipl = cvCreateImage(cvSize(500, 600), IPL_DEPTH_8U,1);
Mat3b img_matches;


Mat descriptors_object1;
Mat descriptors_object2;
Mat descriptors_object3;
Mat descriptors_object4;

//adaptive system
char ob1[25];
char ob2[25];
char ob3[25];
char ob4[25];

char *ob1ptr[25];
char *ob2ptr[25];
char *ob3ptr[25];
char *ob4ptr[25];


//Initialize connection with Kinect sensor
void InitializeKinect()
{
	bool FailToConnect;
	do
	{
		HRESULT hr = NuiInitialize ( NUI_INITIALIZE_FLAG_USES_SKELETON |NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR);
		if( FAILED( hr ) )
		{
			system("cls");
			cout <<"\nFailed to connect, is the sensor plugged in!? \n\n";
			FailToConnect = true;
			system("PAUSE");
			//return hr;
		}
		else
		{
			cout << "\nConnection Established !!! \n \n ";
			FailToConnect = false;
			//return hr;
		}
	}
	while ( FailToConnect);
}


//Create RGB Image
int createRGBImage(HANDLE h, IplImage* colour)
{
	HRESULT hr = NuiImageStreamGetNextFrame(h, 0, &pImageFrame);
	if (FAILED (hr))
	{
		cout << "Create RGB Image Failed\n";
		return -1;
	}

	INuiFrameTexture * pTexture = pImageFrame -> pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch !=0)
	{
		BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		cvSetData(colour, pBuffer, LockedRect.Pitch);
		cvSetImageCOI(colour, 1);
	}

	Mat original_image= Mat (colour);
	cvtColor(original_image, original_image, CV_BGR2GRAY);
	cv::flip(original_image,sceneMat,1);


	imshow("scene", sceneMat);
	waitKey(10);


	SurfFeatureDetector surf(hessianValue);
	surf.detect(sceneMat,keypointsS);
	if(keypointsS.size() < 7) return false; //Not enough keypoints, object not found

	SurfDescriptorExtractor extractor;
	extractor.compute( sceneMat, keypointsS, descriptors_scene );


	BFMatcher matcher(NORM_L1);


	std::vector< vector< DMatch > > matches;

	for (int t=1; t<5; t++){
		if (t==1)
			matcher.knnMatch( descriptors_object1, descriptors_scene, matches1, 4 );
		if (t==2)
			matcher.knnMatch( descriptors_object2, descriptors_scene, matches2, 4 );
		if (t==3)
			matcher.knnMatch( descriptors_object3, descriptors_scene, matches3, 4 );
		if (t==4)
			matcher.knnMatch( descriptors_object4, descriptors_scene, matches4, 4 );


		if(t==1)
			matches=matches1;
		else if(t==2)
			matches=matches2;
		else if(t==3)
			matches=matches3;
		else if(t==4)
			matches=matches4;


		vector< DMatch > good_matches;
		good_matches.reserve(matches.size());

		for (size_t l = 0; l < matches.size(); ++l)
		{
			if (matches[l].size() < 2)
				continue;

			const DMatch &m1 = matches[l][0];
			const DMatch &m2 = matches[l][1];

			if(m1.distance <= nndrRatio * m2.distance)
				good_matches.push_back(m1);
		}


		cout << "matches: " << matches.size() << endl;
		cout << "good matches: " << good_matches.size() << endl;


		//good_matches.size() should be >= 8.. a lower number will give wrong results!
		/*if good_matches.size() is less than 4 it will not work because
		Homography needs at least 4 points to draw the lines*/



		if( (good_matches.size() >=9))
		{


			//cvDestroyWindow("object");
			cvDestroyWindow("scene");



			if (t==1){
				objectMat=objectMat1;
				keypointsO=keypointsO1;
			}
			else if(t==2){
				objectMat=objectMat2;
				keypointsO=keypointsO2;
			}
			else if(t==3){
				objectMat=objectMat3;
				keypointsO=keypointsO3;
			}
			else if(t==4){
				objectMat=objectMat4;
				keypointsO=keypointsO4;
			}


			std::vector< Point2f > obj;
			std::vector< Point2f > scene;


			for( unsigned int r = 0; r < good_matches.size(); r++ )
			{
				//-- Get the keypoints from the good matches
				obj.push_back( keypointsO[ good_matches[r].queryIdx ].pt );
				scene.push_back( keypointsS[ good_matches[r].trainIdx ].pt );
			}

			Mat H = findHomography( obj, scene, CV_RANSAC );

			//-- Get the corners from the image_1 ( the object to be "detected" )
			std::vector< Point2f > obj_corners(100);
			obj_corners[0] = cvPoint(0,0); 
			obj_corners[1] = cvPoint( objectMat.cols, 0 );
			obj_corners[2] = cvPoint( objectMat.cols, objectMat.rows ); 
			obj_corners[3] = cvPoint( 0, objectMat.rows );

			std::vector< Point2f > scene_corners(100);

			perspectiveTransform( obj_corners, scene_corners, H);



			drawMatches( objectMat, keypointsO, sceneMat, keypointsS,
				good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );



			//----------below function fills img_matches with red color
			//img_matches=cv::Scalar(0, 0, 255);




			//--------does the warping
			Mat warpImage;
			cv::warpPerspective(objectMat3, warpImage, H, Size(sceneMat.cols,sceneMat.rows), INTER_CUBIC);

			
			


			//-----trial to replace pixels -- failed		
			//Mat warpDest(Size(objectMat1.cols*2, objectMat1.rows*2),CV_8UC1);;
			//for(int y=0;y<objectMat1.rows;y++)
			//for(int x=0;x<objectMat1.cols;x++)
			//{
			//	warpDest.at<uchar>(y,x) = objectMat1.at<uchar>(y,x) + objectMat2.at<uchar>(y,x);
			//}				
			//imshow ("warp", warpDest);
			//-----trial to replace pixels -- failed


			//-------this print every pixel of object
			//for(int i=0; i<objectMat1.rows; i++) 
			//for(int j=0; j<objectMat1.cols; j++) 
			//std::cout << (int)objectMat1.at<uchar>(j,i) << std::endl;
			//-------this print every pixel of object  



			/*/----- draws a recntangle around detected object
			scene_corners[0] = scene_corners[0]+ Point2f( objectMat.cols, 0), scene_corners[1]+ Point2f( objectMat.cols, 0);
			scene_corners[1] = scene_corners[1]+ Point2f( objectMat.cols, 0), scene_corners[2]+ Point2f( objectMat.cols, 0);
			scene_corners[2] = scene_corners[2]+ Point2f( objectMat.cols, 0), scene_corners[3]+ Point2f( objectMat.cols, 0);
			scene_corners[3] = scene_corners[3]+ Point2f( objectMat.cols, 0), scene_corners[0]+ Point2f( objectMat.cols, 0);
			rectangle(img_matches, scene_corners[1], scene_corners[3], CV_RGB(0,0,255),-5,8);
			//--------------draws a recntangle around detected object */




			/*/-------------- replaces all pixel to certain color in img_matches [work on one channel]
			for(int i=0; i<objectMat1.cols; i++){
			for(int j=0; j<objectMat1.rows; j++){

			objectMat1.data[objectMat1.step[0]*i + objectMat1.step[1]*j + 0] = 0;
			}
			} 
			imshow ("fmsdfds", objectMat1);
			//-------------------replaces all pixel to certain color in img_matches [work on one channel] */

			//std::vector< Point2f > scene_corners(4);



			/*/---- many lines

			for (int i=0; i<5; i++){
			for (int x=0; x<30; x+=10){
			for (int y=0; y<30; y+=10){
			scene_corners[i] = cvPoint(0,0); 
			line( img_matches, scene_corners[i] + Point2f( objectMat.cols, 0) , scene_corners[i+1] + Point2f( objectMat.cols, 0), Scalar(0, 255, 255), 100 );
			}		
			}
			}

			/--- many lines/*/





			//-- Draw lines between the corners (the mapped object in the scene - image_2 )
			line( img_matches, scene_corners[0] + Point2f( objectMat.cols, 0), scene_corners[1] + Point2f( objectMat.cols, 0), Scalar(0, 255, 0), 4 );
			line( img_matches, scene_corners[1] + Point2f( objectMat.cols, 0), scene_corners[2] + Point2f( objectMat.cols, 0), Scalar( 0, 255, 0), 4 );
			line( img_matches, scene_corners[2] + Point2f( objectMat.cols, 0), scene_corners[3] + Point2f( objectMat.cols, 0), Scalar( 0, 255, 0), 4 );
			line( img_matches, scene_corners[3] + Point2f( objectMat.cols, 0), scene_corners[0] + Point2f( objectMat.cols, 0), Scalar( 0, 255, 0), 4 );

			/*/-----iterator
			for (Mat3b::iterator it = img_matches.begin(); it != img_matches.end(); it++) {
			if (*it == Vec3b(0, 255, 0)) {
			*it = Vec3b(0, 0, 0);
			}
			}
			//-----iterator-------/*/


			
			/*/-------------- replaces all pixel to certain color in img_matches [work on one channel]
			for(int i=0; i<objectMat1.cols; i++){
			for(int j=0; j<objectMat1.rows; j++){
				//img_matches.at<Vec3b>(i, j)=0;
			objectMat1.data[objectMat1.step[0]*i + objectMat1.step[1]*j ] = objectMat2.data[objectMat2.step[0]*i + objectMat2.step[1]*j];
			}
			} 
			imshow ("fmsdfds", objectMat1);
			//-------------------replaces all pixel to certain color in img_matches [work on one channel] */






			imshow( "Good Matches & Object detection", img_matches );

			waitKey(100);



			//objectFound=true;

			if (t==1){
				if(*ob1ptr=="\0"){
					cout << "\nNEW Object Detected!!\nPlease Type its Name : " ;
					cin >> ob1;
					*ob1ptr = ob1;
					cout << "Thanks! Object's name was saved as "<< ob1 << endl;
				}else
					cout << ob1 << " is detected!!!" << endl;
			}else if (t==2){
				if(*ob2ptr=="\0"){
					cout << "\nNEW Object Detected!!\nPlease Type its Name : " ;
					cin >> ob2;
					*ob2ptr = ob2;
					cout << "Thanks! Object's name was saved as "<< ob2 << endl;
				}else
					cout << ob2 << " is detected!!!" << endl;
			}else if (t==3){
				if(*ob3ptr=="\0"){
					cout << "\nNEW Object Detected!!\nPlease Type its Name : " ;
					cin >> ob3;
					*ob3ptr = ob3;
					cout << "Thanks! Object's name was saved as "<< ob3 << endl;
				}else
					cout << ob3 << " is detected!!!" << endl;
			}else if (t==4){
				if(*ob4ptr=="\0"){
					cout << "\nNEW Object Detected!!\nPlease Type its Name : " ;
					cin >> ob4;
					*ob4ptr = ob4;
					cout << "Thanks! Object's name was saved as "<< ob4 << endl;
				}else
					cout << ob4 << " is detected!!!" << endl;;
			}


			//img_matches.release();
			cout << "Press Enter to see warp" << endl;
			cin.get();
			cvDestroyWindow("Good Matches & Object detection");

			//shows the warp image with scene in a single window
			Mat final(Size(sceneMat.cols*2, sceneMat.rows),CV_8UC1);
			Mat roi1(final, Rect(0, 0,  sceneMat.cols, sceneMat.rows));
			Mat roi2(final, Rect(sceneMat.cols, 0, warpImage.cols, warpImage.rows));			
			sceneMat.copyTo(roi1);
			warpImage.copyTo(roi2);
			imshow("External Image Warp", final);	

			cin.get();


		}
		else{
			cout <<"Object NOT found!" << endl;

		}

	}
	sceneMat.release();
	img_matches.release();
	NuiImageStreamReleaseFrame(h, pImageFrame);

}



//main
int main(int argc,char * argv[])
{
	//-------------@@@@@--------function to control Kinect Motor Angle
	/*
	NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR);
	NuiCameraElevationSetAngle(15);

	int KB_code=0;
	int kinectAngle = 15;

	cout << "Press Up/Down arrows to adjust Kinect Camera Angle" << endl;
	cout << "Or Press Escape to contniue" << endl;

	while(KB_code != KB_ESCAPE )
	{
	if (kbhit())
	{
	KB_code = getch();
	//printf("KB_code = %i \n",KB_code);

	switch (KB_code)
	{
	case KB_UP:
	kinectAngle += 4;
	NuiCameraElevationSetAngle(kinectAngle);
	break;

	case KB_DOWN:
	kinectAngle -= 4;
	NuiCameraElevationSetAngle(kinectAngle);
	break;
	}
	}
	}
	*/



	ob1ptr[0]="\0";
	ob2ptr[0]="\0";
	ob3ptr[0]="\0";
	ob4ptr[0]="\0";

	cout << "KINECT\n";
	cout << "Establishing connection with Kinect sensor\n\n";

	IplImage* color = cvCreateImage(cvSize(320,240),IPL_DEPTH_8U,4);

	HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX|NUI_INITIALIZE_FLAG_USES_COLOR|NUI_INITIALIZE_FLAG_USES_SKELETON);

	if( hr != S_OK )
	{
		cout<<"\nNuiInitialize failed\n";
		cout<<"Is the Kinect pluged in\n\n?";
		system ("PAUSE");
		return hr;
	}

	HANDLE m_hNextVideoFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	HANDLE m_pVideoStreamHandle = NULL;

	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,NUI_IMAGE_RESOLUTION_640x480, 0, 2, m_hNextVideoFrameEvent, &m_pVideoStreamHandle);

	if( FAILED( hr ) )
	{
		cout<<"Could not open image stream video"<<endl;
		return hr;
	}

	HANDLE m_hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	HANDLE m_pDepthStreamHandle = NULL;

	hr = NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2, m_hNextDepthFrameEvent, &m_pDepthStreamHandle);

	if( FAILED( hr ) )
	{
		cout<<"Could not open depth stream video"<<endl;
		return hr;
	}





	//-- Step 1: Extract keypoints
	SurfFeatureDetector surf(hessianValue);
	SurfDescriptorExtractor extractor;

	surf.detect(objectMat1,keypointsO1);
	if(keypointsO1.size() < 7) return false; //Not enough keypoints, object not found
	extractor.compute( objectMat1, keypointsO1, descriptors_object1 );


	surf.detect(objectMat2,keypointsO2);
	if(keypointsO2.size() < 7) return false; //Not enough keypoints, object not found
	extractor.compute( objectMat2, keypointsO2, descriptors_object2 );


	surf.detect(objectMat3,keypointsO3);
	if(keypointsO3.size() < 7) return false; //Not enough keypoints, object not found
	extractor.compute( objectMat3, keypointsO3, descriptors_object3 );


	surf.detect(objectMat4,keypointsO4);
	if(keypointsO4.size() < 7) return false; //Not enough keypoints, object not found
	extractor.compute( objectMat4, keypointsO4, descriptors_object4 );



	while(1)
	{
		WaitForSingleObject(m_hNextVideoFrameEvent,INFINITE);
		createRGBImage(m_pVideoStreamHandle,color);

		//exit
		int c = cvWaitKey(1);
		if( c == 27 || c == 'q' || c == 'Q' )
			break;
	}
	objectMat.release();
	cvReleaseImageHeader(&color);

	NuiShutdown();
	return 0;


}
