//	Track a moving object using motion features.
#include <vector>
#include <cv.h>
#include <opencv2/core/core.hpp>   // basic OpenCV structures
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <math.h>
#include <cmath> //cmath and define PI headers are for movement and rotation of fishes.
#define PI 3.14159265

using namespace cv;
using namespace std;

int main(  int argc, char** argv )
{ int frameCount = 0;
  VideoCapture cap(0); // open the camera
  if(!cap.isOpened()) {  // check if we succeeded
	  return -1;
  }

  namedWindow("Raw Capture",CV_WINDOW_AUTOSIZE);
  namedWindow("Difference Frame",CV_WINDOW_AUTOSIZE);
  namedWindow("Gray Difference",CV_WINDOW_AUTOSIZE);
  namedWindow("Fishes in the pond",CV_WINDOW_NORMAL);
  Mat frame, prevFrame, frameDiff, grayDiff;
  Mat pond[10];  // pond[0].. pond[9]
  Mat pondFrame;

  Mat RVF1 = imread("Figures\\Fish\\RVF1.png");

  Mat redVirtualFish[24];  // [0] ... [23]
  redVirtualFish[0] = imread("Figures\\Fish\\RVF1.png");
  redVirtualFish[1] = imread("Figures\\Fish\\RVF2.png");
  redVirtualFish[2] = imread("Figures\\Fish\\RVF3.png");
  redVirtualFish[3] = imread("Figures\\Fish\\RVF4.png");
  redVirtualFish[4] = imread("Figures\\Fish\\RVF5.png");
  redVirtualFish[5] = imread("Figures\\Fish\\RVF6.png");
  redVirtualFish[6] = imread("Figures\\Fish\\RVF7.png");
  redVirtualFish[7] = imread("Figures\\Fish\\RVF8.png");
  redVirtualFish[8] = imread("Figures\\Fish\\RVF9.png");
  redVirtualFish[9] = imread("Figures\\Fish\\RVF10.png");
  redVirtualFish[10] = imread("Figures\\Fish\\RVF11.png");
  redVirtualFish[11] = imread("Figures\\Fish\\RVF12.png");
  redVirtualFish[12] = imread("Figures\\Fish\\RVF13.png");
  redVirtualFish[13] = imread("Figures\\Fish\\RVF14.png");
  redVirtualFish[14] = imread("Figures\\Fish\\RVF15.png");
  redVirtualFish[15] = imread("Figures\\Fish\\RVF16.png");
  redVirtualFish[16] = imread("Figures\\Fish\\RVF17.png");
  redVirtualFish[17] = imread("Figures\\Fish\\RVF18.png");
  redVirtualFish[18] = imread("Figures\\Fish\\RVF19.png");
  redVirtualFish[19] = imread("Figures\\Fish\\RVF20.png");
  redVirtualFish[20] = imread("Figures\\Fish\\RVF21.png");
  redVirtualFish[21] = imread("Figures\\Fish\\RVF22.png");
  redVirtualFish[22] = imread("Figures\\Fish\\RVF23.png");
  redVirtualFish[23] = imread("Figures\\Fish\\RVF24.png");

  Mat RVF2;
  RVF1.copyTo(RVF2);

  cap >> frame; // get a new frame
  int xCenter = frame.cols / 2;
  int yCenter = frame.rows / 2;
  int widthObject = frame.cols / 4;   // expected width of the object
  int diffValue;
  int x, y;
  int sum, sumX, sumY;
  int i,j;
  int fx = 550; // Initial location of a fish
  int fy = 330;


  pond[0] = imread("Figures\\Water\\water01.png");
  pond[1] = imread("Figures\\Water\\water02.png");
  pond[2] = imread("Figures\\Water\\water03.png");
  pond[3] = imread("Figures\\Water\\water04.png");
  pond[4] = imread("Figures\\Water\\water05.png");
  pond[5] = imread("Figures\\Water\\water06.png");
  pond[6] = imread("Figures\\Water\\water07.png");
  pond[7] = imread("Figures\\Water\\water08.png");
  pond[8] = imread("Figures\\Water\\water09.png");
  pond[9] = imread("Figures\\Water\\water10.png");



  for(frameCount = 0;  ; frameCount++) {
    frame.copyTo(prevFrame);
    cap >> frame;
    flip( frame,frame,1 );  // flip the image horizontally
    imshow("Raw Capture", frame);
    absdiff( frame,prevFrame,frameDiff );
    imshow("Difference Frame", frameDiff);
    cvtColor( frameDiff, grayDiff, CV_BGR2GRAY, 1);  // convert to gray scale
    imshow("Gray Difference", grayDiff);

    sum = 0;   // these sums will be used to compute the x and y center of mass
    sumX = 0;
    sumY = 0;
    for (y = yCenter-widthObject; y < yCenter+widthObject; y++) {
      for (x = xCenter-widthObject; x < xCenter+widthObject; x++) {
   	  if (y >= 0 && y < frame.rows && x >= 0 && x < frame.cols) {
          diffValue = grayDiff.at<unsigned char>(y,x);
   	    sum += diffValue;
   	    sumX += x * diffValue;

   	    sumY += y * diffValue;
   	  }
   	}
    }
    if (sum > 0) {  // update the center
      xCenter = ((int)(1.0*sumX/sum) + xCenter) / 2; // x and y center of mass,smoothed
      yCenter = ((int)(1.0*sumY/sum) + yCenter) / 2;
    }
    // put a red circle on the object
    circle( frameDiff, Point(xCenter,yCenter),widthObject/2, Scalar(0,0,255), 2, 8 );
    imshow("Difference Frame",frameDiff);

    pond[ (frameCount % 40)/10 ].copyTo( pondFrame );

    int w1,h1;
    w1 = frame.cols; //width of diff frame
    h1 = frame.rows; //height of diff frame

    int w2,h2;
    w2 = pondFrame.cols; //width of pond
    h2 = pondFrame.rows; //height of pond

    int x2,y2;
    x2 = (w2*xCenter)/w1; // computing center of circle (proportional)
    y2 = (h2*yCenter)/h1;

    circle( pondFrame, Point(x2,y2),widthObject/2, Scalar(0,0,255), 2, 8 );

    Vec3b p1,rvf1;
    for( i = 0; i < RVF1.rows ; i++){ //increment i by 1.
       for( j = 0; j < RVF1.cols ; j++) {
    	   int RVFTx = fx+j-128;
    	   int RVFTy = fy+i-128;
    	   if (RVFTy > 0 && RVFTy < pondFrame.rows && RVFTx > 0 && RVFTx < pondFrame.cols){
    		   p1 = pondFrame.at<Vec3b> (RVFTy, RVFTx);
    		   rvf1 = RVF2.at<Vec3b>(i,j);
    		   if(rvf1[2] > 0 ) { // Ignore black pixels.
    		   	   pondFrame.at<Vec3b> (RVFTy, RVFTx) //top left corner where to overlay frame.
    		   		   = Vec3b((p1[0]+rvf1[0])/2,(p1[1]+rvf1[1])/2,(p1[2]+rvf1[2])/2);
    	       }
    	   }
       }
    }

    redVirtualFish[ (frameCount % 24) ].copyTo( RVF1 );
    int centerRVFx = RVF2.cols / 2;
    int centerRVFy = RVF2.rows / 2; // Center point of a fish
    int diffX = x2 - fx;
    int diffY = y2 - fy; // distance between fish and lure
    int angleRVF = 0 + atan2((diffY),(diffX)) * (180/PI);

    Mat matRotation = getRotationMatrix2D( Point(centerRVFy, centerRVFx), (-angleRVF-90), 1 );
    warpAffine( RVF1, RVF2, matRotation, RVF1.size() );
    // Rotation of a fish

    int speed;
    if ((diffX > 200) || (diffX < -200) or (diffY > 200) || (diffY < -200)) speed = 14;
    else if ((diffX > 185) || (diffX < -185) or (diffY > 185) || (diffY < -185)) speed = 12;
    else if ((diffX > 170) || (diffX < -170) or (diffY > 170) || (diffY < -170))  speed = 9;
    else if ((diffX > 155) || (diffX < -155) or (diffY > 155) || (diffY < -155))  speed = 8;
    else if ((diffX > 140) || (diffX < -140) or (diffY > 140) || (diffY < -140))  speed = 5;
    else if ((diffX > 125) || (diffX < -125) or (diffY > 125) || (diffY < -125))  speed = 4;
    else if ((diffX > 110) || (diffX < -110) or (diffY > 110) || (diffY < -110))  speed = 3;
    else speed = 0;
 // Fish would get slower as it gets closer to the tracker.
 // Once distance between fish and tracker in x or y-axis is lower than absolute value of 110, fish would stop.

    fx = fx + (speed * (cos(angleRVF*PI/180)));
	fy = fy + (speed * (sin(angleRVF*PI/180)));
 // Equation to move a fish (Continuous updates in location of fish)


    char line[100];
    sprintf(line,"Move your hand and be aware of your camera! -Made by Jo (s3479811)-");

    if ( frameCount < 280) {
       putText( pondFrame, line, Point(20,50), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255), 2);
    }


    imshow("Fishes in the pond",pondFrame );





    if(waitKey(20) >= 0) break;
  }
  return 0;
}
