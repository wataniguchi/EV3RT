/*
  Compare matching rate between a pair of image files, covering all combinations
  within the training data directory

  matching algorithm: template in blue and red channel respectively with additional checking

  how to use (from workspace directory):
  ./test-match/testMatchTrainData08
 
  how to compile:

  g++ testMatchTrainData08.cpp -std=c++17 `pkg-config --cflags --libs opencv4` -I.. -o testMatchTrainData08
*/

#include "opencv2/opencv.hpp"
#include "opencv2/stitching.hpp"
using namespace cv;

#include <vector>
#include <algorithm>
#include <filesystem>
namespace fs = std::filesystem;
#include <regex>
#include <numeric>
//#include <iostream>
//#include <sstream>
using namespace std;

#define _log(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    0, __PRETTY_FUNCTION__, ##__VA_ARGS__)

#define TRAINING_PATH "./msad2023_pri/trainData"
#define MAX_TARGET 1

int main() 
{
  const fs::path targetPath(TRAINING_PATH);
  const auto dirIter = fs::directory_iterator(targetPath);
  std::vector<std::string> imgFilenameArray;
  std::vector<Mat> imgArray;
  int maxWidth = 0, maxHeight = 0;
 
  /* real all image files in the target path directory by regular expression */
  for (const fs::path& path : dirIter) {
    std::string s = path.filename().string();
    std::smatch m;
    if ( std::regex_match(s, m, std::regex(R"(smp([0-9]+)-[0-9]+.jpg)")) ) {
      std::string f = m[0].str();
      _log("reading image file %s", f.c_str());
      Mat img = imread(path.string(), IMREAD_UNCHANGED);
      if (img.rows > maxHeight) maxHeight = img.rows;
      if (img.cols > maxWidth ) maxWidth  = img.cols;
      imgArray.push_back(img);
      imgFilenameArray.push_back(s);
    }
  }
  if (imgArray.size() < 2) {
    _log("*** ERROR - # of image files less than two");
    return EXIT_FAILURE;
  }

  for (int i = 0; i < imgFilenameArray.size()-1; i++) {
    for (int j = i+1; j < imgFilenameArray.size(); j++) {
      Mat img_src[2], img_tmp[2], img_tmp_mon[2], result;
      Rect matchedRect[2];
      int src, tmp;
      double match[2];

      if (imgArray[i].cols < imgArray[j].cols) {
	tmp = i;
	src = j;
      } else {
	tmp = j;
	src = i;
      }


      for (int k = 0; k < 2; k++) {
	/* copy source image at the center of a larger empty campus for later process */
	img_src[k] = Mat::zeros(imgArray[src].size()*2, CV_8UC3);
	Mat roi = img_src[k](Rect((img_src[k].cols-imgArray[src].cols)/2, (img_src[k].rows-imgArray[src].rows)/2, imgArray[src].cols, imgArray[src].rows));
	imgArray[src].copyTo(roi);
	if (k == 0) {
	  for (int l = 0; l < img_src[k].rows; l++) {
	    for (int m = 0; m < img_src[k].cols; m++) {
	      /* clear red channel */
	      img_src[k].at<Vec3b>(l,m)[2] = 0;
	    }
	  }
	} else if (k == 1) {
	  for (int l = 0; l < img_src[k].rows; l++) {
	    for (int m = 0; m < img_src[k].cols; m++) {
	      /* clear blue channel */
	      img_src[k].at<Vec3b>(l,m)[0] = 0;
	    }
	  }
	}

	/* prepare template image */
	Mat img_copy = imgArray[tmp].clone();
	if (k == 0) {
	  for (int l = 0; l < img_copy.rows; l++) {
	    for (int m = 0; m < img_copy.cols; m++) {
	      /* clear red channel */
	      img_src[k].at<Vec3b>(l,m)[2] = 0;
	    }
	  }
	} else if (k == 1) {
	  for (int l = 0; l < img_copy.rows; l++) {
	    for (int m = 0; m < img_copy.cols; m++) {
	      /* clear blue channel */
	      img_src[k].at<Vec3b>(l,m)[0] = 0;
	    }
	  }
	}
	/* minimize the size of template image for later process */
	int endCol = 0, endRow = 0;
	int staCol = img_copy.cols, staRow = img_copy.rows;
	for (int l = 0; l < img_copy.rows; l++) {
	  for (int m = 0; m < img_copy.cols; m++) {
	    if (img_copy.at<Vec3b>(l,m) != Vec3b(0,0,0)) {
	      if (l > endRow) endRow = l;
	      if (m > endCol) endCol = m;
	      if (l < staRow) staRow = l;
	      if (m < staCol) staCol = m;
	    }
	  }
	}
	if (endCol == 0) endCol = img_copy.cols;
	if (endRow == 0) endRow = img_copy.rows;
	if (staCol == img_copy.cols) staCol = 0;
	if (staRow == img_copy.rows) staRow = 0;
	Rect roi_rect = Rect(staCol, staRow, ((endCol - staCol) > 0 ? (endCol - staCol):1), ((endRow - staRow) > 0 ? (endRow - staRow):1));
	img_tmp[k] = img_copy(roi_rect);
	/* copy template image at the center of a larger empty campus for monitoring */
	img_tmp_mon[k] = Mat::zeros(img_src[k].size(), CV_8UC3);
	Rect img_tmp_rect = Rect((img_tmp_mon[k].cols-img_tmp[k].cols)/2, (img_tmp_mon[k].rows-img_tmp[k].rows)/2, img_tmp[k].cols, img_tmp[k].rows);
	roi = img_tmp_mon[k](img_tmp_rect);
	img_tmp[k].copyTo(roi);
	rectangle(img_tmp_mon[k], img_tmp_rect, Scalar(255,255,255), 1);

	matchTemplate(img_src[k], img_tmp[k], result, TM_CCOEFF_NORMED);
	/* normalize the result from 0 to 1 */
	//normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
	double mMin, mMax;
	Point minP, maxP;
	minMaxLoc(result, &mMin, &mMax, &minP, &maxP);
	match[k] = mMax;
	matchedRect[k] = Rect(maxP.x, maxP.y, img_tmp[k].cols, img_tmp[k].rows);
	rectangle(img_src[k], matchedRect[k], Scalar(255,255,255), 1);
      }

      Mat img_comm1, img_comm2, img_comm;
      vconcat(img_src[0], img_tmp_mon[0], img_comm1);
      vconcat(img_src[1], img_tmp_mon[1], img_comm2);
      vconcat(img_comm1, img_comm2, img_comm);

      /* intersection of matchedArea[blue] and matchedArea[red] */
      Rect intersectRect = matchedRect[0] & matchedRect[1];
      /* calculate overlapping rate */
      float unionArea = matchedRect[0].area() + matchedRect[1].area() - intersectRect.area();
      float iou = intersectRect.area() / unionArea;
      if (iou >= 0.6) {
	_log("compared %s : %s, iou = %f, match[blue] = %lf, match[red] = %lf", imgFilenameArray[i].c_str(), imgFilenameArray[j].c_str(), iou, match[0], match[1]);
	/* display the matching result */
	imshow("matches", img_comm);
	waitKey(0);
      } else {
	_log("compared %s : %s, iou = %f, NOT matched", imgFilenameArray[i].c_str(), imgFilenameArray[j].c_str(), iou);
      }
    }
  }
  
  return EXIT_SUCCESS;
}
      
