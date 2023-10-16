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
#define BLK_AREA_MIN 100
#define LINE_THICKNESS 1
#define CHANNEL_BLUE 0
#define CHANNEL_GREEN 1
#define CHANNEL_RED 2

void locateBlocks(vector<vector<Point>>& contours, vector<Vec4i>& hierarchy,
		  vector<vector<float>>& cnt_idx) { /* cnt_idx: area, idx, w/h, x, y */
  for (int i = 0; i < contours.size(); i++) {
    if (hierarchy[i][3] == -1) { /* if contour is external */
      vector<Point> cnt = contours[i];
      float area = contourArea(cnt);
      Moments mom = moments(cnt);
      /* add 1e-5 to avoid division by zero */
      float x = mom.m10 / (mom.m00 + 1e-5);
      float y = mom.m01 / (mom.m00 + 1e-5);
      /* calculate a bounding box around the identified contour */
      Rect bbcnt = boundingRect(cnt);
      float wh = static_cast<float>(bbcnt.width) / bbcnt.height; /* width / height */
      vector<Point> hull;
      convexHull(cnt, hull);
      if (area > BLK_AREA_MIN && wh > 0.5 && wh < 2.0 &&
	  2.5*area > contourArea(hull)) { /* the contour and its hull are not much different */
	if (hierarchy[i][2] == -1) { /* if the contour has no child */
	  cnt_idx.push_back({area, float(i), wh, x, y});
	} else { /* ensure the contour is not donut-shaped */
	  bool donut = false;
	  for (int j = hierarchy[i][2]; j != -1; j = hierarchy[j][0]){ /* traverse all child */
	    vector<Point> cnt_chd = contours[j];
	    float area_chd = contourArea(cnt_chd);
	    if (10.0 * area_chd > area) donut = true;
	  }
	  if (donut == false) {
	    cnt_idx.push_back({area, float(i), wh, x, y});
	  }
	}
      }
    }
  }
  return;
}

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
      Mat img_src[2], img_src_mon[2], img_tmp[2], img_tmp_mon[2], result;
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
	Mat img_bin;
	/* copy source image at the center of a larger empty campus for later process */
	img_src[k] = Mat::zeros(imgArray[src].size()*2, CV_8UC3);
	Mat roi = img_src[k](Rect((img_src[k].cols-imgArray[src].cols)/2, (img_src[k].rows-imgArray[src].rows)/2, imgArray[src].cols, imgArray[src].rows));
	imgArray[src].copyTo(roi);
	if (k == 0) {
	  for (int l = 0; l < img_src[k].rows; l++) {
	    for (int m = 0; m < img_src[k].cols; m++) {
	      /* clear red channel */
	      img_src[k].at<Vec3b>(l,m)[CHANNEL_RED] = 0;
	    }
	  }
	} else if (k == 1) {
	  for (int l = 0; l < img_src[k].rows; l++) {
	    for (int m = 0; m < img_src[k].cols; m++) {
	      /* clear blue channel */
	      img_src[k].at<Vec3b>(l,m)[CHANNEL_BLUE] = 0;
	    }
	  }
	}
	/* copy source image for monitoring */
	img_src_mon[k] = img_src[k].clone();

	/* locate blocks in template image */
	if (k == 0) {
	  /* extract blue channel */
	  extractChannel(img_src[k], img_bin, CHANNEL_BLUE);
	} else if (k == 1) {
	  /* extract red channel */
	  extractChannel(img_src[k], img_bin, CHANNEL_RED);
	}
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(img_bin, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	vector<vector<float>> cnt_idx; /* cnt_idx: area, idx, w/h, x, y */
	locateBlocks(contours, hierarchy, cnt_idx);
	if (cnt_idx.size() > 0) {
	  sort(cnt_idx.begin(), cnt_idx.end(), greater<>());
	  /* draw the largest contour on the original image in yellow */
	  polylines(img_src_mon[k], contours[cnt_idx[0][1]], true, Scalar(0,255,255), LINE_THICKNESS);
	  _log("area = %f, wh = %f, x = %f, y = %f", cnt_idx[0][0], cnt_idx[0][2], cnt_idx[0][3], cnt_idx[0][4]);
	}

	/* prepare template image */
	Mat img_tmp_copy = imgArray[tmp].clone();
	if (k == 0) {
	  for (int l = 0; l < img_tmp_copy.rows; l++) {
	    for (int m = 0; m < img_tmp_copy.cols; m++) {
	      /* clear red channel */
	      img_tmp_copy.at<Vec3b>(l,m)[CHANNEL_RED] = 0;
	    }
	  }
	} else if (k == 1) {
	  for (int l = 0; l < img_tmp_copy.rows; l++) {
	    for (int m = 0; m < img_tmp_copy.cols; m++) {
	      /* clear blue channel */
	      img_tmp_copy.at<Vec3b>(l,m)[CHANNEL_BLUE] = 0;
	    }
	  }
	}
	/* minimize the size of template image for later process */
	int endCol = 0, endRow = 0;
	int staCol = img_tmp_copy.cols, staRow = img_tmp_copy.rows;
	for (int l = 0; l < img_tmp_copy.rows; l++) {
	  for (int m = 0; m < img_tmp_copy.cols; m++) {
	    if (img_tmp_copy.at<Vec3b>(l,m) != Vec3b(0,0,0)) {
	      if (l > endRow) endRow = l;
	      if (m > endCol) endCol = m;
	      if (l < staRow) staRow = l;
	      if (m < staCol) staCol = m;
	    }
	  }
	}
	if (endCol == 0) endCol = img_tmp_copy.cols;
	if (endRow == 0) endRow = img_tmp_copy.rows;
	if (staCol == img_tmp_copy.cols) staCol = 0;
	if (staRow == img_tmp_copy.rows) staRow = 0;
	Rect roi_rect = Rect(staCol, staRow, ((endCol - staCol) > 0 ? (endCol - staCol):1), ((endRow - staRow) > 0 ? (endRow - staRow):1));
	img_tmp[k] = img_tmp_copy(roi_rect);
	
	/* locate blocks in template image */
	if (k == 0) {
	  /* extract blue channel */
	  extractChannel(img_tmp[k], img_bin, CHANNEL_BLUE);
	} else if (k == 1) {
	  /* extract red channel */
	  extractChannel(img_tmp[k], img_bin, CHANNEL_RED);
	}
	contours.clear();
	hierarchy.clear();
	findContours(img_bin, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	cnt_idx.clear(); /* cnt_idx: area, idx, w/h, x, y */
	locateBlocks(contours, hierarchy, cnt_idx);
	if (cnt_idx.size() > 0) {
	  sort(cnt_idx.begin(), cnt_idx.end(), greater<>());
	  /* draw the largest contour on the original image in yellow */
	  polylines(img_tmp[k], contours[cnt_idx[0][1]], true, Scalar(0,255,255), LINE_THICKNESS);
	  _log("area = %f, wh = %f, x = %f, y = %f", cnt_idx[0][0], cnt_idx[0][2], cnt_idx[0][3], cnt_idx[0][4]);
	}
	
	/* copy template image at the center of a larger empty campus for monitoring */
	img_tmp_mon[k] = Mat::zeros(img_src[k].size(), CV_8UC3);
	Rect img_tmp_rect = Rect((img_tmp_mon[k].cols-img_tmp[k].cols)/2, (img_tmp_mon[k].rows-img_tmp[k].rows)/2, img_tmp[k].cols, img_tmp[k].rows);
	roi = img_tmp_mon[k](img_tmp_rect);
	img_tmp[k].copyTo(roi);
	rectangle(img_tmp_mon[k], img_tmp_rect, Scalar(255,255,255), LINE_THICKNESS);

	matchTemplate(img_src[k], img_tmp[k], result, TM_CCOEFF_NORMED);
	/* normalize the result from 0 to 1 */
	//normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
	double mMin, mMax;
	Point minP, maxP;
	minMaxLoc(result, &mMin, &mMax, &minP, &maxP);
	match[k] = mMax;
	matchedRect[k] = Rect(maxP.x, maxP.y, img_tmp[k].cols, img_tmp[k].rows);	
	rectangle(img_src_mon[k], matchedRect[k], Scalar(255,255,255), LINE_THICKNESS);
      }

      Mat img_comm1, img_comm2, img_comm;
      vconcat(img_src_mon[0], img_tmp_mon[0], img_comm1);
      vconcat(img_src_mon[1], img_tmp_mon[1], img_comm2);
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
      
