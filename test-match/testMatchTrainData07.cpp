/*
  Compare matching rate between a pair of image files, covering all combinations
  within the training data directory

  matching algorithm: template in blue and red channel respectively

  how to use (from workspace directory):
  ./test-match/testMatchTrainData07
 
  how to compile:

  g++ testMatchTrainData07.cpp -std=c++17 `pkg-config --cflags --libs opencv4` -I.. -o testMatchTrainData07
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
      int src, tmp;
      float match[2];

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
	Mat roi = img_src[k](Rect((img_src[k].size().width-imgArray[src].size().width)/2, (img_src[k].size().height-imgArray[src].size().height)/2, imgArray[src].size().width, imgArray[src].size().height));
	imgArray[src].copyTo(roi);
	if (k == 0) {
	  for (int l = 0; l < img_src[k].size().height; l++) {
	    for (int m = 0; m < img_src[k].size().width; m++) {
	      /* copy blue channel to other channels */
	      img_src[k].at<Vec3b>(l,m)[1] = img_src[k].at<Vec3b>(l,m)[0];
	      img_src[k].at<Vec3b>(l,m)[2] = img_src[k].at<Vec3b>(l,m)[0];
	    }
	  }
	} else if (k == 1) {
	  for (int l = 0; l < img_src[k].size().height; l++) {
	    for (int m = 0; m < img_src[k].size().width; m++) {
	      /* copy red channel to other channels */
	      img_src[k].at<Vec3b>(l,m)[0] = img_src[k].at<Vec3b>(l,m)[2];
	      img_src[k].at<Vec3b>(l,m)[1] = img_src[k].at<Vec3b>(l,m)[2];
	    }
	  }
	}

	/* prepare template image */
	Mat img_copy = imgArray[tmp].clone();
	if (k == 0) {
	  for (int l = 0; l < img_copy.size().height; l++) {
	    for (int m = 0; m < img_copy.size().width; m++) {
	      /* copy blue channel to other channels */
	      img_copy.at<Vec3b>(l,m)[1] = img_copy.at<Vec3b>(l,m)[0];
	      img_copy.at<Vec3b>(l,m)[2] = img_copy.at<Vec3b>(l,m)[0];
	    }
	  }
	} else if (k == 1) {
	  for (int l = 0; l < img_copy.size().height; l++) {
	    for (int m = 0; m < img_copy.size().width; m++) {
	      /* copy red channel to other channels */
	      img_copy.at<Vec3b>(l,m)[0] = img_copy.at<Vec3b>(l,m)[2];
	      img_copy.at<Vec3b>(l,m)[1] = img_copy.at<Vec3b>(l,m)[2];
	    }
	  }
	}
	/* minimize the size of template image for later process */
	int endCol = 0, endRow = 0;
	int staCol = img_copy.size().width, staRow = img_copy.size().height;
	for (int l = 0; l < img_copy.size().height; l++) {
	  for (int m = 0; m < img_copy.size().width; m++) {
	    if (img_copy.at<Vec3b>(l,m) != Vec3b(0,0,0)) {
	      if (l > endRow) endRow = l;
	      if (m > endCol) endCol = m;
	      if (l < staRow) staRow = l;
	      if (m < staCol) staCol = m;
	    }
	  }
	}
	if (endCol == 0) endCol = img_copy.size().width;
	if (endRow == 0) endRow = img_copy.size().height;
	if (staCol == img_copy.size().width) staCol = 0;
	if (staRow == img_copy.size().height) staRow = 0;
	Rect roi_rect = Rect(staCol, staRow, ((endCol - staCol) > 0 ? (endCol - staCol):1), ((endRow - staRow) > 0 ? (endRow - staRow):1));
	img_tmp[k] = img_copy(roi_rect);
	/* copy template image at the center of a larger empty campus for monitoring */
	img_tmp_mon[k] = Mat::zeros(img_src[k].size(), CV_8UC3);
	Rect img_tmp_rect = Rect((img_tmp_mon[k].size().width-img_tmp[k].size().width)/2, (img_tmp_mon[k].size().height-img_tmp[k].size().height)/2, img_tmp[k].size().width, img_tmp[k].size().height);
	roi = img_tmp_mon[k](img_tmp_rect);
	img_tmp[k].copyTo(roi);
	rectangle(img_tmp_mon[k], img_tmp_rect, Scalar(255,255,255), 1);

	matchTemplate(img_src[k], img_tmp[k], result, TM_CCOEFF);
	/* normalize the result from 0 to 1 */
	//normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
	/* convert two-dimentional matrix to single-dimention */
	Mat one_row = result.reshape(1, 1);
	std::vector<float> one_row_vec, one_row_index;
	/* convert Mat to std::vector */
	one_row_vec.assign(one_row.begin<float>(), one_row.end<float>());
	std::vector<int> index(one_row_vec.size());
	/* generate sequence of integers 0,1,2,3,4... */
	std::iota(index.begin(), index.end(), 0);
	/* sort */
	std::sort(index.begin(), index.end(), [&](int a, int b) {return one_row_vec[a] > one_row_vec[b];});

	for (int i = 0; i < MAX_TARGET; i++) {
	  int x = index[i] % result.cols;
	  int y = index[i] / result.cols;
	  Rect roi_rect = Rect(x, y, img_tmp[k].size().width, img_tmp[k].size().height);
	  rectangle(img_src[k], roi_rect, Scalar(255,255,255), 1);
	}
	/* save degree of match for later */
	match[k] = one_row_vec[index[0]];
      }

      Mat img_comm1, img_comm2, img_comm;
      vconcat(img_src[0], img_tmp_mon[0], img_comm1);
      vconcat(img_src[1], img_tmp_mon[1], img_comm2);
      vconcat(img_comm1, img_comm2, img_comm);
      /* display the matching result */
      imshow("matches", img_comm);

      _log("compared %s : %s, match blue = %f, match red = %f", imgFilenameArray[i].c_str(), imgFilenameArray[j].c_str(), match[0], match[1]);
      
      waitKey(0);
    }
  }
  
  return EXIT_SUCCESS;
}
      
