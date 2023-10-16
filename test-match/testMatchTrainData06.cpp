/*
  Compare matching rate between a pair of image files, covering all combinations
  within the training data directory

  matching algorithm: template

  how to use (from workspace directory):
  ./test-match/testMatchTrainData06
 
  how to compile:

  g++ testMatchTrainData06.cpp -std=c++17 `pkg-config --cflags --libs opencv4` -I.. -o testMatchTrainData06
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
  Ptr<AKAZE> detector = AKAZE::create();
 
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
      Mat img_src, img_tmp, img_tmp_mon, result;
      int src, tmp;

      if (imgArray[i].cols < imgArray[j].cols) {
	tmp = i;
	src = j;
      } else {
	tmp = j;
	src = i;
      }
      /* copy source image at the center of a larger empty campus for later process */
      img_src = Mat::zeros(imgArray[src].size()*2, CV_8UC3);
      Mat roi = img_src(Rect((img_src.cols-imgArray[src].cols)/2, (img_src.rows-imgArray[src].rows)/2, imgArray[src].cols, imgArray[src].rows));
      imgArray[src].copyTo(roi);
      /* minimize the size of template image for later process */
      int endCol = 0, endRow = 0;
      int staCol = imgArray[tmp].cols, staRow = imgArray[tmp].rows;
      for (int k = 0; k < imgArray[tmp].rows; k++) {
	for (int l = 0; l < imgArray[tmp].cols; l++) {
	  if (imgArray[tmp].at<Vec3b>(k,l) != Vec3b(0,0,0)) {
	    if (k > endRow) endRow = k;
	    if (l > endCol) endCol = l;
	    if (k < staRow) staRow = k;
	    if (l < staCol) staCol = l;
	  }
	}
      }
      Rect roi_rect = Rect(staCol, staRow, (endCol - staCol), (endRow - staRow));
      img_tmp = imgArray[tmp](roi_rect);
      /* copy template image at the center of a larger empty campus for monitoring */
      img_tmp_mon = Mat::zeros(img_src.size(), CV_8UC3);
      Rect img_tmp_rect = Rect((img_tmp_mon.cols-img_tmp.cols)/2, (img_tmp_mon.rows-img_tmp.rows)/2, img_tmp.cols, img_tmp.rows);
      roi = img_tmp_mon(img_tmp_rect);
      img_tmp.copyTo(roi);
      rectangle(img_tmp_mon, img_tmp_rect, Scalar(255,255,255), 1);

      matchTemplate(img_src, img_tmp, result, TM_CCOEFF_NORMED);
      /* normalize the result from 0 to 1 */
      //normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
      double mMin, mMax;
      Point minP, maxP;
      minMaxLoc(result, &mMin, &mMax, &minP, &maxP);
      roi_rect = Rect(maxP.x, maxP.y, img_tmp.cols, img_tmp.rows);
      rectangle(img_src, roi_rect, Scalar(255,255,255), 1);

      Mat img_comm;
      vconcat(img_src, img_tmp_mon, img_comm);
      /* display the matching result */
      imshow("matches", img_comm);

      _log("compared %s : %s, match degree = %f", imgFilenameArray[i].c_str(), imgFilenameArray[j].c_str(), mMax);
      
      waitKey(0);
    }
  }
  
  return EXIT_SUCCESS;
}
      
