/*
  Compare matching rate between a pair of image files, covering all combinations
  within the training data directory

  matching algorithm: histogram

  how to use (from workspace directory):
  ./test-match/testMatchTrainData02
 
  how to compile:

  g++ testMatchTrainData02.cpp -std=c++17 `pkg-config --cflags --libs opencv4` -I.. -o testMatchTrainData02
*/

#include "opencv2/opencv.hpp"
#include "opencv2/stitching.hpp"
using namespace cv;

#include <vector>
#include <algorithm>
#include <filesystem>
namespace fs = std::filesystem;
#include <regex>
//#include <iostream>
//#include <sstream>
using namespace std;

#define _log(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    0, __PRETTY_FUNCTION__, ##__VA_ARGS__)

#define TRAINING_PATH "./msad2023_pri/trainData"
#define NUM_SRC 2
#define MIN_KPTS 20
#define HIST_SIZE 100

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
      Mat img1, img2, roi, channels[3], B1, R1, B2, R2, img_match;
      int hist_size = HIST_SIZE;
      float range[] = {0, 256};
      const float* hist_range = range;
      
      /* copy the original image at the center of a larger empty campus for later process */
      img1 = Mat::zeros(maxHeight, maxWidth, CV_8UC3);
      roi = img1(Rect((maxWidth-imgArray[i].cols)/2, (maxHeight-imgArray[i].rows)/2, imgArray[i].cols, imgArray[i].rows));
      imgArray[i].copyTo(roi);
      img2 = Mat::zeros(maxHeight, maxWidth, CV_8UC3);
      roi = img2(Rect((maxWidth-imgArray[j].cols)/2, (maxHeight-imgArray[j].rows)/2, imgArray[j].cols, imgArray[j].rows));
      imgArray[j].copyTo(roi);
      /* generate histgrams */
      split(img1, channels);
      calcHist(&channels[0], 1, 0, Mat(), B1, 1, &hist_size, &hist_range);
      calcHist(&channels[2], 1, 0, Mat(), R1, 1, &hist_size, &hist_range);
      split(img2, channels);
      calcHist(&channels[0], 1, 0, Mat(), B2, 1, &hist_size, &hist_range);
      calcHist(&channels[2], 1, 0, Mat(), R2, 1, &hist_size, &hist_range);
      /* normalize the histgrams */
      normalize(B1, B1, 0.0, 1.0, NORM_MINMAX, -1, Mat());
      normalize(R1, R1, 0.0, 1.0, NORM_MINMAX, -1, Mat());
      normalize(B2, B2, 0.0, 1.0, NORM_MINMAX, -1, Mat());
      normalize(R2, R2, 0.0, 1.0, NORM_MINMAX, -1, Mat());

      /* compare histgram of two images in two colors respectively, and make their average as the result */
      double distB, distR, avgDist;
      distB = compareHist(B1, B2, HISTCMP_CORREL);
      distR = compareHist(R1, R2, HISTCMP_CORREL);
      avgDist = (distB + distR) / 2.0;
      _log("compared %s : %s, avg dist = %lf", imgFilenameArray[i].c_str(), imgFilenameArray[j].c_str(), avgDist);
      
      //waitKey(0);
    }
  }
  
  return EXIT_SUCCESS;
}
      
