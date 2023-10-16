/*
  Compare matching rate between a pair of image files, covering all combinations
  within the training data directory

  matching algorithm: Hu Moments

  how to use (from workspace directory):
  ./test-match/testMatchTrainData03
 
  how to compile:

  g++ testMatchTrainData03.cpp -std=c++17 `pkg-config --cflags --libs opencv4` -I.. -o testMatchTrainData03
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
      Mat img1, img2, roi, channels1[3], channels2[3];
      
      /* copy the original image at the center of a larger empty campus for later process */
      img1 = Mat::zeros(maxHeight, maxWidth, CV_8UC3);
      roi = img1(Rect((maxWidth-imgArray[i].cols)/2, (maxHeight-imgArray[i].rows)/2, imgArray[i].cols, imgArray[i].rows));
      imgArray[i].copyTo(roi);
      img2 = Mat::zeros(maxHeight, maxWidth, CV_8UC3);
      roi = img2(Rect((maxWidth-imgArray[j].cols)/2, (maxHeight-imgArray[j].rows)/2, imgArray[j].cols, imgArray[j].rows));
      imgArray[j].copyTo(roi);
      /* split channels */
      split(img1, channels1);
      split(img2, channels2);

      /* compare two images in two colors respectively, and make their average as the result */
      double distB, distR, avgDist;
      
      distB = matchShapes(channels1[0], channels2[0], CONTOURS_MATCH_I3, 0);
      distR = matchShapes(channels1[2], channels2[2], CONTOURS_MATCH_I3, 0);
      avgDist = (distB + distR) / 2.0;
      _log("compared %s : %s, B dist = %lf, R dist = %lf, avg dist = %lf", imgFilenameArray[i].c_str(), imgFilenameArray[j].c_str(), distB, distR, avgDist);
      
      //waitKey(0);
    }
  }
  
  return EXIT_SUCCESS;
}
      
