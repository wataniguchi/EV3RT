/*
  Compare matching rate between a pair of image files, covering all combinations
  within the training data directory

  matching algorithm: AKAZE to detect key points, brute-force to match key points, blue and red channel combines as grayscale

  how to use (from workspace directory):
  ./test-match/testMatchTrainData05
 
  how to compile:

  g++ testMatchTrainData05.cpp -std=c++17 `pkg-config --cflags --libs opencv4` -I.. -o testMatchTrainData05
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
#define MIN_KPTS 20

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
      Mat img1, img2, img1gray, img2gray, roi, desc1, desc2, img_match;
      vector<KeyPoint> kpts1, kpts2;
      
      /* copy the original image at the center of a larger empty campus for later process */
      img1 = Mat::zeros(maxHeight, maxWidth, CV_8UC3);
      roi = img1(Rect((maxWidth-imgArray[i].cols)/2, (maxHeight-imgArray[i].rows)/2, imgArray[i].cols, imgArray[i].rows));
      imgArray[i].copyTo(roi);
      img2 = Mat::zeros(maxHeight, maxWidth, CV_8UC3);
      roi = img2(Rect((maxWidth-imgArray[j].cols)/2, (maxHeight-imgArray[j].rows)/2, imgArray[j].cols, imgArray[j].rows));
      imgArray[j].copyTo(roi);
      /* convert to gray scale */
      cvtColor(img1, img1gray, COLOR_BGR2GRAY);
      cvtColor(img2, img2gray, COLOR_BGR2GRAY);
      /* detect key points */
      detector->detectAndCompute(img1gray, noArray(), kpts1, desc1);
      detector->detectAndCompute(img2gray, noArray(), kpts2, desc2);
      /* match key points between the files by brute-force */
      BFMatcher matcher(NORM_HAMMING);
      vector<DMatch> matches;
      matcher.match(desc1, desc2, matches);
      int num_match = matches.size();
      _log("match size = %d", matches.size());
      if (num_match < MIN_KPTS) {
	_log("*** WARNING - number of matched key points not enough");
      }

      /* display the matching result, focusing on highest MIN_KPTS matches */
      nth_element(begin(matches), begin(matches) + (num_match >= MIN_KPTS ? MIN_KPTS:num_match) - 1, end(matches));
      matches.erase(begin(matches) + (num_match >= MIN_KPTS ? MIN_KPTS:num_match), end(matches));
      drawMatches(img1, kpts1, img2, kpts2, matches, img_match);
      imshow("matches", img_match);

      float sumDist = 0.0;
      for (int k = 0; k < matches.size(); k++) {
	sumDist += matches[k].distance;
      };
      _log("compared %s : %s, avg dist = %f", imgFilenameArray[i].c_str(), imgFilenameArray[j].c_str(), sumDist/num_match);
      
      waitKey(0);
    }
  }
  
  return EXIT_SUCCESS;
}
      
