/*
  how to compile:

  g++ batch_stitch.cpp -std=c++17 `pkg-config --cflags --libs opencv4` -I.. -o batch_stitch
*/

#include "opencv2/opencv.hpp"
#include "opencv2/stitching.hpp"
using namespace cv;

#include <vector>
#include <filesystem>
namespace fs = std::filesystem;
#include <regex>

#define _log(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    0, __PRETTY_FUNCTION__, ##__VA_ARGS__)

int main() 
{
  const fs::path targetPath("./msad2023_pri/work");
  const auto dirIter = fs::directory_iterator(targetPath);
  std::vector<Mat> imgArray;
  
  for (const fs::path& path : dirIter) {
    std::string s = path.filename().string();
    std::smatch m;
    if ( std::regex_match(s, m, std::regex(R"(img[0-9]+.jpg)")) ) {
      std::string f = m[0].str();
      _log("reading image file %s", f.c_str());
      Mat img = imread(path.string(), IMREAD_UNCHANGED);
      imgArray.push_back(img);
    }
  }

  Mat pano;
  Stitcher::Mode mode = Stitcher::PANORAMA;
  Ptr<cv::Stitcher> stitcher = Stitcher::create(mode);
  stitcher->stitch(imgArray, pano);
  imshow("panorama", pano);
  waitKey(0);

  return EXIT_SUCCESS;
}
