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
#include <chrono>
#include <iostream>
#include <sstream>

#define _log(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    0, __PRETTY_FUNCTION__, ##__VA_ARGS__)

#define TARGET_PATH "./msad2023_pri/work"
#define MORPH_KERNEL_SIZE 9

int b_min_tre=0,g_min_tre=0,r_min_tre=60,b_max_tre=50,g_max_tre=40,r_max_tre=255;
int b_min_dec=40,g_min_dec=0,r_min_dec=0,b_max_dec=255,g_max_dec=60,r_max_dec=30;
int gs_min=10,gs_max=100;

void binalizeWithColorMask(Mat& img_orig, Scalar& bgr_min, Scalar& bgr_max, int gs_min, int gs_max, Mat& img_bin_mor) {
  Mat img_mask, img_ext, img_gray, img_bin;
  /* extract areas by color */
  inRange(img_orig, bgr_min, bgr_max, img_mask);
  bitwise_and(img_orig, img_orig, img_ext, img_mask);
  /* convert the extracted image from BGR to grayscale */
  cvtColor(img_ext, img_gray, COLOR_BGR2GRAY);
  /* binarize the image */
  inRange(img_gray, gs_min, gs_max, img_bin);
  /* remove noise */
  Mat kernel = Mat::ones(Size(MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), CV_8UC1);
  morphologyEx(img_bin, img_bin_mor, MORPH_CLOSE, kernel);
  return;
}

int main() 
{
  std::chrono::system_clock::time_point tpStart, tpEnd;
  tpStart = std::chrono::system_clock::now();

  const fs::path targetPath(TARGET_PATH);
  const auto dirIter = fs::directory_iterator(targetPath);
  std::vector<Mat> imgArray;

  Scalar bgr_min_tre = Scalar(b_min_tre,g_min_tre,r_min_tre);
  Scalar bgr_max_tre = Scalar(b_max_tre,g_max_tre,r_max_tre);
  Scalar bgr_min_dec = Scalar(b_min_dec,g_min_dec,r_min_dec);
  Scalar bgr_max_dec = Scalar(b_max_dec,g_max_dec,r_max_dec);
  
  /* real all image files in the target path directory by regular expression */
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

  /* stitch all the images into a panoramic image */
  Mat pano;
  Stitcher::Mode mode = Stitcher::PANORAMA;
  Ptr<cv::Stitcher> stitcher = Stitcher::create(mode);
  stitcher->stitch(imgArray, pano);

  /* measure elapsed time */
  tpEnd = std::chrono::system_clock::now();
  std::uint32_t elaps = std::chrono::duration_cast<std::chrono::milliseconds>(tpEnd - tpStart).count();
  _log("elaps = %05u (msec)", elaps);

  /* prepare an empty image */
  Mat img_bin_red, img_bin_blue;
  Mat pano_simple = Mat::zeros(pano.size(), CV_8UC3);
  /* binalize with red mask */
  binalizeWithColorMask(pano, bgr_min_tre, bgr_max_tre, gs_min, gs_max, img_bin_red);
  /* copy the binary image to red channel of simplified image */
  for (int i = 0; i < pano.size().height; i++) {
    for (int j = 0; j < pano.size().width; j++) {
      pano_simple.at<Vec3b>(i,j)[2] = img_bin_red.at<uchar>(i,j);
    }
  }
  /* binalize with blue mask */
  binalizeWithColorMask(pano, bgr_min_dec, bgr_max_dec, gs_min, gs_max, img_bin_blue);
  /* copy the binary image to blue channel of simplified image */
  for (int i = 0; i < pano.size().height; i++) {
    for (int j = 0; j < pano.size().width; j++) {
      pano_simple.at<Vec3b>(i,j)[0] = img_bin_blue.at<uchar>(i,j);
    }
  }

  /* transmit images */
  Mat img_comm;
  vconcat(pano, pano_simple, img_comm);
  imshow("panorama", img_comm);
  int c = waitKey(0);

  if ( c == 's' || c == 'S' ) {
    /* generate a unique file name suffix using time since epoch */
    const std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    const std::chrono::system_clock::duration duration = now.time_since_epoch();
    const long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    std::ostringstream oss;
    oss << TARGET_PATH << "/pan" << std::setw(13) << std::setfill('0') << ms << ".jpg";
    _log("writing panorama as file - %s", oss.str().c_str());
    imwrite(oss.str(), pano);
  }

  return EXIT_SUCCESS;
}
