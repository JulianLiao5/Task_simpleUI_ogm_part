/*************************************************************************
	> File Name: test_ogm.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月28日 星期二 14时41分45秒
 ************************************************************************/

#include<iostream>

#include <opencv2/core.hpp>

#include "occupancy_status_grid_map.h"

using namespace std;



int main(int argc, char *argv[]) {
  cv::Size map_size(800, 800);
  double size_per_pixel = 0.05;
  OccupancyStatusGridMap *OGM = new OccupancyStatusGridMap(size_per_pixel, map_size);
  
  OGM->Reset();



  OGM->DrawLineInMap(cv::Point2d(0, 20), cv::Point2d(0, -20), CellStatus::UNKNOWN);
  OGM->DrawLineInMap(cv::Point2d(-20, 0), cv::Point2d(20, 0), CellStatus::UNKNOWN);
  OGM->DrawLineInMap(cv::Point2d(0, 0), cv::Point2d(20, 20), CellStatus::UNKNOWN);
  OGM->DrawLineInMap(cv::Point2d(0, 0), cv::Point2d(20, -20), CellStatus::UNKNOWN);
  
  for (int i = -2; i <= 16; i += 3) {
    for (int j = -6; j <= 6; j += 4) {
      OGM->DrawRectInMap(Rect(i, j, 0.11, 2), CellStatus::UNKNOWN);
    }
  }

  OGM->DrawRectInMap(Rect(0, 0, 0.2, 0.11), CellStatus::UNKNOWN);
  OGM->DrawRectInMap(Rect(5, 0, 0.2, 0.11), CellStatus::UNKNOWN);
  OGM->DrawRectInMap(Rect(10, 0, 0.2, 0.11), CellStatus::UNKNOWN);
  OGM->DrawRectInMap(Rect(15, 0, 0.2, 0.11), CellStatus::UNKNOWN);





  cv::Mat m = OGM->Visualize();
  // cv::namedWindow("map", cv::WINDOW_NORMAL);
  while (1) {
    cv::imshow("map", m);
    if ('q' == cv::waitKey(20)) {
        break;
    }
  }

  return 0;
}

