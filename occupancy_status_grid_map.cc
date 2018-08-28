#include <glog/logging.h>
#include "grid.h"
#include "occupancy_status_grid_map.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

OccupancyStatusGridMap::OccupancyStatusGridMap(const double cell_size,
                                               const cv::Size &grid_size)
    : Grid(cell_size, grid_size, UNKNOWN) {
  world_to_map << 0, -1 / cell_size, grid_size.width / 2, -1 / cell_size, 0,
      grid_size.height / 2, 0, 0, 1;
  map_to_world << 0, -1 * cell_size, grid_size.height / 2 * cell_size, -1 * cell_size, 0,
          grid_size.width / 2 * cell_size, 0, 0, 1;
}
cv::Mat OccupancyStatusGridMap::Visualize() const {
//  static const uchar kColorFree = uchar(255);
//  static const uchar kColorOccupied = uchar(0);
//  static const uchar kColorUnknown = uchar(128);

  static const cv::Vec3b kColorFree(255,255,255);
  static const cv::Vec3b kColorOccupied(0,0,255);
  static const cv::Vec3b kColorUnknown(128,128,128);

  cv::Mat img(grid_size(), CV_8UC3, kColorUnknown);
  const std::vector<CellStatus> &cells = this->cells();
  for (int r = 0; r < img.rows; r++) {
    for (int c = 0; c < img.cols; c++) {
      const size_t i = static_cast<size_t>((r) * img.cols + c);
      CHECK_LT(i, cells.size());
//      if (cells[i] == CellStatus::FREE) {
//        img.at<uchar>(r, c) = kColorFree;
//      } else if (cells[i] == CellStatus::OCCUPIED) {
//        img.at<uchar>(r, c) = kColorOccupied;
//      } else if (cells[i] == CellStatus::UNKNOWN) {
//        img.at<uchar>(r, c) = kColorUnknown;
//      }
      if (cells[i] == CellStatus::FREE) {
        img.at<cv::Vec3b>(r, c) = kColorFree;
      } else if (cells[i] == CellStatus::OCCUPIED) {
        img.at<cv::Vec3b>(r, c) = kColorOccupied;
      } else if (cells[i] == CellStatus::UNKNOWN) {
        img.at<cv::Vec3b>(r, c) = kColorUnknown;
      }
    }
  }
  return img;
}
bool OccupancyStatusGridMap::DrawRectInMap(const Rect &r,const CellStatus &val) {
  if(r.GetHeight()<=cell_size()||r.GetWidth()<=cell_size())
    return false;
  Eigen::Vector3d vtl_in_map = world_to_map * r.tl();
  Eigen::Vector3d vbr_in_map = world_to_map * r.br();
  for (int i = floor(vtl_in_map.x()); i < floor(vbr_in_map.x()); i++) {
    for (int j = floor(vtl_in_map.y()); j < floor(vbr_in_map.y()); j++) {
//      if (!SetValue(i, j, val)) {
//        return false;
        SetValue(i, j, val);
//      }
    }
  }
  return true;
}

bool OccupancyStatusGridMap::DrawLineInMap(const cv::Point2d &begin, const cv::Point2d &end, const CellStatus &value) {
  Eigen::Vector3d _begin = world_to_map * Eigen::Vector3d(begin.x, begin.y, 1);
  Eigen::Vector3d _end = world_to_map * Eigen::Vector3d(end.x, end.y, 1);
  double x_dif = _end.x()-_begin.x();
  double y_dif = _end.y()-_begin.y();
  double slope = y_dif/x_dif;
  if(fabs(x_dif) >= fabs(y_dif)){
    if(_end.x() >= _begin.x()){
      for(int i=_begin.x();i<_end.x();++i){
        auto temp_x=i;
        auto temp_y=_begin.y()+(temp_x-_begin.x())*slope;
        SetValue(temp_x,temp_y,value);
      }
    }
    else{
      for(int i=_begin.x();i>_end.x();--i){
        auto temp_x=i;
        auto temp_y=_begin.y()+(temp_x-_begin.x())*slope;
        SetValue(temp_x,temp_y,value);
      }
    }
  }
  else{
    if(_end.y() >= _begin.y()){
      for(int i=_begin.y();i<_end.y();++i){
        auto temp_y=i;
        auto temp_x=_begin.x()+(temp_y-_begin.y())/slope;
        SetValue(temp_x,temp_y,value);
      }
    }
    else{
      for(int i=_begin.y();i>_end.y();--i){
        auto temp_y=i;
        auto temp_x=_begin.x()+(temp_y-_begin.y())/slope;
        SetValue(temp_x,temp_y,value);
      }
    }
  }


}

bool OccupancyStatusGridMap::GetOccupiedPointsInMap(const Rect &r, std::multimap<double ,double > &points) {
  Eigen::Vector3d vtl_in_map = world_to_map * r.tl();
  Eigen::Vector3d vbr_in_map = world_to_map * r.br();
  CellStatus temp;
  Eigen::Vector3d point;
  for (int i = floor(vtl_in_map.x()); i < floor(vbr_in_map.x()); i++) {
    for (int j = floor(vtl_in_map.y()); j < floor(vbr_in_map.y()); j++) {
      if (!GetValue(i, j, &temp)) {
        return false;
      } else {
        if (temp == CellStatus::OCCUPIED) {
          point << i, j, 1.0;
          point = map_to_world * point;
          points.insert({point.x(), point.y()});
        }
      }
    }
  }
  return true;
}

bool OccupancyStatusGridMap::SetMapValue(const size_t x_idx, const size_t y_idx,
                                         const CellStatus val) {
  bool ret = SetValue(x_idx, y_idx, val);
  return ret;
}

void OccupancyStatusGridMap::Reset(){
    cells().assign(static_cast<int>(grid_size().area()),CellStatus::FREE);
}

void OccupancyStatusGridMap::ExpandMap(const cv::Size &delta_size) {
  Expand(delta_size);
}
