/** PerceptIn license **/

#pragma once

#ifdef INT
#undef INT
#endif
#ifdef UCHAR
#undef UCHAR
#endif
#include "grid.h"
#include <eigen3/Eigen/Core>
#include <memory>
#include <map>
#include <opencv/cv.hpp>
#include <opencv2/core.hpp>

/**
 * Status of a cell in the grid for planning.
 */
enum CellStatus {
  FREE,      // Cell is free to travel to.
  OCCUPIED,  // Cell is occupied; Can't travel to this cell.
  UNKNOWN,   // Cell's freedom hasn't been determined yet.
  ALL        // For search use, can not assign cell to ALL status
};

class Rect {
public:
    Rect(double x, double y, double width, double height)
            : x_(x), y_(y), width(width), height(height) {};
    double x_ = 0;
    double y_ = 0;

    Eigen::Vector3d tl() const {
      return Eigen::Vector3d(x_ + height / 2, y_ + width / 2, 1);
    }

    Eigen::Vector3d br() const {
      return Eigen::Vector3d(x_ - height / 2, y_ - width / 2, 1);
    }

    double GetHeight() const {
        return height;
    }

    double GetWidth() const {
        return width;
    }

private:
    double width = 0;
    double height = 0;
};

/**
 * Defines a discretied occupancy grid map specifically for planning.
 */
class OccupancyStatusGridMap : public Grid<CellStatus> {
public:  // temporary modify for test should be removed
    /**
     * @note Default cell value is UNKNOWN.
     */
    OccupancyStatusGridMap(const double cell_size, const cv::Size &grid_size);

    /**
     * @brief Visualize the occupancy grid as a cv::Mat image. Each pixel in the
     *        image represents a cell with the following color code:
     *            FREE     -- white
     *            OCCUPIED -- black
     *            UNKNOWN  -- gray
     * @return The image showing the values in the grid.
     */
    cv::Mat Visualize() const;

    bool SetMapValue(const size_t x_idx, const size_t y_idx,
                     const CellStatus val);

    void ExpandMap(const cv::Size &delta_size);

    bool DrawRectInMap(const Rect &r,const CellStatus &value=CellStatus::OCCUPIED);

    bool DrawLineInMap(const cv::Point2d &begin, const cv::Point2d &end, const CellStatus &value=CellStatus::OCCUPIED);

    bool GetOccupiedPointsInMap(const Rect &r, std::multimap<double ,double > &);

    void Reset();


    Eigen::Matrix<double, 3, 3> world_to_map;
    Eigen::Matrix<double, 3, 3> map_to_world;
};
