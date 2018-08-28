/** PerceptIn license **/

#pragma once

#include <memory>
#include <vector>

#include <opencv2/core.hpp>


namespace grid {
/**
 * Index of a cell in the grid.
 */
struct CellIndex {
  /// Horizontal index.
  size_t x_idx;
  /// Vertical index.
  size_t y_idx;

  CellIndex();
  CellIndex(const size_t x_idx, const size_t y_idx);
};

/**
 * @brief Add an offset to a cell index.
 * @note idx_out = idx_1 + indx_2.
 */
CellIndex operator+(const CellIndex &a, const CellIndex &b);

/**
 * @brief Define operator for comparing two cell index.
 */
bool operator==(const CellIndex &a, const CellIndex &b);

/**
 * @brief Define operator for comparing two cell index.
 */
bool operator!=(const CellIndex &a, const CellIndex &b);

/**
 * @brief Define operator to support std::map or std::set.
 */
bool operator<(const CellIndex &a, const CellIndex &b);
}  // namespacd grid

/**
 * A grid covers a square of a 2d metric space, where the center is at (0, 0) of
 * the metric space. Each cell in the grid is also a square. The grid has the
 * same number of cells in both horizontal and vertical directions. The x-axis
 * of the metric space is pointing to the right, and the y-axis of the metric
 * space is pointing down.
 */
template <typename T>
class Grid {
 public:
  /**
   * @brief Constructor.
   * @param cell_size Physical size of a single cell in the grid.
   * @param grid_size Number of cells per side in the grid.
   */
  Grid(const double cell_size, const cv::Size &grid_size, T initial_value,
       const unsigned char max_weight = 48);
  Grid(const Grid &other);
  Grid(Grid &&other);
  ~Grid();
  std::shared_ptr<Grid> clone() const;

  /** Getters **/
  const std::vector<T> &cells() const;
  const double cell_size() const;
  const cv::Size &grid_size() const;
  const cv::Size2d map_size() const;
  const cv::Point2d map_size_max() const;
  const cv::Point2d map_size_min() const;
  const grid::CellIndex idx_max() const;  /// Maximum index in both dimensions.

  /** Setters **/
  // Avoid resizing the cells in using this setter.
  std::vector<T> &cells();

  /**
   * @brief Convert CellIndex to vector based index
   * @param cell_idx The CellIndex.
   * @return The corresponding vector based index.
   */
  size_t CellIndexToVectorIndex(const grid::CellIndex &cell_idx) const;

  /**
   * @brief Convert vector based index to CellIndex
   * @param idx Vector based index
   * @return The corresponding CellIndex.
   */
  grid::CellIndex VectorIndexToCellIndex(const size_t idx) const;

  /**
   * @return The area that the grid covers.
   */
  double GetArea() const;

  /*
   * @return The ROI that the grid covered.
   */
  cv::Rect2d GetRoi() const;

  /**
   * @brief Retrieve the value of a cell in the grid from its 2-d index.
   * @param x_idx Index of the cell along the horizontal direction.
   * @param y_idx Index of the cell along the vertical direction.
   * @param[out] The value at cell (x_idx, y_idx).
   * @return True if the value is retrieved.
   */
  bool GetValue(const size_t &x_idx, const size_t &y_idx, T *value) const;

  /**
   * @brief Retrieve the grid's value from its 2-d index.
   * @param idx Index of the cell.
   * @param[out] The value at cell (x_idx, y_idx).
   * @return True if the value is retrieved.
   */
  bool GetValue(const grid::CellIndex &idx, T *value) const;

  /**
   * @brief Set the value of a cell using weighted average.
   * @param x_idx Horizontal index of the cell to edit.
   * @param y_idx Vertical index of the cell to edit.
   * @param val Value to set to.
   * @return True if (x_idx, y_idx) is a cell in the grid.
   */
  bool SetWeightedValue(const size_t x_idx, const size_t y_idx, const T val);
  /**
   * @brief Set the value of a cell using weighted average.
   * @param idx Index of the cell to edit.
   * @param val Value to set to.
   * @return True if idx is a cell in the grid.
   */
  bool SetWeightedValue(const grid::CellIndex &idx, const T val);

  /**
   * @brief Set the value of a cell.
   * @param x_idx Horizontal index of the cell to edit.
   * @param y_idx Vertical index of the cell to edit.
   * @param val Value to set to.
   * @return True if (x_idx, y_idx) is a cell in the grid.
   */
  bool SetValue(const size_t x_idx, const size_t y_idx, const T val);
  /**
   * @brief Set the value of a cell.
   * @param idx Index of the cell to edit.
   * @param val Value to set to.
   * @return True if idx is a cell in the grid.
   */
  bool SetValue(const grid::CellIndex &idx, const T val);

  /**
   * @brief Get the metric value (x, y) of the center of the cell.
   */
  bool GetLocation(const size_t x_idx, const size_t y_idx,
                   cv::Point2d *loc) const;

  /**
   * @brief Get the metric value (x, y) of the center of the cell.
   */
  bool GetLocation(const grid::CellIndex &idx, cv::Point2d *loc) const;

  /**
   * @brief Calculate which cell a point (x, y) in the metric coordinate belongs
   *        to in the grid.
   * @param loc_x X location of the point in the metric coordinate.
   * @param loc_y Y location of the point in the metric coordinate.
   * @param[out] x_idx Horizontal index of the cell.
   * @param[out] y_idx Vertical index of the cell.
   * @return True if the point (x, y) is within the grid's mapped area.
   */
  bool FindCell(const double loc_x, const double loc_y, size_t *x_idx,
                size_t *y_idx) const;
  /**
   * @brief Calculate which cell a point (x, y) in the metric coordinate belongs
   *        to in the grid.
   * @param loc_x X location of the point in the metric coordinate.
   * @param loc_y Y location of the point in the metric coordinate.
   * @param[out] idx Index of the cell.
   * @return True if the point (x, y) is within the grid's mapped area.
   */
  bool FindCell(const double loc_x, const double loc_y,
                grid::CellIndex *idx) const;
  /**
   * @brief Calculate which cell a point (x, y) in the metric coordinate belongs
   *        to in the grid.
   * @param loc Location of the point in the metric coordinate.
   * @param[out] idx Index of the cell.
   * @return True if the point (x, y) is within the grid's mapped area.
   */
  bool FindCell(const cv::Point2d &loc, grid::CellIndex *idx) const;
  /**
   * @brief Expand the area of current grid map on each side with delta_size.
   * suppose we have a map withn grid size (x,y), with delta_size(dx,dy),
   * the map will be expanded to(x+2*dx, y+2*dy)
   * @param delta_size the additional dimension want to add on existed grid map.
   */
  void Expand(const cv::Size &delta_size);

  /**
   * @brief Expand the area of current gird map with delta size and set this
   * cell to certain value
   * @param delta_size the additional dimension want to add on existed grid map.
   * @param val The cell value want to set
   */
  void Expand(const cv::Size &delta_size, const T val);

 private:
  // NOTE: Setting the cell_size_ and grid_size_ constant until we figure out
  // the best way to resize the grid.

  class Impl;
  std::shared_ptr<Impl> impl_;
};

// Add implementation of the functions.
#include "grid.hpp"
