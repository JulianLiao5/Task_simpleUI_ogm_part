/** PerceptIn license **/

#pragma once

#include <glog/logging.h>

/**
 * @brief Convert 2d cell index to 1d index in #cells_.
 * @param x_idx Horizontal index of the cell.
 * @param y_idx Vertical index of the cell.
 * @param grid_size Size of the grid.
 * @return Index in the cell vector.
 */
inline size_t CellVecIndex(const size_t x_idx, const size_t y_idx,
                           const cv::Size &grid_size) {
  return y_idx * grid_size.width + x_idx;
}

template <typename T>
class Grid<T>::Impl {
 public:
  Impl(const double cell_size, const cv::Size &grid_size, T initial_value,
       const unsigned char max_weight)
      : cell_size_(cell_size),
        grid_size_(grid_size),
        cells_(grid_size.area(), initial_value),
        weights_(grid_size.area(), 1),
        initial_value_(initial_value),
        max_weight_(max_weight) {}
  ~Impl() {}

 private:
  /// Physical size of each cell in the grid map.
  const double cell_size_;
  /// Maximum cell index along each dimension.
  cv::Size grid_size_;
  /// The occupancy grid map. The value is in the range of [0.0, 1.0]. Negative
  /// value means the cell is invalid.
  std::vector<T> cells_;

  std::vector<unsigned char> weights_;

  /// initial value of T
  const T initial_value_;

  /// max weight
  const unsigned char max_weight_;
  // Make life easier.
  friend class Grid<T>;
};

template <typename T>
Grid<T>::Grid(const double cell_size, const cv::Size &grid_size,
              T initial_value, const unsigned char max_weight)
    : impl_(std::make_shared<Impl>(cell_size, grid_size, initial_value,
                                   max_weight)) {
  CHECK_GT(impl_->cell_size_, 0.0);
  CHECK_GT(impl_->grid_size_.width, 0);
  CHECK_GT(impl_->grid_size_.height, 0);
  CHECK_EQ(impl_->cells_.size(), impl_->grid_size_.area());
}

template <typename T>
Grid<T>::Grid(const Grid<T> &other) : impl_(nullptr) {
  CHECK(other.impl_);  // Grid may be moved.
  impl_ = other.impl_;
}

template <typename T>
Grid<T>::Grid(Grid &&other) {
  CHECK(other.impl_);  // Grid may be moved.
  impl_ = other.impl_;
  other.impl_.reset();
}

template <typename T>
Grid<T>::~Grid() {}

template <typename T>
std::shared_ptr<Grid<T>> Grid<T>::clone() const {
  CHECK(impl_);  // Grid may be moved.
  std::shared_ptr<Grid<T>> grid = std::make_shared<Grid<T>>(
      impl_->cell_size, impl_->grid_size, impl_->cells_.front());
  CHECK(grid); CHECK(grid->impl_);
  grid->impl_->cells_ = impl_->cells_;
  return grid;
}

template <typename T>
const std::vector<T> &Grid<T>::cells() const {
  CHECK(impl_);  // Grid may be moved.
  return impl_->cells_;
}

template <typename T>
const double Grid<T>::cell_size() const {
  CHECK(impl_);  // Grid may be moved.
  return impl_->cell_size_;
}

template <typename T>
const cv::Size &Grid<T>::grid_size() const {
  CHECK(impl_);  // Grid may be moved.
  return impl_->grid_size_;
}

template <typename T>
const cv::Size2d Grid<T>::map_size() const {
  CHECK(impl_);  // Grid may be moved.
  return cv::Size2d(
      static_cast<double>(impl_->grid_size_.width) * impl_->cell_size_,
      static_cast<double>(impl_->grid_size_.height) * impl_->cell_size_);
}

template <typename T>
const cv::Point2d Grid<T>::map_size_max() const {
  CHECK(impl_);  // Grid may be moved.
  return cv::Point2d(static_cast<double>((impl_->grid_size_.width - 1) / 2.0) *
                         impl_->cell_size_,
                     static_cast<double>((impl_->grid_size_.height - 1) / 2.0) *
                         impl_->cell_size_);
}

template <typename T>
const cv::Point2d Grid<T>::map_size_min() const {
  CHECK(impl_);  // Grid may be moved.
  return cv::Point2d(
      static_cast<double>(-1.0 * (impl_->grid_size_.width) / 2.0) *
          impl_->cell_size_,
      static_cast<double>(-1.0 * (impl_->grid_size_.height) / 2.0) *
          impl_->cell_size_);
}

template <typename T>
const grid::CellIndex Grid<T>::idx_max() const {
  CHECK(impl_);  // Grid may be moved.
  return grid::CellIndex(impl_->grid_size_.width - 1,
                         impl_->grid_size_.height - 1);
}

template <typename T>
std::vector<T> &Grid<T>::cells() {
  CHECK(impl_);  // Grid may be moved.
  return impl_->cells_;
}

template <typename T>
size_t Grid<T>::CellIndexToVectorIndex(const grid::CellIndex &cell_idx) const {
  CHECK(impl_);  // Grid may be moved.
  return cell_idx.y_idx * impl_->grid_size_.width + cell_idx.x_idx;
}

template <typename T>
grid::CellIndex Grid<T>::VectorIndexToCellIndex(const size_t idx) const {
  CHECK(impl_);  // Grid may be moved.
  return grid::CellIndex(idx % impl_->grid_size_.width,
                         idx / impl_->grid_size_.width);
}

template <typename T>
double Grid<T>::GetArea() const {
  CHECK(impl_);  // Grid may be moved.
  return impl_->cell_size_ * impl_->cell_size_ *
         static_cast<double>(impl_->cells_.size());
}

template <typename T>
cv::Rect2d Grid<T>::GetRoi() const {
  CHECK(impl_);  // Grid may be moved.
  const double map_size_x =
      static_cast<double>(impl_->grid_size_.width) * impl_->cell_size_;
  const double map_size_y =
      static_cast<double>(impl_->grid_size_.height) * impl_->cell_size_;
  return cv::Rect2d(-map_size_x / 2.0, -map_size_y / 2.0, map_size_x,
                    map_size_y);
}

template <typename T>
bool Grid<T>::GetValue(const size_t &x_idx, const size_t &y_idx,
                       T *value) const {
  CHECK(impl_);  // Grid may be moved.
  CHECK_NOTNULL(value);
  if (x_idx >= impl_->grid_size_.width || y_idx >= impl_->grid_size_.height) {
    return false;
  }

  const size_t idx = CellVecIndex(x_idx, y_idx, impl_->grid_size_);
  CHECK_LT(
      idx,
      impl_->cells_.size());  // Avoid using setter cells() to change dimension.
  *value = impl_->cells_[idx];
  return true;
}

template <typename T>
bool Grid<T>::GetValue(const grid::CellIndex &idx, T *value) const {
  CHECK(impl_);  // Grid may be moved.
  CHECK_NOTNULL(value);
  return GetValue(idx.x_idx, idx.y_idx, value);
}

template <typename T>
bool Grid<T>::SetValue(const size_t x_idx, const size_t y_idx, const T val) {
  CHECK(impl_);  // Grid may be moved.
  if (x_idx >= impl_->grid_size_.width || y_idx >= impl_->grid_size_.height) {
    return false;
  }

  if (val < 0) {
    LOG(WARNING) << "Marking a cell (" << x_idx << ", " << y_idx << ") invalid "
                 << "by setting its value to negative (" << val << ").";
  }

  const size_t idx = CellVecIndex(x_idx, y_idx, impl_->grid_size_);
  CHECK_LT(
      idx,
      impl_->cells_.size());  // Avoid using setter cells() to change dimension.
  impl_->cells_[idx] = val;
  return true;
}

template <typename T>
bool Grid<T>::SetWeightedValue(const size_t x_idx, const size_t y_idx,
                               const T val) {
  CHECK(impl_);  // Grid may be moved.
  if (x_idx >= impl_->grid_size_.width || y_idx >= impl_->grid_size_.height) {
    return false;
  }

  if (val < 0) {
    LOG(WARNING) << "Marking a cell (" << x_idx << ", " << y_idx << ") invalid "
                 << "by setting its value to negative (" << val << ").";
  }

  const size_t idx = CellVecIndex(x_idx, y_idx, impl_->grid_size_);
  CHECK_LT(
      idx,
      impl_->cells_.size());  // Avoid using setter cells() to change dimension.
  impl_->cells_[idx] = (impl_->cells_[idx] * impl_->weights_[idx] + val) /
                       (impl_->weights_[idx] + 1);
  if (impl_->weights_[idx] < 40) {
    impl_->weights_[idx]++;
  }
  return true;
}

template <typename T>
bool Grid<T>::SetWeightedValue(const grid::CellIndex &idx, const T val) {
  CHECK(impl_);  // Grid may be moved.
  return SetWeightedValue(idx.x_idx, idx.y_idx, val);
}

template <typename T>
bool Grid<T>::SetValue(const grid::CellIndex &idx, const T val) {
  CHECK(impl_);  // Grid may be moved.
  return SetValue(idx.x_idx, idx.y_idx, val);
}

template <typename T>
bool Grid<T>::GetLocation(const size_t x_idx, const size_t y_idx,
                          cv::Point2d *loc) const {
  CHECK(impl_);  // Grid may be moved.
  CHECK_NOTNULL(loc);
  if (x_idx >= impl_->grid_size_.width || y_idx >= impl_->grid_size_.height) {
    return false;
  }

  // Compute the index at the metric center.
  const double idx_x_origin =
      static_cast<double>(impl_->grid_size_.width - 1) / 2.0;
  const double idx_y_origin =
      static_cast<double>(impl_->grid_size_.height - 1) / 2.0;
  loc->x = (static_cast<double>(x_idx) - idx_x_origin) * impl_->cell_size_;
  loc->y = (static_cast<double>(y_idx) - idx_y_origin) * impl_->cell_size_;
  return true;
}

template <typename T>
bool Grid<T>::GetLocation(const grid::CellIndex &idx, cv::Point2d *loc) const {
  CHECK(impl_);  // Grid may be moved.
  CHECK_NOTNULL(loc);
  return GetLocation(idx.x_idx, idx.y_idx, loc);
}

/**
 * The x and y are in meters in world coordinate from pose side.
 * This function is used in update_ogm and outsiders shouldn't need to
 * explicitly calculate x_idx and y_idx.
*/
template <typename T>
bool Grid<T>::FindCell(const double loc_x, const double loc_y, size_t *x_idx,
                       size_t *y_idx) const {
  CHECK(impl_);  // Grid may be moved.
  CHECK_NOTNULL(x_idx);
  CHECK_NOTNULL(y_idx);

  const int x_idx_int =
      cvRound(loc_x / impl_->cell_size_ +
              static_cast<double>(impl_->grid_size_.width - 1) / 2.0);
  if (x_idx_int < 0 || x_idx_int >= impl_->grid_size_.width) {
    LOG(INFO) << "x " << x_idx_int << ", " << impl_->grid_size_.width;
    return false;
  }
  *x_idx = static_cast<size_t>(x_idx_int);

  const int y_idx_int =
      cvRound(loc_y / impl_->cell_size_ +
              static_cast<double>(impl_->grid_size_.height - 1) / 2.0);
  if (y_idx_int < 0 || y_idx_int >= impl_->grid_size_.height) {
    LOG(INFO) << "y " << y_idx_int << ", " << impl_->grid_size_.height;
    return false;
  }
  *y_idx = static_cast<size_t>(y_idx_int);

  return true;
}

template <typename T>
bool Grid<T>::FindCell(const double loc_x, const double loc_y,
                       grid::CellIndex *idx) const {
  CHECK(impl_);  // Grid may be moved.
  CHECK_NOTNULL(idx);
  return FindCell(loc_x, loc_y, &(idx->x_idx), &(idx->y_idx));
}

template <typename T>
bool Grid<T>::FindCell(const cv::Point2d &loc, grid::CellIndex *idx) const {
  CHECK(impl_);  // Grid may be moved.
  CHECK_NOTNULL(idx);
  return FindCell(loc.x, loc.y, &(idx->x_idx), &(idx->y_idx));
}

template <typename T>
void Grid<T>::Expand(const cv::Size &delta_size) {
  CHECK(impl_);
  // increase the both side of map with delta_size
  cv::Size expand_size = delta_size + delta_size + impl_->grid_size_;
  std::vector<T> new_cells(expand_size.area(), impl_->initial_value_);
  for (size_t i = 0; i < impl_->grid_size_.width; ++i) {
    for (size_t j = 0; j < impl_->grid_size_.height; ++j) {
      new_cells.at(CellVecIndex(i + delta_size.width, j + delta_size.height,
                                expand_size)) =
          impl_->cells_.at(CellVecIndex(i, j, impl_->grid_size_));
    }
  }
  impl_->grid_size_ = expand_size;
  impl_->cells_.swap(new_cells);
}

template <typename T>
void Grid<T>::Expand(const cv::Size &delta_size, const T val) {
  CHECK(impl_);
  // increase the both side of map with delta_size
  cv::Size expand_size = delta_size + delta_size + impl_->grid_size_;
  std::vector<T> new_cells(expand_size.area(), val);
  for (size_t i = 0; i < impl_->grid_size_.width; ++i) {
    for (size_t j = 0; j < impl_->grid_size_.height; ++j) {
      new_cells.at(CellVecIndex(i + delta_size.width, j + delta_size.height,
                                expand_size)) =
          impl_->cells_.at(CellVecIndex(i, j, impl_->grid_size_));
    }
  }
  impl_->grid_size_ = expand_size;
  impl_->cells_.swap(new_cells);
}
