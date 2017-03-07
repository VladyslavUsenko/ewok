/**
* This file is part of Ewok.
*
* Copyright 2017 Vladyslav Usenko, Technical University of Munich.
* Developed by Vladyslav Usenko <vlad dot usenko at tum dot de>,
* for more information see <http://vision.in.tum.de/research/robotvision/replanning>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Ewok is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Ewok is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Ewok. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef EWOK_RING_BUFFER_INCLUDE_EWOK_RAYCAST_RING_BUFFER_H_
#define EWOK_RING_BUFFER_INCLUDE_EWOK_RAYCAST_RING_BUFFER_H_

#include <ewok/ring_buffer_base.h>

#include <vector>

namespace ewok {

template<int _POW, typename _Datatype = int16_t, typename _Scalar = float, typename _Flag = uint8_t>
class RaycastRingBuffer {
 public:

  static const int _N = (1 << _POW);  // 2 to the power of POW

  // Definitions of flags
  static const _Flag occupied_flag = (1 << 0);
  static const _Flag free_flag = (1 << 1);
  static const _Flag free_ray_flag = (1 << 2);
  static const _Flag updated_flag = (1 << 3);

  static const _Flag insertion_flags = (occupied_flag | free_flag | free_ray_flag);


  // Definition of hit/miss and values for the datatype
  static constexpr double min_val = -2;
  static constexpr double max_val = 3.5;

  static constexpr double hit = 0.85;
  static constexpr double miss = -0.4;

  static constexpr _Datatype datatype_max = std::numeric_limits<_Datatype>::max();
  static constexpr _Datatype datatype_min = std::numeric_limits<_Datatype>::min();

  static constexpr double datatype_range = static_cast<double>(datatype_max) - static_cast<double>(datatype_min);

  static constexpr _Datatype datatype_hit = hit * datatype_range/(max_val - min_val);
  static constexpr _Datatype datatype_miss = miss * datatype_range/(max_val - min_val);


  // Other definitions
  typedef Eigen::Matrix<_Scalar, 4, 1> Vector4;
  typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<int, 3, 1> Vector3i;
  typedef std::vector <Vector4, Eigen::aligned_allocator<Vector4>> PointCloud;


  RaycastRingBuffer(const _Scalar &resolution) :
      resolution_(resolution),
      occupancy_buffer_(resolution, _Datatype(0)),
      flag_buffer_(resolution, _Flag(0)) {

    flag_buffer_.setEmptyElement(updated_flag);
    clearUpdatedMinMax();

  }

  inline bool isOccupied(const Vector3i & idx) {
    return isOccupied(occupancy_buffer_.at(idx));
  }

  inline bool isFree(const Vector3i & idx) {
    return isFree(occupancy_buffer_.at(idx));
  }

  inline bool isUpdated(const Vector3i & idx) {
    return flag_buffer_.at(idx) & updated_flag;
  }

  inline bool clearUpdated(const Vector3i & idx) {
    return flag_buffer_.at(idx) &= ~updated_flag;
  }

  void getUpdatedMinMax(Vector3i & updated_min, Vector3i & updated_max) {
    updated_min = updated_min_;
    updated_max = updated_max_;
  }

  void clearUpdatedMinMax() {
    Vector3i offset;
    occupancy_buffer_.getOffset(offset);

    updated_min_ = offset + Vector3i(_N-1, _N-1, _N-1);
    updated_max_ = offset;
  }

  void insertPointCloud(const PointCloud &cloud, const Vector3 &origin) {

    Vector3i origin_idx;
    occupancy_buffer_.getIdx(origin, origin_idx);

    if (!occupancy_buffer_.insideVolume(
        origin_idx)) {
      //ROS_WARN("Origin outside of volume. Skipping pointcloud.");
      return;
    }

    Vector3i min_idx = origin_idx;
    Vector3i max_idx = origin_idx;

    // Iterate over all points in pointcloud and mark occupied.
    // If a point is outside the volume - compute closes point in volume
    // and mark for inserting a free ray
    for (const Vector4 &vec : cloud) {
      Vector3 v = vec.template head<3>();
      Vector3i idx;
      occupancy_buffer_.getIdx(v, idx);

      if (occupancy_buffer_.insideVolume(idx)) {
        flag_buffer_.at(idx) |= occupied_flag;

      } else {
        Vector3 p;
        closestPointInVolume(v, origin, p);
        occupancy_buffer_.getIdx(p, idx);
        flag_buffer_.at(idx) |= free_ray_flag;
      }

      min_idx = min_idx.array().min(idx.array());
      max_idx = max_idx.array().max(idx.array());
    }


    // Iterate over all occupied voxels and insert free rays.
    for (int x = min_idx(0); x <= max_idx(0); ++x) {
      for (int y = min_idx(1); y <= max_idx(1); ++y) {
        for (int z = min_idx(2); z <= max_idx(2); ++z) {

          Vector3i idx(x, y, z);

          if (flag_buffer_.at(idx) & (occupied_flag | free_ray_flag)) {
            insertFreeBresenham3D(idx, origin_idx);
          }
        }
      }
    }

    // Iterate over all marked voxels and update
    for (int x = min_idx(0); x <= max_idx(0); ++x) {
      for (int y = min_idx(1); y <= max_idx(1); ++y) {
        for (int z = min_idx(2); z <= max_idx(2); ++z) {

          Vector3i idx(x, y, z);

          if (flag_buffer_.at(idx) & occupied_flag) {

            _Datatype & occupancy_data = occupancy_buffer_.at(idx);

            bool was_occupied = isOccupied(occupancy_data);
            addHit(occupancy_data);
            bool is_occupied = isOccupied(occupancy_data);

            flag_buffer_.at(idx) &= ~insertion_flags;

            if (was_occupied != is_occupied) {
              flag_buffer_.at(idx) |= updated_flag;

              updated_min_ = updated_min_.array().min(idx.array());
              updated_max_ = updated_max_.array().max(idx.array());
            }

          } else if (flag_buffer_.at(idx) & (free_flag | free_ray_flag)) {

            _Datatype & occupancy_data = occupancy_buffer_.at(idx);

            bool was_occupied = isOccupied(occupancy_data);
            addMiss(occupancy_data);
            bool is_occupied =  isOccupied(occupancy_data);
            flag_buffer_.at(idx) &= ~insertion_flags;

            if (was_occupied != is_occupied) {
              flag_buffer_.at(idx) |= updated_flag;

              updated_min_ = updated_min_.array().min(idx.array());
              updated_max_ = updated_max_.array().max(idx.array());
            }
          }
        }
      }
    }
  }

  virtual void setOffset(const Vector3i &off) {
    occupancy_buffer_.setOffset(off);
    flag_buffer_.setOffset(off);
  }

  virtual void moveVolume(const Vector3i &direction) {
    occupancy_buffer_.moveVolume(direction);
    flag_buffer_.moveVolume(direction);

    Vector3i offset;
    occupancy_buffer_.getOffset(offset);

    Vector3i volume_min = offset;
    Vector3i volume_max = offset.array() + (_N-1);

    for(int i=0; i<3; ++i) {
      if(direction[i] > 0) {
        Vector3i min_point = volume_min;
        Vector3i max_point = volume_max;

        min_point[i] = volume_max[i];

        updated_min_ = updated_min_.array().min(min_point.array());
        updated_max_ = updated_max_.array().max(min_point.array());

        updated_min_ = updated_min_.array().min(max_point.array());
        updated_max_ = updated_max_.array().max(max_point.array());

      } else {
        Vector3i min_point = volume_min;
        Vector3i max_point = volume_max;

        max_point[i] = volume_min[i];

        updated_min_ = updated_min_.array().min(min_point.array());
        updated_max_ = updated_max_.array().max(min_point.array());

        updated_min_ = updated_min_.array().min(max_point.array());
        updated_max_ = updated_max_.array().max(max_point.array());

      }

    }

  }

  void getMarkerFree(visualization_msgs::Marker & m)  {
    occupancy_buffer_.getMarkerHelper(m, "ring_buffer_free", 0, Vector4(0, 1, 0, 0.2),
                    [](const _Datatype & d) { return isFree(d);});
  }

  void getMarkerOccupied(visualization_msgs::Marker & m)  {
    occupancy_buffer_.getMarkerHelper(m, "ring_buffer_occupied", 0, Vector4(1, 0, 0, 0.8),
                    [](const _Datatype & d) { return isOccupied(d);});
  }

  void getMarkerUpdated(visualization_msgs::Marker & m)  {
    flag_buffer_.getMarkerHelper(m, "ring_buffer_occupied", 0, Vector4(1, 1, 0, 0.8),
                                      [](const _Flag & f) { return (f & updated_flag);});
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:

  static inline void addHit(_Datatype & d) {
    int occ = d;
    occ += datatype_hit;
    if(occ > datatype_max) occ = datatype_max;
    d = occ;
  }

  static inline void addMiss(_Datatype & d) {
    int occ = d;
    occ += datatype_miss;
    if(occ < datatype_min) occ = datatype_min;
    d = occ;
  }

  static inline bool isOccupied(const _Datatype & d) {
    return d > datatype_hit;
  }

  static inline bool isFree(const _Datatype & d) {
    return d < datatype_miss;
  }

  void closestPointInVolume(const Vector3 &point,
                           const Vector3 &origin,
                           Vector3 &res) {
    Vector3 diff = point - origin;

    _Scalar min_t = std::numeric_limits<_Scalar>::max();

    for (int i = 0; i < 3; i++) {

      if (std::abs(diff[i]) > 0) {
        _Scalar t1, t2;
        Vector3i offset;
        occupancy_buffer_.getOffset(offset);

        t1 = ((offset[i] + 0.5) * resolution_ - origin[i]) / diff[i];
        if (t1 > 0 && t1 < min_t) min_t = t1;

        t2 = ((offset[i]
            + _N - 0.5)
            * resolution_ - origin[i]) / diff[i];
        if (t2 > 0 && t2 < min_t) min_t = t2;

        //ROS_INFO_STREAM("i: " << i << " t1: " << t1 << " t2 " << t2);
      }

    }
    res = origin + min_t * diff;
  }

  void insertFree(const Vector3i &point_idx,
                  const Vector3i &origin_idx) {

    Vector3 point, origin;
    occupancy_buffer_.getPoint(point_idx, point);
    occupancy_buffer_.getPoint(origin_idx, origin);

    Vector3 dir = origin - point;

    _Scalar length = dir.norm();
    dir /= length;

    for (float i = 0; i <= length; i += this->resolution) {
      Vector3 intermediate_point = point + dir * i;

      Vector3i intermediate_idx;
      occupancy_buffer_.getIdx(intermediate_point, intermediate_idx);

      flag_buffer_.at(intermediate_idx) |= free_flag;
    }
  }

  void insertFreeBresenham3D(const Vector3i &point_idx,
                             const Vector3i &origin_idx) {

    int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2,
        dz2;
    int point[3];

    point[0] = point_idx[0];
    point[1] = point_idx[1];
    point[2] = point_idx[2];

    dx = origin_idx[0] - point_idx[0];
    dy = origin_idx[1] - point_idx[1];
    dz = origin_idx[2] - point_idx[2];

    x_inc = (dx < 0) ? -1 : 1;
    l = abs(dx);
    y_inc = (dy < 0) ? -1 : 1;
    m = abs(dy);
    z_inc = (dz < 0) ? -1 : 1;
    n = abs(dz);
    dx2 = l << 1;
    dy2 = m << 1;
    dz2 = n << 1;

    if ((l >= m) && (l >= n)) {
      err_1 = dy2 - l;
      err_2 = dz2 - l;
      for (i = 0; i < l; i++) {
        flag_buffer_.at(Vector3i(point[0], point[1], point[2])) |= free_flag;
        if (err_1 > 0) {
          point[1] += y_inc;
          err_1 -= dx2;
        }
        if (err_2 > 0) {
          point[2] += z_inc;
          err_2 -= dx2;
        }
        err_1 += dy2;
        err_2 += dz2;
        point[0] += x_inc;
      }
    } else if ((m >= l) && (m >= n)) {
      err_1 = dx2 - m;
      err_2 = dz2 - m;
      for (i = 0; i < m; i++) {
        flag_buffer_.at(Vector3i(point[0], point[1], point[2])) |= free_flag;
        if (err_1 > 0) {
          point[0] += x_inc;
          err_1 -= dy2;
        }
        if (err_2 > 0) {
          point[2] += z_inc;
          err_2 -= dy2;
        }
        err_1 += dx2;
        err_2 += dz2;
        point[1] += y_inc;
      }
    } else {
      err_1 = dy2 - n;
      err_2 = dx2 - n;
      for (i = 0; i < n; i++) {
        flag_buffer_.at(Vector3i(point[0], point[1], point[2])) |= free_flag;
        if (err_1 > 0) {
          point[1] += y_inc;
          err_1 -= dz2;
        }
        if (err_2 > 0) {
          point[0] += x_inc;
          err_2 -= dz2;
        }
        err_1 += dy2;
        err_2 += dx2;
        point[2] += z_inc;
      }
    }

    flag_buffer_.at(Vector3i(point[0], point[1], point[2])) |= free_flag;
  }

  _Scalar resolution_;

  Vector3i updated_min_, updated_max_;

  // buffer to store occupancy information
  RingBufferBase <_POW, _Datatype, _Scalar> occupancy_buffer_;

  // buffer to store insertion information
  RingBufferBase <_POW, _Flag, _Scalar> flag_buffer_;

};

}

#endif // EWOK_RING_BUFFER_INCLUDE_EWOK_RAYCAST_RING_BUFFER_H_
