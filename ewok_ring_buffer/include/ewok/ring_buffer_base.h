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

#ifndef EWOK_RING_BUFFER_INCLUDE_EWOK_RING_BUFFER_BASE_H_
#define EWOK_RING_BUFFER_INCLUDE_EWOK_RING_BUFFER_BASE_H_

#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>

#include <vector>
#include <algorithm>


namespace ewok {

template<int _POW, typename _Datatype, typename _Scalar = float>
class RingBufferBase {
 public:

  static const int _N = (1 << _POW); // 2 to the power of POW
  static const int _N_2 = _N / 2;
  static const int _MASK = (_N - 1);
  static const int _NEG_MASK = ~_MASK;

  typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<_Scalar, 4, 1> Vector4;
  typedef Eigen::Matrix<int, 3, 1> Vector3i;

  explicit RingBufferBase(const _Scalar & resolution,
                          const _Datatype & empty_element = _Datatype())
      : resolution_(resolution), empty_element_(empty_element),
        offset_(-_N_2, -_N_2, -_N_2), buffer_(_N * _N * _N) {

      std::fill(buffer_.begin(), buffer_.end(), empty_element_);
  }

  virtual ~RingBufferBase() {
  }

  void setEmptyElement(const _Datatype & e) {
      empty_element_ = e;
  }

  virtual void setOffset(const Vector3i & offset) {
      offset_ = offset;
  }

  inline void getIdx(const Vector3 & point, Vector3i & idx) const {
      idx = (point / resolution_).array().floor().template cast<int>();
  }

  inline void getPoint(const Vector3i & idx, Vector3 & point) const {
      point = idx.template cast<_Scalar>();
      point.array() += _Scalar(0.5);
      point *= resolution_;
  }

  inline void getOffset(Vector3i & o) const {
      o = offset_;
  }

  // Moves 1 step in the direction
  virtual void moveVolume(const Vector3i &direction) {

      for (int axis = 0; axis < 3; axis++) {
          if (direction[axis] != 0) {

              int slice;

              if (direction[axis] > 0) {
                  offset_[axis]++;
                  slice = offset_[axis] + _N - 1;
              } else {
                  offset_[axis]--;
                  slice = offset_[axis];
              }

              switch (axis) {
                  case 0:setXSlice(slice, empty_element_);
                  break;
                  case 1:setYSlice(slice, empty_element_);
                  break;
                  case 2:setZSlice(slice, empty_element_);
                  break;

              }

          }
      }

  }

  void setXSlice(int slice_idx, const _Datatype &data) {
      for (int i = 0; i < _N; i++) {
          for (int j = 0; j < _N; j++) {
              Vector3i idx(slice_idx, i, j);
              this->at(idx) = data;
          }
      }
  }

  void setYSlice(int slice_idx, const _Datatype &data) {
      for (int i = 0; i < _N; i++) {
          for (int j = 0; j < _N; j++) {
              Vector3i idx(i, slice_idx, j);
              this->at(idx) = data;
          }
      }
  }

  void setZSlice(int slice_idx, const _Datatype &data) {
      for (int i = 0; i < _N; i++) {
          for (int j = 0; j < _N; j++) {
              Vector3i idx(i, j, slice_idx);
              this->at(idx) = data;
          }
      }
  }

  inline bool insideVolume(const Vector3i &coord) {

      static const Vector3i
          neg_mask_vec(_NEG_MASK, _NEG_MASK, _NEG_MASK);

      bool res = true;

      for (int i = 0; i < 3; i++) {
          res &= !((coord[i] - offset_[i]) & neg_mask_vec[i]);
      }

      return res;
  }

  inline _Datatype & at(const Vector3i &coord) {

      Vector3i idx;

      for (int i = 0; i < 3; i++) {
          idx[i] = coord[i] & _MASK;
      }

      return buffer_[_N * _N * idx[0] + _N * idx[1] + idx[2]];
  }

  inline _Datatype at(const Vector3i &coord) const {

      Vector3i idx;

      for (int i = 0; i < 3; i++) {
          idx[i] = coord[i] & _MASK;
      }

      return buffer_[_N * _N * idx[0] + _N * idx[1] + idx[2]];
  }

  inline Vector3i getVolumeCenter() {
      static const Vector3i inc(_N_2, _N_2, _N_2);
      return offset_ + inc;
  }

  template<typename F>
  void getMarkerHelper(visualization_msgs::Marker &m,
                       const std::string &ns, int id, const Vector4 &color,
                       F func) {
      m.header.frame_id = "world";
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::Marker::CUBE_LIST;
      m.action = visualization_msgs::Marker::MODIFY;
      m.scale.x = resolution_ * 0.9;
      m.scale.y = resolution_ * 0.9;
      m.scale.z = resolution_ * 0.9;
      m.color.a = color(3);

      Vector3 offset_point;
      getPoint(offset_, offset_point);

      m.pose.position.x = offset_point.x();
      m.pose.position.y = offset_point.y();
      m.pose.position.z = offset_point.z();

      m.color.r = color(0);
      m.color.g = color(1);
      m.color.b = color(2);

      for (int x = 0; x < _N; x++) {
          for (int y = 0; y < _N; y++) {
              for (int z = 0; z < _N; z++) {

                  Vector3i coord(x, y, z);
                  coord += offset_;

                  geometry_msgs::Point p;
                  p.x = x * resolution_;
                  p.y = y * resolution_;
                  p.z = z * resolution_;

                  _Datatype &data = this->at(coord);

                  if (func(data)) m.points.push_back(p);
              }
          }
      }

  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:

  _Scalar resolution_;
  _Datatype empty_element_;

  Vector3i offset_;
  std::vector<_Datatype> buffer_;
};

}

#endif // EWOK_RING_BUFFER_INCLUDE_EWOK_RING_BUFFER_BASE_H_
