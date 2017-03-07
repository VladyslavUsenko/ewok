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

#include <ewok/ring_buffer_base.h>
#include <gtest/gtest.h>

template<int _POW, typename _Datatype, typename _Scalar>
struct TypeDefinitions {
  static const int POW = _POW;
  typedef _Datatype Datatype;
  typedef _Scalar Scalar;
};

typedef ::testing::Types <
                          TypeDefinitions<6, uint16_t, float>
                          > Implementations;



template <class T>
class RingBudderBaseTest : public testing::Test {
};


TYPED_TEST_CASE(RingBudderBaseTest, Implementations);

// Tests converting integer index of a cell to 3d point and backwards.
//
TYPED_TEST(RingBudderBaseTest, TestPointIndexConversion)
{
  typedef ewok::RingBufferBase<TypeParam::POW,
      typename TypeParam::Datatype, typename TypeParam::Scalar>
      RingBufferBaseType;

  typedef typename RingBufferBaseType::Vector3 Vector3;
  typedef typename RingBufferBaseType::Vector3i Vector3i;

  std::vector<typename TypeParam::Scalar> resolutions = {0.01, 0.05, 0.1, 0.15, 0.2, 0.5, 1.0, 2.0, 5.0};

  const int LIMIST_TO_CHECK = 50;

  for(auto res: resolutions) {
    RingBufferBaseType rbb(res);

    for(int x=-LIMIST_TO_CHECK; x<=LIMIST_TO_CHECK; ++x) {
      for(int y=-LIMIST_TO_CHECK; y<=LIMIST_TO_CHECK; ++y) {
        for(int z=-LIMIST_TO_CHECK; z<=LIMIST_TO_CHECK; ++z) {

          Vector3i idx(x,y,z), idx1, idx2;
          Vector3 point;
          rbb.getPoint(idx, point);
          rbb.getIdx(point, idx1);

          ASSERT_EQ(idx[0], idx1[0]);
          ASSERT_EQ(idx[1], idx1[1]);
          ASSERT_EQ(idx[2], idx1[2]);

          for(int i=0; i<5; i++) {
            rbb.getIdx(point + Vector3::Random()*res*0.499, idx2);
            ASSERT_EQ(idx[0], idx2[0]);
            ASSERT_EQ(idx[1], idx2[1]);
            ASSERT_EQ(idx[2], idx2[2]);
          }

        }
      }
    }
  }


}


int main(int argc, char **argv) {
  //srand((unsigned int) time(0));
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
