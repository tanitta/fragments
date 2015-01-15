#pragma once

#ifdef Success
  #undef Success
#endif
#include <Eigen/Core>
#include <base_entity.hpp>

namespace fragments {
	namespace data {
		class StaticEntity : public fragments::data::BaseEntity{
			public:
				std::vector<Eigen::Vector3d> points_;
				StaticEntity():
					points_(4,Eigen::Vector3d::Zero(3))
					// face_(3,)
				{
				};
				virtual ~StaticEntity(){};

				void SetPoint(const int index, const float x, const float y, const float z){
					points_[index][0] = x;
					points_[index][1] = y;
					points_[index][2] = z;
				};

				Eigen::Vector3d GetPoint(const int index) const {
					return points_[index];
				}


				Eigen::Vector3d GetMaxPoint(const int axis) const{
					int max_index = 0;
					for (int i = 1; i < 3; i++) {
						if (points_[i][axis] > points_[max_index][axis]) {
							max_index = i;
						}
					}
					return points_[max_index];
				};

				Eigen::Vector3d GetMinPoint(const int axis) const{
					int min_index = 0;
					for (int i = 1; i < 3; i++) {
						if (points_[i][axis] < points_[min_index][axis]) {
							min_index = i;
						}
					}
					return points_[min_index];

				};

				Eigen::Vector3d GetCenter() const{
					Eigen::Vector3d center;
					for(int i = 0; i<3; i++){
						center[i] = (GetMaxPoint(i)[i]-GetMinPoint(i)[i])*0.5+GetMinPoint(i)[i];
					}
					return center;
				}
		};
	} // namespace data
} // namespace fragments
