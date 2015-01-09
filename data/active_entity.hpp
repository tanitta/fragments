#pragma once

// #include <boost/numeric/ublas/vector.hpp>
#ifdef Success
  #undef Success
#endif
#include <Eigen/Core>
#include <shape.hpp>
#include <attribute.hpp>
#include <state.hpp>
#include <static_entity.hpp>
namespace fragments {
	namespace data {
		class ActiveEntity{
			public:
				const fragments::data::Shape shape_;
				fragments::data::Attribute attribute_;
				fragments::data::State state_;

				Eigen::Vector3d box_size_max_;
				Eigen::Vector3d box_size_min_;

				ActiveEntity(const fragments::data::ShapeType shape = fragments::data::ShapeType::SQUARE, const float size = 1.0):
					shape_(shape, size),
					state_(),
					attribute_(){};
				virtual ~ActiveEntity(){};

				void SetPosition(Eigen::Vector3d p){
					state_.position_ = p;
				};

				void SetLinearVelocity(Eigen::Vector3d p){
					state_.linear_velocity_ = p;
				};

				void UpdateBoundingBox(){
					for (int i = 0; i < 3; i++) {
						if(state_.position_[i] < state_.position_[i] + state_.linear_velocity_[i]){
							box_size_min_[i] = state_.position_[i];
							box_size_max_[i] = state_.position_[i] + state_.linear_velocity_[i];
						}else{
							box_size_min_[i] = state_.position_[i] + state_.linear_velocity_[i];
							box_size_max_[i] = state_.position_[i];
						}
						//Shape毎にバウンディングボックスの算出方法が異なるので分岐すること
						box_size_min_[i] -= shape_.size_*0.5;
						box_size_max_[i] += shape_.size_*0.5;
					}
				}

				void Integrate(float step){
					state_.position_ += state_.linear_velocity_ * step;
				};
		};
	} // namespace data
} // namespace fragments
