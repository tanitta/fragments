#pragma once
#include <static_entity.hpp>

#ifdef Success
  #undef Success
#endif
#include <Eigen/Core>
namespace  {
	class LessCenter {
		private:
			const int axis_;
		public:
			LessCenter(const int axis):axis_(axis){};
			virtual ~LessCenter(){};

			bool operator()(fragments::data::StaticEntity* a, fragments::data::StaticEntity* b) {
				return a->GetCenter()[axis_] < b->GetCenter()[axis_];
			};

	};
}

namespace fragments{
	namespace data {
		class StaticNode {
			public:
				std::vector<StaticNode> nexts_;
				std::vector<fragments::data::StaticEntity*> static_entity_ptrs_;

				Eigen::Vector3d box_size_max_;
				Eigen::Vector3d box_size_min_;

				StaticNode():
					box_size_min_(Eigen::Vector3d::Zero(3)),box_size_max_(Eigen::Vector3d::Zero(3)){};
				virtual ~StaticNode(){};

				void Setup(std::vector<fragments::data::StaticEntity*> static_entity_ptrs){
					if (static_entity_ptrs.size() > 0){

						MakeNode(static_entity_ptrs);
					};
				};

				void MakeNode(std::vector<fragments::data::StaticEntity*> static_entity_ptrs){
					std::cout<<static_entity_ptrs.size()<<std::endl;

					//axis sort
					int most_large_axis = 0;
					for (int i = 0; i < 3; i++) {
						LessCenter less_center(i);
						std::sort(static_entity_ptrs.begin(), static_entity_ptrs.end(), less_center);

						box_size_min_[i] = static_entity_ptrs[0]->GetMinPoint(i)[i];
						box_size_max_[i] = static_entity_ptrs[0]->GetMaxPoint(i)[i];

						for (auto j : static_entity_ptrs) {
							if (j->GetMinPoint(i)[i] < box_size_min_[i]) {
								box_size_min_[i] = j->GetMinPoint(i)[i];
							}
							if (j->GetMaxPoint(i)[i] > box_size_max_[i]) {
								box_size_max_[i] = j->GetMaxPoint(i)[i];
							}
						}
						if (i != 0) {
							if (box_size_max_[i] - box_size_min_[i] > box_size_max_[i-1] - box_size_min_[i-1]) {
								most_large_axis = i;
							}
						}
					}
					if (most_large_axis != 2) {
						LessCenter less_center(most_large_axis);
						std::sort(static_entity_ptrs.begin(), static_entity_ptrs.end(), less_center);
					}

					//devide
					if (static_entity_ptrs.size() > 1){
						int partial_length = static_entity_ptrs.size()/2;
						std::vector<fragments::data::StaticEntity*> partial_static_entity_ptrs1(0);
						std::vector<fragments::data::StaticEntity*> partial_static_entity_ptrs2(0);
						for(int i = 0; i < static_entity_ptrs.size(); i++){
							if (i < partial_length) {
								partial_static_entity_ptrs1.push_back(static_entity_ptrs[i]);
							}else{
								partial_static_entity_ptrs2.push_back(static_entity_ptrs[i]);
							}
						}

						nexts_.push_back(StaticNode());
						nexts_.push_back(StaticNode());
						nexts_[0].MakeNode(partial_static_entity_ptrs1);
						nexts_[1].MakeNode(partial_static_entity_ptrs2);
					}else{
						static_entity_ptrs_ = static_entity_ptrs;
					};
				};

		};

	} // namespace data
} // namespace fragments
