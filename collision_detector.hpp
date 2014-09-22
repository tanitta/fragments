#pragma once

#include <data/static_entity.hpp>
#include <data/active_entity.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <data/static_node.hpp>

namespace fragments {
	class CollisionDetector {
		private:
			fragments::data::StaticNode static_tree_;
			std::vector<fragments::data::ActiveEntity*> active_entity_ptrs_;
		public:
			CollisionDetector():active_entity_ptrs_(0){};
			virtual ~CollisionDetector(){};

			void Setup(std::vector<fragments::data::StaticEntity>& static_entities,
					std::vector<fragments::data::ActiveEntity>& active_entities){
				std::vector<fragments::data::StaticEntity*> static_entity_ptrs;
				for (int i = 0; i < static_entities.size(); i++) {
					static_entity_ptrs.push_back(&static_entities[i]);
				}
				static_tree_.MakeNode(static_entity_ptrs);

				for (int i = 0; i < active_entities.size(); i++) {
					active_entity_ptrs_.push_back(&active_entities[i]);
				}
			};

			bool DetectCollisionFromAABB(const fragments::data::StaticNode& static_node, const fragments::data::ActiveEntity& active_entity)const{
				for (int i = 0; i < 3; i++) {
					if(active_entity.box_size_max_[i] < static_node.box_size_min_[i] && active_entity.box_size_min_[i] > static_node.box_size_max_[i]){
						return false;
					}
				}
				return true;
			};

			void SearchStaticTree(fragments::data::StaticNode& static_node, fragments::data::ActiveEntity& active_entity){
				if (DetectCollisionFromAABB(static_node,active_entity)){
					// 詳細判定

					for (auto i : static_node.nexts_) {
						SearchStaticTree(i, active_entity);
					}
				}


			};

			void Update(){
				for (auto i : active_entity_ptrs_) {
					SearchStaticTree(static_tree_, *i);
				}
			};



	};
} // namespace fragments
