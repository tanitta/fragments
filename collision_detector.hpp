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

			bool DetectCollisionFromAABB(){
			};

			void SearchStaticTree(fragments::data::StaticNode& static_node){
				std::cout<<static_node.static_entity_ptrs_.size()<<std::endl;
				for (auto i : static_node.nexts_) {
				SearchStaticTree(i);
				}
			};

			void Update(){
				std::cout<<"Update"<<std::endl;
				for (auto i : active_entity_ptrs_) {
					SearchStaticTree(static_tree_);
				}
			};



	};
} // namespace fragments
