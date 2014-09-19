#pragma once

#include <data/static_entity.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <data/static_node.hpp><`0`>

namespace fragments {
	class CollisionDetector {
		private:
			fragments::data::StaticNode static_tree_;

		public:
			CollisionDetector(){};
			virtual ~CollisionDetector(){};

			void Setup(std::vector<fragments::data::StaticEntity>* static_entities_ptr){
				std::vector<fragments::data::StaticEntity*> static_entity_ptrs;

				for (int i = 0; i < static_entities_ptr->size(); i++) {
					static_entity_ptrs.push_back(&static_entities_ptr->operator[](i));
				}

				static_tree_.Setup(static_entity_ptrs);
			};

			void Update(){
			};
	};
} // namespace fragments
