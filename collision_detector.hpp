#pragma once

#include <data/static_entity.hpp>
#include <boost/numeric/ublas/vector.hpp>
namespace  {
	class LessCenter {
		private:
			const int axis_;
		public:
			LessCenter(const int axis):axis_(axis){};
			virtual ~LessCenter(){};

			bool operator()(fragments::data::StaticEntity* a, fragments::data::StaticEntity* b) {
				return a->GetCenter()[0] < b->GetCenter()[0];
			};

	};

	class StaticNode {
		public:
			std::vector<StaticNode*> next_ptrs_;
			std::vector<fragments::data::StaticEntity*> static_entity_ptrs_;

			boost::numeric::ublas::vector<float> box_size_max_;
			boost::numeric::ublas::vector<float> box_size_min_;

			StaticNode(){};
			virtual ~StaticNode(){};

			void Setup(std::vector<fragments::data::StaticEntity*> static_entity_ptrs){
				if (static_entity_ptrs.size() > 0){

					MakeNord(static_entity_ptrs);
				};
			};


			void MakeNord(std::vector<fragments::data::StaticEntity*> static_entity_ptrs){
				std::cout<<static_entity_ptrs[0]->GetCenter()[0]<<std::endl;
				if (static_entity_ptrs.size() > 1){
					LessCenter less_center(0);
					std::sort(static_entity_ptrs.begin(), static_entity_ptrs.end(), less_center);
				}else{

				};
			};

	};
} // namespace

namespace fragments {
	class CollisionDetector {
		private:
			StaticNode static_tree_;

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
