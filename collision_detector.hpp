#pragma once

#ifdef Success
  #undef Success
#endif
#include <Eigen/Core>

#include <data/static_entity.hpp>
#include <data/active_entity.hpp>
#include <data/collidable_pair.hpp>
#include <data/constraint_pair.hpp>
#include <data/static_node.hpp>
#include <data/shape.hpp>

namespace fragments {
	//! 衝突判定を行う
	/*!
	 * 1.BroadPhase
	 * 衝突する可能性のあるActiveEntityとStaticEntityの組み合わせ(CollidablePair)をリストに追加する．
	 *
	 * 2.NarrowPhase
	 * CollidablePairを元により詳細な衝突の計算を行う．ペアが衝突する場合は拘束ペア(ConstraintPair)としてリストに追加する．
	 */
	class CollisionDetector {
		private:
			fragments::data::StaticNode static_tree_;
			std::vector<fragments::data::ActiveEntity*> active_entity_ptrs_;
			std::vector<fragments::data::CollidablePair> collidable_pairs_;
		
		public:
			CollisionDetector():active_entity_ptrs_(0),collidable_pairs_(){};
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
			
			//! StaticNodeとActiveEntityが衝突するかどうかを判定する
			/*!
			 *
			 */
			bool DetectCollisionFromAABB(const fragments::data::StaticNode& static_node, const fragments::data::ActiveEntity& active_entity)const{
				for (int i = 0; i < 3; i++) {
					if(active_entity.box_size_max_[i] < static_node.box_size_min_[i] || active_entity.box_size_min_[i] > static_node.box_size_max_[i]){
						return false;
					}
				}
				return true;
			};
			
			//! 衝突しうるペアをAABBTreeから検索する
			/*!
			 * AABBTreeを探索してStaticNode内のStaticEntityとActiveEntityとが衝突しうるペアをcollidable_pairs_に追加する
			 */
			void SearchStaticTree(fragments::data::StaticNode& static_node, fragments::data::ActiveEntity& active_entity, std::vector<fragments::data::CollidablePair>& collidable_pairs){
				if (DetectCollisionFromAABB(static_node,active_entity)){
					// 末端nodeの場合
					if(static_node.nexts_.size()==0){
						//CollisionPairを追加
						collidable_pairs_.push_back(fragments::data::CollidablePair(*static_node.static_entity_ptrs_[0],active_entity));
						
					}else{
						for (auto i : static_node.nexts_) {
							SearchStaticTree(i, active_entity, collidable_pairs);
						}
					}
				}
			}
			
			//! 衝突判定の詳細計算を行う
			/*!
			 * CollidablePairを元に衝突判定を行い，衝突する場合は拘束ペア(ConstraintPair)をリストに追加する
			 */
			fragments::data::ConstraintPair SearchCollisionDetail(fragments::data::CollidablePair& collidable_pair){
				fragments::data::ActiveEntity& active_entity_ref = collidable_pair.active_entity_ref_;
				fragments::data::StaticEntity& static_entity_ref= collidable_pair.static_entity_ref_;
				
				// ActiveEntityの形状(active_entity_ref_.shape_)ごとに処理を分ける
				
				// 姿勢に依存しない
				// SPHERE
				if (collidable_pair.active_entity_ref_.shape_.type_ == fragments::data::SPHERE) {
					auto start_point = active_entity_ref.state_.position_ - active_entity_ref.state_.linear_velocity_;
					auto end_point = active_entity_ref.state_.position_;
					
					std::cout<<start_point[0]<<std::endl;
				}
				
				// 姿勢(State)に依存
				// SQUARE
				if (collidable_pair.active_entity_ref_.shape_.type_ == fragments::data::SQUARE) {
					
				}
				// DISK
				if (collidable_pair.active_entity_ref_.shape_.type_ == fragments::data::DISK) {
					
				}
				
				return fragments::data::ConstraintPair();
			};
			
			void Update(std::vector<fragments::data::ConstraintPair>& constraint_pair){
				collidable_pairs_.clear();
				// Broadphase
				for (auto i : active_entity_ptrs_) {
					i->UpdateBoundingBox();
					SearchStaticTree(static_tree_, *i, collidable_pairs_);
				}
				
				// Narrowphase
				for (auto&& i : collidable_pairs_) {
					SearchCollisionDetail(i);
				}
			};
			
			fragments::data::StaticNode& GetStaticTree(){
				return static_tree_;
			};
	};
} // namespace fragments
