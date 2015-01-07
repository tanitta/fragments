#pragma once

#include "ofMain.h"

#include <fragments/data/static_entity.hpp>
#include <fragments/data/active_entity.hpp>
#include <fragments/data/collidable_pair.hpp>
#include <fragments/data/constraint_pair.hpp>
#include <../data/static_node.hpp>
#include <fragments/collision_detector.hpp>
#include <fragments/integrator.hpp>

namespace fragments {
	namespace tests {
		class CollidablePairsTest : public ofBaseApp{
			private:
				std::vector<fragments::data::StaticEntity> static_entities_;
				std::vector<fragments::data::ActiveEntity> active_entities_;
				std::vector<fragments::data::ConstraintPair> constraint_pairs_;
				fragments::data::StaticNode static_tree_;
				//pipeline
				fragments::CollisionDetector collision_detector_;
				fragments::Integrator integrator_;

				std::vector<ofMesh> map_;
				ofEasyCam cam_;
			public:
				CollidablePairsTest():
					static_entities_(0),
					active_entities_(0),
					static_tree_(),
					collision_detector_(),
					map_(0),
					cam_(){
					ofSetWindowTitle("fragments");
					ofSetFrameRate(30);
					ofEnableAlphaBlending();
					ofEnableDepthTest();
					ofEnableAntiAliasing();
				};
				virtual ~CollidablePairsTest(){};
				void setup(){
					//StaticEntityの追加
					for (int i = 0; i < 20; i++) {
						static_entities_.push_back(fragments::data::StaticEntity());
						float x = ofRandom(-2000,2000);
						float y = ofRandom(-500,500);
						float z = ofRandom(-2000,2000);
						float a = ofRandom(-50,50);
						static_entities_[i].SetPoint(0,100+x+a,0+y+a*5,0+z+a);
						a = ofRandom(-50,50);
						static_entities_[i].SetPoint(1,-100+x+a,0+y+a*5,100+z+a);
						a = ofRandom(-50,50);
						static_entities_[i].SetPoint(2,-100+x+a,0+y+a*5,-100+z+a);
					}

					//描画するmeshにStaticEntityを追加
					for(fragments::data::StaticEntity i : static_entities_){
						ofMesh mesh;
						mesh.setMode(OF_PRIMITIVE_TRIANGLES);
						for(int j = 0; j<3; j++){
							mesh.addVertex(ofVec3f(i.GetPoint(j)[0],i.GetPoint(j)[1],i.GetPoint(j)[2]));
						}

						for(int j = 0; j<3; j++){
							mesh.addIndex(2-j);
						}
						map_.push_back(mesh);
					}

					collision_detector_.Setup(static_entities_, active_entities_);
				};
				void update(){
					collision_detector_.Update(constraint_pairs_);
					integrator_.Update(active_entities_);
				};
				void draw(){
					cam_.begin();
					ofDrawGrid(500,10,false,true,true,true);
					ofSetColor(255,255,255);
					for(ofMesh i : map_){
						i.draw();
					}
					ofSetColor(255,0,0);
					for(fragments::data::StaticEntity i : static_entities_){
						float x,y,z;
						x = i.GetCenter()[0];
						y = i.GetCenter()[1];
						z = i.GetCenter()[2];
						ofDrawIcoSphere(x,y,z,20);
					}
					cam_.end();
				};
				void drawBox(fragments::data::StaticNode& static_node){
					Eigen::Vector3d bound_box_center(Eigen::Vector3d::Zero(3));
					Eigen::Vector3d bound_box_size(Eigen::Vector3d::Zero(3));

					bound_box_center= (static_node.box_size_max_ - static_node.box_size_min_)*0.5 + static_node.box_size_min_;
					bound_box_size = static_node.box_size_max_ - static_node.box_size_min_;

					ofBoxPrimitive box(bound_box_size[0], bound_box_size[1], bound_box_size[2]);
					box.setPosition(bound_box_center[0], bound_box_center[1], bound_box_center[2]);
					ofSetColor(64,64,64,125);
					box.drawWireframe();
					// static_node.box_size_min_();
					for (auto i : static_node.nexts_) {
						drawBox(i);
					}
				}
		};
	} // namespace tests
} // namespace fragments
