#pragma once

#include "ofMain.h"

#include <fragments/data/static_entity.hpp>
#include <fragments/data/active_entity.hpp>
#include <../data/static_node.hpp>
#include <fragments/collision_detector.hpp><`0`>
namespace fragments {
	namespace tests {
		class SearchCollidableNodeTest : public ofBaseApp{
			private:
				vector< fragments::data::StaticEntity > static_entities_;
				vector< fragments::data::ActiveEntity > active_entities_;
				fragments::data::StaticNode static_tree_;

				fragments::CollisionDetector collision_detector_;
				std::vector<ofMesh> map;
				ofEasyCam cam;

			public:
				SearchCollidableNodeTest():
				static_entities_(0),
				active_entities_(0),
				static_tree_(),
				collision_detector_(),
				map(0),
				cam(){
					ofSetWindowTitle("fragments");
					ofSetFrameRate(30);
					ofEnableAlphaBlending();
					ofEnableDepthTest();
					ofEnableAntiAliasing();
				};

				virtual ~SearchCollidableNodeTest(){};

				void setup(){
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

					for(fragments::data::StaticEntity i : static_entities_){
						ofMesh mesh;
						mesh.setMode(OF_PRIMITIVE_TRIANGLES);
						for(int j = 0; j<3; j++){
							mesh.addVertex(ofVec3f(i.GetPoint(j)[0],i.GetPoint(j)[1],i.GetPoint(j)[2]));
						}

						for(int j = 0; j<3; j++){
							mesh.addIndex(2-j);
						}
						map.push_back(mesh);
					}

					collision_detector_.Setup(static_entities_, active_entities_);
				};
				void update(){
					collision_detector_.Update();
				};
				void draw(){
					cam.begin();
					ofDrawGrid(500,10,false,true,true,true);
					ofSetColor(255,255,255);
					for(ofMesh i : map){
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
					cam.end();
				};
				void drawBox(fragments::data::StaticNode& static_node){
					boost::numeric::ublas::vector<float> bound_box_center(3);
					boost::numeric::ublas::vector<float> bound_box_size(3);

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

				void keyPressed(int key){};
				void keyReleased(int key){};
				void mouseMoved(int x, int y ){};
				void mouseDragged(int x, int y, int button){};
				void mousePressed(int x, int y, int button){};
				void mouseReleased(int x, int y, int button){};
				void windowResized(int w, int h){};
				void dragEvent(ofDragInfo dragInfo){};
				void gotMessage(ofMessage msg){};
		};
	} // namespace tests
} // namespace fragments
