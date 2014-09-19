#pragma once

#include "ofMain.h"

#include <fragments/data/static_entity.hpp>
#include <../data/static_node.hpp>
namespace fragments {
	namespace tests {
		class SortTest : public ofBaseApp{
			private:
				vector< fragments::data::StaticEntity > static_entities_;
				fragments::data::StaticNode static_tree_;

				std::vector<ofMesh> map;
				ofEasyCam cam;

			public:
				SortTest():
				static_entities_(0),
				static_tree_(),
				map(0),
				cam(){
					ofSetWindowTitle("fragments");
					ofEnableAlphaBlending();
					ofEnableDepthTest();
					ofEnableAntiAliasing();
				};

				virtual ~SortTest(){};

				void setup(){
					for (int i = 0; i < 20; i++) {
						static_entities_.push_back(fragments::data::StaticEntity());
						float x = ofRandom(-2000,2000);
						float y = ofRandom(-2000,2000);
						float z = ofRandom(-2000,2000);
						float a = ofRandom(-50,50);
						static_entities_[i].SetPoint(0,100+x+a,0+y+a,0+z+a);
						a = ofRandom(-50,50);
						static_entities_[i].SetPoint(1,-100+x+a,0+y+a,100+z+a);
						a = ofRandom(-50,50);
						static_entities_[i].SetPoint(2,-100+x+a,0+y+a,-100+z+a);
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
					std::vector<fragments::data::StaticEntity*> static_entity_ptrs(0);
					// for (auto i : static_entities_) {
					// 	static_entity_ptrs.push_back(&i);
					// }
					for(int i = 0; i< static_entities_.size(); i++){
						static_entity_ptrs.push_back(&static_entities_[i]);
					}
					static_tree_.MakeNode(static_entity_ptrs);
				};
				void update(){
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
					drawBox(static_tree_);
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
