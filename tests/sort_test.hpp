#pragma once

#include "ofMain.h"

#include <fragments/data/config.hpp>
#include <fragments/data/active_entity.hpp>
#include <fragments/data/static_entity.hpp>
#include <fragments/physics_engine.hpp>
#include <fragments/data/shape.hpp>
namespace fragments {
	namespace tests {
		class SortTest : public ofBaseApp{
			private:
				fragments::data::Config config_;

				vector< fragments::data::StaticEntity > static_entities_;
				vector< fragments::data::ActiveEntity > active_entities_;

				fragments::PhysicsEngine physics_engine_;

				std::vector<ofMesh> map;
				ofEasyCam cam;

			public:
				SortTest():config_(),
				static_entities_(0),
				active_entities_(0,fragments::data::SQUARE),
				physics_engine_(config_),
				map(0),
				cam(){
					// static_entity_.SetPoint(0,100,0,0);
					// static_entity_.SetPoint(1,-100,0,100);
					// static_entity_.SetPoint(2,-100,0,-100);

					// static_entities_.push_back(static_entity_);

					for (int i = 0; i < 10; i++) {
						static_entities_.push_back(fragments::data::StaticEntity());
						float x = ofRandom(-1000,1000);
						float z = ofRandom(-1000,1000);
						static_entities_[i].SetPoint(0,100+x,0,0+z);
						static_entities_[i].SetPoint(1,-100+x,0,100+z);
						static_entities_[i].SetPoint(2,-100+x,0,-100+z);
					}

					ofSetWindowTitle("fragments");
					ofEnableDepthTest();
					ofEnableAntiAliasing();

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
				};

				virtual ~SortTest(){};

				void setup(){
					physics_engine_.Setup(&static_entities_);
				};
				void update(){
				};
				void draw(){
					cam.begin();
					ofDrawGrid(500,10,false,true,true,true);
					for(ofMesh i : map){
						i.draw();
					}
					cam.end();
				};

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
