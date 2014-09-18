#pragma once

#include "ofMain.h"

#include <fragments/data/config.hpp>
#include <fragments/data/active_entity.hpp>
#include <fragments/data/static_entity.hpp>
#include <fragments/physics_engine.hpp>
#include <fragments/data/shape.hpp>
namespace fragments {
	namespace tests {
		class StaticEntityTest : public ofBaseApp{
			private:
				fragments::data::Config config_;

				fragments::data::StaticEntity static_entity_;
				vector< fragments::data::StaticEntity > static_entities_;
				vector< fragments::data::ActiveEntity > active_entities_;

				fragments::PhysicsEngine physics_engine_;

				ofMesh map;
				ofEasyCam cam;

			public:
				StaticEntityTest():config_(),
				static_entity_(),
				static_entities_(0),
				active_entities_(0,fragments::data::SQUARE),
				physics_engine_(config_),
				map(),
				cam(){
					static_entity_.SetPoint(0,100,0,0);
					static_entity_.SetPoint(1,-100,0,100);
					static_entity_.SetPoint(2,-100,0,-100);

					static_entities_.push_back(static_entity_);

					ofSetWindowTitle("fragments");
					ofEnableDepthTest();
					ofEnableAntiAliasing();
					map.setMode(OF_PRIMITIVE_TRIANGLES);

					for(fragments::data::StaticEntity i : static_entities_){
						for(int j = 0; j<3; j++){
							map.addVertex(ofVec3f(i.GetPoint(j)[0],i.GetPoint(j)[1],i.GetPoint(j)[2]));
						}

						for(int j = 0; j<3; j++){
							map.addIndex(2-j);
						}
					}
				};

				virtual ~StaticEntityTest(){};

				void setup(){
					physics_engine_.Setup(&static_entities_);
				};
				void update(){
				};
				void draw(){
					cam.begin();
					ofDrawGrid(500,10,false,true,true,true);
					map.draw();
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
