#pragma once
#include <boost/numeric/ublas/vector.hpp>
namespace fragments {
	namespace data {
		class StaticEntity{
			public:
				std::vector< boost::numeric::ublas::vector<float>> points_;
				StaticEntity():
					points_(4,boost::numeric::ublas::vector<float>(3))
					// face_(3,)
				{
				};
				virtual ~StaticEntity(){};

				void SetPoint(const int index, const float x, const float y, const float z){
					points_[index][0] = x;
					points_[index][1] = y;
					points_[index][2] = z;
				};

				boost::numeric::ublas::vector<float> GetPoint(const int index) const {
					return points_[index];
				}

				float GetMax(const int axis)const{
					float max = points_[0][axis];
					for (int i = 0; i < 3; i++) {
						if (points_[i][axis] > max) {
							max = points_[i][axis];
						}
					}
					return max;
				}

				float GetMin(const int axis)const{
					float min = points_[0][axis];
					for (int i = 0; i < 3; i++) {
						if (points_[i][axis] < min) {
							min = points_[i][axis];
						}
					}
					return min;
				}

				boost::numeric::ublas::vector<float> GetCenter()const{
					boost::numeric::ublas::vector<float> center(3);
					for(int i = 0; i<3; i++){
						center[i] = (GetMax(i)-GetMin(i))*0.5;
					}
					return center;
				}
		};
	} // namespace data
} // namespace fragments
