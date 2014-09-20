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


				boost::numeric::ublas::vector<float> GetMaxPoint(const int axis){
					int max_index = 0;
					for (int i = 1; i < 3; i++) {
						if (points_[i][axis] > points_[max_index][axis]) {
							max_index = i;
						}
					}
					return points_[max_index];
				};

				boost::numeric::ublas::vector<float> GetMinPoint(const int axis){
					int min_index = 0;
					for (int i = 1; i < 3; i++) {
						if (points_[i][axis] < points_[min_index][axis]) {
							min_index = i;
						}
					}
					return points_[min_index];

				};

				boost::numeric::ublas::vector<float> GetCenter(){
					boost::numeric::ublas::vector<float> center(3);
					for(int i = 0; i<3; i++){
						center[i] = (GetMaxPoint(i)[i]-GetMinPoint(i)[i])*0.5+GetMinPoint(i)[i];
					}
					return center;
				}
		};
	} // namespace data
} // namespace fragments
