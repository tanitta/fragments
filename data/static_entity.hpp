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

				boost::numeric::ublas::vector<float> GetPoint(int index){
					return points_[index];
				}

				boost::numeric::ublas::vector<float> GetCenter(){	
				}
		};
	} // namespace data
} // namespace fragments
