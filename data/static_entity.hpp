#pragma once
#include <boost/numeric/ublas/vector.hpp>
namespace fragments {
	namespace data {
		class StaticEntity{
			public:
				std::shared_ptr<std::vector< boost::numeric::ublas::vector<float>>> points_;
				std::vector< std::shared_ptr< boost::numeric::ublas::vector<float> > > face_;
				StaticEntity():
					points_(shared_ptr<std::vector<boost::numeric::ublas::vector<float>>>(new std::vector< boost::numeric::ublas::vector<float>>(4,boost::numeric::ublas::vector<float>(3))))
				{
				};
				virtual ~StaticEntity(){};

				void SetPoint(int index, float x, float y, float z){
					points_->operator[](index)[0] = x;
					points_->operator[](index)[1] = y;
					points_->operator[](index)[2] = z;
				};

				boost::numeric::ublas::vector<float> GetPoint(int index){
					return points_->operator[](index);
				}
		};
	} // namespace data
} // namespace fragments
