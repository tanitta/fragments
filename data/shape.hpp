#pragma once

namespace fragments {
	namespace data {
		//! ActiveEntityの形状
		/*!
		 * 正方形と円盤と球の3種類をサポート
		 */
		enum ShapeType{
			SQUARE,
			DISK,
			SPHERE
		};
		
		/*!
		 * ActiveEntityの形状を定義する
		*/
		class Shape {
			public:
				//! 形状のサイズを定義する
				/*!
				 * DISK : 半径
				 * SPHERE : 半径
				 * 
				 * 以下未定
				 * SQUARE : 1辺の1/2
				 */
				const float size_;
				const fragments::data::ShapeType type_;
				
				Shape(const fragments::data::ShapeType type = fragments::data::ShapeType::SQUARE, float size = 1.0):
					type_(type),
					size_(size){};
				virtual ~Shape(){};
		};
	} // namespace data
} // namespace fragments
