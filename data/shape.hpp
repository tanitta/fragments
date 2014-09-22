#pragma once

namespace fragments {
	namespace data {
		enum ShapeType{
			SQUARE,
			DISK,
			SPHERE
		};

		class Shape {
			public:
				const float size_;
				const fragments::data::ShapeType type_;

				Shape(const fragments::data::ShapeType type = fragments::data::ShapeType::SQUARE, float size = 1.0):
					type_(type),
					size_(size){};
				virtual ~Shape(){};
		};
	} // namespace data
} // namespace fragments
