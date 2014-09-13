#pragma once

namespace fragments {
	class ForceAdder {
		public:
			ForceAdder(){};
			virtual ~ForceAdder(){};

			void Setup(){};
			void Update(){};
	};
} // namespace fragments
