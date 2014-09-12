#pragma once

namespace fragments {
	class NarrowPhase {
		public:
			NarrowPhase(){};
			virtual ~NarrowPhase(){};

			void Setup(){};
			void Update(){};
	};
} // namespace fragments
