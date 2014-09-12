#pragma once

namespace fragments {
	class BroadPhase {
		public:
			BroadPhase(){};
			virtual ~BroadPhase(){};

			void Setup(){};
			void Update(){};
	};
} // namespace fragments
