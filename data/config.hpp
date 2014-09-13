#pragma once

namespace fragments {
	class Config {
		public:
			bool isGravity;

			Config()
				isGravity(true){};
			virtual ~Config(){};
	};
} // namespace fragments
