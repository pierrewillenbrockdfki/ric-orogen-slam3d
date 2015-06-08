#include <slam3d/include/Logger.hpp>
#include <slam3d/include/Clock.hpp>

#include <base/Logging.hpp>

namespace slam3d
{
	class BaseLogger : public slam::Logger
	{
	public:
		BaseLogger() : slam::Logger(slam::Clock()){}
		~BaseLogger(){}
		
		virtual void message(slam::LOG_LEVEL lvl, const std::string& msg)
		{
			switch(lvl)
			{
				case slam::DEBUG:
					LOG_DEBUG("%s", msg.c_str());
					break;
				case slam::INFO:
					LOG_INFO("%s", msg.c_str());
					break;
				case slam::WARNING:
					LOG_WARN("%s", msg.c_str());
					break;
				case slam::ERROR:
					LOG_ERROR("%s", msg.c_str());
					break;
				case slam::FATAL:
					LOG_FATAL("%s", msg.c_str());
					break;
			}
		}
	};
}