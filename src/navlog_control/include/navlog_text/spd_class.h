#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

class spd_class
{
public:
    spd_class()
    {
         info_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("../logs/fileName.txt");//创建sink指针
         info_sink->set_pattern("[%Y-%m-%d %H:%M:%S] %v");//sink_1、设置格式
         info_sink->set_level(spdlog::level::info);//sink_2、设置最低输出等级
    }
    ~spd_class()
    {
            logger->flush();
            info_sink->flush();
    }
    public:
    std::shared_ptr<spdlog::logger> logger = spdlog::default_logger();
    std::shared_ptr<spdlog::sinks::basic_file_sink_mt> info_sink;
};

inline spd_class spd;