#include <iostream>
#include <memory>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
using namespace std;

class Log
{
 
public:
    static void create_log(std::string filename);
    static void info(const std::string& info);
    static void debug(const std::string& info);
    static void error(const std::string& info);
private:
    Log();
    static Log* m_log;
    static shared_ptr<spdlog::logger> g_log;

};