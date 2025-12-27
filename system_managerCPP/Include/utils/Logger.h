#ifndef LOGGER_H
#define LOGGER_H
#include "general.h"

#include <string>
#include <fstream>
#include <mutex>
#include <vector>
#include <map>
#include "utils/CommonConstants.h"

class Logger {
private:
    bool record_log;
    std::string _datatype;
    std::vector<std::string> _field_names;
    int _count_since_last_flush;
    std::string log_name;
    bool _field_name_line_exists;
    bool print_logs_to_console;
    std::mutex file_mutex;
    std::ofstream file;

public:
    Logger(const std::string& log_name, const std::string& log_dir, 
           bool save_log_to_file, bool print_logs_to_console, 
           const std::string& datatype = TEXT, const std::string& suffix_in = "");
    ~Logger();
    
    void write(const std::string& data);
    void log(const std::string& data);
    void log(const std::map<std::string, std::string>& data);
    void log(const std::vector<std::string>& data);
    void log_err(const std::string& data);
    void close();
};

#endif // LOGGER_H

