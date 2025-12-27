#include "utils/Logger.h"
#include "utils/Utils.h"
#include <iostream>
#include <sstream>
#include <chrono>
#ifdef __has_include
    #if __has_include(<filesystem>)
        #include <filesystem>
        namespace fs = std::filesystem;
    #else
        #include <experimental/filesystem>
        namespace fs = std::experimental::filesystem;
    #endif
#else
    #include <filesystem>
    namespace fs = std::filesystem;
#endif

Logger::Logger(const std::string& log_name, const std::string& log_dir,
               bool save_log_to_file, bool print_logs_to_console,
               const std::string& datatype, const std::string& suffix_in)
    : record_log(save_log_to_file), _datatype(datatype), _count_since_last_flush(0),
      log_name(log_name), _field_name_line_exists(false), print_logs_to_console(print_logs_to_console) {
    
    std::lock_guard<std::mutex> lock(file_mutex);
    
    if (record_log) {
        std::string suffix = ".txt";
        std::ios_base::openmode file_write_type = std::ios::out;
        
        if (_datatype == CSV) {
            suffix = ".csv";
            file_write_type = std::ios::out;
        } else if (_datatype == PKL) {
            suffix = ".pkl";
            file_write_type = std::ios::out | std::ios::binary;
        } else if (_datatype == BIN) {
            suffix = ".bin";
            file_write_type = std::ios::out | std::ios::binary;
        } else {
            suffix = ".txt";
            file_write_type = std::ios::out;
        }
        
        if (!suffix_in.empty()) {
            suffix = "." + suffix_in;
        }
        
        fs::create_directories(log_dir);
        std::string file_name = log_dir + "/" + log_name + suffix;
        file.open(file_name, file_write_type);
        
        if (!file.is_open()) {
            record_log = false;
        }
    }
}

Logger::~Logger() {
    close();
}

void Logger::write(const std::string& data) {
    std::lock_guard<std::mutex> lock(file_mutex);
    if (record_log && file.is_open()) {
        file << data;
        if (_datatype == BIN || true) {
            if (_count_since_last_flush > COUNT_BETWEEN_FLUSH_BIN) {
                _count_since_last_flush = 0;
                file.flush();
            } else {
                _count_since_last_flush++;
            }
        } else {
            file.flush();
        }
    }
}

void Logger::log(const std::string& data) {
    if (record_log) {
        if (_datatype == CSV) {
            // CSV logging would need field names setup
            write(data + "\n");
        } else if (_datatype == PKL) {
            // Pickle format not directly supported in C++
            write(data);
        } else if (_datatype == BIN) {
            write(data);
            return;
        } else {
            write(data + "\n");
        }
    }
    
    if (_datatype != BIN && print_logs_to_console) {
        std::cout << data << std::endl;
    }
}

void Logger::log(const std::map<std::string, std::string>& data) {
    if (_datatype == CSV) {
        if (!_field_name_line_exists) {
            _field_name_line_exists = true;
            bool first = true;
            for (const auto& pair : data) {
                _field_names.push_back(pair.first);
                if (first) {
                    write(pair.first);
                    first = false;
                } else {
                    write("," + pair.first);
                }
            }
            write("\n");
        }
        
        bool first = true;
        for (const auto& key : _field_names) {
            auto it = data.find(key);
            if (it != data.end()) {
                if (first) {
                    write(it->second);
                    first = false;
                } else {
                    write("," + it->second);
                }
            }
        }
        write("\n");
    }
}

void Logger::log(const std::vector<std::string>& data) {
    std::ostringstream oss;
    bool first = true;
    for (const auto& val : data) {
        if (first) {
            oss << val;
            first = false;
        } else {
            oss << "," << val;
        }
    }
    log(oss.str());
}

void Logger::log_err(const std::string& data) {
    if (_datatype == BIN || _datatype == PKL || _datatype == CSV) {
        return;
    }
    std::string error_msg = "ERROR: " + data;
    Utils::print_red(error_msg);
    if (record_log) {
        write(error_msg + "\n");
    }
}

void Logger::close() {
    if (record_log) {
        if (print_logs_to_console) {
            std::cout << "closing logger" << std::endl;
        }
        std::lock_guard<std::mutex> lock(file_mutex);
        if (file.is_open()) {
            file.flush();
            file.close();
        }
        record_log = false;
    }
}

