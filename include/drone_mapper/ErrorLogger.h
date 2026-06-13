#pragma once

#include <filesystem>
#include <fstream>
#include <string>

namespace drone_mapper {

class ErrorLogger {
public:
    explicit ErrorLogger(const std::filesystem::path& path);

    void log(const std::string& code, const std::string& message);

private:
    std::ofstream file_;
};

} // namespace drone_mapper
