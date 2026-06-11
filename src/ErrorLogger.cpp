#include <drone_mapper/ErrorLogger.h>

#include <stdexcept>

namespace drone_mapper {

ErrorLogger::ErrorLogger(const std::filesystem::path& path) {
    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    file_.open(path);
    if (!file_) {
        throw std::runtime_error("ErrorLogger: cannot open log file: " + path.string());
    }
}

void ErrorLogger::log(const std::string& code, const std::string& message) {
    file_ << "[" << code << "] " << message << "\n" << std::flush;
}

} // namespace drone_mapper
