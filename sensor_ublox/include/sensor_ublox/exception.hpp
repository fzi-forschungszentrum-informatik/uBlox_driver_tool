#pragma once

#include <exception>
#include <string>

namespace sensor_ublox {
class GNSSException : public std::exception {
public:
    GNSSException(const std::string& msg) : msg_(msg) {
    }
    const char* what() throw() {
        return msg_.c_str();
    }

private:
    std::string msg_;
};
} // namespace sensor_ublox
