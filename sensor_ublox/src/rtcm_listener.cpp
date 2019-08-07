#include "internal/rtcm_listener.hpp"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <generic_logger/generic_logger.hpp>
#include <sys/stat.h>
#include <sys/types.h>

namespace sensor_ublox {
namespace internal {
RTCMListener::RTCMListener(std::shared_ptr<AsyncWorker> asyncWorker, const bfs::path& devicePath)
        : asyncWorker_(asyncWorker), fdRTCMListener(-1) {
    devicePath_ = resolveVariables(devicePath);
    stopListener_.store(false);
    setupListener(devicePath_);
}

RTCMListener::~RTCMListener() {
    stopListener();
}

bfs::path RTCMListener::resolveVariables(const bfs::path& path) const {
    std::string p = path.string();
    size_t openPos = p.find('{');
    while (openPos != std::string::npos) {
        size_t closePos = p.find('}', openPos + 1);
        if (closePos != std::string::npos) {
            std::string envName = p.substr(openPos + 1, closePos - openPos - 1);
            char* resolvedEnvName = std::getenv(envName.c_str());
            INFO_STREAM("ENv: " << envName << " --> " << resolvedEnvName);
            if (resolvedEnvName == nullptr) {
                ERROR_STREAM("Environment variable '" << envName << "' not found. Using empty string.");
                resolvedEnvName = (char*)"";
            }
            p.replace(openPos, closePos - openPos + 1, std::string(resolvedEnvName));
        }
        openPos = p.find('{', closePos + 1);
    }
    return bfs::path(p);
}

void RTCMListener::startListener() {
    listenerThread_ = std::thread([this]() {
        try {
            this->loopListener();
        } catch (GNSSException& e) {
            ERROR_STREAM("Exception during RTCM listener: " << e.what());
        }
    });
}

void RTCMListener::stopListener() {
    if (fdRTCMListener != -1) {
        stopListener_.store(true);
        listenerThread_.join();
        (void)close(fdRTCMListener);
        (void)unlink(devicePath_.string().c_str());
        fdRTCMListener = -1;
    }
}

void RTCMListener::setupListener(const bfs::path& devicePath) {
    if (bfs::is_symlink(devicePath)) {
        int res = unlink(devicePath.string().c_str());
        if (res == -1) {
            throw GNSSException("Failed to delete symbolic link: " + devicePath.string());
        }
    } else if (bfs::exists(devicePath)) {
        throw GNSSException("RTCM device name '" + devicePath.string() + "' does already exist");
    }
    fdRTCMListener = open("/dev/ptmx", O_RDWR);
    if (fdRTCMListener == -1) {
        throw GNSSException(std::string("Failed to open serial multiplexer: ") + std::strerror(errno));
    }
    DEBUG_STREAM("Pseudoterminal device opened: " << fdRTCMListener);
    const auto* pts = ptsname(fdRTCMListener);
    if (pts == nullptr) {
        throw GNSSException(std::string("Failed to obtain number of slave pseudoterminal: ") + std::strerror(errno));
    }
    DEBUG_STREAM("Pseudoterminal device slave: " << pts);
    auto res = grantpt(fdRTCMListener);
    if (res == -1) {
        throw GNSSException(std::string("Failed to grant access for slave pseudoterminal: ") + std::strerror(errno));
    }
    DEBUG_STREAM("Pseudoterminal slave: access granted");
    res = unlockpt(fdRTCMListener);
    if (res == -1) {
        throw GNSSException(std::string("Failed to unlock slave pseudoterminal: ") + std::strerror(errno));
    }
    DEBUG_STREAM("Pseudoterminal slave: unlocked");
    res = symlink(pts, devicePath.string().c_str());
    if (res == -1) {
        throw GNSSException(std::string("Failed to create symlink: ") + std::strerror(errno));
    }
    DEBUG_STREAM("Symlink created from '" << devicePath.string() << "' to '" << pts << "'");
    res = chmod(pts, 00777);
    if (res == -1) {
        throw GNSSException(std::string("Failed to chmod: ") + std::strerror(errno));
    }
    INFO_STREAM("RTCM listener successfully created at: " + devicePath.string());
}

void RTCMListener::loopListener() {
    struct pollfd fdsPoll;
    fdsPoll.fd = fdRTCMListener;
    fdsPoll.events = POLLIN;
    while (stopListener_ == false) {
        const auto ret = poll(&fdsPoll, 1, 500);
        if (ret < 0) {
            throw GNSSException(std::string("Failed to poll file descriptor: ") + std::strerror(errno));
        }
        if (ret == 0) {
            continue;
        }
        if (fdsPoll.revents & POLLHUP) {
            return;
        } else if ((fdsPoll.revents & POLLERR) || (fdsPoll.revents & POLLNVAL)) {
            throw GNSSException(std::string("Poll failure"));
        } else if (fdsPoll.revents & POLLIN) {
            const auto numBytes = read(fdRTCMListener, readBuffer_.data(), BufferSize);
            if (numBytes == -1) {
                throw GNSSException(std::string("Failed to read bytes from serial device: ") + std::strerror(errno));
            }
            if (numBytes > 0) {
                DEBUG_STREAM("Read bytes from slave: " << numBytes);
                DataBuffer buffer(readBuffer_.data(), readBuffer_.data() + numBytes);
                asyncWorker_->send(buffer);
            } else {
                WARN_STREAM("Read 0 bytes from ready file descriptor");
            }
        }
    }
};
} // namespace internal
} // namespace sensor_ublox
