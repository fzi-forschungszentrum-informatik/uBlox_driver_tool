#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <thread>
#include <boost/filesystem.hpp>
#include "async_worker.h"

namespace bfs = boost::filesystem;

namespace sensor_ublox {
namespace internal {
class RTCMListener {
public:
    RTCMListener(std::shared_ptr<AsyncWorker> asyncWorker, const bfs::path& devicePath);
    ~RTCMListener();
    void startListener();
    void stopListener();

private:
    static const size_t BufferSize = 8192;
    void setupListener(const bfs::path& devicePath);
    void loopListener();
    bfs::path resolveVariables(const bfs::path& path) const;
    std::shared_ptr<AsyncWorker> asyncWorker_;
    std::thread listenerThread_;
    int32_t fdRTCMListener;
    std::atomic_bool stopListener_;
    std::array<uint8_t, BufferSize> readBuffer_;
    bfs::path devicePath_;
};
} // namespace internal
} // namespace sensor_ublox
