//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <generic_logger/generic_logger.hpp>

#include "data_types_impl.hpp"

namespace baio = boost::asio;

namespace sensor_ublox {
namespace internal {

class AsyncWorker {
public:
    using CallbackFunc = std::function<void(const uint8_t* data, const size_t)>;

    AsyncWorker(baio::serial_port& serialPort, baio::io_service& ioService);

    ~AsyncWorker();

    void run();

    void setCallback(const CallbackFunc& callback) {
        readCallback_ = callback;
    }

    bool send(const DataBuffer& data);
    void wait(const std::chrono::microseconds& timeout);

    bool isOpen() const {
        return serialPort_.is_open();
    }

    void stop() {
        if (threadRunning_ == true) {
            stopWorker_.store(true);
            backgroundThread_.join();
            threadRunning_.store(false);
        }
    }

private:
    static constexpr int64_t BufferSize = 1024;

    void doRead();
    void readEnd(const boost::system::error_code& errorCode, size_t bytesRead);
    void doWrite(const DataBuffer& data);
    void doClose();
    void printDataBufferDebug(const uint8_t* data, const size_t bytesRead);

    baio::serial_port& serialPort_;
    boost::asio::io_service& ioService_;

    std::mutex readMutex_;
    std::condition_variable readCondVar_;
    std::array<uint8_t, BufferSize> readBuffer_;

    std::mutex writeMutex_;
    std::condition_variable writeCondVar_;

    std::thread backgroundThread_;

    CallbackFunc readCallback_;

    std::atomic_bool stopWorker_;
    std::atomic_bool threadRunning_;
    std::ofstream logFile_;
};

} // namespace internal
} // namespace sensor_ublox
