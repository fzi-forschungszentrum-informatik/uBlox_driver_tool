#include "internal/async_worker.h"

namespace sensor_ublox {
namespace internal {

AsyncWorker::AsyncWorker(baio::serial_port& serialPort, baio::io_service& ioService)
        : serialPort_(serialPort), ioService_(ioService) {
    stopWorker_.store(false);
}

void AsyncWorker::run() {
    ioService_.post([this]() { this->doRead(); });
    threadRunning_.store(true);
    backgroundThread_ = std::thread([this]() { this->ioService_.run(); });
}

AsyncWorker::~AsyncWorker() {
    ioService_.post([this]() { this->doClose(); });
    stop();
    ioService_.reset();
}

bool AsyncWorker::send(const internal::DataBuffer& data) {
    std::lock_guard<std::mutex> guard(writeMutex_);
    ioService_.post([data, this]() { this->doWrite(data); });
    return true;
}

void AsyncWorker::doRead() {
    serialPort_.async_read_some(
        baio::buffer(readBuffer_.data(), BufferSize),
        boost::bind(&AsyncWorker::readEnd, this, baio::placeholders::error, baio::placeholders::bytes_transferred));
}

void AsyncWorker::printDataBufferDebug(const uint8_t* data, const size_t numBytes) {
    std::stringstream hexLine;
    for (uint64_t i = 0; i < numBytes; i++) {
        hexLine << std::setfill('0') << std::setw(2) << std::hex << static_cast<int64_t>(data[i]) << " ";
        if (i > 0 && (((i + 1) % 8 == 0) || i == numBytes - 1)) {
            DEBUG_STREAM("Hex: " << hexLine.str());
            hexLine.clear();
            hexLine.str(std::string());
        }
    }
}

void AsyncWorker::readEnd(const boost::system::error_code& errorCode, size_t bytesRead) {
    if (errorCode) {
        ERROR_STREAM("Failed to read bytes from serial device (err-code: " << errorCode.message());
    } else if (bytesRead > 0) {
        DEBUG_STREAM("Read bytes from serial device: " << bytesRead);
        printDataBufferDebug(readBuffer_.data(), bytesRead);
        const uint8_t* buf = readBuffer_.data();
        // call the read callback
        std::unique_lock<std::mutex> lock(readMutex_);
        if (readCallback_) {
            readCallback_(readBuffer_.data(), bytesRead);
        }
        readCondVar_.notify_all();
    }

    if (stopWorker_ == false) {
        ioService_.post([this]() { this->doRead(); });
    }
}

void AsyncWorker::doWrite(const DataBuffer& data) {
    std::lock_guard<std::mutex> guard(writeMutex_);
    baio::write(serialPort_, boost::asio::buffer(data.data(), data.size()));
    DEBUG_STREAM("Send bytes to serial device: " << data.size());
    printDataBufferDebug(data.data(), data.size());
    writeCondVar_.notify_all();
}

void AsyncWorker::doClose() {
    boost::system::error_code error;
    serialPort_.cancel(error);
}

void AsyncWorker::wait(const std::chrono::microseconds& timeout) {
    std::unique_lock<std::mutex> lock(readMutex_);
    const auto& res = readCondVar_.wait_for(lock, timeout);
    DEBUG_STREAM("Wait ended due to: " << (res == std::cv_status::timeout ? "timeout" : "condition"));
}
} // namespace internal
} // namespace sensor_ublox
