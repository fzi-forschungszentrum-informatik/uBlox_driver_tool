#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

#include "data_types.hpp"
#include "exception.hpp"
#include "ubx_data.h"
#include "internal/async_worker.h"
#include "internal/data_stream_reader.hpp"
#include "internal/rtcm_listener.hpp"

namespace bfs = boost::filesystem;
namespace baio = boost::asio;

namespace sensor_ublox {
enum class UbxAcknowledgement { WAIT, ACK, NACK };

struct GNSSReceiverOptions {
    bfs::path deviceName;
    bfs::path baseConfigFile;
    bfs::path configFile;
};

class GNSSReceiver {
public:
    GNSSReceiver(const GNSSReceiverOptions& opts);
    ~GNSSReceiver();

    void run();
    void stop();

    void waitForStop() {
        std::unique_lock<std::mutex> lock(mutexShutdown_);
        condVarShutdown_.wait(lock);
    }

    std::vector<std::pair<uint8_t, uint8_t>> getEnabledMessages() const;
    double gnssFrequency() const {
        return configHandler_->config().solutionRate;
    }

    template <typename DataType_>
    void registerCallback(CallbackFuncType<DataType_> func) {
        const auto& classID = internal::UbxTypeMappingGetClassID<DataType_, internal::UbxSupportedTypes>::Value;
        const auto& msgID = internal::UbxTypeMappingGetMsgID<DataType_, internal::UbxSupportedTypes>::Value;
        internal::CallbackFactoryEntryPtr callbackHandler =
            std::make_unique<internal::CallbackFactoryEntryImpl<DataType_>>(func);
        internal::ObjectFactory::value_type value{std::make_pair(classID, msgID), std::move(callbackHandler)};
        std::lock_guard<std::mutex> guard(factoryMutex_);
        callbackFactory_.insert(std::move(value));
    }

    template <typename DataType_>
    void registerCallback(void (*func)(const DataType_&)) {
        registerCallback(CallbackFuncType<DataType_>(func));
    }

    template <typename DataType_, typename Func_>
    void registerCallback(Func_& func) {
        registerCallback(CallbackFuncType<DataType_>(func));
    }

private:
    void initializeConnection();
    void loadConfiguration();

    void asyncReadCallback(const uint8_t* data, const size_t numBytes);
    void waitForAcknowledgement(const std::chrono::microseconds& timeout);

    bool uploadConfiguration(const internal::DataBuffer& cfgBuffer);

    GNSSReceiverOptions opts_;
    std::unique_ptr<boost::asio::io_service> ioService_;
    std::unique_ptr<boost::asio::serial_port> serialPort_;
    std::mutex factoryMutex_;
    internal::ObjectFactory callbackFactory_;
    std::shared_ptr<internal::AsyncWorker> asyncWorker_;
    std::unique_ptr<internal::UbxDataStreamReader> dataStreamReader_;
    std::atomic<UbxAcknowledgement> acknowledgement_;
    std::unique_ptr<internal::UbxConfigHandler> configHandler_;

    std::mutex mutexShutdown_;
    std::condition_variable condVarShutdown_;
    std::unique_ptr<internal::RTCMListener> rtcmListener_;
};

} // namespace sensor_ublox
