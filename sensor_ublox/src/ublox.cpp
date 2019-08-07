#include "ublox.hpp"

#include <chrono>
#include <fstream>
#include <thread>
#include <generic_logger/generic_logger.hpp>
#include <boost/algorithm/string.hpp>

#include "exception.hpp"
#include "internal/protocol.hpp"

namespace sensor_ublox {

GNSSReceiver::GNSSReceiver(const GNSSReceiverOptions& opts) : opts_(opts) {
    configHandler_ = std::make_unique<internal::UbxConfigHandler>(opts.configFile, opts.baseConfigFile);
    initializeConnection();
}

GNSSReceiver::~GNSSReceiver() {
    stop();
}

void GNSSReceiver::initializeConnection() {
    ioService_ = std::make_unique<baio::io_service>();
    try {
        serialPort_ = std::make_unique<boost::asio::serial_port>(*ioService_);
        // open serial port
        serialPort_->open(opts_.deviceName.string());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // setup serial port
        serialPort_->set_option(baio::serial_port_base::baud_rate(configHandler_->config().baudRate));
        serialPort_->set_option(baio::serial_port_base::character_size(8));
        serialPort_->set_option(baio::serial_port_base::flow_control(baio::serial_port_base::flow_control::none));
        serialPort_->set_option(baio::serial_port_base::parity(baio::serial_port_base::parity::none));
        serialPort_->set_option(baio::serial_port_base::stop_bits(baio::serial_port_base::stop_bits::one));

        DEBUG_STREAM("Connection to serial port '" << opts_.deviceName.string() << "' established");
    } catch (std::runtime_error& e) {
        throw GNSSException(std::string("Failed to open serial port: ") + e.what());
    }
}

void GNSSReceiver::waitForAcknowledgement(const std::chrono::microseconds& timeout) {
    auto waitTimePoint = std::chrono::high_resolution_clock::now() + timeout;
    while (acknowledgement_ == UbxAcknowledgement::WAIT && std::chrono::high_resolution_clock::now() < waitTimePoint) {
        asyncWorker_->wait(timeout);
    }
}

void GNSSReceiver::run() {
    dataStreamReader_ = std::make_unique<internal::UbxDataStreamReader>();

    asyncWorker_ = std::make_shared<internal::AsyncWorker>(*serialPort_, *ioService_);
    asyncWorker_->setCallback(
        [this](const uint8_t* data, const size_t numBytes) { this->asyncReadCallback(data, numBytes); });
    asyncWorker_->run();
    loadConfiguration();
    if (configHandler_->config().dgps.enabled == true) {
        rtcmListener_ =
            std::make_unique<internal::RTCMListener>(asyncWorker_, configHandler_->config().dgps.serialDevicePath);
        rtcmListener_->startListener();
    }
}

void GNSSReceiver::loadConfiguration() {
    const auto& streams = configHandler_->getConfigStreams();
    auto timeout = std::chrono::milliseconds(20);
    for (const auto& s : streams) {
        for (uint32_t i = 0; i < 10; i++) {
            acknowledgement_ = UbxAcknowledgement::WAIT;
            asyncWorker_->send(s);
            waitForAcknowledgement(timeout);
            if (acknowledgement_ == UbxAcknowledgement::ACK) {
                INFO_STREAM("Uploading of configuration '" << static_cast<int32_t>(s[2]) << " - "
                                                           << static_cast<int32_t>(s[3]) << "' succeeded");
                break;
            } else if (acknowledgement_ == UbxAcknowledgement::NACK) {
                ERROR_STREAM("Uploading of configuration '" << static_cast<int32_t>(s[2]) << " - "
                                                            << static_cast<int32_t>(s[3]) << "' failed");
                break;
            } else {
                DEBUG_STREAM("No status received from device - trying again ...");
            }
        }
    }
}

void GNSSReceiver::stop() {
    if (rtcmListener_) {
        rtcmListener_->stopListener();
    }
    asyncWorker_->stop();
    condVarShutdown_.notify_all();
}

void GNSSReceiver::asyncReadCallback(const uint8_t* data, const size_t numBytes) {
    dataStreamReader_->addData(data, numBytes);
    auto nextUbxObject = dataStreamReader_->get();
    while (nextUbxObject != dataStreamReader_->end()) {
        if (nextUbxObject.classID() == 0x05) {
            // handle acknowledgements differently
            DEBUG_STREAM("Acknowledgement packet received: " << (nextUbxObject.messageID() == 0x00 ? "NACK" : "ACK"));
            acknowledgement_ = (nextUbxObject.messageID() == 0x00) ? UbxAcknowledgement::NACK : UbxAcknowledgement::ACK;
        } else {
            auto key = internal::ObjectFactory::key_type(nextUbxObject.classID(), nextUbxObject.messageID());
            DEBUG_STREAM("Data packet received: " << static_cast<int64_t>(nextUbxObject.classID()) << " / "
                                                  << static_cast<int64_t>(nextUbxObject.messageID()));
            std::lock_guard<std::mutex> guard(factoryMutex_);
            //            for (const auto& m : callbackFactory_) {
            //                DEBUG_STREAM("Mapping: " << (int)m.first.first << " / " << (int)m.first.second);
            //            }
            auto itLower = callbackFactory_.lower_bound(key);
            auto itUpper = callbackFactory_.upper_bound(key);
            for (auto it = itLower; it != itUpper; ++it) {
                it->second->handleData(nextUbxObject);
            }
        }
        nextUbxObject = dataStreamReader_->get();
    }
}

bool GNSSReceiver::uploadConfiguration(const internal::DataBuffer& cfgBuffer) {
    auto timeout = std::chrono::milliseconds(500);
    acknowledgement_ = UbxAcknowledgement::WAIT;
    asyncWorker_->send(cfgBuffer);
    waitForAcknowledgement(timeout);
    return acknowledgement_ == UbxAcknowledgement::ACK;
}

std::vector<std::pair<uint8_t, uint8_t>> GNSSReceiver::getEnabledMessages() const {
    std::vector<std::pair<uint8_t, uint8_t>> enabledMessages;
    for (const auto& m : configHandler_->config().msgs) {
        if (m.rate > 0) {
            enabledMessages.push_back(m.msgID);
        }
    }
    return enabledMessages;
}

} // namespace sensor_ublox
