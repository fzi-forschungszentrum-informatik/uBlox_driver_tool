#include <fstream>
#include <boost/algorithm/string.hpp>
#include "data_types.hpp"
#include "exception.hpp"
#include "internal/data_stream_writer.hpp"
#include "yaml-cpp/yaml.h"

#include "internal/config.hpp"

namespace sensor_ublox {
namespace internal {
std::pair<uint8_t, uint8_t> UbxConfig::getMsgIDByString(const std::string& msg) {
    if (msg == "nav_pvt") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::PVT);
    } else if (msg == "nav_posecef") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::PosECEF);
    } else if (msg == "nav_dop") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::DOP);
    } else if (msg == "nav_sol") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::Sol);
    } else if (msg == "nav_timegps") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::TimeGPS);
    } else if (msg == "nav_timeutc") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::TimeUTC);
    } else if (msg == "nav_dgps") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::DGPS);
    } else if (msg == "nav_sat") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::Sat);
    } else if (msg == "nav_sbas") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::SBAS);
    } else if (msg == "nav_status") {
        return std::make_pair(UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::Status);
    } else if (msg == "rxm_rawx") {
        return std::make_pair(UbxMsgID::ClassID::Rxm, UbxMsgID::RxmMsgID::Rawx);
    } else if (msg == "rxm_sfrbx") {
        return std::make_pair(UbxMsgID::ClassID::Rxm, UbxMsgID::RxmMsgID::Sfrbx);
    } else if (msg == "mon_msgpp") {
        return std::make_pair(UbxMsgID::ClassID::Mon, UbxMsgID::MonMsgID::MsgPP);
    } else {
        throw GNSSException("Invalid messge ID specified: " + msg);
    }
}

UbxConfigHandler::UbxConfigHandler(const bfs::path& configFile, const bfs::path& baseConfigFile)
        : baseConfigFile_(baseConfigFile) {
    loadConfig(configFile);
}

void UbxConfigHandler::loadConfig(const bfs::path& configFile) {
    INFO_STREAM("Config: " << configFile.string());
    if (bfs::exists(configFile) == false) {
        throw GNSSException("Config file '" + configFile.string() + "' does not exist");
    } else if (bfs::is_regular_file(configFile) == false) {
        throw GNSSException("Config file '" + configFile.string() + "' is a directory");
    }
    YAML::Node root = YAML::LoadFile(configFile.string());
    const auto& m8Config = root["ublox_m8_config"];
    // device configuration
    const auto& deviceConfig = m8Config["device"];
    if (deviceConfig.IsDefined()) {
        const auto& baudRate = deviceConfig["baud_rate"];
        if (baudRate.IsDefined()) {
            config_.baudRate = baudRate.as<int>();
        }
    }
    const auto& gnssConfig = m8Config["gnss"];
    if (gnssConfig.IsDefined()) {
        const auto& rate = gnssConfig["rate"];
        if (rate.IsDefined()) {
            config_.solutionRate = rate.as<int>();
        }
    }
    const auto& dgpsConfig = m8Config["dgps"];
    if (dgpsConfig.IsDefined()) {
        const auto& enable = dgpsConfig["enable"];
        if (enable.IsDefined()) {
            config_.dgps.enabled = enable.as<bool>();
        }
        const auto& devicePath = dgpsConfig["device_path"];
        if (devicePath.IsDefined()) {
            config_.dgps.serialDevicePath = devicePath.as<std::string>();
        }
    }
    const auto& msgConfig = m8Config["messages"];
    if (msgConfig.IsDefined()) {
        for (const auto& it : msgConfig) {
            std::string msgName = it.first.as<std::string>();
            int16_t rate = it.second.as<int>();
            config_.msgs.push_back({UbxConfig::getMsgIDByString(msgName), rate});
        }
    }
}

UbxConfigRaw UbxConfigHandler::createConfigDataObject(const std::string& configEntry) const {
    // expected format: <CONFIG_ID> - <BYTE0> <BYTE1> ...
    std::vector<std::string> configComponents;
    boost::split(configComponents, configEntry, boost::is_any_of(" "));
    auto configID = configComponents[0];
    DEBUG_STREAM("Creating config entry for: " << configID);
    UbxConfigRaw rawConfig;
    rawConfig.classID = static_cast<uint8_t>(std::stoul(configComponents[2], nullptr, 16));
    rawConfig.msgID = static_cast<uint8_t>(std::stoul(configComponents[3], nullptr, 16));
    std::for_each(configComponents.cbegin() + 6, configComponents.cend(), [&rawConfig](const std::string& byte) {
        rawConfig.data.push_back(static_cast<uint8_t>(std::stoul(byte, nullptr, 16)));
    });
    return rawConfig;
}

std::vector<DataBuffer> UbxConfigHandler::getConfigStreams() const {
    std::vector<DataBuffer> streams;
    // base config
    std::ifstream configFile(baseConfigFile_.string(), std::ifstream::in);
    while (!configFile.eof()) {
        std::string line;
        std::getline(configFile, line);
        if (!line.empty() && boost::starts_with(line, "#") == false) {
            const UbxConfigRaw& rawConfig = createConfigDataObject(line);
            streams.push_back(UbxDataStreamWriter::writeData(rawConfig, rawConfig.classID, rawConfig.msgID));
        }
    }
    INFO_STREAM("Base configuration uploaded from file: " << baseConfigFile_.string());
    // rate configuration
    UbxConfigRATE rateConfig;
    bzero(&rateConfig, sizeof(rateConfig));
    rateConfig.measRate = 1000 / config_.solutionRate;
    rateConfig.navRate = 1;
    rateConfig.timeRef = 1; /* GPS time */
    streams.push_back(UbxDataStreamWriter::writeData(rateConfig, UbxMsgID::ClassID::Cfg, UbxMsgID::CfgMsgID::Rate));
    // prt configuration
    UbxConfigPRTUSB portConfig;
    bzero(&portConfig, sizeof(portConfig));
    portConfig.portID = 3;
    portConfig.reserved1 = 0;
    portConfig.txReady = 0;
    portConfig.inProtoMask = 0x03;
    if (config_.dgps.enabled) {
        portConfig.inProtoMask |= 0x20;
    }
    portConfig.outProtoMask = 0x01;
    streams.push_back(UbxDataStreamWriter::writeData(portConfig, UbxMsgID::ClassID::Cfg, UbxMsgID::CfgMsgID::Prt));
    // msg configuration
    for (const auto& m : config_.msgs) {
        UbxConfigMSG msgConfig;
        bzero(&msgConfig, sizeof(msgConfig));
        msgConfig.msgClass = m.msgID.first;
        msgConfig.msgID = m.msgID.second;
        msgConfig.rate[3] = m.rate;
        streams.push_back(UbxDataStreamWriter::writeData(msgConfig, UbxMsgID::ClassID::Cfg, UbxMsgID::CfgMsgID::Msg));
    }
    return streams;
}

} // namespace internal
} // namespace sensor_ublox
