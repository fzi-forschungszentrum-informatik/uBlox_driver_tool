#pragma once

#include <map>
#include <memory>
#include <vector>
#include <boost/filesystem.hpp>
#include "async_worker.h"
#include "../data_types.hpp"
#include "../ubx_data.h"

namespace bfs = boost::filesystem;

namespace sensor_ublox {
namespace internal {

struct UbxConfig {
    static std::pair<uint8_t, uint8_t> getMsgIDByString(const std::string& msg);

    UbxConfig() : baudRate(38400), solutionRate(1) {
        dgps.enabled = false;
    }

    struct DGPS {
        bool enabled;
        std::string serialDevicePath;
    };

    struct Msg {
        std::pair<uint8_t, uint8_t> msgID;
        int16_t rate;
    };

    int32_t baudRate;
    int16_t solutionRate;
    DGPS dgps;
    std::vector<Msg> msgs;
};

using UbxConfigMSG = UBX_CFG_MSG_SETU5_t;

using UbxConfigPRTUSB = UBX_CFG_PRT_USB_t;

using UbxConfigRATE = UBX_CFG_RATE_DATA0_t;

struct UbxConfigRaw {
    uint8_t classID;
    uint8_t msgID;
    DataBuffer data;
};

class UbxConfigHandler {
public:
    UbxConfigHandler(const bfs::path& configFile, const bfs::path& baseConfigFile);

    const UbxConfig& config() const {
        return config_;
    }

    std::vector<DataBuffer> getConfigStreams() const;

private:
    void loadConfig(const bfs::path& configFile);
    UbxConfigRaw createConfigDataObject(const std::string& configEntry) const;

    UbxConfig config_;
    bfs::path baseConfigFile_;
};
} // namespace internal
} // namespace sensor_ublox
