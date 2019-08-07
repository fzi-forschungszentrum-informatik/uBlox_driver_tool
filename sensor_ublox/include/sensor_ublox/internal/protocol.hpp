#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>

#include "../data_types.hpp"

namespace sensor_ublox {
namespace internal {
class UbxProtocol {
public:
    static const uint8_t SyncByteA = 0xB5;
    static const uint8_t SyncByteB = 0x62;
    static const size_t ProtocolOverheadSyncLength = 2;
    static const size_t ProtocolOverheadHeaderLength = 6;
    static const size_t ProtocolOverheadChksumLength = 2;
    static const size_t ProtocolOverheadLength = ProtocolOverheadHeaderLength + ProtocolOverheadChksumLength;
    static const size_t ProtocolChksumStart = 2;
    static const size_t ProtocolLengthStart = 4;

    static std::pair<uint8_t, uint8_t> computeChecksum(const DataBuffer& data,
                                                       const uint64_t startPos,
                                                       const uint64_t endPos) {
        auto cksum = std::make_pair<uint8_t, uint8_t>(0, 0);
        for (uint64_t i = startPos; i < endPos; i++) {
            cksum.first += data[i];
            cksum.second += cksum.first;
        }
        return cksum;
    }

    static bool isValidDataPacket(const DataBuffer& data) {
        auto cksum = computeChecksum(data, ProtocolChksumStart, data.size() - ProtocolOverheadChksumLength);
        return (cksum.first == data[data.size() - ProtocolOverheadChksumLength]) &&
               (cksum.second == data[data.size() - ProtocolOverheadChksumLength + 1]);
    }
};
} // namespace internal
} // namespace sensor_ublox
