#pragma once

#include "protocol.hpp"
#include "serializer.hpp"
#include "../data_types.hpp"

namespace sensor_ublox {
namespace internal {
class UbxDataStreamWriter {
public:
    template <typename DataType_>
    static DataBuffer writeData(const DataType_& message, const uint8_t classID, const uint8_t msgID) {
        DataBuffer rawData = Serializer<DataType_>::get(message);
        DataBuffer data;
        data.resize(rawData.size() + UbxProtocol::ProtocolOverheadLength);
        data[0] = UbxProtocol::SyncByteA;
        data[1] = UbxProtocol::SyncByteB;
        data[2] = classID;
        data[3] = msgID;
        *reinterpret_cast<uint16_t*>(data.data() + 4) = static_cast<uint16_t>(rawData.size());
        std::copy(rawData.cbegin(), rawData.cend(), data.begin() + UbxProtocol::ProtocolOverheadHeaderLength);
        const auto& chksum = UbxProtocol::computeChecksum(
            data, UbxProtocol::ProtocolChksumStart, data.size() - UbxProtocol::ProtocolOverheadChksumLength);
        uint64_t chksumPos = UbxProtocol::ProtocolOverheadHeaderLength + rawData.size();
        data[chksumPos] = chksum.first;
        data[chksumPos + 1] = chksum.second;
        return data;
    }
};
} // namespace internal
} // namespace sensor_ublox
