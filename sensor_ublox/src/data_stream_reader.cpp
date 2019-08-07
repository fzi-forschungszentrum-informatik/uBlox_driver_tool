#include "internal/data_stream_reader.hpp"

#include <generic_logger/generic_logger.hpp>

namespace sensor_ublox {
namespace internal {

UbxDataObject UbxDataStreamReader::get() {
    while (buffer_->size() >= 2) {
        if (buffer_->at(0) == UbxProtocol::SyncByteA && buffer_->at(1) == UbxProtocol::SyncByteB) {
            // found the beginning of a new package
            // ToDo: potential problem, it is not contiguous
            uint16_t length = *reinterpret_cast<uint16_t*>(&buffer_->at(UbxProtocol::ProtocolLengthStart));
            size_t totalLength = UbxProtocol::ProtocolOverheadLength + length;
            if (buffer_->size() >= totalLength) {
                // full data package is available
                DataBuffer objectData;
                objectData.insert(objectData.end(), buffer_->begin(), buffer_->begin() + totalLength);
                if (UbxProtocol::isValidDataPacket(objectData)) {
                    DEBUG_STREAM("Valid packet extracted from stream: " << totalLength);
                    buffer_->erase(buffer_->begin(), buffer_->begin() + totalLength);
                    DEBUG_STREAM("Clean up of packet: " << buffer_->size());
                    return UbxDataObject(std::move(objectData), numObjectsRead_++);
                } else {
                    ERROR_STREAM("Invalid data packet received");
                    return end_;
                }
            } else {
                return end_;
            }
        } else {
            // skip the byte
            buffer_->pop_front();
        }
    }
    return end_;
}

constexpr size_t UbxDataStreamReader::BufferSize;

} // namespace internal
} // namespace sensor_ublox
