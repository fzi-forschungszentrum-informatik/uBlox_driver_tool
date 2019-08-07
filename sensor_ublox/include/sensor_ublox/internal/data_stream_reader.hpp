#pragma once

#include <boost/circular_buffer.hpp>
#include <generic_logger/generic_logger.hpp>

#include "../data_types.hpp"

namespace sensor_ublox {
namespace internal {

class UbxDataStreamReader {
public:
    UbxDataStreamReader() : numObjectsRead_(0), end_(DataBuffer(), -1) {
        buffer_ = std::make_unique<boost::circular_buffer<uint8_t>>(BufferSize);
    }

    void addData(const uint8_t* data, const size_t numBytes) {
        DEBUG_STREAM("Adding data to stream reader: " << numBytes);
        buffer_->insert(buffer_->end(), data, data + numBytes);
        DEBUG_STREAM("Data in buffer: " << buffer_->size());
    }

    const UbxDataObject& end() const {
        return end_;
    }

    UbxDataObject get();

private:
    static constexpr size_t BufferSize = 8192;
    std::unique_ptr<boost::circular_buffer<uint8_t>> buffer_;
    uint64_t numObjectsRead_;
    UbxDataObject end_;
};

} // namespace internal
} // namespace sensor_ublox
