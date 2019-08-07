#pragma once

#include "converter.hpp"
#include "serializer.hpp"
#include "supported_types.hpp"
#include "../data_types.hpp"
#include "../exception.hpp"

namespace sensor_ublox {
namespace internal {
class UbxDataObject {
public:
    UbxDataObject(DataBuffer&& data, const int64_t id) : data_(std::move(data)), id_(id) {
    }

    bool operator==(const UbxDataObject& obj) const {
        return id_ == obj.id_;
    }

    bool operator!=(const UbxDataObject& obj) const {
        return id_ != obj.id_;
    }

    uint8_t classID() const {
        return data_[2];
    }
    uint8_t messageID() const {
        return data_[3];
    }

    uint16_t length() const {
        return *reinterpret_cast<const uint16_t*>(&data_[4]);
    }

    template <typename DataType_>
    DataType_ getData() const {
        static_assert(UbxTypeMappingIsSupportedType<DataType_, UbxSupportedTypes>::Value == true,
                      "Unsupported data type requested");
        if (hasType<DataType_>() == false) {
            throw GNSSException("Invalid data type requested for data object");
        }
        const auto& data = Deserializer<DataType_>::get(data_);
        return data;
    }

    template <typename DataType_>
    bool hasType() const {
        static_assert(internal::UbxTypeMappingIsSupportedType<DataType_, internal::UbxSupportedTypes>::Value == true,
                      "Unsupported data type requested");
        const auto& classID = internal::UbxTypeMappingGetClassID<DataType_, internal::UbxSupportedTypes>::Value;
        const auto& msgID = internal::UbxTypeMappingGetMsgID<DataType_, internal::UbxSupportedTypes>::Value;
        return isMessage(classID, msgID);
    }

    bool isMessage(const uint8_t classId, const uint8_t messageId) const {
        return classID() == classId && messageID() == messageId;
    }

private:
    DataBuffer data_;
    int64_t id_;
};

} // namespace internal
} // namespace sensor_ublox
