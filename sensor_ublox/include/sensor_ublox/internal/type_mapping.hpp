#pragma once

#include <utility>

namespace sensor_ublox {
namespace internal {

struct InvalidMappingType {};

/**
 * Container type for a list of function mappings.
 */
template <typename... Mapping_>
struct UbxTypeMappings {};

/**
 * Container type for type mapping.
 */
template <typename DataType_, int64_t MsgClassID_, int64_t MsgID_>
struct UbxTypeMapping {
    static const int64_t MsgClassID = MsgClassID_;
    static const int64_t MsgID = MsgID_;
    using DataType = DataType_;
};

template <typename DataType_, int64_t MsgClassID_, int64_t MsgID_>
const int64_t UbxTypeMapping<DataType_, MsgClassID_, MsgID_>::MsgClassID;

template <typename DataType_, int64_t MsgClassID_, int64_t MsgID_>
const int64_t UbxTypeMapping<DataType_, MsgClassID_, MsgID_>::MsgID;


/**
 * Recursive TMP (implementation) to check if a supported data type is provided
 */
template <typename DataType_, typename... Mappings_>
struct UbxTypeMappingIsSupportedTypeImpl {
    static constexpr bool Value = false;
};

template <typename DataType_, typename... Mappings_>
constexpr bool UbxTypeMappingIsSupportedTypeImpl<DataType_, Mappings_...>::Value;

template <typename DataType_, typename Mapping_, typename... Mappings_>
struct UbxTypeMappingIsSupportedTypeImpl<DataType_, Mapping_, Mappings_...> {
    static constexpr bool Value = (std::is_same<typename Mapping_::DataType, DataType_>::value
                                       ? true
                                       : UbxTypeMappingIsSupportedTypeImpl<DataType_, Mappings_...>::Value);
};

template <typename DataType_, typename Mapping_, typename... Mappings_>
constexpr bool UbxTypeMappingIsSupportedTypeImpl<DataType_, Mapping_, Mappings_...>::Value;

/**
 * TMP (interface) to check that a supported data type is provided
 */
template <typename DataType_, typename>
struct UbxTypeMappingIsSupportedType;

template <typename DataType_, typename... Mappings_>
struct UbxTypeMappingIsSupportedType<DataType_, UbxTypeMappings<Mappings_...>> {
    static constexpr bool Value = UbxTypeMappingIsSupportedTypeImpl<DataType_, Mappings_...>::Value;
};

template <typename DataType_, typename... Mappings_>
constexpr bool UbxTypeMappingIsSupportedType<DataType_, UbxTypeMappings<Mappings_...>>::Value;

/**
 * Recursive TMP (implementation) to retrieve the message ID for a given data type.
 */
template <typename DataType_, typename... Mappings_>
struct UbxTypeMappingGetMsgIDImpl {
    static const int64_t Value = -1;
};

template <typename DataType_, typename... Mappings_>
const int64_t UbxTypeMappingGetMsgIDImpl<DataType_, Mappings_...>::Value;


template <typename DataType_, typename Mapping_, typename... Mappings_>
struct UbxTypeMappingGetMsgIDImpl<DataType_, Mapping_, Mappings_...> {
    // clang-format off
        static const int64_t Value =
                ((std::is_same<DataType_, typename Mapping_::DataType>::value)
                        ? Mapping_::MsgID
                        : UbxTypeMappingGetMsgIDImpl<DataType_, Mappings_...>::Value);
    // clang-format on
};

template <typename DataType_, typename Mapping_, typename... Mappings_>
const int64_t UbxTypeMappingGetMsgIDImpl<DataType_, Mapping_, Mappings_...>::Value;

/**
 * TMP (interface) to retrieve the message ID for a given data type.
 */
template <typename DataType_, typename>
struct UbxTypeMappingGetMsgID;

template <typename DataType_, typename... Mappings_>
struct UbxTypeMappingGetMsgID<DataType_, UbxTypeMappings<Mappings_...>> {
    static_assert(UbxTypeMappingIsSupportedType<DataType_, UbxTypeMappings<Mappings_...>>::Value == true,
                  "Requested data type is not in list");
    static const int64_t Value = UbxTypeMappingGetMsgIDImpl<DataType_, Mappings_...>::Value;
};

template <typename DataType_, typename... Mappings_>
const int64_t UbxTypeMappingGetMsgID<DataType_, UbxTypeMappings<Mappings_...>>::Value;

/**
 * Recursive TMP (implementation) to retrieve the class ID for a given data type.
 */
template <typename DataType_, typename... Mappings_>
struct UbxTypeMappingGetClassIDImpl {
    static const int64_t Value = -1;
};

template <typename DataType_, typename... Mappings_>
const int64_t UbxTypeMappingGetClassIDImpl<DataType_, Mappings_...>::Value;

template <typename DataType_, typename Mapping_, typename... Mappings_>
struct UbxTypeMappingGetClassIDImpl<DataType_, Mapping_, Mappings_...> {
    // clang-format off
        static const int64_t Value =
                ((std::is_same<DataType_, typename Mapping_::DataType>::value)
                        ? Mapping_::MsgClassID
                        : UbxTypeMappingGetClassIDImpl<DataType_, Mappings_...>::Value);
    // clang-format on
};

template <typename DataType_, typename Mapping_, typename... Mappings_>
const int64_t UbxTypeMappingGetClassIDImpl<DataType_, Mapping_, Mappings_...>::Value;

/**
 * TMP (interface) to retrieve the message ID for a given data type.
 */
template <typename DataType_, typename>
struct UbxTypeMappingGetClassID;

template <typename DataType_, typename... Mappings_>
struct UbxTypeMappingGetClassID<DataType_, UbxTypeMappings<Mappings_...>> {
    static_assert(UbxTypeMappingIsSupportedType<DataType_, UbxTypeMappings<Mappings_...>>::Value == true,
                  "Requested data type is not in list");
    static const int64_t Value = UbxTypeMappingGetClassIDImpl<DataType_, Mappings_...>::Value;
};

template <typename DataType_, typename... Mappings_>
const int64_t UbxTypeMappingGetClassID<DataType_, UbxTypeMappings<Mappings_...>>::Value;

} // namespace internal
} // namespace sensor_ublox
