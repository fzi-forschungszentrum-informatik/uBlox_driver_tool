#pragma once

#include "type_mapping.hpp"

namespace sensor_ublox {
namespace internal {
// clang-format off
        using UbxSupportedTypes = UbxTypeMappings<
                /* UBX-Nav-* */
                UbxTypeMapping<UbxDataNavPOSECEF, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::PosECEF>,
                UbxTypeMapping<UbxDataNavSTATUS, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::Status>,
                UbxTypeMapping<UbxDataNavDOP, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::DOP>,
                UbxTypeMapping<UbxDataNavSOL, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::Sol>,
                UbxTypeMapping<UbxDataNavPVT, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::PVT>,
                UbxTypeMapping<UbxDataNavTIMEGPS, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::TimeGPS>,
                UbxTypeMapping<UbxDataNavTIMEUTC, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::TimeUTC>,
                UbxTypeMapping<UbxDataNavDGPS, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::DGPS>,
                UbxTypeMapping<UbxDataNavSAT, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::Sat>,
                UbxTypeMapping<UbxDataNavSBAS, UbxMsgID::ClassID::Nav, UbxMsgID::NavMsgID::SBAS>,

                /* UBX-Rxm-* */
                UbxTypeMapping<UbxDataRxmRAWX, UbxMsgID::ClassID::Rxm, UbxMsgID::RxmMsgID::Rawx>,
                UbxTypeMapping<UbxDataRxmSFRBX, UbxMsgID::ClassID::Rxm, UbxMsgID::RxmMsgID::Sfrbx>,
                /* UBX-MON-* */
                UbxTypeMapping<UbxDataMonMSGPP, UbxMsgID::ClassID::Mon, UbxMsgID::MonMsgID::MsgPP>
            >;

// clang-format on
} // namespace internal
} // namespace sensor_ublox
