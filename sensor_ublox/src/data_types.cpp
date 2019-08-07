#include "data_types.hpp"

namespace sensor_ublox {
// define const variables
const uint8_t UbxMsgID::ClassID::Nav;
const uint8_t UbxMsgID::ClassID::Rxm;
const uint8_t UbxMsgID::ClassID::Cfg;
const uint8_t UbxMsgID::ClassID::Mon;
const uint8_t UbxMsgID::NavMsgID::PosECEF;
const uint8_t UbxMsgID::NavMsgID::Status;
const uint8_t UbxMsgID::NavMsgID::DOP;
const uint8_t UbxMsgID::NavMsgID::Sol;
const uint8_t UbxMsgID::NavMsgID::PVT;
const uint8_t UbxMsgID::NavMsgID::TimeGPS;
const uint8_t UbxMsgID::NavMsgID::TimeUTC;
const uint8_t UbxMsgID::NavMsgID::DGPS;
const uint8_t UbxMsgID::NavMsgID::Sat;
const uint8_t UbxMsgID::NavMsgID::SBAS;
const uint8_t UbxMsgID::RxmMsgID::Rawx;
const uint8_t UbxMsgID::RxmMsgID::Sfrbx;
const uint8_t UbxMsgID::MonMsgID::MsgPP;
const uint8_t UbxMsgID::CfgMsgID::Prt;
const uint8_t UbxMsgID::CfgMsgID::Msg;
const uint8_t UbxMsgID::CfgMsgID::Rate;
} // namespace sensor_ublox
