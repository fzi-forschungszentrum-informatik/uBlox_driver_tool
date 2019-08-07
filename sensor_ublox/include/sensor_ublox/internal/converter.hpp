//#pragma once
//
//#include "protocol.hpp"
//#include "../data_types.hpp"
//
// namespace sensor_ublox {
// namespace internal {
//
// template <typename SrcType_>
// double conv(const SrcType_& src, const double scale) {
//    return static_cast<double>(src) * scale;
//}
//
// template <typename DataType_, typename RawDataType_>
// struct Converter {
//    static DataType_ convert(const RawDataType_& data);
//};
//
// template <>
// struct Converter<UbxDataNavPVT, UbxDataNavPVTInternal> {
//    static UbxDataNavPVT convert(const UbxDataNavPVTInternal& data) {
//        UbxDataNavPVT ubxData;
//        ubxData.iTOW = data.iTOW;
//        ubxData.year = data.year;
//        ubxData.month = data.month;
//        ubxData.day = data.day;
//        ubxData.hour = data.hour;
//        ubxData.min = data.min;
//        ubxData.sec = data.sec;
//        ubxData.valid = data.valid;
//        ubxData.tAcc = data.tAcc;
//        ubxData.nano = data.nano;
//        ubxData.fixType = data.fixType;
//        ubxData.flags = data.flags;
//        ubxData.numSV = data.numSV;
//        ubxData.lon = conv(data.lon, 1e-7);
//        ubxData.lat = conv(data.lat, 1e-7);
//        ubxData.height = conv(data.height, 1e-3);
//        ubxData.hMSL = conv(data.hMSL, 1e-3);
//        ubxData.hAcc = conv(data.hAcc, 1e-3);
//        ubxData.vAcc = conv(data.vAcc, 1e-3);
//        ubxData.velN = conv(data.velN, 1e-3);
//        ubxData.velE = conv(data.velE, 1e-3);
//        ubxData.velD = conv(data.velD, 1e-3);
//        ubxData.gSpeed = conv(data.gSpeed, 1e-3);
//        ubxData.headMot = conv(data.headMot, 1e-5);
//        ubxData.sAcc = conv(data.sAcc, 1e-3);
//        ubxData.headAcc = conv(data.headAcc, 1e-5);
//        ubxData.pDOP = conv(data.pDOP, 1e-1);
//        ubxData.headVeh = conv(data.headVeh, 1e-5);
//        return ubxData;
//    }
//};
//
// template <>
// struct Converter<UbxDataRxmRaw, UbxDataRxmRawInternal> {
//    static UbxDataRxmRaw convert(const UbxDataRxmRawInternal& data) {
//        UbxDataRxmRaw ubxData;
//        ubxData.rcvTow = data.header.rcvTow;
//        ubxData.week = data.header.week;
//        ubxData.leapS = data.header.leapS;
//        ubxData.numMeas = data.header.numMeas;
//        ubxData.recStat = data.header.recStat;
//        ubxData.version = data.header.version;
//        for (int64_t i = 0; i < ubxData.numMeas; i++) {
//            const auto& meaRaw = data.measurements[i];
//            UbxDataRxmRaw::Mea mea;
//            mea.prMes = meaRaw.prMes;
//            mea.cpMes = meaRaw.cpMes;
//            mea.doMes = meaRaw.doMes;
//            mea.gnssId = meaRaw.gnssId;
//            mea.svId = meaRaw.svId;
//            mea.freqId = meaRaw.freqId;
//            mea.locktime = meaRaw.locktime;
//            mea.cno = meaRaw.cno;
//            mea.prStdev = conv((meaRaw.prStdev & 0x0F) << 4, 0.01);
//            if (meaRaw.cpStdev == 0x0F) {
//                mea.cpStdev = -1;
//            } else {
//                mea.cpStdev = conv(meaRaw.cpStdev & 0x0F, 0.004);
//            }
//            mea.doStdev = conv((meaRaw.doStdev & 0x0F) << 4, 0.002);
//            mea.trkStat = meaRaw.trkStat;
//            ubxData.measurements.push_back(mea);
//        }
//        return ubxData;
//    }
//};
//
// template <>
// struct Converter<UbxDataNavDGPS, UbxDataNavDGPSInternal> {
//    static UbxDataNavDGPS convert(const UbxDataNavDGPSInternal& data) {
//        UbxDataNavDGPS ubxData;
//        ubxData.iTOW = data.header.iTOW;
//        ubxData.age = data.header.age;
//        ubxData.baseId = data.header.baseId;
//        ubxData.baseHealth = data.header.baseHealth;
//        ubxData.numCh = data.header.numCh;
//        ubxData.status = data.header.status;
//        for (int64_t i = 0; i < ubxData.numCh; i++) {
//            const auto chRaw = data.channels[i];
//            UbxDataNavDGPS::Channel ch;
//            ch.svid = chRaw.svid;
//            ch.flags = chRaw.flags;
//            ch.ageC = chRaw.ageC;
//            ch.prc = chRaw.prc;
//            ch.prrc = chRaw.prrc;
//            ubxData.channels.push_back(ch);
//        }
//        return ubxData;
//    }
//};
//
// template <>
// struct Converter<UbxDataNavDOP, UbxDataNavDOPInternal> {
//    static UbxDataNavDOP convert(const UbxDataNavDOPInternal& data) {
//        UbxDataNavDOP ubxData;
//
//        ubxData.iTOW = conv(data.iTOW, 1e-1);
//        ubxData.gDOP = conv(data.gDOP, 1e-1);
//        ubxData.pDOP = conv(data.pDOP, 1e-1);
//        ubxData.tDOP = conv(data.tDOP, 1e-1);
//        ubxData.vDOP = conv(data.vDOP, 1e-1);
//        ubxData.hDOP = conv(data.hDOP, 1e-1);
//        ubxData.nDOP = conv(data.nDOP, 1e-1);
//        ubxData.eDOP = conv(data.eDOP, 1e-1);
//        return ubxData;
//    }
//};
//
// template <>
// struct Converter<UbxDataNavPosECEF, UbxDataNavPosECEFInternal> {
//    static UbxDataNavPosECEF convert(const UbxDataNavPosECEFInternal& data) {
//        UbxDataNavPosECEF ubxData;
//
//        ubxData.iTOW = data.iTOW;
//        ubxData.ecefX = conv(data.ecefX, 1e-2);
//        ubxData.ecefY = conv(data.ecefY, 1e-2);
//        ubxData.ecefZ = conv(data.ecefZ, 1e-2);
//        ubxData.pAcc = conv(data.pAcc, 1e-2);
//        return ubxData;
//    }
//};
//
// template <>
// struct Converter<UbxDataNavSAT, UbxDataNavSATInternal> {
//    static UbxDataNavSAT convert(const UbxDataNavSATInternal& data) {
//        UbxDataNavSAT ubxData;
//
//        ubxData.iTOW = data.header.iTOW;
//        ubxData.version = data.header.version;
//        ubxData.numSvs = data.header.numSvs;
//        for (int64_t i = 0; i < ubxData.numSvs; i++) {
//            const auto& satelliteRaw = data.satellites[i];
//            UBX_NAV_SAT_DATA0_GNSSID_t satellite;
//            satellite.gnssId = satelliteRaw.gnssId;
//            satellite.svId = satelliteRaw.svId;
//            satellite.cno = satelliteRaw.cno;
//            satellite.elev = satelliteRaw.elev;
//            satellite.azim = satelliteRaw.azim;
//            satellite.prRes = conv(satelliteRaw.prRes, 0.1);
//            satellite.flags = satelliteRaw.flags;
//        }
//        return ubxData;
//    }
//};
//
//} // namespace internal
//} // namespace sensor_ublox
