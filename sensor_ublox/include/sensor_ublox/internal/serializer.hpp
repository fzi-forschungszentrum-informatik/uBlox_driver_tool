#pragma once

#include <iomanip>
#include <istream>
#include <string>
#include <cereal/cereal.hpp>
#include <generic_logger/generic_logger.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/array.hpp>

#include "config.hpp"
#include "protocol.hpp"
#include "../data_types.hpp"

namespace sensor_ublox {
namespace internal {
namespace {
class Util {
public:
    template <typename RawType_, typename Archive_, typename Type_>
    static void deserializeConv(Archive_& archive,
                                Type_& val,
                                const Type_& scale = Type_(1),
                                const Type_& offset = Type_(0)) {
        RawType_ tmp;
        archive(tmp);
        val = static_cast<Type_>(tmp) * scale + offset;
    }

    template <typename RawType_, typename Archive_, typename Type_>
    static void deserializeConvMask(Archive_& archive,
                                    Type_& val,
                                    const RawType_& mask,
                                    const Type_& scale = Type_(1),
                                    const Type_& offset = Type_(0)) {
        RawType_ tmp;
        archive(tmp);
        val = static_cast<Type_>(tmp & mask) * scale + offset;
    }

    template <typename RawType_, typename Archive_, typename Type_>
    static void deserializeExponent4bitBase2(Archive_& archive, Type_& val, const Type_& scale = Type_(1)) {

        RawType_ exp;
        archive(exp);
        exp = exp & 0x0f;
        int result = 1;
        while (exp) {
            result *= 2;
            --exp;
        }
        val = scale * static_cast<Type_>(result);
    }

    template <typename Archive_, typename Type_>
    static void deserialize(Archive_& archive, Type_& val) {
        archive(val);
    }

    template <typename RawType_, typename Archive_>
    static void consumeReserved(Archive_& archive) {
        RawType_ tmp;
        archive(tmp);
    }

    template <size_t N_, typename RawType_ = uint8_t, typename Archive_>
    static void consumeReserved(Archive_& archive) {
        RawType_ tmp[N_];
        archive(tmp);
    }

    static UbxDataGPSFix getGPSFix(const uint8_t value) {
        switch (value) {
        case 0x00:
            return UbxDataGPSFix::FixNone;
        case 0x01:
            return UbxDataGPSFix::FixDeadReckoningOnly;
        case 0x02:
            return UbxDataGPSFix::Fix2D;
        case 0x03:
            return UbxDataGPSFix::Fix3D;
        case 0x04:
            return UbxDataGPSFix::FixGPSDeadRecknoning;
        case 0x05:
            return UbxDataGPSFix::FixTimeOnly;
        default:
            return UbxDataGPSFix::FixNone;
        }
    }
};
} // namespace

class UbxMemStreamBuffer : public std::basic_streambuf<char> {
public:
    UbxMemStreamBuffer(const DataBuffer& data, const uint64_t startPos, const uint64_t endPos) : data_(data) {
        setg(const_cast<char*>(reinterpret_cast<const char*>((&data[startPos]))),
             const_cast<char*>(reinterpret_cast<const char*>((&data[startPos]))),
             const_cast<char*>(reinterpret_cast<const char*>((&data[endPos]))));
    }

private:
    const DataBuffer& data_;
};

class UbxMemStream : public std::istream {
public:
    UbxMemStream(const DataBuffer& data)
            : std::istream(&streamBuffer_), streamBuffer_(data,
                                                          UbxProtocol::ProtocolOverheadHeaderLength,
                                                          data.size() - UbxProtocol::ProtocolOverheadChksumLength) {
        rdbuf(&streamBuffer_);
    }

private:
    UbxMemStreamBuffer streamBuffer_;
};

template <typename DataType_>
struct Serializer {
    static DataBuffer get(const DataType_& data);
};

template <>
struct Serializer<UbxConfigMSG> {
    static DataBuffer get(const UbxConfigMSG& data) {
        std::stringstream sstr;
        cereal::BinaryOutputArchive ar(sstr);
        ar(data.msgClass);
        ar(data.msgID);
        ar(data.rate);
        const auto& str = sstr.str();
        return DataBuffer(str.cbegin(), str.cend());
    }
};

template <>
struct Serializer<UbxConfigPRTUSB> {
    static DataBuffer get(const UbxConfigPRTUSB& data) {
        std::stringstream sstr;
        cereal::BinaryOutputArchive ar(sstr);
        ar(data.portID);
        ar(data.reserved1);
        ar(data.txReady);
        ar(data.reserved2);
        ar(data.inProtoMask);
        ar(data.outProtoMask);
        ar(data.reserved3);
        ar(data.reserved4);
        const auto& str = sstr.str();
        return DataBuffer(str.cbegin(), str.cend());
    }
};

template <>
struct Serializer<UbxConfigRATE> {
    static DataBuffer get(const UbxConfigRATE& data) {
        std::stringstream sstr;
        cereal::BinaryOutputArchive ar(sstr);
        ar(data.measRate);
        ar(data.navRate);
        ar(data.timeRef);
        const auto& str = sstr.str();
        return DataBuffer(str.cbegin(), str.cend());
    }
};

template <>
struct Serializer<UbxConfigRaw> {
    static DataBuffer get(const UbxConfigRaw& data) {
        return data.data;
    }
};

template <typename DataType_>
struct Deserializer {
    static DataType_ get(const DataBuffer& data);
};

template <>
struct Deserializer<UbxDataNavPVT> {
    static UbxDataNavPVT get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavPVT ubxData;
        Util::deserialize(ar, ubxData.iTOW);
        Util::deserialize(ar, ubxData.year);
        Util::deserialize(ar, ubxData.month);
        Util::deserialize(ar, ubxData.day);
        Util::deserialize(ar, ubxData.hour);
        Util::deserialize(ar, ubxData.min);
        Util::deserialize(ar, ubxData.sec);
        Util::deserialize(ar, ubxData.valid);
        Util::deserialize(ar, ubxData.tAcc);
        Util::deserialize(ar, ubxData.nano);
        Util::deserialize(ar, ubxData.fixType);
        Util::deserialize(ar, ubxData.flags);
        Util::deserialize(ar, ubxData.flags2);
        Util::deserialize(ar, ubxData.numSV);
        Util::deserializeConv<int32_t>(ar, ubxData.lon, 1e-7);
        Util::deserializeConv<int32_t>(ar, ubxData.lat, 1e-7);
        Util::deserializeConv<int32_t>(ar, ubxData.height, 1e-3);
        Util::deserializeConv<int32_t>(ar, ubxData.hMSL, 1e-3);
        Util::deserializeConv<uint32_t>(ar, ubxData.hAcc, 1e-3);
        Util::deserializeConv<uint32_t>(ar, ubxData.vAcc, 1e-3);
        Util::deserializeConv<int32_t>(ar, ubxData.velN, 1e-3);
        Util::deserializeConv<int32_t>(ar, ubxData.velE, 1e-3);
        Util::deserializeConv<int32_t>(ar, ubxData.velD, 1e-3);
        Util::deserializeConv<int32_t>(ar, ubxData.gSpeed, 1e-3);
        Util::deserializeConv<int32_t>(ar, ubxData.headMot, 1e-5);
        Util::deserializeConv<int32_t>(ar, ubxData.sAcc, 1e-3);
        Util::deserializeConv<uint32_t>(ar, ubxData.headAcc, 1e-5);
        Util::deserializeConv<uint16_t>(ar, ubxData.pDOP, 1e-2);
        Util::consumeReserved<6>(ar);
        Util::deserializeConv<int32_t>(ar, ubxData.headVeh, 1e-5);
        Util::consumeReserved<4>(ar);
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataRxmRAWX> {
    static UbxDataRxmRAWX get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataRxmRAWX ubxData;
        Util::deserialize(ar, ubxData.rcvTow);
        Util::deserialize(ar, ubxData.week);
        Util::deserialize(ar, ubxData.leapS);
        Util::deserialize(ar, ubxData.numMeas);
        Util::deserialize(ar, ubxData.recStat);
        Util::deserialize(ar, ubxData.version);
        Util::consumeReserved<2>(ar);
        for (int64_t i = 0; i < ubxData.numMeas; i++) {
            UbxDataRxmRAWX::Mea mea;
            Util::deserialize(ar, mea.prMes);
            Util::deserialize(ar, mea.cpMes);
            float doMes;
            Util::deserialize(ar, doMes);
            mea.doMes = doMes;
            Util::deserialize(ar, mea.gnssId);
            Util::deserialize(ar, mea.svId);
            Util::consumeReserved<1>(ar);
            Util::deserialize(ar, mea.freqId);
            Util::deserialize(ar, mea.locktime);
            Util::deserialize(ar, mea.cno);
            Util::deserializeExponent4bitBase2<uint8_t>(ar, mea.prStdev, 0.01);
            Util::deserializeConvMask<uint8_t>(ar, mea.cpStdev, 0x0f, 0.004);
            Util::deserializeExponent4bitBase2<uint8_t>(ar, mea.doStdev, 0.002);
            Util::deserialize(ar, mea.trkStat);
            Util::consumeReserved<1>(ar);
            ubxData.measurements.push_back(mea);
        }
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataRxmSFRBX> {
    static UbxDataRxmSFRBX get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataRxmSFRBX ubxData;
        Util::deserialize(ar, ubxData.gnssId);
        Util::deserialize(ar, ubxData.svId);
        Util::deserialize(ar, ubxData.reserved1);
        Util::deserialize(ar, ubxData.freqId);
        Util::deserialize(ar, ubxData.numWords);
        Util::deserialize(ar, ubxData.reserved2);
        Util::deserialize(ar, ubxData.version);
        Util::deserialize(ar, ubxData.reserved3);

        ubxData.words.resize(ubxData.numWords);
        for (uint8_t i = 0; i < ubxData.numWords; i++) {
            Util::deserialize(ar, ubxData.words[i]);
        }
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataNavDGPS> {
    static UbxDataNavDGPS get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavDGPS ubxData;

        Util::deserialize(ar, ubxData.iTOW);
        Util::deserialize(ar, ubxData.age);
        Util::deserialize(ar, ubxData.baseId);
        Util::deserialize(ar, ubxData.baseHealth);
        Util::deserialize(ar, ubxData.numCh);
        Util::deserialize(ar, ubxData.status);
        Util::consumeReserved<2>(ar);
        for (int64_t i = 0; i < ubxData.numCh; i++) {
            UbxDataNavDGPS::Channel ch;
            Util::deserialize(ar, ch.svid);
            Util::deserialize(ar, ch.flags);
            Util::deserialize(ar, ch.ageC);
            Util::deserializeConv<uint32_t>(ar, ch.prc);
            Util::deserializeConv<uint32_t>(ar, ch.prrc);
            ubxData.channels.push_back(ch);
        }
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataNavDOP> {
    static UbxDataNavDOP get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavDOP ubxData;

        Util::deserialize(ar, ubxData.iTOW);
        Util::deserializeConv<uint16_t>(ar, ubxData.gDOP, 0.01);
        Util::deserializeConv<uint16_t>(ar, ubxData.pDOP, 0.01);
        Util::deserializeConv<uint16_t>(ar, ubxData.tDOP, 0.01);
        Util::deserializeConv<uint16_t>(ar, ubxData.vDOP, 0.01);
        Util::deserializeConv<uint16_t>(ar, ubxData.hDOP, 0.01);
        Util::deserializeConv<uint16_t>(ar, ubxData.nDOP, 0.01);
        Util::deserializeConv<uint16_t>(ar, ubxData.eDOP, 0.01);
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataNavPOSECEF> {
    static UbxDataNavPOSECEF get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavPOSECEF ubxData;

        Util::deserialize(ar, ubxData.iTOW);
        Util::deserializeConv<int32_t>(ar, ubxData.ecefX, 1e-2);
        Util::deserializeConv<int32_t>(ar, ubxData.ecefY, 1e-2);
        Util::deserializeConv<int32_t>(ar, ubxData.ecefZ, 1e-2);
        Util::deserializeConv<uint32_t>(ar, ubxData.pAcc, 1e-2);
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataNavSAT> {
    static UbxDataNavSAT get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavSAT ubxData;

        Util::deserialize(ar, ubxData.iTOW);
        Util::deserialize(ar, ubxData.version);
        Util::deserialize(ar, ubxData.numSvs);
        Util::consumeReserved<2>(ar);
        for (int64_t i = 0; i < ubxData.numSvs; i++) {
            UbxDataNavSAT::Satellite satellite;
            Util::deserialize(ar, satellite.gnssId);
            Util::deserialize(ar, satellite.svId);
            Util::deserialize(ar, satellite.cno);
            Util::deserialize(ar, satellite.elev);
            Util::deserialize(ar, satellite.azim);
            Util::deserializeConv<int16_t>(ar, satellite.prRes, 0.1);
            Util::deserialize(ar, satellite.flags);
            ubxData.satellites.push_back(satellite);
        }
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataNavSBAS> {
    static UbxDataNavSBAS get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavSBAS ubxData;

        Util::deserialize(ar, ubxData.iTOW);
        Util::deserialize(ar, ubxData.geo);
        Util::deserialize(ar, ubxData.mode);
        Util::deserialize(ar, ubxData.sys);
        Util::deserialize(ar, ubxData.service);
        Util::deserialize(ar, ubxData.cnt);
        Util::consumeReserved<3>(ar);
        for (int64_t i = 0; i < ubxData.cnt; i++) {
            UbxDataNavSBAS::InfoForSingleSat info;
            Util::deserialize(ar, info.svId);
            Util::deserialize(ar, info.flags);
            Util::deserialize(ar, info.udre);
            Util::deserialize(ar, info.svSys);
            Util::deserialize(ar, info.svService);
            Util::consumeReserved<1>(ar);
            Util::deserialize(ar, info.prc);
            Util::consumeReserved<2>(ar);
            Util::deserialize(ar, info.ic);

            ubxData.infos.push_back(info);
        }
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataNavSOL> {
    static UbxDataNavSOL get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavSOL ubxData;

        Util::deserialize(ar, ubxData.iTOW);
        Util::deserialize(ar, ubxData.fTOW);
        Util::deserialize(ar, ubxData.week);
        uint8_t gpsFix;
        Util::deserialize(ar, gpsFix);
        ubxData.gpsFix = Util::getGPSFix(gpsFix);
        Util::consumeReserved<1>(ar);
        Util::deserializeConv<int32_t>(ar, ubxData.ecefX, 1e-2);
        Util::deserializeConv<int32_t>(ar, ubxData.ecefY, 1e-2);
        Util::deserializeConv<int32_t>(ar, ubxData.ecefZ, 1e-2);
        Util::deserializeConv<uint32_t>(ar, ubxData.pAcc, 1e-2);
        Util::deserializeConv<int32_t>(ar, ubxData.ecefVX, 1e-2);
        Util::deserializeConv<int32_t>(ar, ubxData.ecefVY, 1e-2);
        Util::deserializeConv<int32_t>(ar, ubxData.ecefVZ, 1e-2);
        Util::deserializeConv<uint32_t>(ar, ubxData.sAcc, 1e-2);
        Util::deserializeConv<int16_t>(ar, ubxData.pDOP, 0.01);
        Util::consumeReserved<1>(ar);
        Util::deserialize(ar, ubxData.numSV);
        Util::consumeReserved<4>(ar);
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataNavSTATUS> {
    static UbxDataNavSTATUS get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavSTATUS ubxData;

        Util::deserialize(ar, ubxData.iTOW);
        uint8_t gpsFix;
        Util::deserialize(ar, gpsFix);
        ubxData.gpsFix = Util::getGPSFix(gpsFix);
        Util::deserialize(ar, ubxData.flags);
        Util::deserialize(ar, ubxData.fixStat);
        Util::deserialize(ar, ubxData.flags2);
        Util::deserialize(ar, ubxData.ttff);
        Util::deserialize(ar, ubxData.msss);
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataNavTIMEGPS> {
    static UbxDataNavTIMEGPS get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavTIMEGPS ubxData;

        Util::deserialize(ar, ubxData.iTOW);
        Util::deserialize(ar, ubxData.fTOW);
        Util::deserialize(ar, ubxData.week);
        Util::deserialize(ar, ubxData.leapS);
        Util::deserialize(ar, ubxData.valid);
        Util::deserialize(ar, ubxData.tAcc);
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataNavTIMEUTC> {
    static UbxDataNavTIMEUTC get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataNavTIMEUTC ubxData;

        Util::deserialize(ar, ubxData.iTOW);
        Util::deserialize(ar, ubxData.tAcc);
        Util::deserialize(ar, ubxData.nano);
        Util::deserialize(ar, ubxData.year);
        Util::deserialize(ar, ubxData.month);
        Util::deserialize(ar, ubxData.day);
        Util::deserialize(ar, ubxData.hour);
        Util::deserialize(ar, ubxData.min);
        Util::deserialize(ar, ubxData.sec);
        Util::deserialize(ar, ubxData.valid);
        return ubxData;
    }
};

template <>
struct Deserializer<UbxDataMonMSGPP> {
    static UbxDataMonMSGPP get(const DataBuffer& data) {
        UbxMemStream memStream(data);
        cereal::BinaryInputArchive ar(memStream);
        UbxDataMonMSGPP ubxData;
        ar(ubxData.ubx);
        ar(ubxData.nmea);
        ar(ubxData.rtcm2);
        ar(ubxData.rtcm3);
        ar(ubxData.msg5);
        ar(ubxData.msg6);
        ar(ubxData.skipped);
        return ubxData;
    }
};

} // namespace internal
} // namespace sensor_ublox
