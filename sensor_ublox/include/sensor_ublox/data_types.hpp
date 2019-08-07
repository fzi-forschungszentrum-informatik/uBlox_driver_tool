#pragma once

#include <functional>
#include <vector>

#include "ubx_data.h"

namespace sensor_ublox {
template <typename T>
using CallbackFuncType = std::function<void(const T&)>;

struct UbxDataNavPVT {
    uint32_t iTOW;   //!< GPS time of week of the <r href=\"NAV-EPOCH-DESC'>navigation epoch</r>.
    uint16_t year;   //!< Year (UTC)
    uint8_t month;   //!< Month, range 1..12 (UTC)
    uint8_t day;     //!< Day of month, range 1..31 (UTC)
    uint8_t hour;    //!< Hour of day, range 0..23 (UTC)
    uint8_t min;     //!< Minute of hour, range 0..59 (UTC)
    uint8_t sec;     //!< Seconds of minute, range 0..60 (UTC)
    uint8_t valid;   //!< Validity Flags
    uint32_t tAcc;   //!< Time accuracy estimate (UTC)
    uint32_t nano;   //!< Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t fixType; //!< GNSSfix Type, range 0..5
    uint8_t flags;   //!< Fix Status Flags
    uint8_t flags2;
    uint8_t numSV;  //!< Number of satellites used in Nav Solution
    double lon;     //!< Longitude
    double lat;     //!< Latitude
    double height;  //!< Height above ellipsoid
    double hMSL;    //!< Height above mean sea level
    double hAcc;    //!< Horizontal accuracy estimate
    double vAcc;    //!< Vertical accuracy estimate
    double velN;    //!< NED north velocity
    double velE;    //!< NED east velocity
    double velD;    //!< NED down velocity
    double gSpeed;  //!< Ground Speed (2-D)
    double headMot; //!< Heading of motion (2-D)
    double sAcc;    //!< Speed accuracy estimate
    double headAcc; //!< Heading accuracy estimate (both motion and vehicle)
    double pDOP;    //!< Position DOP
    double headVeh; //!< Heading of vehicle (2-D)
};

struct UbxDataRxmRAWX {
    struct Mea {
        double prMes;      //!< Pseudorange measurement [m].
        double cpMes;      //!< Carrier phase measurement [cycles].
        double doMes;      //!< Doppler measurement (positive sign for approaching satellites) [Hz]
        uint8_t gnssId;    //!< GNSS identifier
        uint8_t svId;      //!< Satellite identifier
        uint8_t freqId;    //!< Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
        uint16_t locktime; //!< Carrier phase locktime counter (maximum 64500ms)
        uint8_t cno;       //!< Carrier-to-noise density ratio (signal strength) [dB-Hz]
        double prStdev;    //!< Estimated pseudorange measurement standard deviation
        double cpStdev;    //!< Estimated carrier phase measurement standard deviation
        double doStdev;    //!< Estimated Doppler measurement standard deviation.
        uint8_t trkStat;   //!< Tracking status bitfield
    };

    double rcvTow;   //!< Measurement time of week in
    uint16_t week;   //!< GPS week number in
    int8_t leapS;    //!< GPS leap seconds (GPS-UTC)
    uint8_t numMeas; //!< Number of measurements to follow
    uint8_t recStat; //!< Receiver tracking status bitfield
    uint8_t version; //!< Message version
    std::vector<Mea> measurements;
};

struct UbxDataRxmSFRBX {
    uint8_t gnssId;    //!< GNSS identifier (see <r href=\"DESC-SVNumbering-UBX'>Satellite Numbering</r>)
    uint8_t svId;      //!< Satellite identifier (see <r href=\"DESC-SVNumbering-UBX'>Satellite Numbering</r>)
    uint8_t reserved1; //!< <r href='DESC-UBX-Reserved'>Reserved</r>
    uint8_t freqId;    //!< Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
    uint8_t numWords;  //!< The number of data words contained in this message (0..16)
    uint8_t reserved2; //!< <r href='DESC-UBX-Reserved'>Reserved</r>
    uint8_t version;   //!< Message version, (=1 for this version)
    uint8_t reserved3; //!< <r href='DESC-UBX-Reserved'>Reserved</r>
                       // REPEAT: UBX_RXM_SFRBX_DATA1_DWRD_t repeat0[numWords];

    std::vector<uint32_t> words; //!< The data words
};

struct UbxDataNavDGPS {
    struct Channel {
        uint8_t svid;  //!< Satellite ID
        uint8_t flags; //!< Channel number and usage
        uint16_t ageC; //!< Age of latest correction data
        double prc;    //!< Pseudorange correction
        double prrc;   //!< Pseudorange rate correction
    };

    uint32_t iTOW;      //!< GPS time of week of the <r href=\"NAV-EPOCH-DESC'>navigation epoch</r>.
    int32_t age;        //!< Age of newest correction data
    int16_t baseId;     //!< DGPS base station identifier
    int16_t baseHealth; //!< DGPS base station health status
    uint8_t numCh;      //!< Number of channels for which correction data is following
    uint8_t status;     //!< DGPS correction type status:
    std::vector<Channel> channels;
};

struct UbxDataNavDOP {
    uint32_t iTOW; //!< GPS time of week
    double gDOP;   //!< Geometric DOP
    double pDOP;   //!< Position DOP
    double tDOP;   //!< Time DOP
    double vDOP;   //!< Vertical DOP
    double hDOP;   //!< Horizontal DOP
    double nDOP;   //!< Northing DOP
    double eDOP;   //!< Easting DOP
};

struct UbxDataNavPOSECEF {
    uint32_t iTOW; //!< GPS time of week of the <r href=\"NAV-EPOCH-DESC'>navigation epoch</r>.
    double ecefX;  //!< ECEF X coordinate
    double ecefY;  //!< ECEF Y coordinate
    double ecefZ;  //!< ECEF Z coordinate
    double pAcc;   //!< Position Accuracy Estimate
};

struct UbxDataNavSAT {
    struct Satellite {
        uint8_t gnssId; //!< GNSS identifier
        uint8_t svId;   //!< Satellite identifier
        uint8_t cno;    //!< Carrier to noise ratio (signal strength)
        int8_t elev;    //!< Elevation (range: +/-90), unknown if out of range
        int16_t azim;   //!< Azimuth (range +/-180), unknown if elevation is out of range
        double prRes;   //!< Pseudo range residual
        uint32_t flags; //!< Bitmask
    };

    uint32_t iTOW;
    uint8_t version; //!< Message version (1 for this version)
    uint8_t numSvs;  //!< Number of satellites
    std::vector<Satellite> satellites;
};


struct UbxDataNavSBAS {
    struct InfoForSingleSat {
        U1 svId;
        U1 flags;
        U1 udre;
        U1 svSys;
        U1 svService;
        I2 prc;
        I2 ic;
    };

    U4 iTOW;
    U1 geo;
    U1 mode;
    I1 sys;
    X1 service;
    U1 cnt;
    std::vector<InfoForSingleSat> infos;
};

enum class UbxDataGPSFix : uint8_t {
    FixNone = 0x00,
    FixDeadReckoningOnly = 0x01,
    Fix2D = 0x02,
    Fix3D = 0x03,
    FixGPSDeadRecknoning = 0x04,
    FixTimeOnly = 0x05,
};

struct UbxDataNavSOL {
    uint32_t iTOW;        //!< GPS time of week of the
    uint32_t fTOW;        //!< Fractional part of iTOW
    int16_t week;         //!< GPS week number
    UbxDataGPSFix gpsFix; //!< GPSfix Type, range 0..5
    uint8_t flags;        //!< Fix Status Flags
    double ecefX;         //!< ECEF X coordinate
    double ecefY;         //!< ECEF Y coordinate
    double ecefZ;         //!< ECEF Z coordinate
    double pAcc;          //!< 3D Position Accuracy Estimate
    double ecefVX;        //!< ECEF X velocity
    double ecefVY;        //!< ECEF Y velocity
    double ecefVZ;        //!< ECEF Z velocity
    double sAcc;          //!< Speed Accuracy Estimate
    double pDOP;          //!< Position DOP
    uint8_t numSV;        //!< Number of SVs used in Nav Solution
};

struct UbxDataNavSTATUS {
    uint32_t iTOW;        //!< GPS time of week
    UbxDataGPSFix gpsFix; //!< GPSfix Type
    uint8_t flags;        //!< Navigation Status Flags
    uint8_t fixStat;      //!< Fix Status Information
    uint8_t flags2;       //!< further information about navigation output
    uint32_t ttff;        //!< Time to first fix (millisecond time tag)
    uint32_t msss;        //!< Milliseconds since Startup / Reset
};

struct UbxDataNavTIMEGPS {
    uint32_t iTOW; //!< GPS time of week
    int32_t fTOW;  //!< Fractional part of iTOW
    int16_t week;  //!< GPS week number
    int8_t leapS;  //!< GPS leap seconds (GPS-UTC)
    uint8_t valid; //!< Validity Flags
    uint32_t tAcc; //!< Time Accuracy Estimate
};

struct UbxDataNavTIMEGLONASS {
    uint32_t iTOW; //!< GPS time of week
    uint32_t TOD;
    int32_t fTOD; //!< Fractional part of iTOW

    uint32_t tAcc; //!< Time Accuracy Estimate
};

struct UbxDataNavTIMEUTC {
    uint32_t iTOW; //!< GPS time of week
    uint32_t tAcc; //!< Time accuracy estimate (UTC)
    int32_t nano;  //!< Fraction of second, range -1e9 .. 1e9 (UTC)
    uint16_t year; //!< Year, range 1999..2099 (UTC)
    uint8_t month; //!< Month, range 1..12 (UTC)
    uint8_t day;   //!< Day of month, range 1..31 (UTC)
    uint8_t hour;  //!< Hour of day, range 0..23 (UTC)
    uint8_t min;   //!< Minute of hour, range 0..59 (UTC)
    uint8_t sec;   //!< Seconds of minute, range 0..60 (UTC)
    uint8_t valid; //!< Validity Flags
};

struct UbxDataMonMSGPP {
    static const uint64_t PORT_USB = 3;

    std::array<uint16_t, 8> ubx;
    std::array<uint16_t, 8> nmea;
    std::array<uint16_t, 8> rtcm2;
    std::array<uint16_t, 8> rtcm3;
    std::array<uint16_t, 8> msg5;
    std::array<uint16_t, 8> msg6;
    std::array<uint32_t, 6> skipped;
};

struct UbxMsgID {
    struct ClassID {
        static const uint8_t Nav = 0x01;
        static const uint8_t Rxm = 0x02;
        static const uint8_t Cfg = 0x06;
        static const uint8_t Mon = 0x0A;
    };

    struct NavMsgID {
        static const uint8_t PosECEF = 0x01;
        static const uint8_t Status = 0x03;
        static const uint8_t DOP = 0x04;
        static const uint8_t Sol = 0x06;
        static const uint8_t PVT = 0x07;
        static const uint8_t TimeGPS = 0x20;
        static const uint8_t TimeUTC = 0x21;
        static const uint8_t DGPS = 0x31;
        static const uint8_t SBAS = 0x32;
        static const uint8_t Sat = 0x35;
    };

    struct RxmMsgID {
        static const uint8_t Rawx = 0x15;
        static const uint8_t Sfrbx = 0x13;
    };

    struct MonMsgID {
        static const uint8_t MsgPP = 0x06;
    };

    struct CfgMsgID {
        static const uint8_t Prt = 0x00;
        static const uint8_t Msg = 0x01;
        static const uint8_t Rate = 0x08;
    };
};

namespace internal {
using DataBuffer = std::vector<uint8_t>;

} // namespace internal

} // namespace sensor_ublox

#include "internal/data_types_impl.hpp"
