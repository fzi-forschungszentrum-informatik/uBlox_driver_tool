#include <cstddef>
#include <iostream>
#include <signal.h>
#include <string>
#include <generic_logger/generic_logger.hpp>
#include <sensor_ublox/data_types.hpp>
#include <sensor_ublox/ublox.hpp>
#include <spdlog/sinks/stdout_sinks.h>

std::unique_ptr<sensor_ublox::GNSSReceiver> gnss;

void callbackNavPVT(const sensor_ublox::UbxDataNavPVT& data) {
    std::cout << "UBX-NAV-PVT: " << std::setprecision(10) << data.lat << " / " << data.lon << " / " << data.height
              << " - " << data.hAcc << std::endl;
}

void callbackRxmRaw(const sensor_ublox::UbxDataRxmRAWX& data) {
    std::cout << "UBX-Rmx-Raw: " << (int)data.numMeas << " / " << data.version << std::endl;
    for (int64_t i = 0; i < data.numMeas; i++) {
        const auto& mea = data.measurements[i];
        std::cout << mea.prMes << " - " << mea.prStdev << std::endl;
    }
}

void callbackMonMsgPP(const sensor_ublox::UbxDataMonMSGPP& data) {
    std::cout << "RTCM3: " << (int)data.rtcm3[5] << std::endl;
    std::cout << "Skipped[USB]: " << (int)data.skipped[sensor_ublox::UbxDataMonMSGPP::PORT_USB] << std::endl;
}

void callbackNavSOL(const sensor_ublox::UbxDataNavSOL& data) {
    std::cout << "SOL: " << std::setprecision(12) << data.ecefX << " - " << data.ecefY << " - " << data.ecefZ
              << std::endl;
}

void callbackNavStatus(const sensor_ublox::UbxDataNavSTATUS& data) {
    std::cout << "STATUS: " << (int)data.gpsFix << std::endl;
}

void callbackNavTimeUTC(const sensor_ublox::UbxDataNavTIMEUTC& data) {
    std::cout << "TIMEUTC: " << data.iTOW << std::endl;
}

void handleSigINT(int sig) {
    if (gnss) {
        gnss->stop();
    }
}

int main(int argc, char* argv[]) {
    if (argc <= 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_device> <config> <base_config>" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    signal(SIGINT, handleSigINT);
    generic_logger::set_sink(spdlog::sink_ptr(new spdlog::sinks::stdout_sink_st()));
    generic_logger::set_level("info");
    try {
        sensor_ublox::GNSSReceiverOptions opts;
        opts.deviceName = argv[1];
        opts.configFile = argv[2];
        opts.baseConfigFile = argv[3];
        gnss = std::make_unique<sensor_ublox::GNSSReceiver>(opts);
        gnss->registerCallback(&callbackNavPVT);
        gnss->registerCallback(&callbackMonMsgPP);
        gnss->registerCallback(&callbackNavSOL);
        gnss->registerCallback(&callbackNavStatus);
        gnss->registerCallback(&callbackNavTimeUTC);
        gnss->run();
        gnss->waitForStop();
    } catch (sensor_ublox::GNSSException& e) {
        std::cerr << "Failure: " << e.what() << std::endl;
    }
}
