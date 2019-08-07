#pragma once

#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>

#include <generic_logger/generic_logger.hpp>

#include "data_object.hpp"
#include "supported_types.hpp"

namespace sensor_ublox {
namespace internal {

class CallbackFactoryEntry {
public:
    virtual void handleData(const UbxDataObject& object) = 0;
    virtual bool wait(const std::chrono::microseconds& timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condVar_.wait_for(lock, timeout) == std::cv_status::no_timeout;
    }

    virtual ~CallbackFactoryEntry() {
    }

protected:
    std::mutex mutex_;
    std::condition_variable condVar_;
};

using CallbackFactoryEntryPtr = std::unique_ptr<CallbackFactoryEntry>;

template <typename DataType_>
class CallbackFactoryEntryImpl : public CallbackFactoryEntry {
public:
    CallbackFactoryEntryImpl(CallbackFuncType<DataType_>& callback) : callback_(callback) {
    }

    void handleData(const UbxDataObject& data) override {
        if (callback_) {
            const auto& d = data.getData<DataType_>();
            callback_(d);
        }
        condVar_.notify_all();
    }

    const DataType_& getData() {
        return data_;
    }

private:
    CallbackFuncType<DataType_> callback_;
    DataType_ data_;
    std::mutex mutex_;
    std::condition_variable condVar_;
};

using ObjectFactory = std::multimap<std::pair<int64_t, int64_t>, CallbackFactoryEntryPtr>;

} // namespace internal
} // namespace sensor_ublox
