#ifndef _WIRE_H_
#define _WIRE_H_

#include <i2c_slave.h>

#ifndef WIRE_BUFFER_LENGTH
#define WIRE_BUFFER_LENGTH 32
#endif

using WireReceiveHandler = void (*)(int count);
using WireRequestHandler = void (*)();

class TwoWire final
{
    public:
        TwoWire();
        i2c_inst_t *i2c() const;
        void begin();
        void begin(uint8_t selfAddress);
        void beginTransmission(uint8_t address);
        uint8_t endTransmission(bool sendStop = true);
        uint8_t requestFrom(uint8_t address, size_t count, bool sendStop);
        size_t available() const;
        int peek() const;
        int read();
        size_t write(uint8_t value);
        size_t write(const uint8_t *data, size_t size);
        void onReceive(WireReceiveHandler handler);
        void onRequest(WireRequestHandler handler);

    private:
        static constexpr uint8_t NO_ADDRESS = 255;
        enum Mode : uint8_t
        {
            Unassigned,
            Master,
            Slave,
        };
        TwoWire(const TwoWire &other) = delete;
        TwoWire &operator=(const TwoWire &other) = delete;
        static void handleEvent(i2c_inst_t *i2c, i2c_slave_event_t event);
        WireReceiveHandler receiveHandler_ = nullptr;
        WireRequestHandler requestHandler_ = nullptr;
        Mode mode_ = Unassigned;
        uint8_t txAddress_ = 255;
        uint8_t buf_[WIRE_BUFFER_LENGTH];
        uint8_t bufLen_ = 0;
        uint8_t bufPos_ = 0;
};

extern TwoWire Wire;
extern TwoWire Wire1;

inline size_t TwoWire::available() const 
{
    assert(mode_ != Unassigned);
    assert(txAddress_ == NO_ADDRESS);
    return bufLen_ - bufPos_;
}

inline int TwoWire::peek() const 
{
    assert(mode_ != Unassigned);
    assert(txAddress_ == NO_ADDRESS);
    return bufPos_ < bufLen_ ? (int)buf_[bufPos_] : -1;
}

inline int TwoWire::read()
{
    assert(mode_ != Unassigned);
    assert(txAddress_ == NO_ADDRESS);
    return bufPos_ < bufLen_ ? (int)buf_[bufPos_++] : -1;
}

#endif