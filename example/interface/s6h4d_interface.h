#ifndef S6H4D_INTERFACE_H
#define S6H4D_INTERFACE_H

#include "serial/serial.h"
#include "interface_base.h"



class S6h4dInterface : public InterfaceBase
{
public:
    explicit S6h4dInterface(std::string port, uint32_t baud = 115200) : port_(port), baud_(baud) {}

    virtual void Init() override {
        ser_.setPort(port_);
        ser_.setBaudrate(baud_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(timeout);
        ser_.open();
    }
    virtual void DeInit() override {
        if (ser_.isOpen()) {
            ser_.close();
        }
    }
    
    virtual size_t WriteBytes(uint8_t *bytes, size_t size) override {
        return ser_.write(bytes, size);
    }

    virtual size_t ReadBytes(uint8_t *bytes, size_t size) override {
        return ser_.read(bytes, size);
    }

private:
    serial::Serial ser_;
    std::string port_;
    uint32_t baud_;
};

#endif 