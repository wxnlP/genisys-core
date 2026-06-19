#include "dm_motor_driver.h"
#include "canfd_bus.h"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    SocketCANBus socketcan0("vcan0", true);
    DmMotorDriver dm4310(socketcan0, 0x01);
    std::cout << "SocketCAN Init" << std::endl;
    int sock_fd = socketcan0.Init();
    uint32_t ids = 0x11;
    socketcan0.SetReceiverId(&ids, 1);
    // std::cout << "socket fd: " << sock_fd << std::endl;
    // uint8_t frame[4] = { 0x01, 0x02, 0x03, 0x04 };
    // int ret = socketcan0.Send(0x01, frame, 4);
    // std::cout << "ret: " << ret << std::endl;
    std::cout << "Motor Enable" << std::endl;
    dm4310.Enable(true);

    std::jthread receive_thread([&dm4310] () {
        DmMotorDriver::MotorState state{};
        while (true) {
            int ret = dm4310.GetLatestState(state);
            if (ret != -1) {
                std::cout << "Motor State: " << std::endl;
                std::cout << " id: " << state.id << " pos: " << state.pos
                          << " vel: " << state.vel << " error: " << state.state << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    DmMotorDriver::CtrlCmd cmd{2.0, 0.0, 0, 0.0, 0.5};
    std::cout << "MIT velocity control" << std::endl;
    dm4310.MoveCtrl(cmd);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "Motor Disable" << std::endl;
    dm4310.Enable(false);

    return 0;
}