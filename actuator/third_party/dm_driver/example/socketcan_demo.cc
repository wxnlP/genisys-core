#include "canfd_bus.h"
#include <iostream>
#include <thread>

int main() {
  SocketCANBus can_bus("vcan0", true);
  if (can_bus.Init() < 0) {
    std::cerr << "Failed to initialize CAN bus" << std::endl;
    return -1;
  }

  uint32_t recv_data[8] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};
  can_bus.SetReceiverId(recv_data, 7);

  std::jthread send_thread([&can_bus]() {
    std::cout << "Sending data to CAN bus..." << std::endl;
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint32_t id = 0x11;
    while (true) {
      can_bus.Send(id, data, 8);
      data[id - 0x10 - 1]++; // 修改数据以便观察变化
      id++;
      if (id > 0x18) {
        id = 0x11;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // std::jthread t1([&can_bus]() {
  //   std::cout << "Sampling CAN bus..." << std::endl;
  //   while (true) {
  //     can_bus.Sample();
  //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //   }
  // });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::jthread t2([&can_bus]() {
    std::cout << "Checking for received data..." << std::endl;
    uint32_t id = 0x01;
    while (true) {
      uint8_t recv_buf[8]{};
      if (can_bus.Receive(id, recv_buf) == 0) {
        std::cout << "Received data: ";
        for (int i = 0; i < 8; ++i) {
          std::cout << std::hex << static_cast<int>(recv_buf[i]) << " ";
        }
        std::cout << std::dec << std::endl;
      }
      id++;
      if (id > 0x08) {
        id = 0x01;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  return 0;
}