#pragma once
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>

namespace CanDrivers {

const int32_t CAN_FRAME_SIZE = 8;
const int32_t MAX_CAN_RECV_FRAME_LEN = 10;
const int32_t CANBUS_MESSAGE_LENGTH = 8;  // according to ISO-11891-1

class Canbus {
 public:
  Canbus();
  ~Canbus();
  bool Init(std::string can_name);
  void Stop(void);
  int32_t Receive(std::vector<can_frame> *const frames,
                  int32_t *const frame_num);
  bool flag_init_cout_;
  bool flag_receive_cout_;

 private:
  can_frame recv_frames_[MAX_CAN_RECV_FRAME_LEN];
  bool is_started_;
  int32_t dev_handler_;
  std::string port_;
};
}  // namespace CanDrivers
