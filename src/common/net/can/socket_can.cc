#include "socket_can.h"
#include <string.h>
#include <iomanip>
#include "../../../common/log.h"

namespace CanDrivers {

Canbus::Canbus() {
  is_started_ = false;
  dev_handler_ = 0;
  port_ = "";
  flag_receive_cout_ = true;
  flag_init_cout_ = true;
}

Canbus::~Canbus() {
  if (dev_handler_) {
    Stop();
  }
}

bool Canbus::Init(std::string can_name) {
  if (is_started_) return true;
  struct sockaddr_can addr;
  struct ifreq ifr;
  dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if ((dev_handler_ < 0) && flag_init_cout_) {
    AERROR << " open device error code [" << dev_handler_ << "]";
    return false;
  }

  port_ = can_name;
  if (flag_init_cout_) {
    AINFO << "device name : " << can_name.c_str();
  }
  strcpy(ifr.ifr_name, can_name.c_str());
  if ((ioctl(dev_handler_, SIOCGIFINDEX, &ifr) < 0) && flag_init_cout_) {
    AERROR << " ioctl error ";
    return false;
  }

  // init config and state
  // 1. set receive message_id filter, ie white list
  struct can_filter filter[2048];
  for (int i = 0; i < 2048; ++i) {
    filter[i].can_id = 0x000 + i;
    filter[i].can_mask = CAN_SFF_MASK;
  }

  // if you don't want to receive any CAN dat.
  // int ret = setsockopt(dev_handler_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
  int ret = setsockopt(dev_handler_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                       sizeof(filter));

  // 2. enable reception of can frames.
  int enable = 1;
  ret = setsockopt(dev_handler_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable,
                   sizeof(enable));
  if ((ret < 0) && flag_init_cout_) {
    AERROR << "enable reception of can frame error code: " << ret;
    return false;  // CAN_CLIENT_ERROR_BASE
  }
  // 3. bind socket to network interface
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),
             sizeof(addr));
  if ((ret < 0) && flag_init_cout_) {
    AERROR << " bind socket to network interface error code: " << ret;
    return false;
  }

  is_started_ = true;
  return true;
}

void Canbus::Stop() {
  if (is_started_) {
    is_started_ = false;
    int ret = close(dev_handler_);
    if (ret < 0) {
      AERROR << " close error code : " << ret;
    } else {
      AINFO << " close socket can ok. port : " << port_;
    }
  }
}

int32_t Canbus::Receive(std::vector<can_frame> *const frames,
                        int32_t *const frame_num) {
  if ((!is_started_) && flag_receive_cout_) {
    AERROR << " canbus is not been initialized! please init first!" << __LINE__;
    return 2104;  // CAN_CLIENT_ERROR_RECV_FAILED
  }

  if ((static_cast<size_t>(*frame_num) > MAX_CAN_RECV_FRAME_LEN ||
      static_cast<size_t>(*frame_num) < 0) && flag_receive_cout_) {
    AERROR << " recv can frame not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num : " << static_cast<size_t>(*frame_num);
    return 2102;  // CAN_CLIENT_ERROR_FRAME_NUM
  }

  for (int32_t i = 0;
       i < static_cast<size_t>(*frame_num) && i < MAX_CAN_RECV_FRAME_LEN; ++i) {
    can_frame cf;
    auto ret = read(dev_handler_, &recv_frames_[i], sizeof(recv_frames_[i]));
    if ((ret < 0) && flag_receive_cout_) {
      AERROR << " receive message failed, error code : " << ret;
      return 2100;  // CAN_CLIENT_ERROR_BASE
    }
    cf.can_id = recv_frames_[i].can_id & 0x1fffffff;
    cf.can_dlc = static_cast<size_t>(recv_frames_[i].can_dlc);
    memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].can_dlc);
    frames->push_back(cf);
  }
  return 0;  // OK
}

}  // namespace CanDrivers
