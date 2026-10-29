#include "radar_can.h"

void RadarCan_Conti::ParseHead(const std::vector<can_frame> &buf) {
  target_num = (__u8)(buf[0].data[0]) + (__u8)(buf[0].data[1]);
  meas_counter = (__u16)(buf[0].data[2] << 8) + (__u8)(buf[0].data[3]);
}

void RadarCan_Conti::ParseBody(const std::vector<can_frame> &buf) {
  target_id = (__u8)(buf[0].data[0]);
  target_dist_long = ((__u16)(buf[0].data[1] << 5)
            + (__u8)((buf[0].data[2] & 0xf8) >> 3)) * 0.2 - 500;
  target_dist_lat = ((__u16)((buf[0].data[2] & 0x03) << 8)
            + (__u8)(buf[0].data[3])) * 0.2 - 102.3;
  target_vre_long = ((__u16)(buf[0].data[4] << 2)
            + (__u8)((buf[0].data[5] & 0xc0) >> 6)) * 0.25 - 128;
  target_vre_lat = ((__u16)((buf[0].data[5] & 0x3f) << 3)
            + (__u8)((buf[0].data[6] & 0xe0) >> 5)) * 0.25 - 64;
  target_dyn_prop = (__u8)(buf[0].data[6] & 0x07);
  target_rcs = (__u8)(buf[0].data[7]) * 0.5 - 64.0;
}