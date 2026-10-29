#pragma once
#include <iostream>
#include "../../../common/net/can/socket_can.h"

class RadarCan_Conti {
  public:
    // from 0x600 -- cluster list status
    int target_num;
    int meas_counter;
    // from 0x701 -- cluster general information
    int target_id;
    float target_dist_long;
    float target_dist_lat;
    float target_vre_long;
    float target_vre_lat;
    int target_dyn_prop;
    float target_rcs;
    
    // parse 0x600 -- cluster list status
    void ParseHead(const std::vector<can_frame> &buf);
    // parse 0x701 -- cluster general information
    void ParseBody(const std::vector<can_frame> &buf);
};
