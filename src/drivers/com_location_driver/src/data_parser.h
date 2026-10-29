#pragma once
#include <string>

#ifdef USE_ROS1
#include "com_location_msgs/BESTGNSSPOS.h"
#include "com_location_msgs/HEADINGA.h"
#include "com_location_msgs/INSPVAXA.h"
#include "com_location_msgs/RAWIMUA.h"
#else
#include "com_location_msgs/msg/bestgnsspos.hpp"
#include "com_location_msgs/msg/headinga.hpp"
#include "com_location_msgs/msg/inspvaxa.hpp"
#include "com_location_msgs/msg/rawimua.hpp"
#endif

typedef std::function<void(void)> MessageParseReadyCallback;
 
class DataParser {
	public:

    void Init(MessageParseReadyCallback best_pos_call,
        MessageParseReadyCallback heading_call,
        MessageParseReadyCallback inspvaxa_call,
        MessageParseReadyCallback rawimu_call);

		void Parse(std::string &data, 
				com_location_msgs::BESTGNSSPOS &pos_msg,
				com_location_msgs::HEADINGA &heading_msg,
				com_location_msgs::INSPVAXA &ins_msg,
				com_location_msgs::RAWIMUA &imu_msg);

    void ParseHeader(com_location_msgs::Oem7Header &header,
        std::string &data);

	private:

    MessageParseReadyCallback best_pos_call_;
    MessageParseReadyCallback heading_call_;
    MessageParseReadyCallback inspvaxa_call_;
    MessageParseReadyCallback rawimu_call_;
};
