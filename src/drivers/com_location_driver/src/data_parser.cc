#include "data_parser.h"
#include <vector>
#include <iostream>
#ifdef GNSS_TEST
#include "common/common.h"
#define AERROR std::cout
#define AINFO std::cout
#else
#include "../../common/common.h"
#include "../../common/log.h"
#endif
#include "gnss_data.h"

// 解算状态
typedef struct {
	SolutionStatus status;
	char *name;
} SolutionStutusTag;
SolutionStutusTag SolutionStutusMap[] = {
	{ SOL_COMPUTED, "SOL_COMPUTED" },
  { INSUFFICIENT_OBS, "INSUFFICIENT_OBS" },
  { NO_CONVERGENCE, "NO_CONVERGENCE" },
  { SINGULARITY, "SINGULARITY" },
  { COV_TRACE, "COV_TRACE" },
  { TEST_DIST, "TEST_DIST" },
  { COLD_START, "COLD_START" },
  { V_H_LIMIT, "V_H_LIMIT" },
  { VARIANCE, "VARIANCE" },
  { RESIDUALS, "RESIDUALS" },
  { INTEGRITY_WARNING, "INTEGRITY_WARNING" },
  { PENDING, "PENDING" },
  { INVALID_FIX, "INVALID_FIX" },
  { UNAUTHORIZED, "UNAUTHORIZED" },
  { INVALID_RATE, "INVALID_RATE" }
};

typedef struct {
  InertialSolutionStatus status;
  char *name;
} InertialSolutionStatusTag;
InertialSolutionStatusTag InertialSolutionStatusMap[] = {
  { INS_INACTIVE, "INS_INACTIVE" },
  { INS_ALIGNING, "INS_ALIGNING" },
  { INS_HIGH_VARIANCE, "INS_HIGH_VARIANCE" },
  { INS_SOLUTION_GOOD, "INS_SOLUTION_GOOD" },
  { INS_SOLUTION_FREE, "INS_SOLUTION_FREE" },
  { INS_ALIGNMENT_COMPLETE, "INS_ALIGNMENT_COMPLETE" },
  { DETERMINING_ORIENTATION, "DETERMINING_ORIENTATION" },
  { WAITING_INITIAL_POS, "WAITING_INITIAL_POS" },
  { WAITING_AZIMUTH, "WAITING_AZIMUTH" },
  { INITIALIZING_BIASES, "INITIALIZING_BIASES" },
  { MOTION_DETECT, "MOTION_DETECT" }
};

typedef struct {
  PositionOrVelocityType type;
  char *name;
} PositionOrVelocityTypeTag;
PositionOrVelocityTypeTag PositionOrVelocityTypeMap [] = {
  //未解算
  { NONE, "NONE" },
  //位置已由FIXPOSITION命令固定
  { FIXEDPOS, "FIXEDPOS" },
  //位置已由FIXHEIGHT或FIXAUTO命令固定
  { FIXEDHEIGHT, "FIXEDHEIGHT" },
  //使用瞬间多普勒计算速度
  { DOPPLER_VELOCITY, "DOPPLER_VELOCITY" },
  //单点解
  { SINGLE, "SINGLE" },
  //伪距差分
  { PSRDIFF, "PSRDIFF" },
  //SBAS解
  { WAAS, "WAAS" },
  //由卡尔曼滤波器在没有新观测的情况下推算解
  { PROPAGATED, "PROPAGATED" },
  //L1浮点解
  { L1_FLOAT, "L1_FLOAT" },
  //窄带浮点解
  { NARROW_FLOAT, "NARROW_FLOAT" },
  //L1固定解
  { L1_INT, "L1_INT" },
  //宽带固定解
  { WIDE_INT, "WIDE_INT" },
  //窄带固定解
  { NARROW_INT, "NARROW_INT" },
  //RTK状态，其中RTK直接通过INS初始化
  { RTK_DIRECT_INS, "RTK_DIRECT_INS" },
  //无线矫正后INS位置
  { INS_SBAS, "INS_SBAS" },
  //
  { INS_PSRSP, "INS_PSRSP" },
  { INS_PSRDIFF, "INS_PSRDIFF" },
  { INS_RTKFLOAT, "INS_RTKFLOAT" },
  { INS_RTKFIXED, "INS_RTKFIXED" },
  { PPP_CONVERGING, "PPP_CONVERGING" },
  { PPP, "PPP" },
  { OPERATIONAL, "OPERATIONAL" },
  { WARNING, "WARNING" },
  { OUT_OF_BOUNDS, "OUT_OF_BOUNDS" },
  { INS_PPP_CONVERGING, "INS_PPP_CONVERGING" },
  { INS_PPP, "INS_PPP" },
  { PPP_BASIC_CONVERGING, "PPP_BASIC_CONVERGING" },
  { PPP_BASIC, "PPP_BASIC" },
  { INS_PPP_BASIC_CONVERGING, "INS_PPP_BASIC_CONVERGING" },
  { INS_PPP_BASIC, "INS_PPP_BASIC" }
};

static std::vector<std::string> string_split(std::string &str,
    std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern;
  int size = str.size();

  for (int i=0; i<size; i++)
  {
    pos = str.find(pattern, i);
    if ((int)pos < size)
    {
      std::string s = str.substr(i, pos-i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}
    
void DataParser::Init(MessageParseReadyCallback best_pos_call,
    MessageParseReadyCallback heading_call,
    MessageParseReadyCallback inspvaxa_call,
    MessageParseReadyCallback rawimu_call)
{
    best_pos_call_ = best_pos_call;
    heading_call_ = heading_call;
    inspvaxa_call_ = inspvaxa_call;
    rawimu_call_ = rawimu_call;
}

static void ParseGnssTime(std::string date_str, std::string time_str,
    double &gnss_time)
{
  // data time
  try {
    int day = std::stoi(date_str.substr(0, 2));
    int month = std::stoi(date_str.substr(2, 2));
    int year = 2000 + std::stoi(date_str.substr(4, 2));

    // UTC -> localtime
    int hour = std::stoi(date_str.substr(0, 2)) + 8;
    if (hour >= 24) {
      if (day < 31) day += 1;
      hour -= 24;
    }
    int minute = std::stoi(time_str.substr(2, 2));
    int second = std::stoi(time_str.substr(4, 2));
    int msec = std::stoi(time_str.substr(7, 2));
    std::string sd = time_str.substr(4, 2);
    std::string mss = time_str.substr(7, 2);

    struct tm ts;
    ts.tm_year = year - 1900;
    ts.tm_mon = month - 1;
    ts.tm_mday = day;
    ts.tm_hour = hour;
    ts.tm_min = minute;
    ts.tm_sec = second;
    ts.tm_isdst = -1;
    time_t gnss_sec = mktime(&ts);
    gnss_time = gnss_sec * 1.0 + (msec * 0.01);
  } catch(const std::exception &e) {
    AERROR << __func__ << " exception: " << e.what();
  }
}

void DataParser::Parse(std::string &data,
    com_location_msgs::BESTGNSSPOS &pos_msg,
    com_location_msgs::HEADINGA &heading_msg,
    com_location_msgs::INSPVAXA &ins_msg,
    com_location_msgs::RAWIMUA &imu_msg)
{
  if (data.empty()) return;
  // "$GPGGA,025724.20,2811.3394819,N,11250.8155580,E,5,09,1.8,55.873,M,-17.063,M,1.200,1714*54
  // $GPRMC,025724.20,A,2811.3394819,N,11250.8155580,E,0.016,315.4,130124,0.0,E,F*33
  // #BESTGNSSPOSA,ICOM4,0,0.0,FINESTEERING,2109,367696.000,00000000,0000,82;SOL_COMPUTED,NARROW_INT,28.23315515415,112.87713068512,82.5990,-17.0381,WGS84,0.0106,0.011 0,0.0250,"0",1.000,0.058,33,33,33,25,00,00,30,33*9ea908f7
  // #HEADINGA,COM3,0,0,FINESTEERING,1975,394129.000,00000000,0000,113;SOL_COMPUTED,NARROW_INT,1.328605294,296.248487535,­71.075350314,0,0.200,0.500,"0000",29,24,29,7,00,00, 10,01*63131FA1
  // #INSPVAXA,ICOM4,0,0.0,FINESTEERING,2107,35489.000,00000000,03de,68;INS_ALIGNMENT_COMPLETE,INS_RTKFIXED,28.23316396165,112.87713086609,82.7966,-17.0382,0.0020,-0.0191, 0.0006,179.789714292,-0.387541550,1.405962922,0.0240,0.0168,0.0218,0.0047,0.0049,0.005 4,0.0553,0.0553,1.0818,00000000,0*fd6e3a89

  std::vector<std::string> cmds;
  std::vector<std::string> values;
  cmds = string_split(data, "\r\n");
  //AINFO << __func__ << " debug:\n" << data << "\nsize:" << cmds.size() << std::endl;

  try {
    for (size_t i = 0; i < cmds.size(); i++) {
      std::string &line = cmds[i];

      values.clear();
      std::size_t pos = line.find(";");
      if (pos != std::string::npos && pos < line.size()) {
        values = string_split(line, ",");

        // Parse header
        std::string header_str = line.substr(0, pos);
        if ("#BESTGNSSPOSA" == values[0])
          ParseHeader(pos_msg.nov_header, header_str);
        else if ("#HEADINGA" == values[0])
          ParseHeader(heading_msg.nov_header, header_str);
        else if ("#INSPVAXA" == values[0])
          ParseHeader(ins_msg.nov_header, header_str);
        else if ("%RAWIMUSA" == values[0])
          ParseHeader(imu_msg.nov_header, header_str);

        std::string sign = values[0];
        std::string real_line = line.substr(pos + 1);
        values.clear();
        values = string_split(real_line, ",");
        std::vector<std::string>::iterator it = values.begin();
        values.insert(it, sign);
      }
      else 
        values = string_split(line, ",");
      //for (int j = 0; j < values.size(); j++)
      //  std::cout << "i " << i << " j " << j << " v " << values[j] << std::endl;
#if 0 
      // GPGGA
      if (values.size() > 9 && "$GPGGA" == values[0]) {
        gnss_msg.state = std::stoi(values[6]);
        gnss_msg.satellite_num = std::stoi(values[7]);
        gnss_msg.height = std::stod(values[9]);
        if (0 == gnss_msg.state) {
          AERROR << __func__ << " gnss invalid state " << gnss_msg.state;
        }
      }

      // GPRMC
      if (values.size() > 9 && "$GPRMC" == values[0]) {
        gnss_msg.is_valid = "A" == values[2] ? true : false;
        gnss_msg.speed = std::stod(values[7]) * 0.5144444; // jie -> m/s
        gnss_msg.direction = std::stod(values[8]);
        ParseGnssTime(values[9], values[1], gnss_msg.utc_time);
      }
#endif
      // BESTGNSSPOSA
      if (values.size() > 18 && "#BESTGNSSPOSA" == values[0]) {
        // 解算状态
        // SolutionStatus;
        for (int i = 0; i < ARRAY_SIZE(SolutionStutusMap); i++) {
          if (SolutionStutusMap[i].name == values[1]) {
            pos_msg.sol_status.status = SolutionStutusMap[i].status;
            break;
          }
        }
        for (int i = 0; i < ARRAY_SIZE(PositionOrVelocityTypeMap); i++) {
          if (PositionOrVelocityTypeMap[i].name == values[2]) {
            pos_msg.pos_type.type = PositionOrVelocityTypeMap[i].type;
            break;
          }
        }
        // 位置类型
        // 纬度
        pos_msg.lat = std::stod(values[3]);
        // 经度
        pos_msg.lon = std::stod(values[4]);
        // 海拔高
        pos_msg.hgt = std::stod(values[5]);
        // 基站ID
        pos_msg.stn_id = values[11];
        // 差分延时时间(s)
        pos_msg.diff_age = std::stod(values[12]);
        // 解算延时时间(s)
        pos_msg.sol_age = std::stod(values[13]);
        // 解算卫星数
        pos_msg.num_sol_svs = std::stoi(values[14]);
        // L1/E1/B1解算卫星数
        pos_msg.num_sol_multi_svs = std::stoi(values[16]);
        best_pos_call_();
      }

      // HEADINGA
      if (values.size() > 12 && "#HEADINGA" == values[0]) {
        for (int i = 0; i < ARRAY_SIZE(SolutionStutusMap); i++) {
          if (SolutionStutusMap[i].name == values[1]) {
            heading_msg.sol_status.status = SolutionStutusMap[i].status;
            break;
          }
        }
        // 解算状态
        // 定向类型
        for (int i = 0; i < ARRAY_SIZE(PositionOrVelocityTypeMap); i++) {
          if (PositionOrVelocityTypeMap[i].name == values[2]) {
            heading_msg.pos_type.type = PositionOrVelocityTypeMap[i].type;
            break;
          }
        }
        // 方位角
        heading_msg.heading = std::stod(values[4]);
        // 俯仰角
        //heading_msg.pitch = std::stod(values[5]);
        // FIXME
        heading_msg.pitch = atof(values[5].data());
        // 定向天线的可见卫星数
        heading_msg.num_sv_tracked = std::stoi(values[10]);
        // 参与定向的卫星数
        heading_msg.num_sv_in_sol = std::stoi(values[11]);
        heading_call_();
      }

      // INSPVAXA
      if (values.size() > 13 && "#INSPVAXA" == values[0]) {
        // InertialSolutionStatus
        for (int i = 0; i < ARRAY_SIZE(InertialSolutionStatusMap); i++) {
          if (InertialSolutionStatusMap[i].name == values[1]) {
            ins_msg.ins_status.status = InertialSolutionStatusMap[i].status;
            break;
          }
        }
        // PositionOrVelocityType
        // 定向类型
        for (int i = 0; i < ARRAY_SIZE(PositionOrVelocityTypeMap); i++) {
          if (PositionOrVelocityTypeMap[i].name == values[2]) {
            ins_msg.pos_type.type = PositionOrVelocityTypeMap[i].type;
            break;
          }
        }
        // 纬度
        ins_msg.latitude = std::stod(values[3]);
        // 经度
        ins_msg.longitude = std::stod(values[4]);
        // 海拔高
        ins_msg.height = std::stod(values[5]);
        // 横滚角
        ins_msg.roll = std::stod(values[10]);
        // 俯仰角
        ins_msg.pitch = std::stod(values[11]);
        // 航向角
        ins_msg.azimuth = std::stod(values[12]);
        inspvaxa_call_();
      }

      // RAWIMUA
      // #RAWIMUA,ICOM1,0,98.3,FINESTEERING,2310,93548.121,00000000,0000,781;2310,93548.121182000,00fc0000,3207563,5374,27758,-780,-5181,3749*4c7ed351
      if (values.size() > 10 && "%RAWIMUSA" == values[0]) {
        // PositionOrVelocityType
        // 定向类型
        for (int i = 0; i < ARRAY_SIZE(PositionOrVelocityTypeMap); i++) {
          if (PositionOrVelocityTypeMap[i].name == values[2]) {
            ins_msg.pos_type.type = PositionOrVelocityTypeMap[i].type;
            break;
          }
        }
        // Z 轴的速度增量* 
        imu_msg.z_accel = std::stod(values[4]);
        // -Y 轴的速度增量*
        imu_msg.n_y_accel = std::stod(values[5]);
        // X 轴的速度增量* 
        imu_msg.x_accel = std::stod(values[6]);
        // Z 轴的角度增量*
        imu_msg.z_gyro = std::stod(values[7]);
        // -Y 轴的角度增量* 
        imu_msg.n_y_gyro = std::stod(values[8]);
        std::string gyro = values[9].substr(0, 4);
        // X 轴的角度增量*
        imu_msg.x_gyro  = std::stod(gyro);
        rawimu_call_();
      }
    }
  } catch(std::exception &e) {
    std::cout << __func__ << " parse err " << e.what() << std::endl;
  }
}

void DataParser::ParseHeader(com_location_msgs::Oem7Header &header,
    std::string &data)
{
  std::vector<std::string> values;
  values.clear();
  values = string_split(data, ",");
  if (values.size() > 5) {
    try {
      header.gps_week_number = std::stoi(values[5]);
      header.gps_week_milliseconds = std::stof(values[6]);
    } catch (std::exception &e) {
      std::cout << __func__ << " parse err " << e.what() << std::endl;
    }
  }
}

#ifdef GNSS_TEST
int main(int ac, char **av)
#else
int test_gnss(
    com_location_msgs::BESTGNSSPOS &pos_msg,
    com_location_msgs::HEADINGA &heading_msg,
    com_location_msgs::INSPVAXA &ins_msg,
    com_location_msgs::RAWIMUA &imu_msg)
#endif
{
#ifdef GNSS_TEST
  com_location_msgs::BESTGNSSPOS pos_msg;
  com_location_msgs::HEADINGA heading_msg;
  com_location_msgs::INSPVAXA ins_msg;
  com_location_msgs::RAWIMUA imu_msg;
#endif
  DataParser parser;
  //std::string data = "$GPGGA,025724.20,2811.3394819,N,11250.8155580,E,5,09,1.8,55.873,M,-17.063,M,1.200,1714*54\r\n$GPRMC,025724.20,A,2811.3394819,N,11250.8155580,E,0.016,315.4,130124,0.0,E,F*33\r\n#BESTGNSSPOSA,ICOM4,0,0.0,FINESTEERING,2109,367696.000,00000000,0000,82;SOL_COMPUTED,NARROW_INT,28.23315515415,112.87713068512,82.5990,-17.0381,WGS84,0.0106,0.011 0,0.0250,\"0\",1.000,0.058,33,33,33,25,00,00,30,33*9ea908f7\r\n#HEADINGA,COM3,0,0,FINESTEERING,1975,394129.000,00000000,0000,113;SOL_COMPUTED,NARROW_INT,1.328605294,296.248487535,­71.075350314,0,0.200,0.500,\"0000\",29,24,29,7,00,00, 10,01*63131FA1\r\n#INSPVAXA,ICOM4,0,0.0,FINESTEERING,2107,35489.000,00000000,03de,68;INS_ALIGNMENT_COMPLETE,INS_RTKFIXED,28.23316396165,112.87713086609,82.7966,-17.0382,0.0020,-0.0191, 0.0006,179.789714292,-0.387541550,1.405962922,0.0240,0.0168,0.0218,0.0047,0.0049,0.005 4,0.0553,0.0553,1.0818,00000000,0*fd6e3a89";
  std::string data = "#INSPVAXA,ICOM1,0,89.3,FINESTEERING,2324,470496.400,00000000,017d,781;INS_ALIGNMENT_COMPLETE,INS_RTKFIXED,28.18532872130,112.93963675918,49.6446,-16.8064,-0.0096,-0.0015,0.0005,2.563425919,-1.549976730,48.125271381,0.0160,0.0172,0.0300,0.0049,0.0051,0.0035,0.0099,0.0099,0.3385,00040017,0*54793bff\r\n$GPGGA,030816.00,2811.1149002,N,11256.4017374,E,1,17,1.1,66.918,M,-16.805,M,0.000,0000*59\r\n#BESTGNSSPOSA,ICOM1,0,98.3,FINESTEERING,2305,356914.000,00000000,0001,781;SOL_COMPUTED,SINGLE,28.18524833680,112.94002895592,66.9182,-16.8052,WGS84,1.1259,1.0108,1.4472,\"\",0.000,0.024,17,17,17,17,00,00,30,37*804b0e53\r\n#HEADINGA,ICOM1,0,98.3,FINESTEERING,2305,356914.000,00000000,0000,781;SOL_COMPUTED,NARROW_INT,0.010827017,0.000000000,0.000000000,0.0,0.000000000,0.000000000,\"0\",15,14,15,0,0,00,30,37*3b8c849e";

  parser.Parse(data, pos_msg, heading_msg, ins_msg, imu_msg);

  printf("pos_msg: sol_status %d pos_type %d lat %.9f lng %.9f height %.6f stn_id %s diff_ag %.3f sol_age %.3f num_sol_svs %d sol_multi_svs %d\n",
      pos_msg.sol_status.status, pos_msg.pos_type.type,
      pos_msg.lat, pos_msg.lon, pos_msg.hgt, pos_msg.stn_id.data(), pos_msg.diff_age,
      pos_msg.sol_age, pos_msg.num_sol_svs, pos_msg.num_sol_multi_svs);

  printf("heading_msg: sol_status %d pos_type %d heading %.6f pitch %.6f num_sv_tracked %d num_sv_in_sol %d\n",
      heading_msg.sol_status.status, heading_msg.pos_type.type,
      heading_msg.heading, heading_msg.pitch, heading_msg.num_sv_tracked,
      heading_msg.num_sv_in_sol);

  printf("ins_msg: ins_status %d pos_type %d lat %.9f lng %.9f height %.6f roll %.3f pitch %.3f yaw %.3f\n",
      ins_msg.ins_status.status, ins_msg.pos_type.type,
      ins_msg.latitude, ins_msg.longitude, ins_msg.height,
      ins_msg.roll, ins_msg.pitch, ins_msg.azimuth);
  return 0;
}

// g++ -DGNSS_TEST src/drivers/com_location_driver/src/gnss_data.cc src/drivers/com_location_driver/src/data_parser.cc -Iinstall/com_location_msgs/include -I/opt/ros/galactic/include -Isrc
