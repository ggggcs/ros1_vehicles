#include "subscriber.h"
#include <string>
#include <thread>
#include "../../common/data_sync.h"

class Calibrator {
  public:
    Calibrator(rclcpp::Node::SharedPtr node);
    void Init(std::string path);

    void Start(void);

  private:
    CalibrationSubscriber subs;
    rclcpp::Node::SharedPtr node_;
};

Calibrator::Calibrator(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  Init("");
}

void Calibrator::Init(std::string path)
{
  subs.Init(node_);

  std::thread th(&Calibrator::Start, this);
  th.detach();
}

void Calibrator::Start(void)
{
  while (1) {
    SyncFrame frame;
    if (!DataSync::Instance()->GetSyncFrame(frame)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      break;
    }

    // TODO
  };
}

//主运行程序
int main(int argc,char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("calibration_Subscriber");
  Calibrator calib(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
