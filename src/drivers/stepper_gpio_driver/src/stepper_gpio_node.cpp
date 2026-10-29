#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>

#include <stepper_gpio_driver/MotorCommand.h>
#include <stepper_gpio_driver/MotorStatus.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace {

std::runtime_error systemError(const std::string& action, const std::string& path) {
  std::ostringstream stream;
  stream << action << " " << path << ": " << std::strerror(errno);
  return std::runtime_error(stream.str());
}

class GpioLine {
 public:
  GpioLine(const std::string& path, bool dry_run) : path_(path), dry_run_(dry_run) {
    if (!dry_run_) {
      fd_ = ::open(path_.c_str(), O_WRONLY | O_CLOEXEC);
      if (fd_ < 0) {
        throw systemError("Cannot open", path_);
      }
    }
  }

  ~GpioLine() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  GpioLine(const GpioLine&) = delete;
  GpioLine& operator=(const GpioLine&) = delete;

  void set(bool high) {
    if (dry_run_) {
      state_ = high;
      return;
    }

    if (::lseek(fd_, 0, SEEK_SET) < 0) {
      throw systemError("Cannot seek", path_);
    }

    const char value[2] = {high ? '1' : '0', '\n'};
    const ssize_t written = ::write(fd_, value, sizeof(value));
    if (written != static_cast<ssize_t>(sizeof(value))) {
      throw systemError("Cannot write", path_);
    }
    state_ = high;
  }

 private:
  std::string path_;
  bool dry_run_{false};
  bool state_{false};
  int fd_{-1};
};

struct Motion {
  uint64_t pulses{0};
  double frequency_hz{0.0};
  double angle_deg{0.0};
  double speed_rpm{0.0};
  int8_t direction{1};
  bool is_test{false};
};

timespec monotonicNow() {
  timespec value{};
  if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw systemError("clock_gettime failed for", "CLOCK_MONOTONIC");
  }
  return value;
}

void addNanoseconds(timespec* value, int64_t nanoseconds) {
  value->tv_sec += nanoseconds / 1000000000LL;
  value->tv_nsec += nanoseconds % 1000000000LL;
  if (value->tv_nsec >= 1000000000L) {
    ++value->tv_sec;
    value->tv_nsec -= 1000000000L;
  }
}

bool isEarlier(const timespec& left, const timespec& right) {
  return left.tv_sec < right.tv_sec ||
         (left.tv_sec == right.tv_sec && left.tv_nsec < right.tv_nsec);
}

void sleepForEdge(timespec* deadline, int64_t duration_ns) {
  addNanoseconds(deadline, duration_ns);
  const timespec now = monotonicNow();
  if (isEarlier(*deadline, now)) {
    *deadline = now;
    addNanoseconds(deadline, duration_ns);
  }

  int result = 0;
  do {
    result = ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, deadline, nullptr);
  } while (result == EINTR);

  if (result != 0) {
    errno = result;
    throw systemError("clock_nanosleep failed for", "pulse timing");
  }
}

}  // namespace

class StepperGpioDriver {
 public:
  StepperGpioDriver() : private_nh_("~") {
    private_nh_.param<std::string>("pulse_gpio_path", pulse_gpio_path_,
                                   "/sys/class/tz_gpio/gpio-output_1/value");
    private_nh_.param<std::string>("direction_gpio_path", direction_gpio_path_,
                                   "/sys/class/tz_gpio/gpio-output_2/value");
    private_nh_.param("pulses_per_revolution", pulses_per_revolution_, 200.0);
    private_nh_.param("gear_ratio", gear_ratio_, 1.0);
    private_nh_.param("min_frequency_hz", min_frequency_hz_, 100.0);
    private_nh_.param("max_frequency_hz", max_frequency_hz_, 20000.0);
    private_nh_.param("invert_direction", invert_direction_, false);
    private_nh_.param("dry_run", dry_run_, false);
    private_nh_.param("test_mode", test_mode_, false);
    private_nh_.param("test_on_start", test_on_start_, false);
    private_nh_.param("test_frequency_hz", test_frequency_hz_, 100.0);
    private_nh_.param("test_delay_s", test_delay_s_, 2.0);
    private_nh_.param("direction_setup_us", direction_setup_us_, 1000);

    int test_pulses = 5;
    int test_direction = 1;
    int max_command_pulses = 1000000;
    private_nh_.param("test_pulses", test_pulses, 5);
    private_nh_.param("test_direction", test_direction, 1);
    private_nh_.param("max_command_pulses", max_command_pulses, 1000000);
    test_pulses_ = test_pulses > 0 ? static_cast<uint64_t>(test_pulses) : 0;
    test_direction_ = static_cast<int8_t>(test_direction);
    max_command_pulses_ = max_command_pulses > 0
                              ? static_cast<uint64_t>(max_command_pulses)
                              : 0;

    validateParameters();

    pulse_gpio_.reset(new GpioLine(pulse_gpio_path_, dry_run_));
    direction_gpio_.reset(new GpioLine(direction_gpio_path_, dry_run_));
    pulse_gpio_->set(false);
    direction_gpio_->set(false);

    status_publisher_ = private_nh_.advertise<stepper_gpio_driver::MotorStatus>("status", 10, true);
    command_subscriber_ = private_nh_.subscribe("command", 10, &StepperGpioDriver::commandCallback, this);
    stop_subscriber_ = private_nh_.subscribe("stop", 10, &StepperGpioDriver::stopCallback, this);
    test_service_ = private_nh_.advertiseService("run_test", &StepperGpioDriver::testCallback, this);

    worker_ = std::thread(&StepperGpioDriver::workerLoop, this);
    publishStatus("idle", Motion{}, 0, "Ready");

    ROS_INFO_STREAM("Stepper GPIO driver ready: P+=" << pulse_gpio_path_
                    << ", D+=" << direction_gpio_path_
                    << ", test_mode=" << std::boolalpha << test_mode_
                    << ", dry_run=" << dry_run_);

    if (test_mode_ && test_on_start_) {
      test_timer_ = private_nh_.createWallTimer(ros::WallDuration(test_delay_s_),
                                                &StepperGpioDriver::testTimerCallback,
                                                this, true);
    }
  }

  ~StepperGpioDriver() {
    shutting_down_.store(true);
    stop_requested_.store(true);
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    try {
      if (pulse_gpio_) {
        pulse_gpio_->set(false);
      }
      if (direction_gpio_) {
        direction_gpio_->set(false);
      }
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM("Failed to leave GPIO outputs low during shutdown: " << error.what());
    }
  }

 private:
  void validateParameters() const {
    if (!std::isfinite(pulses_per_revolution_) || pulses_per_revolution_ <= 0.0) {
      throw std::runtime_error("~pulses_per_revolution must be positive");
    }
    if (!std::isfinite(gear_ratio_) || gear_ratio_ <= 0.0) {
      throw std::runtime_error("~gear_ratio must be positive");
    }
    if (!std::isfinite(min_frequency_hz_) || !std::isfinite(max_frequency_hz_) ||
        min_frequency_hz_ <= 0.0 || max_frequency_hz_ < min_frequency_hz_) {
      throw std::runtime_error("Invalid pulse frequency limits");
    }
    if (test_pulses_ == 0 || test_pulses_ > max_command_pulses_) {
      throw std::runtime_error("~test_pulses must be between 1 and ~max_command_pulses");
    }
    if (test_direction_ != 1 && test_direction_ != -1) {
      throw std::runtime_error("~test_direction must be +1 or -1");
    }
    if (test_frequency_hz_ < min_frequency_hz_ || test_frequency_hz_ > max_frequency_hz_) {
      throw std::runtime_error("~test_frequency_hz is outside the configured frequency limits");
    }
    if (direction_setup_us_ < 0) {
      throw std::runtime_error("~direction_setup_us cannot be negative");
    }
  }

  bool directionLevel(int8_t direction) const {
    const bool positive = direction > 0;
    return invert_direction_ ? !positive : positive;
  }

  void commandCallback(const stepper_gpio_driver::MotorCommand::ConstPtr& message) {
    if (test_mode_) {
      publishStatus("rejected", Motion{}, 0,
                    "Normal commands are disabled while ~test_mode is true");
      ROS_WARN("Rejected normal motion command in test mode");
      return;
    }

    Motion motion;
    std::string error;
    if (!buildMotion(*message, &motion, &error)) {
      publishStatus("rejected", motion, 0, error);
      ROS_WARN_STREAM("Rejected motion command: " << error);
      return;
    }

    if (!queueMotion(motion, &error)) {
      publishStatus("rejected", motion, 0, error);
      ROS_WARN_STREAM("Rejected motion command: " << error);
    }
  }

  bool buildMotion(const stepper_gpio_driver::MotorCommand& message,
                   Motion* motion, std::string* error) const {
    motion->angle_deg = message.angle_deg;
    motion->speed_rpm = message.speed_rpm;
    motion->direction = message.direction;

    if (!std::isfinite(message.angle_deg) || message.angle_deg <= 0.0) {
      *error = "angle_deg must be finite and greater than zero";
      return false;
    }
    if (!std::isfinite(message.speed_rpm) || message.speed_rpm <= 0.0) {
      *error = "speed_rpm must be finite and greater than zero";
      return false;
    }
    if (message.direction != 1 && message.direction != -1) {
      *error = "direction must be +1 or -1";
      return false;
    }

    const long double pulse_count =
        static_cast<long double>(message.angle_deg) / 360.0L *
        static_cast<long double>(pulses_per_revolution_) *
        static_cast<long double>(gear_ratio_);
    if (!std::isfinite(static_cast<double>(pulse_count)) || pulse_count < 0.5L ||
        pulse_count > static_cast<long double>(max_command_pulses_)) {
      *error = "angle converts to zero pulses or exceeds ~max_command_pulses";
      return false;
    }

    const double frequency = message.speed_rpm / 60.0 *
                             pulses_per_revolution_ * gear_ratio_;
    if (!std::isfinite(frequency) || frequency < min_frequency_hz_ ||
        frequency > max_frequency_hz_) {
      std::ostringstream stream;
      stream << "speed converts to " << frequency << " Hz; allowed range is "
             << min_frequency_hz_ << "-" << max_frequency_hz_ << " Hz";
      *error = stream.str();
      return false;
    }

    motion->pulses = static_cast<uint64_t>(std::llround(pulse_count));
    motion->frequency_hz = frequency;
    return true;
  }

  void stopCallback(const std_msgs::Empty::ConstPtr&) {
    bool had_pending = false;
    bool is_busy = false;
    Motion active_motion;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      had_pending = pending_;
      pending_ = false;
      is_busy = busy_;
      active_motion = active_motion_;
    }

    stop_requested_.store(true);
    condition_.notify_all();
    if (is_busy) {
      publishStatus("stopping", active_motion, completed_pulses_.load(),
                    "Stop requested");
    } else if (had_pending) {
      publishStatus("stopped", Motion{}, 0, "Pending command cancelled");
    } else {
      publishStatus("idle", Motion{}, 0, "No motion was active");
    }
  }

  bool testCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (!test_mode_) {
      response.success = false;
      response.message = "Set ~test_mode:=true before running the point test";
      return true;
    }

    Motion motion = testMotion();
    std::string error;
    response.success = queueMotion(motion, &error);
    response.message = response.success ? "Point test queued" : error;
    return true;
  }

  void testTimerCallback(const ros::WallTimerEvent&) {
    Motion motion = testMotion();
    std::string error;
    if (!queueMotion(motion, &error)) {
      ROS_ERROR_STREAM("Could not start automatic point test: " << error);
    }
  }

  Motion testMotion() const {
    Motion motion;
    motion.pulses = test_pulses_;
    motion.frequency_hz = test_frequency_hz_;
    motion.direction = test_direction_;
    motion.is_test = true;
    return motion;
  }

  bool queueMotion(const Motion& motion, std::string* error) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (busy_ || pending_) {
        *error = "Motor is busy; send ~stop or wait for completion";
        return false;
      }
      pending_motion_ = motion;
      pending_ = true;
      stop_requested_.store(false);
    }
    condition_.notify_one();
    publishStatus("queued", motion, 0, motion.is_test ? "Point test queued" : "Motion queued");
    return true;
  }

  void workerLoop() {
    while (!shutting_down_.load()) {
      Motion motion;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return shutting_down_.load() || pending_; });
        if (shutting_down_.load()) {
          break;
        }
        motion = pending_motion_;
        pending_ = false;
        busy_ = true;
        active_motion_ = motion;
        completed_pulses_.store(0);
      }

      executeMotion(motion);

      {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
        active_motion_ = Motion{};
      }
      stop_requested_.store(false);
    }
  }

  void executeMotion(const Motion& motion) {
    uint64_t completed = 0;
    try {
      direction_gpio_->set(directionLevel(motion.direction));
      if (direction_setup_us_ > 0) {
        timespec setup_deadline = monotonicNow();
        sleepForEdge(&setup_deadline,
                     static_cast<int64_t>(direction_setup_us_) * 1000LL);
      }

      publishStatus("running", motion, 0,
                    motion.is_test ? "Point test running" : "Motion running");
      ROS_INFO_STREAM((motion.is_test ? "Point test" : "Motion") << " started: "
                      << motion.pulses << " pulses at " << motion.frequency_hz << " Hz");

      const int64_t half_period_ns = static_cast<int64_t>(
          std::llround(500000000.0 / motion.frequency_hz));
      timespec edge_deadline = monotonicNow();
      ros::WallTime last_status = ros::WallTime::now();

      for (uint64_t index = 0; index < motion.pulses; ++index) {
        if (stop_requested_.load() || shutting_down_.load()) {
          break;
        }

        pulse_gpio_->set(true);
        sleepForEdge(&edge_deadline, half_period_ns);
        pulse_gpio_->set(false);
        ++completed;
        completed_pulses_.store(completed);
        sleepForEdge(&edge_deadline, half_period_ns);

        if ((ros::WallTime::now() - last_status).toSec() >= 0.5) {
          publishStatus("running", motion, completed, "Motion running");
          last_status = ros::WallTime::now();
        }
      }

      pulse_gpio_->set(false);
      if (completed == motion.pulses) {
        publishStatus("completed", motion, completed,
                      motion.is_test ? "Point test pulse sequence completed"
                                     : "Motion completed");
        ROS_INFO_STREAM((motion.is_test ? "Point test" : "Motion")
                        << " completed: " << completed << "/" << motion.pulses
                        << " pulses");
      } else {
        publishStatus("stopped", motion, completed, "Motion stopped");
        ROS_WARN_STREAM("Motion stopped after " << completed << "/" << motion.pulses
                        << " pulses");
      }
    } catch (const std::exception& error) {
      try {
        pulse_gpio_->set(false);
      } catch (const std::exception& low_error) {
        ROS_ERROR_STREAM("Also failed to force pulse GPIO low: " << low_error.what());
      }
      publishStatus("error", motion, completed, error.what());
      ROS_ERROR_STREAM("GPIO motion failed: " << error.what());
    }
  }

  void publishStatus(const std::string& state, const Motion& motion,
                     uint64_t completed, const std::string& text) {
    stepper_gpio_driver::MotorStatus status;
    status.header.stamp = ros::Time::now();
    status.state = state;
    status.commanded_pulses = motion.pulses;
    status.completed_pulses = completed;
    status.requested_angle_deg = motion.angle_deg;
    status.requested_speed_rpm = motion.speed_rpm;
    status.direction = motion.direction;
    status.message = text;
    status_publisher_.publish(status);
  }

  ros::NodeHandle private_nh_;
  ros::Publisher status_publisher_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber stop_subscriber_;
  ros::ServiceServer test_service_;
  ros::WallTimer test_timer_;

  std::string pulse_gpio_path_;
  std::string direction_gpio_path_;
  double pulses_per_revolution_{200.0};
  double gear_ratio_{1.0};
  double min_frequency_hz_{100.0};
  double max_frequency_hz_{20000.0};
  double test_frequency_hz_{100.0};
  double test_delay_s_{2.0};
  bool invert_direction_{false};
  bool dry_run_{false};
  bool test_mode_{false};
  bool test_on_start_{false};
  int direction_setup_us_{1000};
  uint64_t test_pulses_{5};
  uint64_t max_command_pulses_{1000000};
  int8_t test_direction_{1};

  std::unique_ptr<GpioLine> pulse_gpio_;
  std::unique_ptr<GpioLine> direction_gpio_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  Motion pending_motion_;
  Motion active_motion_;
  bool pending_{false};
  bool busy_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutting_down_{false};
  std::atomic<uint64_t> completed_pulses_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "stepper_gpio");
  try {
    StepperGpioDriver driver;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Stepper GPIO driver could not start: " << error.what());
    return 1;
  }
  return 0;
}
