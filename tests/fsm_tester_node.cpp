// fsm_tester_node.cpp
//
// Minimal ROS 2 test node for AMR Sweeper FSM.
//
// Features (minimal set):
//   1) startup_delay_ms
//   2) exit_after_ms / crash_after_ms
//   3) log_script
//   4) scenario preset name
//
// Additional feature:
//   - Service endpoint to apply parameter updates at runtime.
//     This enables tests to dynamically change behavior (e.g. schedule an exit).
//
// This node is intended to be launched as a "process" under an FSM profile.
// The FSM can use it to test:
//   - readiness timeouts (topic appears late or never appears)
//   - rosout-triggered DEGRADED / FAULT transitions
//   - process death detection (clean exit vs crash)
//
// Readiness signal:
//   Publishes std_msgs/Empty on "fsm_tester" within the node namespace, after startup_delay_ms.
//   (e.g. /amr_sweeper/fsm_tester if launched with __ns:=/amr_sweeper)
//
// log_script format:
//   Semicolon-separated events, each event is: t_ms,LEVEL,Message
//   Example:
//     "1000,WARN,simulated warn;2000,ERROR,simulated error;2500,FATAL,simulated fatal"
//
// scenario presets (applied only when corresponding explicit params keep default values):
//   - ready_timeout: never advertises the readiness topic
//   - late_ready: startup_delay_ms=1500
//   - warn_then_ok: log_script="500,WARN,simulated degraded condition"
//   - error_to_fault: log_script="500,ERROR,simulated fault condition"
//   - exit_after_ready: exit_after_ms=2000
//   - crash_after_ready: crash_after_ms=2000
//
// Runtime parameter update service:
//   - Service name (private): "~/apply_parameters" (resolves to /<ns>/<node_name>/apply_parameters)
//   - Type: rcl_interfaces/srv/SetParametersAtomically
//
// Example: schedule node to exit in 1000ms from now:
//   ros2 service call /amr_sweeper/fsm_tester_node/apply_parameters rcl_interfaces/srv/SetParametersAtomically
//     "{parameters: [{name: 'exit_after_ms', value: {type: 3, integer_value: 1000}}]}"
//
// NOTE: ROS 2 nodes also expose the standard parameter services
//       (set_parameters / set_parameters_atomically) by default. This service is a convenience alias
//       that also ensures internal timers are reconfigured consistently.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "std_msgs/msg/empty.hpp"

// Severity constants (RCUTILS_LOG_SEVERITY_*).
#include "rcutils/logging.h"

// Parameter service type
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

namespace
{

struct LogEvent
{
  std::chrono::milliseconds at{0};
  int severity = RCUTILS_LOG_SEVERITY_INFO;
  std::string message;
};

static inline std::string trim_copy(const std::string & s)
{
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

static inline bool iequals(const std::string & a, const std::string & b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    const char ca = static_cast<char>(std::tolower(a[i]));
    const char cb = static_cast<char>(std::tolower(b[i]));
    if (ca != cb) return false;
  }
  return true;
}

static inline int level_to_severity(const std::string & level)
{
  if (iequals(level, "DEBUG")) return RCUTILS_LOG_SEVERITY_DEBUG;
  if (iequals(level, "INFO")) return RCUTILS_LOG_SEVERITY_INFO;
  if (iequals(level, "WARN") || iequals(level, "WARNING")) return RCUTILS_LOG_SEVERITY_WARN;
  if (iequals(level, "ERROR")) return RCUTILS_LOG_SEVERITY_ERROR;
  if (iequals(level, "FATAL")) return RCUTILS_LOG_SEVERITY_FATAL;
  return RCUTILS_LOG_SEVERITY_INFO;
}

static std::vector<LogEvent> parse_log_script(const std::string & script)
{
  std::vector<LogEvent> out;
  const std::string s = trim_copy(script);
  if (s.empty()) return out;

  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ';')) {
    item = trim_copy(item);
    if (item.empty()) continue;

    // Split into 3 fields: t_ms,LEVEL,Message (message may contain commas)
    const auto c1 = item.find(',');
    if (c1 == std::string::npos) continue;
    const auto c2 = item.find(',', c1 + 1);
    if (c2 == std::string::npos) continue;

    const std::string t_str = trim_copy(item.substr(0, c1));
    const std::string lvl_str = trim_copy(item.substr(c1 + 1, c2 - (c1 + 1)));
    const std::string msg_str = trim_copy(item.substr(c2 + 1));

    try {
      const auto t_ms = static_cast<int64_t>(std::stoll(t_str));
      LogEvent ev;
      ev.at = std::chrono::milliseconds(t_ms < 0 ? 0 : t_ms);
      ev.severity = level_to_severity(lvl_str);
      ev.message = msg_str;
      out.push_back(std::move(ev));
    } catch (...) {
      continue;
    }
  }

  std::stable_sort(out.begin(), out.end(), [](const LogEvent & a, const LogEvent & b) {
    return a.at < b.at;
  });
  return out;
}

static inline bool is_default_int(const int value, const int default_value)
{
  return value == default_value;
}

static inline bool is_default_str(const std::string & value, const std::string & default_value)
{
  return value == default_value;
}

}  // namespace

class FsmTesterNode final : public rclcpp::Node
{
public:
  explicit FsmTesterNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("fsm_tester_node", options)
  {
    // Parameters (minimal set)
    startup_delay_ms_ = this->declare_parameter<int>("startup_delay_ms", 0);
    exit_after_ms_ = this->declare_parameter<int>("exit_after_ms", -1);
    crash_after_ms_ = this->declare_parameter<int>("crash_after_ms", -1);
    log_script_ = this->declare_parameter<std::string>("log_script", "");
    scenario_ = this->declare_parameter<std::string>("scenario", "");

    apply_scenario_presets();

    // Ensure internal behavior is updated consistently for any parameter updates.
    on_set_params_handle_ = this->add_on_set_parameters_callback(
      std::bind(&FsmTesterNode::on_set_parameters, this, std::placeholders::_1));

    // Convenience alias service.
    apply_params_srv_ = this->create_service<rcl_interfaces::srv::SetParametersAtomically>(
      "~/apply_parameters",
      std::bind(&FsmTesterNode::on_apply_parameters, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize runtime machinery.
    start_time_ = this->now();
    configure_from_current_parameters(/*from_param_update=*/false);

    RCLCPP_INFO(
      this->get_logger(),
      "fsm_tester_node started (startup_delay_ms=%d exit_after_ms=%d crash_after_ms=%d scenario='%s')",
      startup_delay_ms_, exit_after_ms_, crash_after_ms_, scenario_.c_str());
  }

private:
  void apply_scenario_presets()
  {
    // Presets apply only when caller keeps parameter at its default value.
    if (scenario_.empty()) return;

    if (scenario_ == "ready_timeout") {
      // Special: never advertise readiness.
      advertise_ready_ = false;
      return;
    }

    if (scenario_ == "late_ready") {
      if (is_default_int(startup_delay_ms_, 0)) startup_delay_ms_ = 1500;
      return;
    }

    if (scenario_ == "warn_then_ok") {
      if (is_default_str(log_script_, "")) log_script_ = "500,WARN,simulated degraded condition";
      return;
    }

    if (scenario_ == "error_to_fault") {
      if (is_default_str(log_script_, "")) log_script_ = "500,ERROR,simulated fault condition";
      return;
    }

    if (scenario_ == "exit_after_ready") {
      if (is_default_int(exit_after_ms_, -1) && is_default_int(crash_after_ms_, -1)) exit_after_ms_ = 2000;
      return;
    }

    if (scenario_ == "crash_after_ready") {
      if (is_default_int(exit_after_ms_, -1) && is_default_int(crash_after_ms_, -1)) crash_after_ms_ = 2000;
      return;
    }

    RCLCPP_WARN(this->get_logger(), "Unknown scenario preset '%s' (ignored).", scenario_.c_str());
  }

  // --- Parameter updates -----------------------------------------------------

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Validate without mutating
    for (const auto & p : params) {
      const auto & name = p.get_name();

      if (name == "startup_delay_ms") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          result.successful = false;
          result.reason = "startup_delay_ms must be an integer";
          return result;
        }
        if (p.as_int() < 0) {
          result.successful = false;
          result.reason = "startup_delay_ms must be >= 0";
          return result;
        }
      } else if (name == "exit_after_ms" || name == "crash_after_ms") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          result.successful = false;
          result.reason = name + " must be an integer";
          return result;
        }
        // -1 disables; otherwise must be > 0 (0 is ambiguous/no-op)
        const auto v = p.as_int();
        if (!(v == -1 || v > 0)) {
          result.successful = false;
          result.reason = name + " must be -1 (disabled) or > 0";
          return result;
        }
      } else if (name == "log_script") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "log_script must be a string";
          return result;
        }
      } else if (name == "scenario") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "scenario must be a string";
          return result;
        }
      }
    }

    // Apply mutations to internal state based on the parameters that were set.
    for (const auto & p : params) {
      const auto & name = p.get_name();

      if (name == "startup_delay_ms") {
        startup_delay_ms_ = static_cast<int>(p.as_int());
      } else if (name == "exit_after_ms") {
        exit_after_ms_ = static_cast<int>(p.as_int());
      } else if (name == "crash_after_ms") {
        crash_after_ms_ = static_cast<int>(p.as_int());
      } else if (name == "log_script") {
        log_script_ = p.as_string();
      } else if (name == "scenario") {
        scenario_ = p.as_string();
        // Scenario presets are intended at startup. Changing scenario at runtime does not
        // auto-overwrite the other parameters.
      }
    }

    // Reconfigure internal timers/publishers for any relevant changes.
    configure_from_current_parameters(/*from_param_update=*/true);

    return result;
  }

  void on_apply_parameters(
    const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
    std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response)
  {
    // Convert msg Parameters -> rclcpp::Parameter and apply atomically.
    std::vector<rclcpp::Parameter> params;
    params.reserve(request->parameters.size());
    for (const auto & pm : request->parameters) {
      params.push_back(rclcpp::Parameter::from_parameter_msg(pm));
    }

    const auto r = this->set_parameters_atomically(params);
    response->result = r;
  }

  // --- Runtime configuration -------------------------------------------------

  void configure_from_current_parameters(const bool from_param_update)
  {
    (void)from_param_update;

    if (crash_after_ms_ > 0 && exit_after_ms_ > 0) {
      // Deterministic precedence: crash wins.
      RCLCPP_WARN(
        this->get_logger(),
        "Both crash_after_ms (%d) and exit_after_ms (%d) set; crash will take precedence.",
        crash_after_ms_, exit_after_ms_);
    }

    // Rebuild log events and restart script timer.
    events_ = parse_log_script(log_script_);
    next_event_index_ = 0;

    if (script_timer_) {
      script_timer_->cancel();
      script_timer_.reset();
    }
    if (!events_.empty()) {
      script_timer_ = this->create_wall_timer(50ms, std::bind(&FsmTesterNode::on_script_tick, this));
    }

    // Configure readiness advertisement.
    if (ready_timer_) {
      ready_timer_->cancel();
      ready_timer_.reset();
    }
    if (ready_pub_timer_) {
      ready_pub_timer_->cancel();
      ready_pub_timer_.reset();
    }
    // Only drop publisher if it doesn't exist yet; once created it stays (simpler and fine for tests).
    if (!ready_pub_ && advertise_ready_) {
      if (startup_delay_ms_ <= 0) {
        enable_readiness_publisher();
      } else {
        ready_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(startup_delay_ms_),
          std::bind(&FsmTesterNode::on_startup_delay_elapsed, this));
      }
    }

    // Configure exit/crash timers from *now*.
    if (exit_timer_) {
      exit_timer_->cancel();
      exit_timer_.reset();
    }
    if (crash_timer_) {
      crash_timer_->cancel();
      crash_timer_.reset();
    }

    if (crash_after_ms_ > 0) {
      crash_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(crash_after_ms_),
        std::bind(&FsmTesterNode::do_crash, this));
    } else if (exit_after_ms_ > 0) {
      exit_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(exit_after_ms_),
        std::bind(&FsmTesterNode::do_exit, this));
    }
  }

  void on_startup_delay_elapsed()
  {
    if (ready_timer_) {
      ready_timer_->cancel();
      ready_timer_.reset();
    }
    enable_readiness_publisher();
  }

  void enable_readiness_publisher()
  {
    if (ready_pub_) return;

    ready_pub_ = this->create_publisher<std_msgs::msg::Empty>("fsm_tester", rclcpp::QoS(1));
    ready_pub_timer_ = this->create_wall_timer(1s, std::bind(&FsmTesterNode::publish_ready, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Readiness topic enabled: %s/fsm_tester (publishing std_msgs/Empty @ 1 Hz)",
      this->get_namespace());
  }

  void publish_ready()
  {
    if (!ready_pub_) return;
    std_msgs::msg::Empty msg;
    ready_pub_->publish(msg);
  }

  void on_script_tick()
  {
    const rclcpp::Time now = this->now();
    const auto elapsed_ms = std::chrono::milliseconds(
      static_cast<int64_t>((now - start_time_).nanoseconds() / 1000000LL));

    while (next_event_index_ < events_.size() && events_[next_event_index_].at <= elapsed_ms) {
      const auto & ev = events_[next_event_index_];

      switch (ev.severity) {
        case RCUTILS_LOG_SEVERITY_DEBUG:
          RCLCPP_DEBUG(this->get_logger(), "%s", ev.message.c_str());
          break;
        case RCUTILS_LOG_SEVERITY_INFO:
          RCLCPP_INFO(this->get_logger(), "%s", ev.message.c_str());
          break;
        case RCUTILS_LOG_SEVERITY_WARN:
          RCLCPP_WARN(this->get_logger(), "%s", ev.message.c_str());
          break;
        case RCUTILS_LOG_SEVERITY_ERROR:
          RCLCPP_ERROR(this->get_logger(), "%s", ev.message.c_str());
          break;
        case RCUTILS_LOG_SEVERITY_FATAL:
          RCLCPP_FATAL(this->get_logger(), "%s", ev.message.c_str());
          break;
        default:
          RCLCPP_INFO(this->get_logger(), "%s", ev.message.c_str());
          break;
      }

      ++next_event_index_;
    }

    if (next_event_index_ >= events_.size() && script_timer_) {
      script_timer_->cancel();
    }
  }

  void do_exit()
  {
    RCLCPP_ERROR(this->get_logger(), "Exiting as requested (exit_after_ms=%d).", exit_after_ms_);
    rclcpp::shutdown();
  }

  void do_crash()
  {
    RCLCPP_FATAL(this->get_logger(), "Crashing as requested (crash_after_ms=%d).", crash_after_ms_);
    std::abort();
  }

private:
  // Params
  int startup_delay_ms_{0};
  int exit_after_ms_{-1};
  int crash_after_ms_{-1};
  std::string log_script_;
  std::string scenario_;

  bool advertise_ready_{true};

  // Timing
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  // Scripted logging
  std::vector<LogEvent> events_;
  size_t next_event_index_{0};
  rclcpp::TimerBase::SharedPtr script_timer_;

  // Readiness
  rclcpp::TimerBase::SharedPtr ready_timer_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr ready_pub_timer_;

  // Exit/crash
  rclcpp::TimerBase::SharedPtr exit_timer_;
  rclcpp::TimerBase::SharedPtr crash_timer_;

  // Parameter updates
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr apply_params_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FsmTesterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
