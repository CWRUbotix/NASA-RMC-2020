#include "threads/motor_thread.h"

#include <ros/spinner.h>

#include <boost/smart_ptr/make_shared.hpp>
#include <boost/range/adaptor/map.hpp>

#include <string>
#include <map>

#include "hardware/bmc.h"
#include "hardware/motor.h"
#include "hardware/vesc.h"
#include "hwctrl.h"

MotorThread::MotorThread(ros::NodeHandle nh) : m_nh(nh), m_loop_rate(1000) {
  m_motor_set_sub = m_nh.subscribe("motor_setpoints", 128,
                                   &MotorThread::set_motor_callback, this);
  m_estop_sub = m_nh.subscribe("estop", 128, &MotorThread::estop_callback, this);
  // Get all limit switch topics. Doesnt matter which one as long as they specify
  // which motor theyre addressing within the message
  auto ls_topics = get_limit_switch_topics();
  
  // subscribe to all limitswitch messages and send them to our callback
  m_ls_subs.reserve(ls_topics.size());
  for(auto name : ls_topics)
    m_ls_subs.push_back(m_nh.subscribe(name, 128, &MotorThread::limit_switch_callback, this));

  read_from_server();
}

std::vector<std::string> MotorThread::get_limit_switch_topics() {
  std::vector<std::string> topics;
  const std::string base = hwctrl::param_base + "/sensor/";

  for (auto name : hwctrl::sensor_param_names) {
    const std::string full_base = base + name;
    const std::string topic_path = full_base + "/topic";
    const std::string type_path = full_base + "/type";
    if(m_nh.hasParam(topic_path)) {
      std::string type_str;
      m_nh.getParam(type_path, type_str);
      if(type_str.compare("limit") != 0) {
        continue;
      }
    } else continue;

    if(m_nh.hasParam(topic_path)) {
      std::string temp;
      m_nh.getParam(topic_path, temp);
      topics.push_back(temp);
    }
  }
  return topics;
}

void MotorThread::read_from_server() {
  ROS_INFO("Reading motor configs from param server...");
  const std::string base = hwctrl::param_base + "/motor";
  uint32_t sys_id_idx = 0;
  for (auto name = hwctrl::motor_param_names.begin(); name != hwctrl::motor_param_names.end();
       ++name) {
    const std::string full_name = base + "/" + *name;
    ROS_INFO("Checking for parameters under %s/...", full_name.c_str());
    const std::string name_param = full_name + "/name";
    const std::string type_param = full_name + "/type";
    const std::string id_param = full_name + "/id";
    const std::string can_id_param = full_name + "/can_id";
    const std::string gear_reduc_param = full_name + "/gear_reduction";
    const std::string max_rpm_param = full_name + "/max_rpm";
    const std::string max_accel_param = full_name + "/max_acceleration";
    const std::string period_param = full_name + "/update_period";

    std::string name_str;
    MotorType type;
    ControlType c_type;
    int id;
    int can_id;
    double gearing;
    double max_rpm;
    double max_acc;
    ros::Duration update_pd;
    ros::Duration timeout = ros::Duration(2.0);

    boost::shared_ptr<Motor> motor;
    int found = 0;

    if (m_nh.hasParam(name_param)) {
      m_nh.getParam(name_param, name_str);
      ROS_INFO(" - Found motor name: %s", name_str.c_str());
      found++;
    }
    if (m_nh.hasParam(type_param)) {
      std::string type_str;
      m_nh.getParam(type_param, type_str);
      ROS_INFO(" - Found motor type: %s", type_str.c_str());
      type = get_motor_type(type_str);
      found++;
    }
    if(m_nh.hasParam(id_param)) {
      m_nh.getParam(id_param, id);
      ROS_INFO(" - Found system id: %d", id);
      found++;
    }
    if (m_nh.hasParam(can_id_param)) {
      m_nh.getParam(can_id_param, can_id);
      ROS_INFO(" - Found CAN id: %d", can_id);
      found++;
    }
    if (m_nh.hasParam(gear_reduc_param)) {
      m_nh.getParam(gear_reduc_param, gearing);
      ROS_INFO(" - Found gear reduction: %.2f", gearing);
      found++;
    }
    if (m_nh.hasParam(max_rpm_param)) {
      m_nh.getParam(max_rpm_param, max_rpm);
      ROS_INFO(" - Found max RPM: %.2f", max_rpm);
      found++;
    }
    if (m_nh.hasParam(max_accel_param)) {
      m_nh.getParam(max_accel_param, max_acc);
      ROS_INFO(" - Found max acceleration: %.2f", max_acc);
      found++;
    }
    if (m_nh.hasParam(period_param)) {
      double pd;
      m_nh.getParam(period_param, pd);
      update_pd = ros::Duration(pd);
      ROS_INFO(" - Found motor update period: %.3fs", pd);
    }

    switch (type) {
      case MotorType::Vesc: {
        auto temp = boost::make_shared<VescMotor>(
            m_nh, name_str, id, (uint32_t)can_id, update_pd,
            (float)max_acc, (float)max_acc, (float)max_rpm, (float)gearing,
            timeout);
        motor = temp;
        break;
      }
      case MotorType::BMC: {
        auto temp = boost::make_shared<BMCMotor>(
            m_nh, name_str, id, (uint32_t)can_id, update_pd,
            (float)max_acc, (float)max_acc, (float)max_rpm, (float)gearing,
            timeout);
        break;
      }
      case MotorType::Sabertooth:
        // I wont implement this for now
        motor = nullptr;
      default:
        break;
    }

    if (found > 0 && motor != nullptr) m_motors.insert({id, motor});
  }
}

void MotorThread::set_motor_callback(boost::shared_ptr<hwctrl::MotorCmd> msg) {
  m_motors.at(msg->id)->set_setpoint(ros::Time::now(), msg->setpoint,
                                     msg->acceleration);
}

void MotorThread::estop_callback(boost::shared_ptr<std_msgs::Bool> msg) {
  // If estop is not enabled, powerup the motors
  if (!msg->data) {
    if(!m_sys_power_on) {
      setup_motors();
    }
    m_sys_power_on = true;
  } else {
    m_sys_power_on = false;
  }
}

void MotorThread::limit_switch_callback(boost::shared_ptr<hwctrl::LimitSwState> msg) {
  // TODO: Fix this
  // This should not just stop the motors, it should
  // check which directioon is allowed before stopping
  auto id = msg->motor_id;
  auto allowed_dir = msg->allowed_dir;

  // check setpoint before stopping?
  m_motors.at(id)->stop();
}

void MotorThread::setup_motors() {
  for (auto motor : m_motors | boost::adaptors::map_values)
    motor->setup();
}

void MotorThread::update_motors() {
  auto time = ros::Time::now();
  for (auto motor : m_motors | boost::adaptors::map_values) {
    if (motor->ready_to_update()) {
      motor->update(time);
    }
  }
}

void MotorThread::shutdown() {
  // stop motors maybe
  for(auto motor : m_motors | boost::adaptors::map_values)
    motor->stop();
}

void MotorThread::sleep() { m_loop_rate.sleep(); }

void MotorThread::operator()() {
  ROS_INFO("Starting motor_thread");
  m_nh.setCallbackQueue(&m_cb_queue);
  ros::AsyncSpinner spinner(1, &m_cb_queue);
  spinner.start();
  // setup_motors();
  while (ros::ok()) {
    update_motors();
    sleep();
  }
  shutdown();
}
