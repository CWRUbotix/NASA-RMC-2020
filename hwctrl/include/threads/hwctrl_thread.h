#pragma once

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <ros/time.h>

#include <string>

class HwctrlThread {
 public:
  HwctrlThread() = default;
  ~HwctrlThread() = default;

  // delete copy constructors
  HwctrlThread(HwctrlThread const&) = delete;
  void operator=(HwctrlThread const&) = delete;

  // delete move constructors
  HwctrlThread(HwctrlThread&&) = delete;
  void operator=(HwctrlThread&&) = delete;
};
