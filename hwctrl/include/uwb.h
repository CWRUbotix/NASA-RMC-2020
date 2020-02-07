#ifndef _UWB_H_
#define _UWB_H_

#include <string>
#include <cstdio>
#include <cstdlib>
#include <inttypes.h>
#include <hwctrl/CanFrame.h>
#include <hwctrl/UwbData.h>

#define UWB_CAN_HDDR_SIZE 2

typedef enum {
  UWB_TIMESTAMP = 0,
  UWB_X,
  UWB_Y,
  UWB_Z,
  UWB_DISTANCE,
  UWB_RX_POWER,
  UWB_FP_POWER,
  UWB_FP_SNR,
  UWB_RANGING_DONE,
  UWB_LEGACY
} uwb_can_data_t;

class AnchorData{
  uint32_t id;
  uint32_t timestamp;
  float x;
  float y;
  float z;
  float distance;
  float rx_power;
  float fp_power;
  float fp_snr;
  bool data_ready;
  std::string to_string();
}

class UwbNode{
  uint32_t id;
private:
  std::vector<AnchorData> anchors;
public:
  UwbNode(uint32_t id) : id(id) { }
  uint32_t get_id() : {return id;}
  uint32_t get_num_anchors() : {return anchors.size();}
  AnchorData* get_anchor_by_id(uint32_t id);
  void add_can_data(uint8_t* can_data); // assimilate a CAN frame's worth of data
  int get_msgs_from_anchors(hwctrl::UwbData* uwb_msgs, int max_size);
};



#endif
