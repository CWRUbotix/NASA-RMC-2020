#include <uwb.h>

std::vector<AnchorData>::iterator UwbNode::get_anchor_by_id(uint32_t anc_id){
	auto anchor = this->anchors.begin();
	bool found = false;
	while(anchor != this->anchors.end() && !found){
		found = anchor->id == anc_id;
		++anchor;
	}
	if(!found){
		AnchorData new_anc;
		new_anc.id = anc_id;
		this->anchors.push_back(new_anc);
		anchor = this->anchors.end();
	}
	return anchor;
}

void UwbNode::add_can_data(uint8_t* can_data, uint8_t can_dlc){

	uint8_t frame_data[can_dlc] = {};
	memcpy(frame_data, can_data, can_dlc);

	uint32_t anchor_id = 0;
	uint8_t data_type = 0;
	float f_data = 0.0;
	if(can_dlc == 8){
		ROS_INFO("Legacy UWB frame");
		// this is legacy data
		data_type = UWB_LEGACY;
		// uint8_t frame_type = can_data[0];
		anchor_id  = (uint32_t)frame_data[1];
	}else{
		// new frame format
		anchor_id = (uint32_t)frame_data[0]; // which anchor is this about?
		data_type = frame_data[1];      // what kind of anchor data is this?	
	}
	// memcpy(&f_data, frame_data+2, 4); // copy the data, which is almost always a float

	auto anchor = this->get_anchor_by_id(anchor_id); // get existing anchor, or append new anchor
	ROS_INFO("Got anchor %x", anchor->id);
  switch (data_type) {
	  case UWB_TIMESTAMP:
      anchor->timestamp = frame_data[2] | frame_data[3] << 8 | frame_data[4] << 16 | frame_data[5] << 24;
      break;
    case UWB_X:
      memcpy(&(anchor->x), frame_data + 2, 4);
      break;
    case UWB_Y:
      memcpy(&(anchor->y), frame_data + 2, 4);
      break;
    case UWB_Z:
      memcpy(&(anchor->z), frame_data + 2, 4);
      break;
    case UWB_DISTANCE:
      memcpy(&(anchor->distance), frame_data + 2, 4);
      break;
    case UWB_RX_POWER:
      memcpy(&(anchor->rx_power), frame_data + 2, 4);
      break;
    case UWB_FP_POWER:
      memcpy(&(anchor->fp_power), frame_data + 2, 4);
      break;
    case UWB_FP_SNR:
      memcpy(&(anchor->fp_snr), frame_data + 2, 4);
      break;
    case UWB_RANGING_DONE:
      anchor->data_ready = true;
    case UWB_LEGACY:
      anchor->distance = f_data;
      anchor->data_ready = true;
      break;
    default:
      break;
  }

}
/**
 * @param uwb_msgs an array of UwbData types
 * @param max_size the max number of UwbData messages to populate
 * @return how many UwbData messages were actually populated
 */
int UwbNode::get_msgs_from_anchors(hwctrl::UwbData* uwb_msgs, int max_size){
	ROS_INFO("Making messages from anchor data...");
	if(max_size > this->anchors.size()){
		max_size = this->anchors.size();
	}
	int retval = 0;
  //AnchorData* anc_temp = this->anchors.data();
  //for(int i = 0; i < this->anchors.size(); i++){
	for(auto anc = this->anchors.begin(); anc != this->anchors.end(); ++anc){
		//AnchorData* anc = &(anc_temp[i]);
		if(anc->data_ready){
		// data is ready, make a new UwbData message
		hwctrl::UwbData msg;
		msg.node_id = this->id;
		msg.anchor_id = anc->id;
		msg.anchor_x = anc->x;
		msg.anchor_y = anc->y;
		msg.anchor_z = anc->z;
		msg.distance = anc->distance;
		msg.rx_power = anc->rx_power;
		 msg.fp_power = anc->fp_power;
		msg.fp_snr   = anc->fp_snr;
		uwb_msgs[retval] = msg;
		anc->data_ready = false;
		retval++;
    }
  }
  return retval;
}
