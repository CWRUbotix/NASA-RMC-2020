#include <uwb.h>

AnchorData* UwbNode::get_anchor_by_id(uint32_t id){
  auto i = this->anchors.begin();
	AnchorData* anchor = NULL;
	while(i != this->uwb_nodes.end() && i->id != id){
		++i; // increment to next UWB Node
		anchor = &(*i);
	}
	return node;
}

void UwbNode::add_can_data(uint8_t* can_data){

  uint32_t anchor_id = can_data[0]; // which anchor is this about?
  int data_type = can_data[1];      // what kind of anchor data is this?
  uint8_t* frame_data = can_data + UWB_CAN_HDDR_SIZE;

  float f_data = 0.0;
  memcpy(&f_data, frame_data, 4); // floats are size 4

  AnchorData* anchor = this->get_anchor_by_id(anchor_id);
  if(anchor == NULL){
    AnchorData anc;
    anc.id = anchor_id;
    this->anchors.push_back(anc); // puts a copy into the vector
    anchor = this->get_anchor_by_id(anchor_id); // get pointer to the object in the vector
  }
  switch (data_type) {
    case UWB_TIMESTAMP:
      anchor->timestamp = frame_data[0] | frame_data[1] << 8 | frame_data[2] << 16 | frame_data[3] << 24;
      break;
    case UWB_X:
      memcpy(&(anchor->x), frame_data, 4);
      break;
    case UWB_Y:
      memcpy(&(anchor->y), frame_data, 4);
      break;
    case UWB_Z:
      memcpy(&(anchor->z), frame_data, 4);
      break;
    case UWB_DISTANCE:
      memcpy(&(anchor->distance), frame_data, 4);
      break;
    case UWB_RX_POWER:
      memcpy(&(anchor->rx_power), frame_data, 4);
      break;
    case UWB_FP_POWER:
      memcpy(&(anchor->fp_power), frame_data, 4);
      break;
    case UWB_FP_SNR:
      memcpy(&(anchor->fp_snr), frame_data, 4);
      break;
    case UWB_RANGING_DONE:
      anchor->data_ready = true;
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
  if(max_size > this->anchors.size()){
    max_size = this->anchors.size();
  }
  int retval = 0;
  AnchorData* anc_temp = this->anchors.data();
  for(int i = 0; i < max_size; i++){
    AnchorData* anc = &(anc_temp[i]);
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
      uwb_msgs[i] = msg;
      anc->data_ready = false;
      retval++;
    }
  }
  return retval;
}
