#include <hwctrl.h>

////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
InterfaceType get_if_type(std::string type_str){
	if(type_str.compare("can") == 0){
		return IF_CANBUS;
	}else if(type_str.compare("uart") == 0){
		return IF_UART;
	}else if(type_str.compare("spi") == 0){
		return IF_SPI;
	}else if(type_str.compare("gpio") == 0){
		return IF_GPIO;
	}else{
		return IF_NONE;
	}
}

DeviceType get_device_type(std::string type_str){
	if      (type_str.compare("vesc") 		== 0){
		return DEVICE_VESC;
	}else if(type_str.compare("sabertooth") == 0){
		return DEVICE_SABERTOOTH;
	}else if(type_str.compare("uwb") 		== 0){
		return DEVICE_UWB;
	}else if(type_str.compare("quad_enc") 	== 0){
		return DEVICE_QUAD_ENC;
	}else if(type_str.compare("limit") 	== 0){
		return DEVICE_LIMIT_SW;
	}else{
		return DEVICE_NONE;
	}
}

/*
// OBSOLTETE
void can_rx_sub_callback(const boost::shared_ptr<hwctrl::CanFrame>& frame){
	uint32_t rx_id = (uint32_t)frame->can_id;

	if((rx_id & ~0xFF) != 0){
		// we can assume this is a VESC message
		uint8_t cmd = (uint8_t)(rx_id >> 8);
		int8_t id 	= (int8_t)(rx_id & 0xFF);
		uint8_t* frame_data = frame->data.data(); // get the back-buffer of the vector
		ROS_INFO("VESC %d sent msg type %d", id, cmd);
		HwMotor* vesc = this->get_vesc_from_can_id(id);
		switch(cmd){
			case CAN_PACKET_STATUS:{
				if(vesc == NULL){
					break;
				}
				VescData* vesc_data = &(vesc->vesc_data);

				// store a ROS timestamp
				vesc_data->timestamp = ros::Time::now();

				// store data with the correct vesc object
				fill_data_from_status_packet(frame_data, vesc_data);
				break;
			}
			case CAN_PACKET_FILL_RX_BUFFER:{
				if(id != 0x00){
						// this is not intended for us (our canID is 0)
					break;
				}
				// copy the CAN data into the corresponding vesc rx buffer
				memcpy(vesc->vesc_data.vesc_rx_buf + frame_data[0], frame_data + 1, frame->can_dlc - 1);
				break;
			}
			case CAN_PACKET_PROCESS_RX_BUFFER:{
				if(id != 0x00){
					// not intended for us
					break;
				}
				if(vesc == NULL){
					// we know of no VESC with the given CAN ID
					break;
				}
				int ind = 0;
				uint16_t packet_len;
				uint16_t crc;
				int vesc_id = frame_data[ind++]; // which vesc sent the data
				int n_cmds 	= frame_data[ind++]; // how many commands
				packet_len 	= (frame_data[ind++] << 8);
				packet_len 	|= frame_data[ind++];
				crc 		= (frame_data[ind++] << 8);
				crc 		|= frame_data[ind++];

				uint16_t chk_crc = crc16(vesc->vesc_data.vesc_rx_buf, packet_len);
				// ROS_INFO("PROCESSING RX BUFFER\nPacket Len:\t%d\nRcvd CRC:\t%x\nComputed CRC:\t%x", packet_len, crc, chk_crc);

				if(crc != chk_crc){
					ROS_INFO("Error: Checksum doesn't match");
					// error in transmission
					break;
				}

				ind = 0;
				int comm_cmd = vesc->vesc_data.vesc_rx_buf[ind++];
				switch(comm_cmd){
					case COMM_GET_VALUES:{
						vesc->vesc_data.timestamp 	= ros::Time::now();
						vesc->vesc_data.motor_type 	= "vesc";
						vesc->vesc_data.can_id 		  = vesc_id;

						fill_data_from_buffer(vesc->vesc_data.vesc_rx_buf, &(vesc->vesc_data));

						break;}
					}

				break;
			}
				// indicates the end of the cmd buffer
		}
	}else{
		// we can assume this is either UWB message or Quad Encoder message
		rx_id = rx_id & CAN_SFF_MASK;
		uint8_t type = frame->data[0];
		if(type == 0 || type == 0xFF){
			// yes this is an UWB msg
			hwctrl::UwbData uwb_msg;
			anchor_frames++;
			float distance;
			int16_t confidence;
			memcpy(&distance, frame->data+2, 4);
			memcpy(&confidence, frame->data+6, 2);
			// ROS_INFO("Distance from node %d to anchor %d: %.3f m", rx_id, dist_data.anchor_id, dist_data.distance);
			uwb_msg.timestamp 	= ros::Time::now();
			uwb_msg.node_id 	= rx_id;
			uwb_msg.anchor_id 	= frame->data[1];
			uwb_msg.distance 	= distance;
			uwb_msg.confidence = confidence;

			uwb_pub.publish(uwb_msg);
		}else{
			// quad encoder msg
		}
	}
} */
