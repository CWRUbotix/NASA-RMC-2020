import serial
from ctypes import *
import time
import tkinter as tk

HEADER_LEN 		= 5
CMD_SET_OUTPUTS = 0x51
CMD_READ_VALUES = 0x52
CMD_TEST 		= 0x53
CMD_SYNC_TIME 	= 0x54
ITEM_LENGTH 	= 5
ITEM_LENGTH_L 	= 9

TEST_DATA 		= int(0xA5A5A5A5)

OUTPUT_LUT 		= [
	"Port Drive",
	"Starboard Drive",
	"deposition winch",
	"bucket conveyor belt drive",
	"bucket conveyor translation",
	"bucket conveyor attitude",
	"port-side looky",
	"starboard-side looky",
	"bucket conveyor RGB LEDs"]

VALUE_LUT 		= [
	"Port Drive Encoder",
	"Starboard Drive encoder",
	"deposition winch encoder",
	"bucket conveyor belt drive encoder",
	"bucket conveyor translation encoder",
	"bucket conveyor attitude pot port",
	"bucket conveyor attitude pot starboard",
	"port-side looky encoder",
	"starboard-side looky encoder",
	"deposition port-side load-cell",
	"deposition starboard-side load-cell",
	"gyroscope_0, X axis",
	"gyroscope_0, Y axis",
	"gyroscope_0, Z axis",
	"accelerometer_0, X axis",
	"accelerometer_0, Y axis",
	"accelerometer_0, Z axis",
	"gyroscope_1, X axis",
	"gyroscope_1, Y axis",
	"gyroscope_1, Z axis",
	"accelerometer_1, X axis",
	"accelerometer_1, Y axis",
	"accelerometer_1, Z axis",
	"deposition lower limit switch",
	"deposition upper limit switch",
	"excavation fore limit switch",
	"excavation aft limit switch",
	"bucket conveyor lower limit switch",
	"bucket conveyor upper limit switch",
	"E-Stop state",
	"angular displacement, X-axis",
	"angular displacement, Y-axis",
	"angular displacement, Z-axis"]

sync_time = 0
start_time = time.time()


def float_to_bytes(f):
	dest 	= bytes(4) 			# allocating empty memory
	temp 	= memmove(dest, addressof(c_float(float(f))), 4)
	return dest

def bytes_to_float(data):
	value 	= c_float(0.0) 	# allocate memory
	temp 	= memmove(addressof(value), data, 4)
	return value.value

def bytes_to_int(data):
	value 	= c_int(0)
	temp 	= memmove(addressof(value), data, 4)
	return value.value

def int_to_bytes(n):
	dest 	= bytes(4)
	temp 	= memmove(dest, addressof(c_int(int(n))), 4)
	return dest


def chksum(data):
	s = 0
	for b in data:
		s = s + b

	return int(s)

def get_time():
	return int( (time.time()-start_time)*1000000 )

def print_reply(rpy, LUT):
	for i in range(0, len(rpy), ITEM_LENGTH):
		item_id = rpy[i]
		temp = bytearray(4)
		temp[0] = rpy[i+1]
		temp[1] = rpy[i+2]
		temp[2] = rpy[i+3]
		temp[3] = rpy[i+4]
		value = bytes_to_float(bytes(temp))
		print("{:<7}\t{:<25}\t{:<10}".format(item_id, LUT[item_id], value))

def print_reply_L(rpy, LUT):
	print("{:<7}\t{:<25}\t{:<10}\t{:<10}".format("ID", "NAME", "VALUE", "TIME STAMP"))
	for i in range(0, len(rpy), ITEM_LENGTH_L):
		item_id = rpy[i]
		temp = bytearray(4)
		temp[0] = rpy[i+1]
		temp[1] = rpy[i+2]
		temp[2] = rpy[i+3]
		temp[3] = rpy[i+4]
		value 	= bytes_to_float(bytes(temp))
		temp[0] = rpy[i+5]
		temp[1] = rpy[i+6]
		temp[2] = rpy[i+7]
		temp[3] = rpy[i+8]
		t_stamp = bytes_to_int(bytes(temp)) - sync_time
		print("{:<7}\t{:<25}\t{:<10}\t{:<20}".format(item_id, LUT[item_id], value, t_stamp))

def main():

	com_port = input("COM Port\t: ")

	teensy = serial.Serial(com_port, timeout=5)
	teensy.baudrate = 115200 	# maximum available option, communication will happen at full USB speed

	done = False

	while not done:
		# do things
		print("TEST COMMS\t: TEST")
		print("SET OUTPUTS\t: SET  ID_0,val_0 ID_1,val_1 ...")
		print("READ VALUES\t: READ ID_0 ID_1 ...")
		print("SYNC TIME\t: SYNC")
		line 			= input(">>> ")
		line 			= line.strip().split(' ')
		items 			= len(line)-1
		cmd 			= line[0].upper()
		bytes_expected 	= 0
		packet_ready 	= False

		if cmd == 'TEST':
			length 		= 4
			packet 		= bytearray(length + HEADER_LEN)
			data 		= TEST_DATA.to_bytes(4, byteorder='big')
			chksum_bytes= chksum(data).to_bytes(2, byteorder='big')
			len_bytes 	= length.to_bytes(2, byteorder='big')

			packet[0] 	= CMD_TEST
			packet[1] 	= len_bytes[0]
			packet[2] 	= len_bytes[1]
			packet[3] 	= chksum_bytes[0]
			packet[4] 	= chksum_bytes[1]

			for i in range(0,len(data)):
				packet[HEADER_LEN + i] = data[i]

			bytes_expected 	= len(packet)
			packet_ready 	= True

		elif cmd == 'SET':
			length 		= items*ITEM_LENGTH 	# 2 strings per item, 5 bytes per item
			packet 		= bytearray(length + HEADER_LEN)
			data 		= bytearray(length)

			j = 0
			for i in range(1, len(line), 1):
				temp 		= line[i].split(',')
				item_id 	= int(temp[0]).to_bytes(1, byteorder='big')
				item_value 	= float_to_bytes(float(temp[1]))
				data[j]   = item_id[0]
				data[j+1] = item_value[0]
				data[j+2] = item_value[1]
				data[j+3] = item_value[2]
				data[j+4] = item_value[3]
				j = j + 5

			chksum_bytes= chksum(data).to_bytes(2, byteorder='big')
			len_bytes 	= length.to_bytes(2, byteorder='big')

			packet[0] 	= CMD_SET_OUTPUTS
			packet[1] 	= len_bytes[0]
			packet[2] 	= len_bytes[1]
			packet[3] 	= chksum_bytes[0]
			packet[4] 	= chksum_bytes[1]

			for i in range(0,len(data)):
				packet[HEADER_LEN + i] = data[i]

			bytes_expected 	= len(packet) 	# true for now, may not be later on
			packet_ready 	= True

		elif cmd == 'READ':
			length 		= items 	# 1 byte per item
			packet 		= bytearray(length + HEADER_LEN)
			data 		= bytearray(length)

			for i in range(1, len(line), 1):
				item_id 	= int(line[i]) #.to_bytes(1, byteorder='big')
				data[i-1] = item_id

			chksum_bytes= chksum(data).to_bytes(2, byteorder='big')
			len_bytes 	= length.to_bytes(2, byteorder='big')

			packet[0] 	= CMD_READ_VALUES
			packet[1] 	= len_bytes[0]
			packet[2] 	= len_bytes[1]
			packet[3] 	= chksum_bytes[0]
			packet[4] 	= chksum_bytes[1]

			print(len(data))
			for i in range(0,len(data)):
				packet[HEADER_LEN + i] = data[i]

			bytes_expected 	= HEADER_LEN + (items*ITEM_LENGTH_L)
			packet_ready 	= True
		elif cmd == 'SYNC':
			length 	= 4
			packet 	= bytearray(length + HEADER_LEN)
			data 	= bytearray(length)

			sync_time = get_time()
			data 	= sync_time.to_bytes(4, byteorder='big')

			chksum_bytes= chksum(data).to_bytes(2, byteorder='big')
			len_bytes 	= length.to_bytes(2, byteorder='big')

			packet[0] 	= CMD_SYNC_TIME
			packet[1] 	= len_bytes[0]
			packet[2] 	= len_bytes[1]
			packet[3] 	= chksum_bytes[0]
			packet[4] 	= chksum_bytes[1]

			for i in range(0,len(data)):
				packet[HEADER_LEN + i] = data[i]

			bytes_expected 	= HEADER_LEN + 4
			packet_ready 	= True
		else:
			print("unknown command")


		if packet_ready and bytes_expected > 0:
			start_t = time.time()
			teensy.write(packet)

			reply 		= teensy.read(5)
			end_t 		= time.time()
			rpy_type 	= reply[0]
			rpy_len 	= (reply[1] << 8) + reply[2]
			rpy_chksum 	= (reply[3] << 8) + reply[4]

			rpy_body 	= teensy.read(rpy_len)

			if rpy_type == CMD_TEST:
				rpy_items = int(len(rpy_body)/ITEM_LENGTH)

			elif rpy_type == CMD_SET_OUTPUTS:
				print_reply(rpy_body, OUTPUT_LUT)

			elif rpy_type == CMD_READ_VALUES:
				print_reply_L(rpy_body, VALUE_LUT)
			elif rpy_type == CMD_SYNC_TIME:
				print("Time sync successful")

			else:
				print("error occured: " + str(hex(rpy_type)))


			print("Approx round-trip time\t: {}".format(end_t-start_t))

if __name__ == '__main__':
	main()