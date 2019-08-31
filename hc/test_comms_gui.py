import serial
from ctypes import *
import time
import tkinter as tk
from tkinter import font
import threading
import _thread
import sys

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
	"deposition load-cell",
	"excavation load-cell",
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


#===============================================================================
#===============================================================================
class Application(tk.Frame):
	poll_sensors_period = 0.5
	def __init__(self, master=None):
		tk.Frame.__init__(self, master)
		self.top_level = self.winfo_toplevel()
		self.top_level.title('Hardware Controller Hack-Script')
		self.grid()
		self.createWidgets()
		self.teensy = None
		self.debug 	= None
		self.is_paused = True
		self.debug_is_paused = True

	def createWidgets(self):
		self.connection_frame 	= tk.LabelFrame(self, text='Hardware Controller Connection')
		self.values_frame_main 	= tk.LabelFrame(self, text='Sensor Values')
		self.values_frame 		= tk.Frame(self.values_frame_main)
		self.values_frame_2 	= tk.Frame(self.values_frame_main)
		self.set_frame_1		= tk.LabelFrame(self, text='Set Outputs')
		self.debug_frame 		= tk.LabelFrame(self, text='Debugging Interface')

		self.connection_frame.grid(column=0, row=0, sticky=tk.N+tk.S)
		self.debug_frame.grid(column=1, row=0, sticky=tk.W)
		self.values_frame_main.grid(column=1, row=1, sticky=tk.W+tk.E)
		self.values_frame.grid(column=0, row=0)
		self.values_frame_2.grid(column=1, row=0)
		self.set_frame_1.grid(column=0, row=1, sticky=tk.N+tk.S)

		# CONNECTION FRAME
		tk.Label(self.connection_frame, text="COM Port").grid(column=0, row=0, padx=10, pady=10)

		self.com_port_entry = tk.Entry(self.connection_frame, width=10)
		self.com_port_entry.grid(column=1, row=0, padx=10, pady=10)

		self.connectBtn = tk.Button(self.connection_frame)
		self.connectBtn['text'] = "CONNECT"
		self.connectBtn['command'] 	= self.open_com_port
		self.connectBtn.grid(column=2, row=0, padx=10, pady=10)

		self.test_comms_btn = tk.Button(self.connection_frame)
		self.test_comms_btn['text'] = 'TEST COMMS'
		self.test_comms_btn['command'] = self.test_comms
		self.test_comms_btn.grid(column=0, row=1, padx=10, pady=10)

		self.round_trip_label = tk.Label(self.connection_frame, text='---')
		self.round_trip_label.grid(column=1, row=1, padx=10, pady=10)

		self.pause_btn = tk.Button(self.connection_frame)
		self.pause_btn['text'] = "RESUME"
		self.pause_btn['command'] = self.toggle_paused
		self.pause_btn.grid(column=2, row = 2, padx=5, pady=5)

		# self.resume_btn = tk.Button(self.connection_frame)
		# self.resume_btn['text'] = "RESUME"
		# self.resume_btn['command'] = self.toggle_paused
		# self.resume_btn.grid(column=2, row = 2, , padx=5, pady=5)


		# VALUES FRAME 1
		self.value_btns 	= []
		self.value_labels 	= []
		self.time_labels 	= []
		start_row 	= 0
		col 		= 0
		for i in range(0, int(len(VALUE_LUT)/2)):
			self.value_btns.append(tk.Label(self.values_frame, text=VALUE_LUT[i]) ) 
			self.value_labels.append(tk.Label(self.values_frame, text="0.0"))
			self.time_labels.append(tk.Label(self.values_frame, text="0") )
			
			self.value_btns[i].grid(column=col, row=start_row+i, padx=5, pady=0, ipady=0, sticky=tk.E)
			self.value_labels[i].grid(column=col+1, row=start_row+i, padx=5, pady=0, ipady=0, sticky=tk.W)  
			self.time_labels[i].grid(column=col+2, row=start_row+i, padx=5, pady=0, ipady=0, sticky=tk.W) 

		# VALUES FRAME 2
		for i in range(int(len(VALUE_LUT)/2), len(VALUE_LUT)):
			self.value_btns.append(tk.Label(self.values_frame_2, text=VALUE_LUT[i]) ) 
			self.value_labels.append(tk.Label(self.values_frame_2, text="0.0"))
			self.time_labels.append(tk.Label(self.values_frame_2, text="0") )
			
			self.value_btns[i].grid(column=col, row=start_row+i, padx=5, pady=0, ipady=0, sticky=tk.E)
			self.value_labels[i].grid(column=col+1, row=start_row+i, padx=5, pady=0, ipady=0, sticky=tk.W)  
			self.time_labels[i].grid(column=col+2, row=start_row+i, padx=5, pady=0, ipady=0, sticky=tk.W) 

		# SET FRAME 1
		self.set_labels 	= []
		self.set_entries 	= []
		self.set_btns 		= []
		for i in range(0, len(OUTPUT_LUT)):
			self.set_labels.append(tk.Label(self.set_frame_1, text=OUTPUT_LUT[i]))
			self.set_entries.append(tk.Entry(self.set_frame_1, width=6))
			# self.set_btns.append(tk.Button(self.set_frame_1, text='SET'))

			self.set_labels[i].grid(column=0, row=i, padx=5, pady=0, sticky=tk.E)
			self.set_entries[i].grid(column=1, row=i, padx=5, pady=0)
			# self.set_btns[i].grid(column=2, row=i, padx=5, pady=0)

		self.update_outputs_btn = tk.Button(self.set_frame_1, text='Update Outputs', command=self.update_outputs)
		self.update_outputs_btn.grid(column=1, row=len(OUTPUT_LUT), padx=5, pady=0)

		# DEBUG FRAME
		self.debug_com_port 	= tk.Entry(self.debug_frame, width=8)
		self.debug_com_port.grid(column=0, row=0, padx=10, pady=5, sticky=tk.E)
		self.begin_debug_btn 	= tk.Button(self.debug_frame, text='BEGIN', command=self.begin_debug)
		self.begin_debug_btn.grid(column=1, row=0, padx=10, pady=5, sticky=tk.W)
		self.mono_font = font.Font(family='Courier', size=10)
		self.debug_text_box 	= tk.Text(self.debug_frame, height=20, width=80, bg='black', fg='green', font=self.mono_font)
		self.debug_text_box.grid(column=1, row=2, padx=5, pady=0, sticky=tk.W)
		# self.debug_text = tk.StringVar()
		# self.debug_text_area 	= tk.Label(self.debug_frame, height=20, width=40, anchor=tk.SW, textvariable=self.debug_text, bg='black', fg='green', justify=tk.LEFT, font=self.mono_font)
		# self.debug_text_area.grid(column=1, row=2, padx=5, pady=0, sticky=tk.W)
		self.pause_debug_btn 	= tk.Button(self.debug_frame, text='PAUSE', command=self.toggle_debug_paused)
		self.pause_debug_btn.grid(column=1, row=1, padx=5, pady=5, sticky=tk.W)



		self.QUIT = tk.Button(self, text="QUIT", fg="red",command=self.end_all)
		self.QUIT.grid(column=0, row=2, padx=10, pady=10)

	def begin_debug(self):
		debug_com = self.debug_com_port.get()
		if len(debug_com) >= 4:
			self.debug = serial.Serial(debug_com)
			self.debug.baudrate = 115200
			self.debug_is_paused = False
		else:
			self.debug_is_paused = True

	def check_debug(self):
		if not (self.debug == None) and not (self.debug_is_paused):
			if self.debug.in_waiting > 0:
				line = self.debug.read(self.debug.in_waiting)
				line = line.decode(encoding='utf-8')
				self.debug_text_box.insert(tk.END, line, None)
				self.debug_text_box.yview_scroll(line.count('\n'), tk.UNITS)
				# line = self.debug_text.get() + line
				# self.debug_text.set(line)
				# self.debug_text_area.config(text=)

	def toggle_debug_paused(self):
		self.debug_is_paused = (not self.debug_is_paused)
		if self.debug_is_paused:
			self.pause_debug_btn.config(text="RESUME")
		else:
			self.pause_debug_btn.config(text="PAUSE")
	def update_outputs(self):
		if not self.teensy == None:
			cmd 	= 'SET'
			cmd_str = ''
			for i in range(0, len(OUTPUT_LUT)):
				id_n 	= i
				try:
					s = self.set_entries[i].get()
					if len(s) > 0: 		# don't send update if no number is entered
						value 	= float(s)
						cmd_str += '{:d},{:.6f} '.format(id_n, value)
				except ValueError:
					value 	= 0.0
				
			rpy_type, rpy_body, round_trip_time = send_cmd('SET', cmd_str, self.teensy)


	def toggle_paused(self):
		self.is_paused = (not self.is_paused)
		if self.is_paused:
			self.pause_btn.config(text="RESUME")
		else:
			self.pause_btn.config(text="PAUSE")

	def print_entry(self):
		pass
		# print(self.com_port_entry.get())

	def open_com_port(self):
		self.teensy = serial.Serial(self.com_port_entry.get(), timeout=5)
		self.teensy.baudrate = 115200 	# maximum available option, communication will happen at full USB speed
		stuff = send_cmd('SYNC', '', self.teensy)


	def poll_sensors(self):
		if not self.teensy == None and not self.is_paused:
			# print('polling sensors')
			cmd_str = ''
			for n_id in range(0, len(VALUE_LUT)):
				cmd_str += str(n_id) + ' '
			rpy_type, rpy_body, round_trip_time = send_cmd('READ', cmd_str, self.teensy)

			values = parse_reply_L(rpy_body, VALUE_LUT)

			for l in values:
				self.value_labels[l[0]].config(text='{:.6f}'.format(l[2]))
				self.time_labels[l[0]].config(text='{:.3f}'.format(l[3]/1000000.0))

			# for i in range(0, len(values)):
			# 	self.value_labels[i].config(text='{:.6f}'.format(values[i][2]))
			# 	self.time_labels[i].config(text='{:.3f}'.format(values[i][3]/1000000.0))

	def set_output(self, item_id, item_value):
		if not self.teensy == None:
			cmd_str = '{:d},{:.6f} '.format(item_id,item_value)
			rpy_type, rpy_body, round_trip_time = send_cmd('SET', cmd_str, self.teensy)

	def test_comms(self):
		if not self.teensy == None:
			temp, temp_2, round_trip_t = send_cmd('TEST', '', self.teensy)
			self.round_trip_label.config(text='Round-trip t: {:.3f} ms'.format(round_trip_t*1000.0))


	def stop(self):
		self._stop_event.set()

	def is_stopped(self):
		return self._stop_event.is_set()

	def end_all(self):
		self.master.destroy()

#===============================================================================
#===============================================================================
def float_to_bytes(f):
	dest 	= bytes(4) 			# allocating empty memory
	temp 	= memmove(dest, addressof(c_float(float(f))), 4)
	return dest

#===============================================================================
def bytes_to_float(data):
	value 	= c_float(0.0) 	# allocate memory
	temp 	= memmove(addressof(value), data, 4)
	return value.value

#===============================================================================
def bytes_to_int(data):
	value 	= c_int(0)
	temp 	= memmove(addressof(value), data, 4)
	return value.value

#===============================================================================
def int_to_bytes(n):
	dest 	= bytes(4)
	temp 	= memmove(dest, addressof(c_int(int(n))), 4)
	return dest


#===============================================================================
def chksum(data):
	s = 0
	for b in data:
		s = s + b

	return int(s)

#===============================================================================
def get_time():
	return int( (time.time()-start_time)*1000000 )

#===============================================================================
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

#===============================================================================
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
#===============================================================================
def parse_reply(rpy, LUT):
	values = []
	for i in range(0, len(rpy), ITEM_LENGTH_L):
		item_id = rpy[i]
		temp = bytearray(4)
		temp[0] = rpy[i+1]
		temp[1] = rpy[i+2]
		temp[2] = rpy[i+3]
		temp[3] = rpy[i+4]
		value 	= bytes_to_float(bytes(temp))
		values.append([item_id, LUT[item_id], value, 0.0])
	return values

#===============================================================================
def parse_reply_L(rpy, LUT):
	values = []
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
		values.append([item_id, LUT[item_id], value, t_stamp])
	return values

#===============================================================================
def send_cmd(cmd, line, teensy):
	line 			= line.strip().split(' ')
	items 			= len(line)
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
			for i in range(0, len(line), 1):
				print(line[i])
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

			for b in packet:
				print(int(b))

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

		# print(len(data))
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
			pass
			# print_reply(rpy_body, OUTPUT_LUT)

		elif rpy_type == CMD_READ_VALUES:
			pass
			# print_reply_L(rpy_body, VALUE_LUT)
		elif rpy_type == CMD_SYNC_TIME:
			print("Time sync successful")

		else:
			print("error occured: " + str(hex(rpy_type)))

		round_trip_time = end_t-start_t

		return (rpy_type, rpy_body, round_trip_time)

#===============================================================================
def main():

	root = tk.Tk()
	app = Application(master=root)
	ref_t 	= time.time()
	ellapsed_t = 0

	done = False

	while not done:
		ellapsed_t = time.time() - ref_t

		if ellapsed_t >= app.poll_sensors_period:
			app.poll_sensors()
			ref_t = time.time()

		app.update_idletasks()
		app.update()
		app.check_debug()
	
		# do things
		# print("TEST COMMS\t: TEST")
		# print("SET OUTPUTS\t: SET  ID_0,val_0 ID_1,val_1 ...")
		# print("READ VALUES\t: READ ID_0 ID_1 ...")
		# print("SYNC TIME\t: SYNC")
		

if __name__ == '__main__':
	main()