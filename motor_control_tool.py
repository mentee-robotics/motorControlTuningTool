import struct
import serial
import matplotlib.pyplot as plt
import enum
import serial.tools.list_ports
import threading
import time

class MotorMode(enum.Enum):
    STOP = 0
    SINE = 1
    SQUARE = 2

class InjectedParams(enum.Enum):
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2
    CURRENT = 3

class MessageTypes(enum.Enum):
    MOVE = 0X01
    CURRENT = 0X02
    IMPEDANCE = 0X03

class ArduinoController:
    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(self.port, baudrate=3000000, timeout=1, write_timeout=1)

        self.desired_position_list = []
        self.actual_position_list = []

        self.desired_velocity_list = []
        self.actual_velocity_list = []

        self.desired_ff_list = []
        self.actual_torque_list = []
        self.actual_torque_filtered_list = []

        self.actual_current_list = []
        self.desired_current_list = []

        self.injection_params = None
        self.ampl = 0
        self.freq = 0

        self.bw_i = 0
        self.bw_gain = 0
        self.bw_ki = 0

        self.imp_kp = 0
        self.imp_kd = 0

        self.write_flag = False
        self.message_type = None

        self.mode = None
        self.is_running = False
        self.lock = threading.Lock()
        self.i = 0
        self.fig, self.ax = plt.subplots(figsize=(12, 9))
        self.samples_to_plot = 100
        self.start_time = time.time()
        self.read_thread = threading.Thread(target=self.serial_manager)
        self.read_thread.start()
       
    def serial_manager(self):
        while True:
            print("Serial manager") 
    
            if not self.write_flag:
                if not self.is_running:
                    continue
                print("Reading from serial")
                self.read_state_machine()

            elif self.write_flag:
                print("Writing to serial")
                self.send_state_machine()
                print("after write state machine")
                self.write_flag = False
                print(f" writing? {self.write_flag}")


    def read_state_machine(self):
        while True:
            if not self.is_running:
                continue

            if self.injection_params == InjectedParams.POSITION.value:
                # print("Reading position")
                self.read_position()
                self.plot_data(data_series_1=self.desired_position_list,
                            data_series_2=self.actual_position_list,
                            samples_to_plot=self.samples_to_plot,
                            title="Position",
                            xlabel="Sample Number",
                            series1_label="desired Position",
                            series2_label="actual Position")

            elif self.injection_params == InjectedParams.VELOCITY.value:
                self.read_velocity()
                self.plot_data(data_series_1=self.desired_velocity_list,
                            data_series_2=self.actual_velocity_list,
                            samples_to_plot=self.samples_to_plot,
                            title="Velocity",
                            xlabel="Sample Number",
                            series1_label="Desired Velocity",
                            series2_label="Actual Velocity")

            elif self.injection_params == InjectedParams.TORQUE.value:
                self.read_torque()
                self.plot_data(data_series_1=self.desired_ff_list,
                            data_series_2=self.actual_torque_list,
                            data_series_3=self.actual_torque_filtered_list,  
                            samples_to_plot=self.samples_to_plot,
                            title="Torque",
                            xlabel="Sample Number",
                            series1_label="Desired Torque",
                            series2_label="Actual Torque",
                            series3_label="Filtered Actual Torque")  

            elif self.injection_params == InjectedParams.CURRENT.value:
                self.read_current()
                self.plot_data(data_series_1=self.desired_current_list,  
                            data_series_2=self.actual_current_list,   
                            samples_to_plot=self.samples_to_plot,
                            title="Current",
                            xlabel="Sample Number",
                            series1_label="Desired Current",
                            series2_label="Actual Current")


    def plot_data(self, data_series_1, data_series_2=None, data_series_3=None, samples_to_plot=100, 
                    title="", series1_label="", series2_label="", series3_label="", xlabel=""):
        # Clear the previous plot
        self.start_time = time.time()
        self.ax.clear()

        # Ensure we have enough data to plot
        available_samples_1 = min(samples_to_plot, len(data_series_1))
        if available_samples_1:
            self.ax.plot(range(len(data_series_1)-available_samples_1, len(data_series_1)), 
                        data_series_1[-available_samples_1:], label=series1_label)

        if data_series_2 is not None:
            available_samples_2 = min(samples_to_plot, len(data_series_2))
            if available_samples_2:
                self.ax.plot(range(len(data_series_2)-available_samples_2, len(data_series_2)), 
                            data_series_2[-available_samples_2:], label=series2_label)

        if data_series_3 is not None:
            available_samples_3 = min(samples_to_plot, len(data_series_3))
            if available_samples_3:
                self.ax.plot(range(len(data_series_3)-available_samples_3, len(data_series_3)), 
                            data_series_3[-available_samples_3:], label=series3_label)


        self.ax.set_title(title)
        self.ax.set_xlabel(xlabel)
        
        # Update the x-axis limits
        if len(data_series_1) >= available_samples_1:
            self.ax.set_xlim(len(data_series_1)-available_samples_1, len(data_series_1))
        self.ax.legend(loc ='upper right')

        self.fig.canvas.draw()
        # print(f"delta time is: {time.time() - self.start_time}")



    def read_position(self):
        try:
            line = self.ser.readline().decode('ascii').strip()
            desired_position = float(line[71:75])
            actual_position = float(line[75:78])
            self.desired_position_list.append(desired_position)
            self.actual_position_list.append(actual_position)
            print(f"desired position: {desired_position}, actual position: {actual_position}")
        except ValueError:
            pass

    def read_velocity(self):
        try:
            line = self.ser.readline().decode('ascii').strip()
            desired_velocity = float(line[71:75])
            actual_velocity = float(line[75:78])
            self.desired_velocity_list.append(desired_velocity)
            self.actual_velocity_list.append(actual_velocity)
        except ValueError:
            pass

    def read_torque(self):
        try:
            line = self.ser.readline().decode('ascii').strip()
            desired_ff = float(line[71:75])
            actual_torque = float(line[75:78])
            actual_torque_filtered = float(line[78:81])  # assuming the next float starts immediately after the previous one
            self.desired_ff_list.append(desired_ff)
            self.actual_torque_list.append(actual_torque)
            self.actual_torque_filtered_list.append(actual_torque_filtered)
        except ValueError:
            pass

    def read_current(self):
        try:
            line = self.ser.readline().decode('ascii').strip()
            desired_current = float(line[71:75])
            actual_current = float(line[75:78])
            self.desired_current_list.append(desired_current)
            self.actual_current_list.append(actual_current)
        except ValueError:
            pass
    

    def send_state_machine(self):
        if self.message_type == MessageTypes.MOVE.value:
            print("Sending move")
            self.send_move(self.mode, self.injection_params, self.ampl, self.freq)
        elif self.message_type == MessageTypes.CURRENT.value:
            print("Sending current")
            self.send_current(self.bw_i, self.bw_gain, self.bw_ki)
        elif self.message_type == MessageTypes.IMPEDANCE.value:
            print("Sending impedance")  
            self.send_impedance(self.imp_kp, self.imp_kd)
        # Lock is released when we exit the 'with' block


    def send_move(self, mode, injected_param, ampl, freq):  # mode - uint8_t, injected_param - uint8_t, ampl - float, freq - float
        packet_to_send = bytearray(13)
        print(f"Mode: {mode}, injected_param: {injected_param}, ampl: {ampl}, freq: {freq}")
        packet_to_send[0] = MessageTypes.MOVE.value
        packet_to_send[1] = mode
        packet_to_send[2] = injected_param
         
        ampl_bytes = struct.pack('f', float(ampl))
        for i, byte in enumerate(ampl_bytes):
            packet_to_send[3+i] = byte
        
        freq_bytes = struct.pack('f', float(freq))
        for i, byte in enumerate(freq_bytes):
            packet_to_send[7+i] = byte
        
        packet_to_send[11] = 0
        packet_to_send[12] = 0
        print("send move before write")
        try:
            print("im trying")
            self.ser.write(packet_to_send)
        except Exception as e:
            print(f"Error during write: {e}")
        print("after write")
        self.is_running = True
        print(f"Sent move command: {packet_to_send}")

  
    def send_current(self, bw_i, bw_gain, bw_ki):
        print("send current")
        packet_to_send = bytearray(13)
        packet_to_send[0] = MessageTypes.CURRENT.value

        # Packing and inserting bw_i
        bw_i_bytes = struct.pack('f', float(bw_i))
        for i, byte in enumerate(bw_i_bytes):
            packet_to_send[1+i] = byte

        # Packing and inserting bw_gain
        bw_gain_bytes = struct.pack('f', float(bw_gain))
        for i, byte in enumerate(bw_gain_bytes):
            packet_to_send[5+i] = byte

        # Packing and inserting bw_ki
        bw_ki_bytes = struct.pack('f', float(bw_ki))
        for i, byte in enumerate(bw_ki_bytes):
            packet_to_send[9+i] = byte

        self.ser.write(packet_to_send)
        print(f"Sent current command: {packet_to_send}")

    def send_impedance(self, imp_kp, imp_kd):
        print("send impedance")
        packet_to_send = bytearray(13)  # Fixed size of 13 bytes
        packet_to_send[0] = MessageTypes.IMPEDANCE.value  # Assuming you have this enum value defined

        # Packing and inserting imp_kp
        imp_kp_bytes = struct.pack('f', float(imp_kp))
        for i, byte in enumerate(imp_kp_bytes):
            packet_to_send[1+i] = byte

        # Packing and inserting imp_kd
        imp_kd_bytes = struct.pack('f', float(imp_kd))
        for i, byte in enumerate(imp_kd_bytes):
            packet_to_send[5+i] = byte

        self.ser.write(packet_to_send)
        print(f"Sent impedance command: {packet_to_send}")


    
    def stop(self):
        self.mode = MotorMode.STOP.value
        self.send_state_machine(MessageTypes.MOVE)
        self.is_running = False






