import struct
import serial
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import enum
import serial.tools.list_ports
from motor_control_tool import ArduinoController , InjectedParams, MessageTypes , MotorMode
import time




class App:
    def __init__(self, master):
        # Initialize main properties
        self.master = master
        self.configure_master()
        
        # Initialize Arduino-related attributes
        self.init_arduino_attributes()
        
        # Create GUI elements
        self.create_port_controls()
        self.create_attribute_controls()
        self.create_buttons()
        self.create_graph()
        self.create_sample_input()

        # Start plot updater
        self.master.after(200, self.update_plot)
    
    def configure_master(self):
        """Configure main window properties"""
        self.master.title("MOTOR CALIBRATION GUI")
        # self.master.attributes('-fullscreen', True)
    
    def init_arduino_attributes(self):
        """Initialize Arduino-related attributes."""
        self.arduino = None  # Placeholder for the ArduinoController instance
        self.injection_params = None
        self.mode = None
        self.ampl = 0
        self.freq = 0
        self.bw_i = 0
        self.bw_gain = 0
        self.bw_ki = 0
        self.imp_kp = 0
        self.imp_kd = 0
        self.is_running = False
        self._after_id = None

    def create_port_controls(self):
        """Create controls for port selection."""
        ttk.Label(self.master, text="Select Port:").grid(row=0, column=0)
        self.port_var = tk.StringVar()
        self.ports_dropdown = ttk.Combobox(self.master, textvariable=self.port_var)
        self.ports_dropdown.grid(row=0, column=1)
        self.update_ports()  # Initial population of ports
        ttk.Button(self.master, text="scan ports", command=self.update_ports).grid(row=0, column=2)
        ttk.Button(self.master, text="Connect", command=self.connect_to_port).grid(row=1, column=1)
        self.status_label = ttk.Label(self.master, text="")
        self.status_label.grid(row=2, column=0, columnspan=3)

    def create_attribute_controls(self):
        """Create controls for various attributes."""
        attributes = ['injection_params', 'mode', 'ampl', 'freq', 'bw_i', 'bw_gain', 'bw_ki', 'imp_kp', 'imp_kd']
        for idx, attr in enumerate(attributes, start=3):
            ttk.Label(self.master, text=f"{attr}:").grid(row=idx, column=0)
            setattr(self, f"{attr}_label", ttk.Label(self.master))
            getattr(self, f"{attr}_label").grid(row=idx, column=1)
            
            # Attribute specific control (Comboboxes and Entries)
            if attr in ['injection_params', 'mode']:
                self.create_combobox_for_attribute(attr, idx)
            else:
                self.create_entry_for_attribute(attr, idx)
    
    def create_combobox_for_attribute(self, attr, idx):
        """Create combobox for specific attributes."""
        combo_var = tk.StringVar()
        values = [e.name for e in (InjectedParams if attr == 'injection_params' else MotorMode)]
        combo = ttk.Combobox(self.master, textvariable=combo_var, values=values)
        combo.grid(row=idx, column=1, pady=3)
        setattr(self, f"{attr}_combo_var", combo_var)
        combo_var.trace("w", lambda *args, attr=attr: self.delayed_update(attr))

    def create_entry_for_attribute(self, attr, idx):
        """Create entry field for specific attributes."""
        entry_var = tk.StringVar()
        setattr(self, f"{attr}_entry_var", entry_var)
        entry = ttk.Entry(self.master, textvariable=entry_var)
        entry.grid(row=idx, column=1, pady=3)
        entry_var.trace("w", lambda *args, attr=attr: self.delayed_update(attr))
        
    def create_buttons(self):
        """Create control buttons."""
        self.start_button = ttk.Button(self.master, text="Start", command=self.start_callback, width=20)
        self.start_button.grid(row=4, column=2, pady=10) 

        self.stop_button = ttk.Button(self.master, text="Stop", command=self.stop_callback, width=20)
        self.stop_button.grid(row=5, column=2, pady=10) 

        self.update_current_button = ttk.Button(self.master, text="Update Current", command=self.update_current_callback, width=20)
        self.update_current_button.grid(row=6, column=2, pady=10) 

        self.update_impedance_button = ttk.Button(self.master, text="Update Impedance", command=self.update_impedance_callback, width=20)
        self.update_impedance_button.grid(row=7, column=2, pady=10)

        close_button = ttk.Button(self.master, text="Close", command=self.close_window)
        close_button.grid(row=0, column=100, pady=20)
    
    def create_graph(self):
        """Create the graph."""
        if self.arduino and hasattr(self.arduino, 'ax') and hasattr(self.arduino, 'fig'):
            self.ax = self.arduino.ax
            self.fig = self.arduino.fig
        else:
            self.fig, self.ax = plt.subplots(figsize=(12, 9))

        self.canvas = FigureCanvasTkAgg(self.fig, self.master)
        self.canvas.get_tk_widget().grid(row=0, column=10, columnspan=30, rowspan=20)
    

    def create_sample_input(self):
        """Create sample input section."""
        self.sample_label = ttk.Label(self.master, text="Number of Samples to plot:")
        self.sample_label.grid(row=20, column=1)
        self.sample_input_var = tk.StringVar(value="1000")  # Default value
        self.sample_input = ttk.Entry(self.master, textvariable=self.sample_input_var)
        self.sample_input.grid(row=20, column=2)
        # Add a trace on the sample_input_var
        self.sample_input_var.trace("w", self.update_samples_to_plot)

    def update_samples_to_plot(self, *args):
        """Update the samples_to_plot value in Arduino."""
        try:
            samples = int(self.sample_input_var.get())
            self.arduino.samples_to_plot = samples
        except ValueError:
            # Handle cases where the input is not a valid integer
            pass

        

    def delayed_update(self, attr):
        print("HEEEELOOOOOOO")
        if self._after_id:
            self.master.after_cancel(self._after_id)
        self._after_id = self.master.after(500, lambda: self.update_attribute(attr))


    def get_serial_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports


    def update_ports(self):

        """Updates the ports dropdown with the currently available ports."""
        ports = self.get_serial_ports()
        self.ports_dropdown['values'] = ports
        if ports:
            self.port_var.set(ports[0])  # Set the first port as default
        else:
            self.port_var.set('No Ports Available')


    def connect_to_port(self):
        selected_port = self.port_var.get()
        if selected_port != 'No Ports Available':
            try:
                self.arduino = ArduinoController(selected_port)
                self.status_label.config(text=f"Connected to {selected_port} successfully!", foreground="green")
                
                # Recreate the graph after connecting to the port.
                self.create_graph()
            except Exception as e:
                self.status_label.config(text=f"Failed to connect to {selected_port}.", foreground="red")



    def update_attribute(self, attr):
        # If motor is running, do not allow updates to the specified attributes
        if self.is_running and attr in ['injection_params', 'ampl', 'freq', 'mode']:
            return

        if attr == 'injection_params':
            value = getattr(self, f"{attr}_combo_var").get()
            if value in InjectedParams.__members__:  # Check if the value is a valid InjectedParams member
                setattr(self, attr, InjectedParams[value])
                
        elif attr == 'mode':
            value = getattr(self, f"{attr}_combo_var").get()
            if value in MotorMode.__members__:
                setattr(self, attr, MotorMode[value])
        else:
            try:
                value = float(getattr(self, f"{attr}_entry_var").get())
                setattr(self, attr, value)
            except ValueError:
                # Handle the error, maybe display a message to the user
                print(f"The input for {attr} is not a valid float.")
                return

        # Now, update the ArduinoController attribute
        if self.arduino:  # Check if the ArduinoController instance exists
            if attr in ['injection_params', 'mode']:  # Enum attributes
                setattr(self.arduino, attr, getattr(self, attr).value)
            else:
                setattr(self.arduino, attr, getattr(self, attr))



    def start_callback(self):
        self.arduino.is_running = True
        self.is_running = True
        
        self.arduino.message_type = MessageTypes.MOVE.value
        self.arduino.write_flag = True
        # self.arduino.start_reading()
        

    def stop_callback(self):
        print("front stopping")
        self.arduino.is_running = False
        self.arduino.mode = MotorMode.STOP.value
        self.arduino.message_type = MessageTypes.MOVE.value
        self.arduino.write_flag = True

        self.is_running = False

        if self.arduino.read_thread.is_alive():
            print("Thread is alive!")
        else:
            print("Thread is dead!")
        # self.arduino.desired_position_list = []
        # self.arduino.actual_position_list = []

        # self.arduino.desired_velocity_list = []
        # self.arduino.actual_velocity_list = []

        # self.arduino.desired_ff_list = []
        # self.arduino.actual_torque_list = []
        # self.arduino.actual_torque_filtered_list = []

        # self.arduino.actual_current_list = []
        # self.arduino.desired_current_list = []
        

    def update_current_callback(self):
        print("front updating current")
        self.arduino.message_type = MessageTypes.CURRENT.value
        self.arduino.write_flag = True
        # pass  # You can populate this function later

    def update_impedance_callback(self):
        print("front updating impedance")
        self.arduino.message_type = MessageTypes.IMPEDANCE.value
        self.arduino.write_flag = True

    def update_plot(self):
        if self.arduino is None:
            # Handle the case where the arduino is not initialized. Maybe print a message or return early.
            print("Arduino object is not initialized.")
            self.master.after(200, self.update_plot)
            return
        print("updating graph")
        # Simply draw the updated figure from ArduinoController
        self.canvas.draw()
        
        # Set an interval for updating the canvas to show the latest figure
        self.master.after(200, self.update_plot)


    def close_window(self):
        root.destroy()
        self.arduino.is_running = False


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        root.destroy()