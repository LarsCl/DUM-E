import serial
import threading
import time
import queue
import struct
import re


class UART:
    def __init__(self, port, baud_rate=115200):
        """
        Initializes the UART instance with a command queue.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.ser = serial.Serial(port, baud_rate, timeout=0.1)
        self.lock = threading.Lock()  # Ensure thread safety for serial access
        self.command_queue = queue.Queue()
        self.running = True
        self.thread = threading.Thread(target=self._process_queue)
        self.thread.daemon = True  # Stop the thread when main program exits
        self.thread.start()
        print(f"[INFO] Connected to UART on {port} at {baud_rate} baud.")

    def _process_queue(self):
        """
        Continuously processes commands from the queue.
        Each item is a tuple: (command, feedback, result_queue)
          - command: the command string to send.
          - feedback: True if the caller expects a response.
          - result_queue: if feedback is True, a temporary queue to deliver the response.
        """
        while self.running:
            try:
                # Wait for a command to be queued.
                command, feedback, result_queue = self.command_queue.get(timeout=1)
            except queue.Empty:
                continue

            if feedback:
                response = self._send_feedback_command_now(command)
                if result_queue is not None:
                    result_queue.put(response)
            else:
                self._send_command_now(command)
            self.command_queue.task_done()

    def _send_feedback_command_now(self, command):
        """
        Sends a command and waits for its response.
        This method is used internally by the queue thread.
        """
        full_command = f"{command}\n"  # UART expects newline-terminated commands
        with self.lock:
            self.ser.write(full_command.encode())
            response = self._wait_for_response()
        if response:
            return response.decode(errors='ignore').strip()
        return None

    def _send_command_now(self, command):
        """
        Sends a command without waiting for a response.
        This method is used internally by the queue thread.
        """
        full_command = f"{command}\n"  # UART expects newline-terminated commands
        with self.lock:
            self.ser.write(full_command.encode())

    def _wait_for_response(self):
        """
        Waits for data to become available on the serial port.
        Once data is available, returns the data read.
        """
        # Wait until there is data available (busy-waiting)
        while self.ser.in_waiting == 0:
            time.sleep(0.001)  # small sleep to avoid maxing out the CPU
        # Read all available bytes
        response = self.ser.read(self.ser.in_waiting)
        return response

    def queue_feedback_command(self, command, block=True, timeout=None):
        """
        Queues a command that expects a response.
        If block is True, waits until the response is available.
        """
        result_queue = queue.Queue()
        self.command_queue.put((command, True, result_queue))
        if block:
            try:
                return result_queue.get(timeout=timeout)
            except queue.Empty:
                return None
        return None

    def queue_command(self, command):
        """
        Queues a command that does not expect a response.
        """
        self.command_queue.put((command, False, None))

    def close(self):
        """
        Stops the command processing thread and closes the UART connection.
        """
        self.running = False
        self.thread.join()
        self.ser.close()
        print(f"[INFO] Connection closed for {self.port}.")


class ODriveUART(UART):
    def __init__(self, port, baud_rate=115200):
        """
        Initializes an ODrive instance using a given UART port.
        Inherits the UART class for communication.
        """
        super().__init__(port, baud_rate)

    def get_bus_voltage(self):
        """
        Requests the bus voltage from the ODrive.
        Uses a queued command to ensure sequential processing.
        """
        response = self.queue_feedback_command("r vbus_voltage")
        if response:
            try:
                return float(response)
            except ValueError:
                print(f"[ERROR] Invalid voltage response from {self.port}: {response}")
        return None

    def get_bus_current(self):
        """
        Requests the bus current from the ODrive.
        """
        response = self.queue_feedback_command("r ibus")
        if response:
            try:
                return float(response)
            except ValueError:
                print(f"[ERROR] Invalid current response from {self.port}: {response}")
        return None

    def get_joint_position(self):
        """
        Requests the position estimate from the ODrive (axis 0).
        """
        response = self.queue_feedback_command("r axis0.pos_estimate")
        if response:
            try:
                return float(response)
            except ValueError:
                print(f"[ERROR] Invalid position response from {self.port}: {response}")
        return None

    def get_joint_velocity(self):
        """
        Requests the velocity estimate from the ODrive (axis 0).
        """
        response = self.queue_feedback_command("r axis0.vel_estimate")
        if response:
            try:
                return float(response)
            except ValueError:
                print(f"[ERROR] Invalid velocity response from {self.port}: {response}")
        return None

    def get_active_errors(self):
        """
        Requests the active errors on the ODrive (axis 0).
        """
        response = self.queue_feedback_command("r axis0.active_errors")
        if response:
            return response
        return None

    def get_serial_number(self):
        """
        Requests the serial number of the ODrive. Returns the integer representation.
        """
        response = self.queue_feedback_command("r serial_number")
        if response:
            try:
                return int(response)
            except ValueError:
                print(f"[ERROR] Invalid serial number response from {self.port}: {response}")
        return None

    def is_armed(self):
        """
        Requests whether the ODrive is armed.
        """
        response = self.queue_feedback_command("r axis0.is_armed")
        if response:
            try:
                return bool(int(response))
            except ValueError:
                print(f"[ERROR] Invalid armed state response from {self.port}: {response}")
        return None

    def is_homed(self):
        """
        Requests whether the ODrive is homed.
        """
        response = self.queue_feedback_command("r axis0.is_homed")
        if response:
            try:
                return bool(int(response))
            except ValueError:
                print(f"[ERROR] Invalid homed state response from {self.port}: {response}")
        return None

    def identify_on(self):
        """
        Turns on the ODrive identification LED.
        """
        self.queue_command("w identify 1")
        print(f"[INFO] ODrive at {self.port} identifying.")

    def identify_off(self):
        """
        Turns off the ODrive identification LED.
        """
        self.queue_command("w identify 0")
        print(f"[INFO] ODrive at {self.port} stopped identifying.")

    def move_pos(self, target_pos: float, target_vel: float):
        """
        Moves to a new position setpoint relative to the current joint position.
        """
        self.queue_command(f"p 0 {target_pos} {target_vel}")
        print(f"[INFO] ODrive at {self.port} moving to {target_pos}")
        

    def home(self):
        """
        Requests the built-in homing sequence for axis 0 of the ODrive.
        """
        self.queue_command("w axis0.requested_state 11")
        print(f"[INFO] ODrive at {self.port} homing.")

    def arm(self):
        """
        Arms the motor.
        """
        self.queue_command("w axis0.requested_state 8")
        print(f"[INFO] ODrive at {self.port} arming.")

    def disarm(self):
        """
        Disarms the motor.
        """
        self.queue_command("w axis0.requested_state 1")
        print(f"[INFO] ODrive at {self.port} disarming.")

    def reboot(self):
        """
        Reboots the ODrive.
        """
        self.queue_command("sr")
        print(f"[INFO] ODrive at {self.port} rebooting.")
        return self.get_active_errors()

    def clear_errors(self):
        """
        Clears all errors in the ODrive.
        """
        self.queue_command("sc")
        print(f"[INFO] ODrive at {self.port} errors cleared.")

    def close(self):
        """
        Closes the UART connection.
        """
        super().close()


class ArduinoUART(UART):
    def __init__(self, port, baud_rate=115200):
        """
        Initializes an Arduino instance using a given UART port.
        """
        super().__init__(port, baud_rate)
        print(f"[INFO] Connected to Arduino on {port} at {baud_rate} baud.")
    
    def set_postition(self, baseAngle, rotation, endRotation, endAngle):
        """
        set arduino position 
        """
        print(f"[Set positions]: M B{baseAngle} W{rotation} ER{endRotation} EA{endAngle}")
        
        packet = struct.pack(
        '>c B c h c h c h c h',     # Format string
        b'C', 1,                    # Command header
        b'B', baseAngle,            # 'B' value (16-bit)
        b'W', rotation,             # 'W' value (16-bit)
        b'R', endRotation,
        b'A', endAngle
        )           
        self.ser.write(packet)
        return None
        
    def set_motor_config(self,unit, velocity,acceleration, sleep, block, drv):
        """
        arduino motor config
        """
        print(f"[Configuring {unit}]: velo: {velocity} accel: {acceleration} sleep: {sleep} block: {block} drive: {drv}")
        bitmask = (sleep << 2) | (block << 1) | drv
        
        packet = struct.pack(
        '>c B c h c h c h c h',     # Format string
        b'C', 2,                    # Command header
        b'B', unit,                 # 'B' value (16-bit)
        b'W', acceleration,         # 'W' value (16-bit)
        b'R', velocity,
        b'A', bitmask
        )           
        self.ser.write(packet)
        return None
    
    def start_homing_proc(self):
        """
        makes arduino go crazy
        """
        packet = struct.pack(
        '>c B c h c h c h c h',     # Format string
        b'C', 3,                    # Command header
        b'B', 10,                   # 'B' value (16-bit)
        b'W', 0,                    # 'W' value (16-bit)
        b'R', 0,
        b'A', 0
        )           
        self.ser.write(packet)
        return None

    def get_positions(self):
        """
        return individual motor positions [j1, j4, j5, j6]
        """
        #print(f"[request joint positions]")
        
        cmd_packet = struct.pack(
        '>c B c h c h c h c h',  # Format: big-endian
        b'C', 4,    # Command header
        b'B', 0,    # Placeholders (values ignored by Arduino)
        b'W', 0,
        b'R', 0,
        b'A', 0
        )
        self.ser.write(cmd_packet)

        # Read incoming data, filtering out debug prints
        start_time = time.time()
        start_time = time.time()
        while time.time() - start_time < 1:
            buffer = self.ser.read(self.ser.in_waiting).decode('ascii', 'ignore')
        
            if match := re.search(r'C4B(-?\d+)W(-?\d+)R(-?\d+)A(-?\d+)', buffer):
                try:
                    received_b = int(match.group(1))
                    received_w = int(match.group(2))
                    received_r = int(match.group(3))
                    received_a = int(match.group(4))
                
                    # For debugging/transparency
                    #print(f"B: {received_b}, W: {received_w}, R: {received_r}, A: {received_a}")
                
                    # dan nog /10 ergens
                    return [received_b, received_w, received_r, received_a]
                except ValueError:
                    print("Invalid numeric values in response")
                break

        #print("Timeout waiting for positions")
        return [None, None, None, None]
        

    def get_velocities(self):
        """
        Returns individual motor velocities [j1, j4, j5, j6].
        Filters out Arduino debug messages by looking for a valid response sequence.
        """
        #print("[INFO] Requesting joint velocities...")

        cmd_packet = struct.pack(
        '>c B c h c h c h c h',  # Format: big-endian
        b'C', 5,    # Command header
        b'B', 0,    # Placeholders (values ignored by Arduino)
        b'W', 0,
        b'R', 0,
        b'A', 0
        )
        self.ser.write(cmd_packet)

         # Read incoming data, filtering out debug prints
        start_time = time.time()
        start_time = time.time()
        while time.time() - start_time < 1:
            buffer = self.ser.read(self.ser.in_waiting).decode('ascii', 'ignore')
            if match := re.search(r'C5B(-?\d+)W(-?\d+)R(-?\d+)A(-?\d+)', buffer):
                try:
                    received_b = int(match.group(1))
                    received_w = int(match.group(2))
                    received_r = int(match.group(3))
                    received_a = int(match.group(4))
                
                    # For debugging/transparency
                    #print(f"B: {received_b}, W: {received_w}, R: {received_r}, A: {received_a}")
                
                    # dan nog /10 ergens
                    return [received_b, received_w, received_r, received_a]
                except ValueError:
                    print("Invalid numeric values in response")
                break

        #print("Timeout waiting for positions")
        return [None, None, None, None]
    
    def send_joint_targets_velocity(self, joint1, joint4, joint5, joint6):
        """
        set arduino positions format: 
        ```python
        [vel_joint1, vel_joint4, vel_joint5, vel_joint6] 
        ```
        """
        
        packet = struct.pack(
        '>c B c h c h c h c h',     # Format string
        b'C', 6,                    # Command header
        b'B', joint1,               # 'B' value (16-bit)
        b'W', joint4,               # 'W' value (16-bit)
        b'R', joint5,
        b'A', joint6
        )           
        self.ser.write(packet)
        return None

    def send_joint_targets_position(self, joint1, joint4, joint5, joint6):
        """
        set arduino position
        ```python
        [pos_joint1, pos_joint4, pos_joint5, pos_joint6] 
        ```
        """
        
        packet = struct.pack(
        '>c B c h c h c h c h',     # Format string
        b'C', 7,                    # Command header
        b'B', joint1,               # 'B' value (16-bit)
        b'W', joint4,               # 'W' value (16-bit)
        b'R', joint5,
        b'A', joint6
        )           
        self.ser.write(packet)
        return None


    def close(self):
        """
        Closes the UART connection.
        """
        super().close()



