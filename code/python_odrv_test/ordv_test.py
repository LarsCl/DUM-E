import pigpio
import time

# Start pigpio daemon
pi = pigpio.pi()

# Define TX and RX pins for each ODrive
TX1, RX1 = 17, 27  # ODrive 1
TX2, RX2 = 22, 23  # ODrive 2

# Set RX pins as input
pi.set_mode(RX1, pigpio.INPUT)
pi.set_mode(RX2, pigpio.INPUT)

# Configure Software UARTs (TX is specified, RX is assumed to be the next available pin)
soft_uart1 = pi.serial_open(TX1, 115200, 8)  # TX = 17, RX = 27
soft_uart2 = pi.serial_open(TX2, 115200, 8)  # TX = 22, RX = 23

def send_command_soft(soft_uart, command):
    """Send command over software UART and read response."""
    pi.serial_write(soft_uart, command + "\n")
    time.sleep(0.01)  # Small delay for stability
    (count, data) = pi.serial_read(soft_uart)
    return data.decode().strip() if count else ""

# Real-time control loop

# Set velocity of both ODrives
send_command_soft(soft_uart1, "p 0 1.0")
print(send_command_soft(soft_uart1, "f 0"))
time.sleep(3)
send_command_soft(soft_uart1, "p 0 0.0")
print(send_command_soft(soft_uart2, "f 0"))
