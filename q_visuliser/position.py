import serial
import re
import collections
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
MAX_TRAIL_LENGTH = 50  # How many physical points to keep on screen

# Regex to catch: "Final Position: X: 0.066, Y: -0.145, Z: 0.090"
pattern = r"X:\s*(-?\d+\.\d+),\s*Y:\s*(-?\d+\.\d+),\s*Z:\s*(-?\d+\.\d+)"

# Use deques to automatically push old data out when the buffer is full
x_vals = collections.deque(maxlen=MAX_TRAIL_LENGTH)
y_vals = collections.deque(maxlen=MAX_TRAIL_LENGTH)
z_vals = collections.deque(maxlen=MAX_TRAIL_LENGTH)

print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"Failed to connect: {e}")
    exit(1)

# --- Setup the Plot ---
plt.ion()  # Turn on interactive mode for real-time updates
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

print("Listening for 3D coordinates... (Press Ctrl+C to stop)")

try:
    while True:
        if ser.in_waiting > 0:
            raw_line = ""
            
            # --- THE MAGIC FIX: DRAIN THE BUFFER ---
            # Fast-forward through all piled-up old data until we get the newest line
            while ser.in_waiting > 0:
                try:
                    raw_line = ser.readline().decode('utf-8').strip()
                except UnicodeDecodeError:
                    continue

            # Now, raw_line contains the absolute most recent position.
            match = re.search(pattern, raw_line)
            if match:
                # 1. Extract the data
                x = float(match.group(1)) * 100
                y = float(match.group(2)) * 100
                z = float(match.group(3)) * 100
                print(f"x:{x:.3f}, y:{y:.3f}, z:{z:.3f}")

                x_vals.append(x)
                y_vals.append(y)
                z_vals.append(z)
                #z_vals.append(0)  # Forced 2D for now

                # 2. Clear the old frame
                ax.cla()

                # 3. Draw the Origin (Transmitter)
                ax.scatter([0], [0], [0], color='red', s=100, label='Transmitter', marker='^')

                # 4. Draw the Receiver trail
                ax.scatter(x_vals, y_vals, z_vals, color='blue', s=20, alpha=0.5)
                ax.plot(x_vals, y_vals, z_vals, color='blue', alpha=0.3)

                # Highlight the most current position in bright green
                ax.scatter([x], [y], [z], color='lime', s=80, edgecolors='black', label='Current Position')

                # 5. Formatting
                ax.set_xlabel('X Position')
                ax.set_ylabel('Y Position')
                ax.set_zlabel('Z Position')
                ax.set_title('Live 6-DOF Magnetic Tracking')
                
                # Keep the viewing box stable
                # Lock the camera to the Transmitter (Origin) so the room stops moving
                view_window = 100  # Set this to the max range of your tracking volume
                ax.set_xlim(-view_window, view_window)
                ax.set_ylim(-view_window, view_window)
                ax.set_zlim(-view_window, view_window)

                ax.legend(loc='upper left')

                # 6. Force the screen to update
                plt.pause(0.0001)

except KeyboardInterrupt:
    print("\nClosing connection...")
    ser.close()
    plt.close()
