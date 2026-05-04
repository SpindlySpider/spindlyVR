from threading import Thread
import queue
import serial
import re


class serial_reader():
    def __init__(self, output_queue):
        self.queue = output_queue
        self.baudrate = 115200
        self.port = "/dev/ttyACM0"

    def start_thread(self):
        # just start reasding from serial when you can and appending it to the queue
        search_pattern = re.compile(r"(\w+): ((?:\d|\.|-)+)")
        # groups 2 and 3
        ser = serial.Serial(self.port, self.baudrate, timeout=1)
        print(f"Connected to {self.port}")
        ser.reset_input_buffer()
        while True:
           while ser.in_waiting > 0:
                try:
                    raw_line = ser.readline().decode('utf-8').strip()
                except Exception:
                    continue
                matchs = search_pattern.findall(raw_line)
                matches_dict = {}
                [matches_dict.update({k:v}) for (k,v) in matchs]
                try:
                    # print(raw_line)
                    # print(f"phase: {matches_dict['phase']} | q: {matches_dict['q0']},{matches_dict['q1']},{matches_dict['q1']},{matches_dict['q3']}")
                    # pass the dict up
                    self.queue.put_nowait(matches_dict)
                except KeyError:
                    pass
                # then once worked out need to put the extracted data on a queue





