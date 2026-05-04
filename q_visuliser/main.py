from PySide6 import QtWidgets
from phase_quaternion_visuliser import MainWindow
from serial_thread import serial_reader
from vispy.app import use_app
import queue
import threading

shared_queue = queue.Queue(maxsize=1000)


use_app("pyside6")
app = QtWidgets.QApplication()

#setup serial thead


reader = serial_reader(shared_queue)
thread = threading.Thread(target=reader.start_thread)
thread.start()

# reader.start_thread()

#setup GUI
window = MainWindow(shared_queue)
window.config_app()
window.show()

app.exec()
