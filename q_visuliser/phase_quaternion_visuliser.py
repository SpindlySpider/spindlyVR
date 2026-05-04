from PySide6 import QtWidgets
from PySide6 import QtCore
from vispy import scene
from vispy import util
import numpy as np
from vispy.app import use_app
from vispy import geometry
import queue


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, in_queue):
        super().__init__()
        self.queue = in_queue
        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        self.canvas = scene.SceneCanvas(keys="interactive", bgcolor="#403a3a")
        self.log_box = QtWidgets.QPlainTextEdit()
        self.grid = self.canvas.central_widget.add_grid()
        self.grid.spacing = 5
        self.view3d = self.grid.add_view(
            row=0, col=0, border_color="black", border_width=2)
        self.view2d = self.grid.add_view(
            row=0, col=1, border_color="black", border_width=2)

    def config_app(self):

        # VisPy canvas

        self.view3d.camera = scene.TurntableCamera(
            fov=45, azimuth=30, elevation=25)

        self.view2d.camera = scene.PanZoomCamera(rect=(0, -3.2, 10, 6.4))

        # add visuals to view3d.scene and view2d.scene here

        # Log widget
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("background-color:grey;")
        self.splitter.addWidget(self.canvas.native)
        self.splitter.addWidget(self.log_box)
        self.setCentralWidget(self.splitter)

        scene.visuals.Text(parent=self.view3d, text="Quaterion visulisation", pos=[
            350, 200, 0], face="Poppins", color="white")

        self.resize(1400, 800)
        self.config_2d()

        self.add_cube()
        self.start_demo_timer()

    def config_2d(self):
        self.x_axis = scene.AxisWidget(orientation="top")
        self.view2d.add_widget(self.x_axis)
        self.x_axis.link_view(self.view2d)

        self.y_axis = scene.AxisWidget(orientation="right")
        self.view2d.add_widget(self.y_axis)
        self.y_axis.link_view(self.view2d)

        self.line = scene.visuals.Line(
            pos=[(0, 0)], color="yellow", parent=self.view2d.scene)

    def start_queue_checker(self):
        pass
        # this function looks at a shared queue and pops the entries and updates phase and quaternion data

    def start_demo_timer(self):
        self.angle = 0
        self.phase = [(0, 0)]
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.demo_update)
        self.timer.start(5)

    def demo_update(self):
        # self.angle += 0.05
        # q = util.quaternion.Quaternion(
        #     1, np.cos(self.angle), np.sin(self.angle), 0)
        # serial values
        sv = None
        try:
            sv = self.queue.get_nowait()
            self.queue.queue.clear()
        except KeyError:
            pass
        except Exception as e:
            print(e)

        if sv is None:
            return

        q = util.quaternion.Quaternion(sv["q0"], sv["q1"], sv["q2"], sv["q3"])

        self.rotate_cube(q)
        # self.phase.append(sv["phase"])
        # keep x axis always increasing as time and y as the phase value
        self.phase.append((len(self.phase), sv["phase"]))

        self.line.set_data(pos=self.phase)

        camera_x = self.phase[-1][0]
        # center the camera on the increasing X axis but keep Y stable
        camera_length = (camera_x, 0)
        # print(camera_length)
        self.view2d.camera.center = camera_length
        self.canvas.update()

    def add_cube(self):
        self.cube = scene.visuals.Cube(
            parent=self.view3d.scene, edge_color="black")

    def rotate_cube(self, q):
        # q = util.quaternion.Quaternion(1,0,5,0)
        transform_value = q.get_matrix()

        transform = scene.MatrixTransform()
        transform.matrix = transform_value
        self.cube.transform = transform
