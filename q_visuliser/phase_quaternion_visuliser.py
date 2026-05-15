from PySide6 import QtWidgets
from PySide6 import QtCore
from vispy import scene
from vispy import util
import numpy as np
from vispy.app import use_app
from vispy import geometry
from vispy.visuals.transforms import STTransform
import queue
import re

import numpy as np


def wrap_to_pi(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


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
        self.view3d_pos = self.grid.add_view(
            row=1, col=0, border_color="black", border_width=2)
        self.view2d = self.grid.add_view(
            row=0, col=1, border_color="black", border_width=2)

    def config_app(self):

        # VisPy canvas
        self.view3d.camera = scene.TurntableCamera(
            fov=45, azimuth=30, elevation=25)

        self.view3d_pos.camera = scene.TurntableCamera(
            fov=45, up="z")

        self.view2d.camera = scene.PanZoomCamera(rect=(0, -3.2, 10, 6.4))

        # add visuals to view3d.scene and view2d.scene here

        # Log widget
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("background-color:grey;")
        self.splitter.addWidget(self.canvas.native)
        self.splitter.addWidget(self.log_box)
        self.setCentralWidget(self.splitter)

        scene.visuals.Text(parent=self.view3d, text="Quaternion visulisation", pos=[
            350, 200, 0], face="Poppins", color="white")

        self.axis = scene.visuals.XYZAxis(parent=self.view3d.scene)

        self.resize(1400, 800)
        self.config_2d()
        self.config_3d_pos()

        self.cube = self.add_cube(self.view3d.scene)
        self.start_demo_timer()

    def config_3d_pos(self):
        # https://vispy.org/gallery/scene/surface_plot.html#sphx-glr-gallery-scene-surface-plot-py
        xax = scene.Axis(pos=[[-2, -2], [2, -2]], tick_direction=(0, -1),
                         font_size=16, axis_color='red', tick_color='k', text_color='k',
                         parent=self.view3d_pos.scene)

        # xax.transform = scene.STTransform(translate=(0, 0, -0.2))

        yax = scene.Axis(pos=[[-2, -2], [-2, 2]], tick_direction=(-1, 0),
                         font_size=16, axis_color='red', tick_color='k', text_color='k',
                         parent=self.view3d_pos.scene)
        # yax.transform = scene.STTransform(translate=(0, 0, -0.2))

        self.position_cube = self.add_cube(self.view3d_pos.scene)

        self.pos_file = open("./logs/first_pos_log.txt")

        # Add a 3D axis to keep us oriented
        axis = scene.visuals.XYZAxis(parent=self.view3d_pos.scene)

    def config_2d(self):
        self.x_axis = scene.AxisWidget(orientation="top", axis_label="Time")
        self.view2d.add_widget(self.x_axis)
        self.x_axis.link_view(self.view2d)

        self.y_axis = scene.AxisWidget(orientation="right", axis_label="Phase")
        self.view2d.add_widget(self.y_axis)
        self.y_axis.link_view(self.view2d)

        self.raw_phase_line = scene.visuals.Line(
            pos=[(0, 0)], color="yellow", parent=self.view2d.scene)

        self.synced_phase_line = scene.visuals.Line(
            pos=[(0, 0)], color="blue", parent=self.view2d.scene)

        self.error_phase_line = scene.visuals.Line(
            pos=[(0, 0)], color="red", parent=self.view2d.scene)

    def start_demo_timer(self):
        self.angle = 0
        self.phase = [(0, 0)]
        self.synced_phase = [(0, 0)]
        self.error_phase = [(0, 0)]
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.demo_update)
        self.timer.start(5)

    def demo_update(self):
        # call back for GUI thread
        # serial values
        sv = None
        try:
            sv = self.queue.get_nowait()
            # empty queue so we always use new data
            self.queue.queue.clear()
        except Exception:
            pass

        try:
            q = util.quaternion.Quaternion(
                sv["q0"], sv["q1"], sv["q2"], sv["q3"])
            self.rotate_cube(q)
        except Exception:
            # if we have no quaternion data pass :)
            pass

        try:
            # try to update phase data :)
            self.update_phase_data(sv)
        except Exception:
            # if we have no quaternion data pass :)
            pass

        try:
            self.update_cube_pos(sv)
        except Exception:
            # if we have no quaternion data pass :)
            pass

        # print(sv)
        self.canvas.update()
        self.log_box.appendPlainText(sv["raw"])

    def update_cube_pos(self, sv):
        # reg = re.compile(r"(\w+): ((?:\d|\.|-)+)")
        # line = reg.findall(self.pos_file.readline())
        # self.position_cube.transform = STTransform(
        #     translate=(float(line[0][1]), float(line[1][1]), float(line[2][1])))
        # print(line)
        self.position_cube.transform = STTransform(
            translate=(float(sv["X"]), float(sv["Y"]), float(sv["Z"])))
        # print(line)

    def update_phase_data(self, sv):
        # sv is serial values
        self.phase.append((len(self.phase), np.sin(
            (len(self.phase)) + float(sv["phase"]))))
        self.synced_phase.append((len(self.synced_phase), np.sin(
            (len(self.phase)) + float(sv["syncedPhase"]))))
        self.error_phase.append((len(self.error_phase), wrap_to_pi(
            float(sv["phase"]) - float(sv["syncedPhase"]))))

        self.raw_phase_line.set_data(pos=self.phase)
        self.synced_phase_line.set_data(pos=self.synced_phase)
        self.error_phase_line.set_data(pos=self.error_phase)

        camera_x = self.phase[-1][0]
        # center the camera on the increasing X axis but keep Y stable
        camera_length = (camera_x - 10, 0)
        # print(camera_length)
        self.view2d.camera.center = camera_length

    def make_cube_face_colors(self):
        # setting colours for orientation testing
        red = [1.0, 0.0, 0.0, 1.0]   # +X
        darkred = [0.5, 0.0, 0.0, 1.0]   # -X
        green = [0.0, 1.0, 0.0, 1.0]   # +Y
        darkgrn = [0.0, 0.5, 0.0, 1.0]   # -Y
        blue = [0.0, 0.0, 1.0, 1.0]   # +Z
        darkblu = [0.0, 0.0, 0.5, 1.0]   # -Z

        return [
            darkblu, darkblu,
            blue, blue,
            darkgrn, darkgrn,
            green, green,
            darkred, darkred,
            red, red,
        ]

    def add_cube(self, parent):
        return scene.visuals.Cube(
            parent=parent,
            edge_color="black",
            face_colors=self.make_cube_face_colors(),
            size=(0.1,0.1,0.1)
        )

    def rotate_cube(self, q):
        # q = util.quaternion.Quaternion(1,0,5,0)
        q = q.conjugate()
        transform_value = q.get_matrix()

        transform = scene.MatrixTransform()
        transform.matrix = transform_value
        self.cube.transform = transform
