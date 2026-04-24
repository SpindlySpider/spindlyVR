import pygame
import serial
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'   # CHANGE THIS
BAUD_RATE = 115200

def resize(width, height):
    if height == 0: height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def draw_cube():
    glBegin(GL_QUADS)
    # Coloring faces to help identify orientation
    glColor3f(0.0, 1.0, 0.0); glVertex3f( 1.0, 0.2, -1.0); glVertex3f(-1.0, 0.2, -1.0); glVertex3f(-1.0, 0.2,  1.0); glVertex3f( 1.0, 0.2,  1.0) # Top (Green)
    glColor3f(1.0, 0.5, 0.0); glVertex3f( 1.0,-0.2,  1.0); glVertex3f(-1.0,-0.2,  1.0); glVertex3f(-1.0,-0.2, -1.0); glVertex3f( 1.0,-0.2, -1.0) # Bot (Orange)
    glColor3f(1.0, 0.0, 0.0); glVertex3f( 1.0, 0.2,  1.0); glVertex3f(-1.0, 0.2,  1.0); glVertex3f(-1.0,-0.2,  1.0); glVertex3f( 1.0,-0.2,  1.0) # Front (Red)
    glColor3f(1.0, 1.0, 0.0); glVertex3f( 1.0,-0.2, -1.0); glVertex3f(-1.0,-0.2, -1.0); glVertex3f(-1.0, 0.2, -1.0); glVertex3f( 1.0, 0.2, -1.0) # Back (Yellow)
    glColor3f(0.0, 0.0, 1.0); glVertex3f(-1.0, 0.2,  1.0); glVertex3f(-1.0, 0.2, -1.0); glVertex3f(-1.0,-0.2, -1.0); glVertex3f(-1.0,-0.2,  1.0) # Left (Blue)
    glColor3f(1.0, 0.0, 1.0); glVertex3f( 1.0, 0.2, -1.0); glVertex3f( 1.0, 0.2,  1.0); glVertex3f( 1.0,-0.2,  1.0); glVertex3f( 1.0,-0.2, -1.0) # Right (Magenta)
    glEnd()

def main():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Zephyr Madgwick Visualization")
    resize(640, 480)
    init()

    # Open Serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}")
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Serial Error: {e}")
        return

    # Quaternions
    w, x, y, z = 1.0, 0.0, 0.0, 0.0

    while True:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        # Read Serial Data
        while ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                parts = line.split(',')
                if len(parts) == 4:
                    w, x, y, z = map(float, parts)
            except ValueError:
                pass
        
        # Render
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0.0, -7.0)

# Convert Quaternion to Axis-Angle
        import math
        angle = 2 * math.acos(w) * 180.0 / math.pi
        s = math.sqrt(1 - w*w)
        if s < 0.001: 
            ax, ay, az = x, y, z 
        else:
            ax, ay, az = x/s, y/s, z/s
        
        # --- THE FIX: MAP IMU AXES TO OPENGL SCREEN AXES ---
        # Assuming your IMU X is Forward, Y is Right, Z is Up:
        # OpenGL X (Screen Right) = IMU Y (Physical Right)
        # OpenGL Y (Screen Up)    = IMU Z (Physical Up)
        # OpenGL Z (Screen Out)   = IMU -X (Physical Backward)
        
        opengl_x = -ay
        opengl_y = az
        opengl_z = -ax
        
        # Apply the MAPPED rotation
        glRotatef(angle, opengl_x, opengl_y, opengl_z)

        draw_cube()
        pygame.display.flip()
        pygame.time.wait(1)

if __name__ == '__main__':
    main()
