from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import math

# Simple joint angles
theta1 = 0.0
theta2 = 0.0

# Link lengths
L1 = 1.0
L2 = 0.7

def forward_kinematics():
    x1 = L1 * math.cos(math.radians(theta1))
    y1 = L1 * math.sin(math.radians(theta1))

    x2 = x1 + L2 * math.cos(math.radians(theta1 + theta2))
    y2 = y1 + L2 * math.sin(math.radians(theta1 + theta2))

    return (0, 0), (x1, y1), (x2, y2)

def display():
    glClear(GL_COLOR_BUFFER_BIT)
    glLoadIdentity()
    
    glColor3f(1,1,1)
    glLineWidth(4)

    base, j1, j2 = forward_kinematics()

    glBegin(GL_LINES)
    glVertex2f(*base)
    glVertex2f(*j1)

    glVertex2f(*j1)
    glVertex2f(*j2)
    glEnd()

    glutSwapBuffers()

def keyboard(key, x, y):
    global theta1, theta2

    if key == b'a': theta1 += 2
    if key == b'd': theta1 -= 2
    if key == b'w': theta2 += 2
    if key == b's': theta2 -= 2

    glutPostRedisplay()

def main():
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB)
    glutInitWindowSize(600, 600)
    glutCreateWindow(b"Robotic Arm - Line Rendering")

    glClearColor(0, 0, 0, 1)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glOrtho(-2, 2, -2, 2, -1, 1)

    glutDisplayFunc(display)
    glutKeyboardFunc(keyboard)
    glutMainLoop()

if __name__ == "__main__":
    main()
