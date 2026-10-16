
import tkinter as tk
from pymodbus.client import ModbusTcpClient
import numpy as np
from PIL import Image, ImageTk

from robot_control import RobotHandController
from appInterface import HandApp
from robot_control import RobotHandController

if __name__ == "__main__":
    robot = RobotHandController(ip="192.168.123.211", port=6000)
    root = tk.Tk()
    app = HandApp(root, robot)
    root.mainloop()