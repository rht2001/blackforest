import tkinter as tk
from pymodbus.client import ModbusTcpClient
import numpy as np
from PIL import Image, ImageTk
import os
import json
import time

# ----------------------------
# Verbindung
# ----------------------------
IP = "192.168.123.211"
PORT = 6000

client = ModbusTcpClient(IP, port=PORT)
client.connect()

# ----------------------------
# Datei für Speicher
# ----------------------------
SAVE_FILE = "positions.json"

# ----------------------------
# GUI
# ----------------------------
root = tk.Tk()
root.title("Greifhand – mit Positionsspeicher")
root.geometry("1200x900")

left_frame = tk.Frame(root)
left_frame.pack(side="left", fill="y", padx=10)

right_frame = tk.Frame(root)
right_frame.pack(side="right", fill="both", expand=True)

# ----------------------------
# Bild
# ----------------------------
img = Image.open(os.path.join(os.path.dirname(__file__), "hand.png"))
img = img.resize((700, 900))
hand_img = ImageTk.PhotoImage(img)

# ----------------------------
# SENSOR LAYOUT (UNVERÄNDERT)
# ----------------------------
sensor_groups = [
    ("Little Tip", 3000, 3, 3, "Little", (8, 10)),
    ("Little Nail", 3018, 12, 8, "Little", (-2, 40)),
    ("Little Pad", 3210, 10, 8, "Little", (4, 190)),

    ("Ring Tip", 3370, 3, 3, "Ring", (-9, -20)),
    ("Ring Nail", 3388, 12, 8, "Ring", (-23, 30)),
    ("Ring Pad", 3580, 10, 8, "Ring", (-18, 200)),

    ("Middle Tip", 3740, 3, 3, "Middle", (-21, -25)),
    ("Middle Nail", 3758, 12, 8, "Middle", (-39, 22)),
    ("Middle Pad", 3950, 10, 8, "Middle", (-35, 210)),

    ("Index Tip", 4110, 3, 3, "Index", (-38, -19)),
    ("Index Nail", 4128, 12, 8, "Index", (-49, 17)),
    ("Index Pad", 4320, 10, 8, "Index", (-54, 200)),

    ("Thumb Tip", 4480, 3, 3, "Thumb", (185, 140)),
    ("Thumb Nail", 4498, 12, 8, "Thumb", (145, 170)),
    ("Thumb Mid", 4690, 3, 3, "Thumb", (90, 240)),
    ("Thumb Pad", 4708, 12, 8, "Thumb", (-30, 240)),

    ("Palm", 4900, 8, 14, "Palm", (-15, 0))
]

offsets = {
    "Little": (150, 120),
    "Ring":   (230, 110),
    "Middle": (310, 100),
    "Index":  (390, 110),
    "Thumb":  (420, 360),
    "Palm":   (240, 520)
}

pixel = 6
sensor_offsets = {}

# ----------------------------
# MODBUS
# ----------------------------
def safe_read(addr, length):
    values = []
    for i in range(0, length, 120):
        res = client.read_holding_registers(
            address=addr+i,
            count=min(120, length-i),
            device_id=1
        )
        if not res.isError():
            values.extend(res.registers)
    return values

# ----------------------------
# OFFSET KALIBRIERUNG
# ----------------------------
def calibrate_offsets():
    print("Kalibriere Sensoren...")
    time.sleep(1)

    for name, addr, h, w, key, rel in sensor_groups:
        data = safe_read(addr, h*w)
        sensor_offsets[name] = np.array(data) if data else np.zeros(h*w)

# ----------------------------
# STEUERUNG
# ----------------------------
def send_angles():
    client.write_registers(
        address=1486,
        values=[s.get() for s in sliders],
        device_id=1
    )

def öffnen():
    for s in sliders:
        s.set(1000)
    send_angles()

def schließen():
    # ✅ DEINE WERTE!
    vals = [0, 0, 0, 0, 234, 670]
    for i in range(6):
        sliders[i].set(vals[i])
    send_angles()

def step_plus():
    for s in sliders:
        s.set(min(1000, s.get()+20))
    send_angles()

def step_minus():
    for s in sliders:
        s.set(max(0, s.get()-20))
    send_angles()

# ----------------------------
# 💾 SPEICHERFUNKTION
# ----------------------------
def save_position():

    name = entry.get()
    if not name:
        return

    values = [s.get() for s in sliders]

    positions[name] = values
    save_to_file()

    create_button(name, values)

def create_button(name, values):

    def apply():
        for i in range(6):
            sliders[i].set(values[i])
        send_angles()

    tk.Button(saved_frame, text=name, command=apply).pack(fill="x")

def save_to_file():
    with open(SAVE_FILE, "w") as f:
        json.dump(positions, f)

def load_positions():
    if not os.path.exists(SAVE_FILE):
        return

    with open(SAVE_FILE, "r") as f:
        data = json.load(f)

    for name, values in data.items():
        positions[name] = values
        create_button(name, values)

# ----------------------------
# VISUALISIERUNG
# ----------------------------
def draw():

    canvas.delete("all")
    canvas.create_image(350, 450, image=hand_img)

    for name, addr, h, w, key, rel in sensor_groups:

        data = safe_read(addr, h*w)
        if not data:
            continue

        raw = np.array(data)
        offset = sensor_offsets.get(name, np.zeros(len(raw)))

        matrix = (raw - offset).reshape(h, w)
        matrix[matrix < 0] = 0

        x0 = offsets[key][0] + rel[0]
        y0 = offsets[key][1] + rel[1]

        # Heatmap
        for i in range(h):
            for j in range(w):

                v = matrix[i, j] / 4000

                if v < 0.2:
                    color="#00aa00"
                elif v < 0.5:
                    color="#cccc00"
                elif v < 0.8:
                    color="#ff8800"
                else:
                    color="#ff0000"

                canvas.create_rectangle(
                    x0+j*pixel, y0+i*pixel,
                    x0+(j+1)*pixel, y0+(i+1)*pixel,
                    fill=color, width=0, stipple="gray25"
                )

        # ✅ ROBUSTER KONTAKT
        active = matrix > 50
        if np.sum(active) < 2:
            continue

        y_idx, x_idx = np.where(active)

        cx = int(np.mean(x_idx))
        cy = int(np.mean(y_idx))

        force = np.mean(matrix[active])

        size = int(3 + force/200)  # smooth scaling

        canvas.create_oval(
            x0+cx*pixel,
            y0+cy*pixel,
            x0+cx*pixel+size,
            y0+cy*pixel+size,
            fill="blue"
        )

# ----------------------------
def loop():
    draw()
    root.after(120, loop)

# ----------------------------
# UI
# ----------------------------
sliders=[]
positions={}

names=["Kleiner Finger","Ringfinger","Mittelfinger","Zeigefinger","Daumen B","Daumen R"]

for i in range(6):
    f=tk.Frame(left_frame)
    f.pack()

    tk.Label(f,text=names[i],width=15).pack(side="left")

    s=tk.Scale(f,from_=0,to=1000,orient="horizontal",length=250)
    s.pack(side="left")
    s.bind("<ButtonRelease-1>",lambda e: send_angles())

    sliders.append(s)

def read_current_angles():
    res=client.read_holding_registers(address=1486,count=6,device_id=1)
    if not res.isError():
        for i in range(6):
            sliders[i].set(res.registers[i])

root.after(500,read_current_angles)
root.after(1000,calibrate_offsets)

# Buttons
bf=tk.Frame(left_frame)
bf.pack(pady=10)

tk.Button(bf,text="Öffnen",command=öffnen).pack(side="left")
tk.Button(bf,text="Schließen",command=schließen).pack(side="left")
tk.Button(bf,text="Step +",command=step_plus).pack(side="left")
tk.Button(bf,text="Step -",command=step_minus).pack(side="left")

# ----------------------------
# 💾 Speicher UI
# ----------------------------
entry = tk.Entry(left_frame)
entry.pack(fill="x", pady=5)

tk.Button(left_frame, text="Position speichern", command=save_position).pack(fill="x")

saved_frame = tk.Frame(left_frame)
saved_frame.pack(fill="both", expand=True)

load_positions()

# ----------------------------
# Canvas
# ----------------------------
canvas=tk.Canvas(right_frame,width=700,height=900)
canvas.pack()

loop()
root.mainloop()
