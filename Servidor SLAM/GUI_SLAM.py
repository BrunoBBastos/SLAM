import tkinter as tk
import cv2
import requests
from PIL import Image, ImageTk
import threading
import numpy as np
import json

import qrcode as qc

class InterfaceModule:
    def __init__(self, ip):
        # Initialize variables and create the Tkinter window
        self.ip = f'http://{ip}'
        self.video_url = f'{self.ip}:81'
        self.page_url = f'{self.ip}/robot'
        self.interval_ms = 100
        self.speed = 255
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.medicao = 0
        self.odometriaDelta = np.array([0.0, 0.0, 0.0])
        self.vision = qc.qrcode()
        self.distanceSensor = -1
        self.state = "Driving"

        self.root = tk.Tk()
        self.root.title("Robot Interface")
        self.root.state('zoomed')

        # Create UI components
        self.create_video_frame()
        self.create_pose_frame()
        self.create_observation_frame() 
        self.create_toggle_button()
        self.create_connect_button()

        self.gui_thread = threading.Thread(target=self.start, daemon=True)
        self.gui_thread.start()
        # Start threads
        # self.start_video_stream()
        # self.start_page_content_fetching()
        # self.start_keyboard_control()

    def create_connect_button(self):
        # Create a button to connect
        self.connect_button = tk.Button(self.root, text="Connect", command=self.connect)
        self.connect_button.grid(row=3, column=0, padx=10, pady=10)
        self.connected = False

    def getConnectionStatus(self):
        return self.connected

    def connect(self):
        # Establish the connection to the server and start threads
        self.start_video_stream()
        self.start_page_content_fetching()
        self.start_keyboard_control()
        # Put a test to confirm connection
        self.connected = True
        # return True

    def create_video_frame(self):
        # Create frame for displaying video
        self.video_frame = tk.Frame(self.root, width = 800, height= 600)
        self.video_frame.grid(row=0, column=0, rowspan=3, columnspan=4, padx=10, pady=10)

        self.video_label = tk.Label(self.video_frame)
        self.video_label.pack()

    def create_pose_frame(self):
        # Create frame for displaying pose values
        self.pose_frame = tk.LabelFrame(self.root, text="Pose", padx=10, pady=10)
        self.pose_frame.grid(row=0, column=4, padx=10, pady=10)

        self.x_label = self.create_label(self.pose_frame, "x:")
        self.y_label = self.create_label(self.pose_frame, "y:")
        self.theta_label = self.create_label(self.pose_frame, "\u03B8:")

    def create_observation_frame(self):
        # Create a frame for displaying observations
        self.observation_frame = tk.LabelFrame(
            self.root, text="Observation", padx=10, pady=10
        )
        self.observation_frame.grid(row=1, column=4, padx=10, pady=10, sticky="n")

        # Create a label to display the distance sensor value
        self.distance_label = tk.Label(self.observation_frame, text="Distance: --")
        self.distance_label.pack()
        self.angle_label = tk.Label(self.observation_frame, text="Angle: --")
        self.angle_label.pack()
        self.ID_label = tk.Label(self.observation_frame, text="ID: --")
        self.ID_label.pack()

    def update_distance_label(self):
        # Update the label's text with the current distance sensor value
        if self.distanceSensor == 0 or self.distanceSensor == -1:
            return
        else:
            self.distance_label.config(text=f"Distance: {self.distanceSensor[0]:.3f}")
            self.angle_label.config(text=f"Angle: {self.distanceSensor[1]:.3f}")
            self.ID_label.config(text=f"ID: {self.distanceSensor[2]:.3f}")


    def create_label(self, parent, text):
        # Create and return a label
        label = tk.Label(parent, text=text)
        label.pack(side="top", padx=5, pady=5)
        return label

    def create_toggle_button(self):
        # Create a button for toggling state
        self.toggle_button = tk.Button(self.root, text="Toggle", command=self.toggle_state)
        self.toggle_button.grid(row=2, column=4, pady=10)

    def start_video_stream(self):
        threading.Thread(target=self.video_stream_worker, daemon=True).start()

    def video_stream_worker(self):
        try:
            cap = cv2.VideoCapture(self.video_url)
            if not cap.isOpened():
                print("Failed to open video stream")
                return

            while True:
                ret, frame = cap.read()
                if ret:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    image = Image.fromarray(frame)
                    photo = ImageTk.PhotoImage(image=image)
                    self.video_label.configure(image=photo)
                    self.video_label.image = photo
                    self.distanceSensor = self.vision.detectAnddecod(frame, ret)
                    self.update_distance_label()

        except cv2.error as e:
            print(f"An error occurred: {e}")

    def getDist(self):
        distance = self.distanceSensor
        self.distanceSensor = -1
        return distance
    
    def getOdometry(self):
        delta = np.copy(self.odometriaDelta)
        self.odometriaDelta = np.array([[0], [0], [0]])
        return delta

    def start_page_content_fetching(self):
        threading.Thread(target=self.page_content_fetching_worker, daemon=True).start()

    def page_content_fetching_worker(self):
        try:
            while True:
                response = requests.get(self.page_url)
                if response.status_code == 200:
                    content = response.text
                    self.update_page_content(content)
                else:
                    print(f"Request failed with status code: {response.status_code}")

                threading.Event().wait(self.interval_ms / 1000)

        except requests.RequestException as e:
            print(f"An error occurred: {e}")

    def update_page_content(self, content):
        try:
            data = json.loads(content)
            pose_values = data.get('pose')
            odometria_values = data.get('odometria')

            if odometria_values:
                odometria_values = odometria_values.split(', ')
                if len(odometria_values) == 3:
                    x, y, theta = map(float, odometria_values)
                    self.odometriaDelta = np.array([[x], [y], [theta]])
                    # self.x += x
                    # self.y += y
                    # self.theta += theta
                    # theta = theta * 180 / np.pi
                    self.x_label.configure(text=f"x: {self.x:.3f}")
                    self.y_label.configure(text=f"y: {self.y:.3f}")
                    self.theta_label.configure(text=f"\u03B8: {self.theta:.3f}")
                else:
                    raise ValueError("Invalid content format or missing values")

        except (ValueError, json.JSONDecodeError) as e:
            print(f"Error updating from page content: {e}")
    
    def setPose(self, newPose):
        # pass
        # self.x = newPose[0,0]
        # self.y = newPose[1,0]
        # self.theta = newPose[2,0]
        self.x_label.configure(text=f"x: {newPose[0, 0]:.3f}")
        self.y_label.configure(text=f"y: {newPose[1, 0]:.3f}")
        self.theta_label.configure(text=f"\u03B8: {newPose[2, 0]:.3f}")

    def start_keyboard_control(self):
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.key_state = {
            "w": False,
            "s": False,
            "a": False,
            "d": False
        }
        threading.Thread(target=self.update_robot_command, daemon=True).start()

    def on_key_press(self, event):
        key = event.keysym.lower()

        if self.state == "Driving":
            if key in self.key_state:
                self.key_state[key] = True

    def on_key_release(self, event):
        key = event.keysym.lower()

        if self.state == "Driving":
            if key in self.key_state:
                self.key_state[key] = False

    def update_robot_command(self):
        key_combinations = {
            ("w",): (self.speed, self.speed),
            ("s",): (-self.speed, -self.speed),
            ("a",): (-self.speed/3, self.speed/3),
            ("d",): (self.speed/3, -self.speed/3),
            ("w", "a"): (0, self.speed),
            ("w", "d"): (self.speed, 0),
            ("s", "a"): (0, -self.speed/2),
            ("s", "d"): (-self.speed/2, 0),
        }

        while True:
            keys = tuple(key for key, state in self.key_state.items() if state)
            l, r = key_combinations.get(keys, (0, 0))

            url = f"{self.ip}/slam?type=vel&l={l}&r={r}"
            response = requests.get(url)
            if response.status_code != 200:
                print(f"Failed to send robot command: {response.status_code}")

            threading.Event().wait(50 / 1000)

    def toggle_state(self):
        state = ""
        if self.state == "Driving":
            self.state = "Following"
            self.toggle_button.configure(text="Following")
            state = "FLW"
        else:
            self.state = "Driving"
            self.toggle_button.configure(text="Driving")
            state = "DRV"

        url = f"{self.ip}/slam?type={state}"
        response = requests.get(url)
        if response.status_code != 200:
            print(f"Failed to send robot command: {response.status_code}")

    def start(self):
        # Start the Tkinter event loop
        self.root.mainloop()
    

if __name__ == "__main__":
    # ipEsp32 = '10.0.0.104'
    ipEsp32 = '192.168.4.84'
    interface = InterfaceModule(ipEsp32)
    interface.start()
