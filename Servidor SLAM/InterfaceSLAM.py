import tkinter as tk
import cv2
import requests
from PIL import Image, ImageTk
import threading
import json


class InterfaceModule:
    def __init__(self, ip, video_url, page_url, interval_ms):
        self.ip = f'http://{ip}'
        self.video_url = f'{self.ip}{video_url}'
        self.page_url = f'{self.ip}{page_url}'
        self.interval_ms = interval_ms

        self.speed = 100

        self.root = tk.Tk()
        self.root.title("Robot Interface")

        self.create_pose_frame()
        self.create_referencia_frame()
        self.create_video_frame()

        self.state = "Starting"  # Initial state
        self.toggle_button = tk.Button(
            self.root, text="Toggle", command=self.toggle_state
        )
        self.toggle_button.pack(pady=10)

        self.start_video_stream()
        self.start_page_content_fetching()
        self.start_keyboard_control()

    def create_pose_frame(self):
        self.pose_frame = tk.LabelFrame(
            self.root, text="Pose", padx=10, pady=10
        )
        self.pose_frame.pack(side="left", padx=10, pady=10)

        self.x_label = self.create_label(self.pose_frame, "x:")
        self.y_label = self.create_label(self.pose_frame, "y:")
        self.theta_label = self.create_label(self.pose_frame, "\u03B8:")

    def create_referencia_frame(self):
        self.referencia_frame = tk.LabelFrame(
            self.root, text="Referencia", padx=10, pady=10
        )
        self.referencia_frame.pack(side="left", padx=10, pady=10)

        self.referencia_x_label = self.create_label(self.referencia_frame, "x:")
        self.referencia_y_label = self.create_label(self.referencia_frame, "y:")
        self.referencia_theta_label = self.create_label(
            self.referencia_frame, "\u03B8:"
        )

    def create_video_frame(self):
        self.video_frame = tk.Frame(self.root)
        self.video_frame.pack(side="right", padx=10, pady=10)

        self.video_label = tk.Label(self.video_frame)
        self.video_label.pack()

    def create_label(self, parent, text):
        label = tk.Label(parent, text=text)
        label.pack(side="top", padx=5, pady=5)
        return label

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

        except cv2.error as e:
            print(f"An error occurred: {e}")

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
            referencia_values = data.get('Referencia')

            if pose_values:
                pose_values = pose_values.split(', ')
                if len(pose_values) == 3:
                    x, y, theta = map(float, pose_values)
                    self.x_label.configure(text=f"x: {x:.3f}")
                    self.y_label.configure(text=f"y: {y:.3f}")
                    self.theta_label.configure(text=f"\u03B8: {theta:.3f}")
                else:
                    raise ValueError("Invalid content format or missing values")

            if referencia_values:
                referencia_values = referencia_values.split(', ')
                if len(referencia_values) == 3:
                    x, y, theta = map(float, referencia_values)
                    self.referencia_x_label.configure(text=f"x: {x:.3f}")
                    self.referencia_y_label.configure(text=f"y: {y:.3f}")
                    self.referencia_theta_label.configure(text=f"\u03B8: {theta:.3f}")
                else:
                    raise ValueError("Invalid content format or missing values")

        except (ValueError, json.JSONDecodeError) as e:
            print(f"Error updating page content: {e}")

    def start_keyboard_control(self):
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.key_state = {
            "w": False,
            "s": False,
            "a": False,
            "d": False
        }
        self.update_robot_command()

    def on_key_press(self, event):
        key = event.keysym.lower()

        if self.state == "Driving":
            if key in self.key_state:
                self.key_state[key] = True

        self.update_robot_command()

    def on_key_release(self, event):
        key = event.keysym.lower()

        if self.state == "Driving":
            if key in self.key_state:
                self.key_state[key] = False

        self.update_robot_command()

    def update_robot_command(self):
        key_combinations = {
            ("w",): (self.speed, self.speed),
            ("s",): (-self.speed, -self.speed),
            ("a",): (-self.speed, self.speed),
            ("d",): (self.speed, -self.speed),
            ("w", "a"): (0, self.speed),
            ("w", "d"): (self.speed, 0),
            ("s", "a"): (0, -self.speed),
            ("s", "d"): (-self.speed, 0),
        }

        keys = tuple(key for key, state in self.key_state.items() if state)
        l, r = key_combinations.get(keys, (0, 0))

        url = f"{self.ip}/slam?type=vel&l={l}&r={r}"
        response = requests.get(url)
        if response.status_code == 200:
            print("Robot command sent successfully")
        else:
            print(f"Failed to send robot command: {response.status_code}")


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
        print(url)
        response = requests.get(url)
        if response.status_code == 200:
            print("Robot command sent successfully")
        else:
            print(f"Failed to send robot command: {response.status_code}")

    def start(self):
        # Start the Tkinter event loop
        self.root.mainloop()






ipEsp32 = '192.168.1.85'

if __name__ == "__main__":
    interface = InterfaceModule(ipEsp32, ':81', '/robot', 100)
    interface.start()