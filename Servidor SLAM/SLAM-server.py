import tkinter as tk
import cv2
import requests
from PIL import Image, ImageTk
import threading
import json

class Application(tk.Tk):
    def __init__(self, ip, video_url, page_url, interval_ms):
        super().__init__()
        self.ip = ip
        self.video_url = video_url
        self.page_url = page_url
        self.interval_ms = interval_ms

        self.video_label = tk.Label(self)
        self.video_label.grid(row=0, column=0, padx=10, pady=10)

        self.pose_frame = tk.LabelFrame(self, text="Pose")
        self.pose_frame.grid(row=0, column=1, padx=10, pady=10)

        self.x_label = tk.Label(self.pose_frame, text="x:")
        self.x_label.grid(row=0, column=0, sticky="e")

        self.y_label = tk.Label(self.pose_frame, text="y:")
        self.y_label.grid(row=1, column=0, sticky="e")

        self.theta_label = tk.Label(self.pose_frame, text="theta:")
        self.theta_label.grid(row=2, column=0, sticky="e")

        self.start_video_stream()
        self.start_page_content_fetching()

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

                # Wait for the specified interval
                threading.Event().wait(self.interval_ms / 1000)

        except requests.RequestException as e:
            print(f"An error occurred: {e}")

    def update_page_content(self, content):
        try:
            data = json.loads(content)
            pose_values = data.get('Pose')
            if pose_values:
                pose_values = pose_values.split(', ')
                if len(pose_values) == 3:
                    x, y, theta = map(float, pose_values)

                    self.x_label.configure(text=f"x: {x}")
                    self.y_label.configure(text=f"y: {y}")
                    self.theta_label.configure(text=f"theta: {theta}")
                else:
                    raise ValueError("Invalid content format or missing values")

        except (ValueError, json.JSONDecodeError) as e:
            print(f"Error updating page content: {e}")


    def start(self):
        self.mainloop()


# Usage
# ip_address = "192.168.4.85"
# ip_address = "192.168.1.85"
ip_address = "10.0.0.103"
video_url = f"http://{ip_address}:81"
page_url = f"http://{ip_address}/robot"
page_interval = 100  # 100 milliseconds


if __name__ == '__main__':
    app = Application(ip_address, video_url, page_url, page_interval)
    app.start()
