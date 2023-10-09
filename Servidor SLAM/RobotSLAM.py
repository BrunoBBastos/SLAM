import numpy as np
import requests
import json
import threading
import time
import cv2
import subprocess # pinging

import qrcode as qc


class Robot:

    def __init__(self, ip = None):

        self.x = 0
        self.y = 0
        self.orientation = 0
        self._state = np.zeros((3, 1))
        self.pose = np.array([[0.0], [0.0], [0.0]])
        self.odometryDelta = np.zeros((3, 1))

        self._distanceSensorMeasurement = -1
        self.vision = qc.qrcode()
        self.stream_frames = None
        self.stream_new_frames = False

        self.ip = ip
        self.video_url = None
        self.state_url = None
        if self.ip is not None:
            self.video_url = f"http://{self.ip}:81"
            self.state_url = f"http://{self.ip}/robot"

        self.connection_status = False
        self.start_ping_check()

        print("Connecting...")
        while self.connection_status == False:
            time.sleep(1)
        print("Connected")

        self.start_url_content_fetching()
        self.start_video_stream()
        self.start_distance_sensor()

#------------------------------------------------------------- METHODS

    @property
    def state(self):
        return self._state
    
    @state.setter
    def state(self, new_pose):
        if len(new_pose) == 3:
            self._state[0, 0] = new_pose[0]  # Set x
            self._state[1, 0] = new_pose[1]  # Set y
            self._state[2, 0] = new_pose[2]  # Set theta
        else:
            raise ValueError("New pose must be a 3-element list or array [x, y, theta].")

    @property
    def distanceSensorMeasurement(self):
        if (self._distanceSensorMeasurement != 0) & (self._distanceSensorMeasurement != -1):
            distance = np.array([[[self._distanceSensorMeasurement[0]], [self._distanceSensorMeasurement[1]]], self._distanceSensorMeasurement[2]]) 
        else:
            distance = [-1, -1]        
        self._distanceSensorMeasurement = -1
        return distance
    
    @distanceSensorMeasurement.setter
    def distanceSensorMeasurement(self, dist_ang_id):
        self._distanceSensorMeasurement = dist_ang_id
    
    def computeOdometry(self, Odometry):
        
        self.odometryDelta += Odometry
        self._state += self.odometryDelta

    def readOdometry(self):
        odometry = np.copy(self.odometryDelta)
        self.odometryDelta = np.zeros((3, 1))
        return odometry

    def printState(self):
        state = self._state
        s = ''
        s += f'x: {state[0,0]:.3f}, '
        s += f'y: {state[1,0]:.3f}, '
        s += f'\u03B8: {state[2,0]:.3f}'
        # s+= f' {self.pose[0,0]}, {self.pose[1,0]}, {self.pose[2,0]}'
        print(s)
    
#------------------------------------------------------------- WI-FI CONNECTION
    def start_ping_check(self):
        threading.Thread(target=self.pingServer, daemon=True).start()

    def pingServer(self):
        while True:
            try:
                # Use the ping command to send ICMP echo requests to the server
                result = subprocess.run(['ping', self.ip], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=50)
                
                if result.returncode == 0:
                    # Server is reachable
                    self.connection_status = True
                else:
                    # Server is unreachable
                    self.connection_status = False
                
            except Exception as e:
                return f"An error occurred: {str(e)}"
            
            time.sleep(1)

    def start_url_content_fetching(self):
        #Defining some parameters and states the robot should run by:
        #Start in 'drive' mode
        url = f"http://{self.ip}/slam?type=DRV"
        response = requests.get(url)
        if response.status_code != 200:
            print(f"Failed to send robot command: {response.status_code}")

        threading.Thread(target=self.state_url_fetching_worker, daemon=True).start()

    def state_url_fetching_worker(self):
        try:
            while True:
                response = requests.get(self.state_url)
                if response.status_code == 200:
                    content = response.text
                    self.update_from_state_url(content)
                else:
                    print(f"Request failed with status code: {response.status_code}")

        except requests.RequestException as e:
            print(f"An error occurred: {e}")
        time.sleep(0.1)

    def update_from_state_url(self, content):
        try:
            data = json.loads(content)
            odometry_values = data.get('odometria')
            pose_values = data.get('pose')

            # if odometry_values:
            #     odometry_values = odometry_values.split(', ')
            #     if len(odometry_values) == 3:
            #         x, y, theta = map(float, odometry_values)
            #         self.computeOdometry(np.array([[x], [y], [theta]]))
            #     else:
            #         raise ValueError("Invalid content format or missing values")
                
            if pose_values:
                pose_values = pose_values.split(', ')
                if len(pose_values) == 3:
                    x, y, theta = map(float, pose_values)
                    odometry = np.array([[x], [y], [theta]]) - self.pose
                    self.pose = np.array([[x], [y], [theta]])
                    self.computeOdometry(odometry)
                else:
                    raise ValueError("Invalid content format or missing values")

        except (ValueError, json.JSONDecodeError) as e:
            print(f"Error updating from page content: {e}")

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
                if not ret:
                    print("Error: Could not read frame.")
                    break
                
                self.stream_frames = (frame, ret)
                self.stream_new_frames = True

                cv2.imshow('Video Stream', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()
            print("Closing stream")

        except cv2.error as e:
            print(f"An error occurred: {e}")

#------------------------------------------------------------ SENSOR
    def start_distance_sensor(self):
        # threading.Thread(target=self.run_detection).start()
        threading.Thread(target=self.run_detection, daemon=True).start()

    def run_detection(self):
        while True:
            if self.stream_new_frames:
                self._distanceSensorMeasurement = self.vision.detectAnddecod(self.stream_frames[0], self.stream_frames[1])
                self.stream_new_frames = False

#------------------------------------------------------------ ROBOT MAIN
    def start_robot_mainloop(self):
        threading.Thread(target=self.mainLoop, daemon=True).start()

    def mainLoop(self):
        while True:
            pass

#---------------------------------------------------------------------------------------------- MAIN
if __name__ == "__main__":
    robot = Robot("10.0.0.105")
    while robot.connection_status == False:
        pass

    while True:
        pass
