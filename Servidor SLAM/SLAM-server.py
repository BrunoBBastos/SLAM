import cv2
import requests
import threading

def fetch_page_content(url):
    try:
        response = requests.get(url)
        if response.status_code == 200:
            print(response.text)
        else:
            print(f"Request failed with status code: {response.status_code}")
    except requests.RequestException as e:
        print(f"An error occurred: {e}")

def capture_and_display_video(url):
    try:
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            print("Failed to open video stream")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame from video stream")
                break

            cv2.imshow("Video", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
    except cv2.error as e:
        print(f"An error occurred: {e}")

def make_requests(ip, interval_ms):
    page_url = f"http://{ip}/robot"
    video_url = f"http://{ip}:81"

    while True:
        threading.Thread(target=fetch_page_content, args=(page_url,), daemon=True).start()
        threading.Thread(target=capture_and_display_video, args=(video_url,), daemon=True).start()
        threading.Event().wait(interval_ms / 1000)  # Convert milliseconds to seconds

# Usage
ip_address = "esp32-cam"
interval = 1000  # 1000 milliseconds (1 second)

make_requests(ip_address, interval)
