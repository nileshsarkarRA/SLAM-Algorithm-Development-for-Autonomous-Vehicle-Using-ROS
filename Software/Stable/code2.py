import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

# Global variables
frame = None
histogram_lane = []
histogram_lane_end = []
left_lane = right_lane = line_center = frame_center = diff = lane_end = 0

camera = PiCamera()
camera.resolution = (400, 240)
camera.framerate = 32
raw_capture = PiRGBArray(camera, size=(400, 240))

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

# Region of Interest Points
source = np.float32([[80, 160], [300, 160], [40, 210], [340, 210]])
destination = np.float32([[130, 0], [310, 0], [130, 240], [310, 240]])

def capture():
    global frame
    for frame_raw in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        frame = frame_raw.array
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        break  # Capture one frame
    raw_capture.truncate(0)

def region_of_interest():
    global frame_perspective
    for pt in source:
        cv2.line(frame, tuple(pt), tuple(source[(source.tolist().index(pt) + 1) % len(source)]), (0, 0, 255), 1)

    matrix = cv2.getPerspectiveTransform(source, destination)
    frame_perspective = cv2.warpPerspective(frame, matrix, (400, 240))

def threshold():
    global frame_final, frame_gray, frame_threshold, frame_edge
    frame_gray = cv2.cvtColor(frame_perspective, cv2.COLOR_RGB2GRAY)
    _, frame_threshold = cv2.threshold(frame_gray, 180, 255, cv2.THRESH_BINARY)
    frame_edge = cv2.Canny(frame_gray, 150, 400)
    frame_final = cv2.add(frame_threshold, frame_edge)

def histogram():
    global histogram_lane, histogram_lane_end, lane_end
    histogram_lane = []
    histogram_lane_end = []

    for i in range(frame_final.shape[1]):
        roi_lane = frame_final[140:240, i]
        histogram_lane.append(np.sum(roi_lane) // 255)

    for i in range(frame_final.shape[1]):
        roi_lane_end = frame_final[:, i]
        histogram_lane_end.append(np.sum(roi_lane_end) // 255)

    lane_end = sum(histogram_lane_end)

def line_center():
    global line_center, diff
    line_center = (right_lane - left_lane) // 2 + left_lane
    frame_center = 216
    diff = line_center - frame_center

    cv2.line(frame_final, (line_center, 0), (line_center, 240), (255, 255, 0), 3)
    cv2.line(frame_final, (frame_center, 0), (frame_center, 240), (255, 0, 0), 3)

def line_detector():
    global left_lane, right_lane
    left_lane = np.argmax(histogram_lane[:200])
    right_lane = np.argmax(histogram_lane[240:]) + 240

    cv2.line(frame_final, (left_lane, 0), (left_lane, 240), (0, 255, 0), 3)
    cv2.line(frame_final, (right_lane, 0), (right_lane, 240), (0, 255, 0), 3)

def screen_output(direction):
    cv2.putText(frame, f"Direction: {direction}", (110, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 216, 173), 2)

# Main loop
try:
    while True:
        capture()
        region_of_interest()
        threshold()
        histogram()
        line_detector()
        line_center()

        # Determine movement based on 'diff'
        if -5 < diff < 5:
            GPIO.output(21, GPIO.LOW)
            GPIO.output(22, GPIO.LOW)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)
            screen_output("Forward")
        elif 5 <= diff < 15:
            GPIO.output(21, GPIO.HIGH)
            GPIO.output(22, GPIO.LOW)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)
            screen_output("Right")
        elif 15 <= diff < 25:
            GPIO.output(21, GPIO.LOW)
            GPIO.output(22, GPIO.HIGH)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)
            screen_output("Right")
        elif diff >= 25:
            GPIO.output(21, GPIO.HIGH)
            GPIO.output(22, GPIO.HIGH)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)
            screen_output("Right")
        elif -15 < diff <= -5:
            GPIO.output(21, GPIO.LOW)
            GPIO.output(22, GPIO.HIGH)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.LOW)
            screen_output("Left")
        elif -25 < diff <= -15:
            GPIO.output(21, GPIO.HIGH)
            GPIO.output(22, GPIO.LOW)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.LOW)
            screen_output("Left")
        elif diff <= -25:
            GPIO.output(21, GPIO.LOW)
            GPIO.output(22, GPIO.HIGH)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.LOW)
            screen_output("Left")

        # Display frames
        cv2.imshow("Frame RGB", frame)
        cv2.imshow("Perspective", frame_perspective)
        cv2.imshow("Frame Final", frame_final)

        # Break loop on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    GPIO.cleanup()
    cv2.destroyAllWindows()
