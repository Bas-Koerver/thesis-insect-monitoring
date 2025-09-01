import time

import pypylon.pylon as py
import matplotlib.pyplot as plt
from ultralytics import YOLO
import cv2
import math

cam = py.InstantCamera(py.TlFactory.GetInstance().CreateFirstDevice())
cam.Open()
cam.PixelFormat.SetValue("BGR8")

cam.LineSelector.SetValue("Line2")
cam.LineMode.SetValue("Output")
cam.LineSource.SetValue("UserOutput1")
cam.UserOutputSelector.SetValue("UserOutput1")


model = YOLO("yolo11n.pt")

# FPS calculation
start_time = time.time()
# Update time
update_time = 2
frames = 0
fps = 0

while True:
    res = cam.GrabOne(1000)
    frame = res.GetArray()

    # coordinates
    results = model.track(frame, persist=True, conf=0.5)[0]

    # Visualize the results on the frame
    annotated_frame = results.plot()

    if not cam.UserOutputValue.Value and 0 in results.boxes.cls:
        cam.UserOutputValue.SetValue(True)
        print("Signal high")
    elif not 0 in results.boxes.cls:
        cam.UserOutputValue.SetValue(False)


    # FPS calculation
    frames += 1
    duration = time.time() - start_time

    if duration > update_time:
        fps = frames / duration
        frames = 0
        start_time = time.time()

    fps_disp = "FPS: " + str(fps)[:5]


    cv2.putText(frame, fps_disp, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("YOLO11 Tracking", annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cam.Close()
cv2.destroyAllWindows()