import time

import pypylon.pylon as pylon
import matplotlib.pyplot as plt
from ultralytics import YOLO
import cv2
import math

cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
cam.Open()

print(f"Theoretical FPS: {cam.BslResultingAcquisitionFrameRate.Value}")

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

cam.LineSelector.SetValue("Line2")
cam.LineMode.SetValue("Output")
cam.LineSource.SetValue("UserOutput1")
cam.UserOutputSelector.SetValue("UserOutput1")

cam.ChunkModeActive.Value = True
cam.ChunkSelector.Value = "Timestamp"
cam.ChunkEnable.Value = True
cam.ChunkSelector.Value = "PayloadCRC16"
cam.ChunkEnable.Value = True


model = YOLO("../yolo11n.pt")

# Update calculation
start_time = time.time()
previous_time = time.time()
update_time = 1
internal_fps = 0
prev_timestamp = 0
fps_disp = ""


cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

while cam.IsGrabbing():
    grab_result = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grab_result.GrabSucceeded():
        TIME = time.time() - previous_time

        if grab_result.HasCRC() and grab_result.CheckCRC() == True:
            timestamp = grab_result.ChunkTimestamp.Value
            internal_fps = 1 / ((timestamp - prev_timestamp) * 1e-9)
            prev_timestamp = timestamp

        if (TIME) >= update_time:
            previous_time = time.time()
            fps_disp = f"FPS: {internal_fps:.2f}"
            print(fps_disp)

        image = converter.Convert(grab_result)
        frame = image.GetArray()

        # coordinates
        results = model.track(frame, persist=True, conf=0.5, verbose=False)[0]

        # Visualize the results on the frame
        # annotated_frame = results.plot()

        # if not cam.UserOutputValue.Value and 0 in results.boxes.cls:
        #     cam.UserOutputValue.SetValue(True)
        #     print("Signal high")
        # elif not 0 in results.boxes.cls:
        #     cam.UserOutputValue.SetValue(False)

        # cv2.putText(frame, fps_disp, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # cv2.imshow("YOLO11 Tracking", annotated_frame)

        # Break the loop if 'q' is pressed
        #! Reduces FPS significantly ~100 FPS
        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #     break

cam.Close()
cv2.destroyAllWindows()
