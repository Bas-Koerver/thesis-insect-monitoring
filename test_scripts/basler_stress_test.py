import time

import cv2
import pandas as pd
import math
from pypylon import pylon

cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
cam.Open()

cam.UserSetSelector.SetValue("Default")
cam.UserSetLoad.Execute()
cam.PixelFormat.SetValue("BayerRG8")

print(f"Theoretical FPS: {cam.BslResultingAcquisitionFrameRate.Value}")

# converting to opencv bgr format
converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

def stress_test(test_duration: int, record_sensor_temp: bool = False, view_feed: bool = False, debug: bool = False):
    data = []

    cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    start_time = time.time()
    previous_time = time.time()
    # Update calculations time in seconds
    update_time = 1
    frame_count = 0
    fps_disp = ""
    coreboard_temp_disp = ""
    sensor_temp_disp = ""

    while cam.IsGrabbing():
        grabResult = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if time.time() - start_time >= test_duration:
            print("Test duration reached, exiting...")
            break

        if grabResult.GrabSucceeded():
            frame_count += 1
            TIME = time.time() - previous_time

            if (TIME) >= update_time:
                # Calculate the FPS value
                FPS = frame_count / (TIME)
                frame_count = 0
                previous_time = time.time()
                fps_disp = f"FPS: {FPS:.2f}"

                # Get the coreboard temperature
                cam.DeviceTemperatureSelector.Value = "Coreboard"
                coreboard_temp = cam.DeviceTemperature.Value
                coreboard_temp_disp = f"Coreboard: {coreboard_temp:.2f} C"

                if record_sensor_temp:
                    # Get the sensor temperature
                    cam.StopGrabbing()
                    cam.DeviceTemperatureSelector.Value = "Sensor"
                    sensor_temp = cam.DeviceTemperature.Value
                    sensor_temp_disp = f"Sensor: {sensor_temp:.2f} C"
                    cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

                data.append({
                    "time": time.time() - start_time,
                    "fps": FPS,
                    "coreboard_temp": coreboard_temp,
                    "sensor_temp": sensor_temp if record_sensor_temp else None
                })

                if debug:
                    # Print the info to console
                    print(fps_disp)
                    print(coreboard_temp_disp)
                    print(sensor_temp_disp)

            if view_feed:
                # Access the image data
                image = converter.Convert(grabResult)
                frame = image.GetArray()

                # Add info to image and display
                cv2.putText(
                    frame,
                    fps_disp,
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    coreboard_temp_disp,
                    (510, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    sensor_temp_disp,
                    (1010, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2,
                )
                cv2.namedWindow('Basler camera stress test', cv2.WINDOW_NORMAL)
                cv2.imshow('Basler camera stress test', frame)
                k = cv2.waitKey(1)
                if k == 27:
                    break
        grabResult.Release()

    # Releasing the resources
    cam.StopGrabbing()
    cv2.destroyAllWindows()

    return data

def cooldown():
    # Amount of times the temperature needs to be close to each other before stopping the cooldown
    stable_temp_count = 5
    count = 0
    coreboard_temp = 0

    # Turn off the sensor.
    cam.BslSensorOff.Execute()
    cam.DeviceTemperatureSelector.Value = "Coreboard"

    while True:
        prev_coreboard_temp = coreboard_temp
        coreboard_temp = cam.DeviceTemperature.Value
        print(coreboard_temp)
        print(count)

        if math.isclose(coreboard_temp, prev_coreboard_temp, abs_tol=5e-2):
            count += 1
            if count >= stable_temp_count:
                print("Coreboard temperature stabilized, ending cooldown.")
                return
        else:
            count = 0
        time.sleep(1)


if __name__ == "__main__":
    # Test durations in seconds
    durations = [10, 30, 60, 300, 600, 1200, 1800]
    with_sensor_temp = [False, True]

    for with_sensor_temp in with_sensor_temp:
        for duration in durations:
            print(f"Starting stress test for {duration} seconds..., with sensor_temp: {with_sensor_temp}")
            data = stress_test(test_duration=duration, record_sensor_temp=True)
            df = pd.DataFrame(data)
            df.to_csv(f"basler_stress_test_{duration}_{with_sensor_temp}.csv", index=False)
            print(f"Stress test for {duration} seconds completed and data saved.")
            print("Cooling down before next test...")

            # Cooldown
            cooldown()
