import math
import time

import cv2
import pandas as pd
from pypylon import pylon

cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
cam.Open()

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

    # Initialize variables
    frame_count = 0
    prev_timestamp = 0
    fps_disp = ""
    coreboard_temp_disp = ""
    sensor_temp_disp = ""

    while cam.IsGrabbing():
        grab_result = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if time.time() - start_time >= test_duration:
            print("Test duration reached, exiting...")
            break

        if grab_result.GrabSucceeded():
            frame_count += 1
            TIME = time.time() - previous_time

            # Calculate the fps value based on frame timestamps
            if grab_result.HasCRC() and grab_result.CheckCRC() == True:
                timestamp = grab_result.ChunkTimestamp.Value
                internal_fps = 1 / ((timestamp - prev_timestamp) * 1e-9)
                prev_timestamp = timestamp

            # Calculate some values every update_time seconds and append to data dict
            if (TIME) >= update_time:
                # Calculate the external fps value
                external_fps = frame_count / (TIME)
                frame_count = 0
                previous_time = time.time()
                fps_disp = f"FPS: {external_fps:.2f}"

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
                    "external_fps": external_fps,
                    "internal_fps": internal_fps if grab_result.HasCRC() and grab_result.CheckCRC() == True else None,
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
                image = converter.Convert(grab_result)
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
        grab_result.Release()

    # Releasing the resources
    cam.StopGrabbing()
    cv2.destroyAllWindows()

    return data

def temp_stabilization():
    # Amount of times the temperature needs to be close to each other before stopping the cooldown
    stable_temp_count = 8
    count = 0
    coreboard_temp = 0

    # Turn off the sensor.
    cam.BslSensorOff.Execute()
    cam.DeviceTemperatureSelector.Value = "Coreboard"

    while True:
        prev_coreboard_temp = coreboard_temp
        coreboard_temp = cam.DeviceTemperature.Value
        print(coreboard_temp)

        if math.isclose(coreboard_temp, prev_coreboard_temp, abs_tol=0.6e-2):
            count += 1
            print(f"{count=}")
            if count >= stable_temp_count:
                print("Coreboard temperature stabilized, ending cooldown.")
                return
        else:
            count = 0
            print(f"{count=}")
        time.sleep(4)

def set_chunkmode():
    cam.ChunkModeActive.Value = True
    cam.ChunkSelector.Value = "Timestamp"
    cam.ChunkEnable.Value = True
    cam.ChunkSelector.Value = "PayloadCRC16"
    cam.ChunkEnable.Value = True


if __name__ == "__main__":
    print("Stabilising temperature")
    temp_stabilization()

    # Test durations in seconds
    durations = [10, 30, 60, 300, 600, 1200, 1800, 3600]
    external_cooling = True

    # Test with compression beyond
    cam.UserSetSelector.SetValue("UserSet1")
    cam.UserSetLoad.Execute()
    set_chunkmode()

    for with_sensor_temp in [False, True]:
        for duration in durations:
            print(f"Starting stress test for {duration} seconds..., with sensor_temp: {with_sensor_temp}")
            data = stress_test(test_duration=duration, record_sensor_temp=with_sensor_temp)
            df = pd.DataFrame(data)
            df.to_csv(f"./test_data/basler_stress_test_{duration}_sensortemp_{with_sensor_temp}_cooling_{external_cooling}_CB.csv", index=False)
            print(f"Stress test for {duration} seconds completed and data saved.")

            print("Cooling down before next test...")
            temp_stabilization()

    # Test with default settings
    # cam.UserSetSelector.SetValue("Default")
    # cam.UserSetLoad.Execute()
    # set_chunkmode()
    #
    # for with_sensor_temp in [False]:
    #     for duration in durations:
    #         print(f"Starting stress test for {duration} seconds..., with sensor_temp: {with_sensor_temp}")
    #         data = stress_test(test_duration=duration, record_sensor_temp=with_sensor_temp)
    #         df = pd.DataFrame(data)
    #         df.to_csv(f"./test_data/basler_stress_test_{duration}_sensortemp_{with_sensor_temp}_cooling_{external_cooling}_Default.csv", index=False,)
    #         print(f"Stress test for {duration} seconds completed and data saved.")
    #
    #         print("Cooling down before next test...")
    #         temp_stabilization()
