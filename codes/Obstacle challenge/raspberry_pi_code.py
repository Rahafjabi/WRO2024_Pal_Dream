import cv2
import numpy as np
import serial
from picamera.array import PiRGBArray
import picamera
import time

# Set up the serial connection (adjust the parameters as needed)
ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)  # Adjust port as needed
flag=0

#Function to Find the Largest Contour
def find_largest_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None, 0
    largest_contour = max(contours, key=cv2.contourArea)
    largest_area = cv2.contourArea(largest_contour)
    return largest_contour, largest_area

#Function to Determine Object Position
def position(image, max_contour):
    image_height, image_width = image.shape[:2]
    midpointx = image_width // 2
    midpointy = image_height // 2
    x, y, w, h = cv2.boundingRect(max_contour)
    contour_center_x = x + w // 2
    contour_center_y = y + h // 2
    side = "left" if contour_center_x < midpointx else "right"
    sidey = "up" if contour_center_y < midpointy else "down"
    return midpointx, image_height, contour_center_x, y + h

#Function to Check Frame for Object Color and Position
def check_frame(image):
    print("chek")
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_image)
    v_equalized = cv2.equalizeHist(v)
    hsv_equalized = cv2.merge([h, s, v_equalized])

    lower_color = np.array([68, 200, 20])
    upper_color = np.array([78, 255, 40])
    redl = np.array([0, 1, 1])
    redu = np.array([10, 255, 255])
    lower_red2 = np.array([170, 1, 1])
    upper_red2 = np.array([180, 255, 255])
    greenl = np.array([67, 170, 25])
    greenu = np.array([83, 255, 80])

    mask = cv2.inRange(hsv_image, lower_color, upper_color)
    mask2 = cv2.inRange(hsv_image, redl, redu)
    mask3 = cv2.inRange(hsv_image, greenl, greenu)
    mask4 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask5 = cv2.bitwise_or(mask2, mask4)

    mm = mask5
    selected_objects = cv2.bitwise_and(image, image, mask=mm)
    red_contour, red_area = find_largest_contour(mask5)
    green_contour, green_area = find_largest_contour(mask3)

    if red_area > green_area:
        largest_color = "Red"
        largest_contour = red_contour
    else:
        largest_color = "Green"
        largest_contour = green_contour

    imgx, imgh, contour_x, contourbottom = position(image, largest_contour)
    if largest_color == "Red" and contourbottom > 350 and cv2.contourArea(largest_contour) > 7000:
        if contour_x <= imgx and contour_x >= imgx / 2:
            return "Redmove1\n"
        elif contour_x <= imgx * 1.5 and contour_x > imgx:
            return "Redmove2\n"
        elif contour_x > imgx * 1.5:
            return "Redmove3\n"
    elif largest_color == "Green" and contourbottom > 350 and cv2.contourArea(largest_contour) > 7000:
        if contour_x <= imgx and contour_x >= imgx / 2:
            return "Greenmove3\n"
        elif contour_x <= imgx * 1.5 and contour_x > imgx:
            return "Greenmove2\n"
        elif contour_x > imgx * 1.5:
            return "Greenmove1\n"
    else:
        return "none"

#Function to Capture Video and Process Frames:
def take_video():
    message = "started\n"
    ser.write(message.encode('utf-8'))
    ser.flush()
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)  # You can adjust the resolution
        camera.framerate = 30
        raw_capture = PiRGBArray(camera, size=(640, 480))

    
        time.sleep(0.1)  # Give the camera some time to warm up
        camera.start_preview()
        try:
            for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
                image = frame.array
                pic = check_frame(image)
                print(pic)

                if pic != "none":
                    ser.write(str(pic).encode('utf-8'))
                    ser.flush()
                    #stop_capture(camera)
                    #break
                cv2.imshow("Video", image)
                raw_capture.truncate(0)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    #stop_capture(camera)
                    break
        except Exception as e:
        # Generic exception handler
            print(f"An unexpected error occurred: {e}")
        finally:
            camera.stop_preview()
            camera.close()
            cv2.destroyAllWindows()

#Function to Listen for Serial Commands    
def listen():
    global ser
    global flag
    if not ser.is_open:
            ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)  # Adjust port as needed
    ser.flush()
    ser.reset_input_buffer()
    try:
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"Received: {line}")
                    if line == "request":
                        take_video()
    except KeyboardInterrupt:
        print("Program interrupted")
    finally:
        ser.close()
        print("Serial connection closed")


listen()
