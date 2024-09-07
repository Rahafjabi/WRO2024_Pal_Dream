import cv2
import numpy as np
import serial
from picamera.array import PiRGBArray
import picamera
import time

ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)  # Adjust port as needed
flag=0
camera =None
color_obj=''
flag=0
# Set up the serial connection (adjust the parameters as needed)
ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)  # Adjust port as needed

def find_largest_contour(mask):
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Return None if no contours are found
    if len(contours) == 0:
        return None, 0

    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    largest_area = cv2.contourArea(largest_contour)
    
    return largest_contour, largest_area

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

def check_frame(image):
    global color_obj
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

    elif green_area>red_area:
        largest_color = "Green"
        largest_contour = green_contour
    
    else:
        return "none"
    
    imgx, imgh, contour_x, contourbottom = position(image, largest_contour)
    if largest_color == "Red" and contourbottom > 350 and cv2.contourArea(largest_contour) > 7000:
        color_obj='red'
        return color_obj
    
    elif largest_color == "Green" and contourbottom > 350 and cv2.contourArea(largest_contour) > 7000:
        color_obj='green'
        return color_obj
        
    else:
        return "none"

def take_video():
    message = "started\n"
    ser.write(message.encode('utf-8'))
    ser.flush()
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)  # You can adjust the resolution
        camera.framerate = 30
        raw_capture = PiRGBArray(camera, size=(640, 480))

    # raw_capture.truncate(0)
        time.sleep(0.1)  # Give the camera some time to warm up
        camera.start_preview()
        try:
            for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
                image = frame.array
                pic = check_frame(image)
                print(pic)

                if pic != "none":
                    if pic=='red':
                        ser.write(b"red\n")
                    elif pic=='green':
                        ser.write(b"green\n")

                    #ser.write(str(pic).encode('utf-8'))
                    #ser.write(b"stop\n")
                    ser.flush()
                    time.sleep(0.5)
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        print(f"Received: {line}")
                    #stop_capture(camera)
                    #break
                    return   
                #cv2.imshow("Video", image)
                raw_capture.truncate(0)
                #if cv2.waitKey(1) & 0xFF == ord('q'):
                    #stop_capture(camera)
                #break
        except Exception as e:
        # Generic exception handler
            print(f"An unexpected error occurred: {e}")
        finally:
            camera.stop_preview()
            camera.close()
            cv2.destroyAllWindows()

#----------------------------------------------------------------------------------------------------------------------------Turn

def see_edge(image,max_contour,color):
    image_height, image_width = image.shape[:2]
    midpointx = image_width // 2
    midpointy = image_height//2
    x, y, w, h = cv2.boundingRect(max_contour)
    # Calculate the center of the contour
    left=x
    right=x+w
    bottom=y+h
    upper=y
    print("color=",color)
    print("image_width =",image_width,"image_hieght=",image_height,"contour_left=",left,"right=",right,"contourbottom=",bottom,"upper=",upper,"area=",cv2.contourArea(max_contour))
    if color=="Red":
        if right>image_width//8:
            return True
        else:
            return False
    elif color=="Green":
        if left< image_width*7//8:
            return True
        else:
            return False

def check_object_edge(image):
    # Convert the image to the HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    
    
    # Split the HSV channels
    h, s, v = cv2.split(hsv_image)
    
    # Apply histogram equalization to the Value channel to enhance contrast
    v_equalized = cv2.equalizeHist(v)
    
    # Merge the equalized V channel back with the original H and S channels
    hsv_equalized = cv2.merge([h, s, v_equalized])
    
    
    # Define the color range for green (adjusted range)
    lower_color = np.array([68, 200, 20])
    upper_color = np.array([78, 255, 40])
    #color for red
    redl=np.array([0, 1, 1])
    redu=np.array([10, 255, 255])
    
    lower_red2 = np.array([170, 1, 1])
    upper_red2 = np.array([180, 255, 255])
    
    greenl=np.array([67, 170, 25])
    greenu=np.array([83, 255, 80])
    
    
    # Create a mask for the specific color range
    mask = cv2.inRange(hsv_image, lower_color, upper_color)
    mask2 = cv2.inRange(hsv_image, redl, redu)
    mask3 = cv2.inRange(hsv_image, greenl, greenu)
    mask4= cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask5 = cv2.bitwise_or(mask2, mask4)
    mm=mask5
    
    # Apply the mask to the original image to extract the desired color regions
    selected_objects = cv2.bitwise_and(image, image, mask=mm)
    
    # Find the largest red object
    red_contour, red_area = find_largest_contour(mask5)
    
        # Find the largest green object
    green_contour, green_area = find_largest_contour(mask3)
     # Compare areas to find the largest object
    lergest_color=None
    largest_contour=None
    if red_area > green_area:
       largest_color = "Red"
       largest_contour = red_contour
       print("red")
    elif green_area>red_area:
       largest_color = "Green"
       largest_contour = green_contour
       print("green")
    else:
        return False

    result=see_edge(image,largest_contour,largest_color)

    return result

def detect():
    message="started_detect\n"
    ser.write(message.encode('utf-8'))
 
    ser.flush()
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)  # You can adjust the resolution
        camera.framerate = 30
        raw_capture = PiRGBArray(camera, size=(640, 480))

    # raw_capture.truncate(0)
        time.sleep(0.1)  # Give the camera some time to warm up
        camera.start_preview()
        try:
            for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
                image = frame.array
                pic = check_object_edge(image)
                print("pic=",pic)

                if pic == False:
                    #ser.write(str(pic).encode('utf-8'))
                    ser.write(b"stop\n")
                    ser.flush()
                    time.sleep(0.5)
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        print(f"Received: {line}")
                    #stop_capture(camera)
                    return 
                cv2.imshow("Video", image)
                raw_capture.truncate(0)
                '''
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    #stop_capture(camera)
                    break
                '''
        except Exception as e:
        # Generic exception handler
            print(f"An unexpected error occurred: {e}")
        finally:
            camera.stop_preview()
            camera.close()
            cv2.destroyAllWindows()

#----------------------------------------------------------------------------------------------------------------------------move

def see_edge_move(image,max_contour,color):
    image_height, image_width = image.shape[:2]
    midpointx = image_width // 2
    midpointy = image_height//2
    x, y, w, h = cv2.boundingRect(max_contour)
    # Calculate the center of the contour
    left=x
    right=x+w
    bottom=y+h
    upper=y
    print("color=",color)
    print("image_width =",image_width,"image_hieght=",image_height,"contour_left=",left,"right=",right,"contourbottom=",bottom,"upper=",upper,"area=",cv2.contourArea(max_contour))
    if color=="Red":
        if right>10:
            return True
        else:
            return False
    elif color=="Green":
        if left< image_width-10:
            return True
        else:
            return False
    else:
        return False

def check_object_edge_move(image):#check_frame
    # Convert the image to the HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    
    
    # Split the HSV channels
    h, s, v = cv2.split(hsv_image)
    
    # Apply histogram equalization to the Value channel to enhance contrast
    v_equalized = cv2.equalizeHist(v)
    
    # Merge the equalized V channel back with the original H and S channels
    hsv_equalized = cv2.merge([h, s, v_equalized])
    
    
    # Define the color range for green (adjusted range)
    lower_color = np.array([68, 200, 20])
    upper_color = np.array([78, 255, 40])
    #color for red
    redl=np.array([0, 1, 1])
    redu=np.array([10, 255, 255])
    
    lower_red2 = np.array([170, 1, 1])
    upper_red2 = np.array([180, 255, 255])
    
    greenl=np.array([67, 170, 25])
    greenu=np.array([83, 255, 80])
    
    
    # Create a mask for the specific color range
    mask = cv2.inRange(hsv_image, lower_color, upper_color)
    mask2 = cv2.inRange(hsv_image, redl, redu)
    mask3 = cv2.inRange(hsv_image, greenl, greenu)
    mask4= cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask5 = cv2.bitwise_or(mask2, mask4)
    mm=mask5
    
    # Apply the mask to the original image to extract the desired color regions
    selected_objects = cv2.bitwise_and(image, image, mask=mm)
    
    # Find the largest red object
    red_contour, red_area = find_largest_contour(mask5)
    
        # Find the largest green object
    green_contour, green_area = find_largest_contour(mask3)
     # Compare areas to find the largest object
    lergest_color=None
    largest_contour=None
    if red_area > green_area:
       largest_color = "Red"
       largest_contour = red_contour
       print("red")
    elif green_area>red_area:
       largest_color = "Green"
       largest_contour = green_contour
       print("green")
    
    else:
        return False
    
    result=see_edge_move(image,largest_contour,largest_color)
    
    if largest_contour is not None:
            cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 3)
    
    #image_thread = threading.Thread(target=show_image, args=(image,))
    #image_thread.start()
    #cv2.imshow('Detected Objects', image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
       
    return result

def detect_move():  #take_video
    message="started_move\n"
    ser.write(message.encode('utf-8'))
 
    ser.flush()
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)  # You can adjust the resolution
        camera.framerate = 30
        raw_capture = PiRGBArray(camera, size=(640, 480))

    # raw_capture.truncate(0)
        time.sleep(0.1)  # Give the camera some time to warm up
        camera.start_preview()
        try:
            for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
                image = frame.array
                pic = check_object_edge_move(image)
                print(pic)

                if pic == False:
                    #ser.write(str(pic).encode('utf-8'))
                    ser.write(b"stop\n")
                    ser.flush()
                    time.sleep(0.5)
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        print(f"Received: {line}")
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

###########            
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
                print("hh")
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"Received: {line}")
                    if line == "request":
                        take_video()
                    elif line=="Right_Rotate":
                        detect()
                    elif line=="Left_Rotate":
                        detect()
                    elif line=="Move1":
                        detect_move()
                    
    except KeyboardInterrupt:
        print("Program interrupted")
    finally:
        ser.close()
        print("Serial connection closed")


listen()
    
