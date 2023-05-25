import cv2
import serial

# Setting up the serial connection with the Arduino
ser = serial.Serial('/dev/ttyACM0', 9600)

# Setting up the camera object called 'cap' to find OpenCV
cap = cv2.VideoCapture(0)

# QR code detection method
detector = cv2.QRCodeDetector()

# Infinite loop to keep the camera searching for data at all times
while True:
    # Capturing an image of the QR code
    _, img = cap.read()

    # Reading the QR code by detecting the bounding box coords and decoding the hidden QR data
    data, bbox, _ = detector.detectAndDecode(img)

    # Drawing a blue box around the data, and displaying the data along with the top
    if bbox is not None:
        for i in range(len(bbox)):
            cv2.line(img, tuple(bbox[i][0]), tuple(bbox[(i + 1) % len(bbox)][0]), color=(255, 0, 0), thickness=2)
        cv2.putText(img, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 250, 120), 2)

        # If the data is found, send the corresponding command to the Arduino and print the data to the terminal
        if data:
            print("Data found:", data)
            if data == '1':
                ser.write(b'1')
            elif data == '2':
                ser.write(b'1')
            elif data == '3':
                ser.write(b'1')
            else:
                ser.write(b'0')
        else:
            ser.write(b'0')

    # Displaying the live camera feed to the desktop on Raspberry Pi OS preview
    cv2.imshow("Code detector", img)

    # Pressing 'q' on the keyboard to stop the code
    if cv2.waitKey(1) == ord("q"):
        break

# Releasing the camera and closing all the windows that the above has created
cap.release()
cv2.destroyAllWindows()
ser.close()
