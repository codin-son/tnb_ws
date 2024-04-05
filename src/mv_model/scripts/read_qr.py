import cv2
from pyzbar.pyzbar import decode

# Initialize camera capture
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Decode QR codes
    decoded_objects = decode(gray)

    # Loop over detected objects
    for obj in decoded_objects:
        # Extract QR code data
        qr_data = obj.data.decode('utf-8')
        
        # Draw rectangle around the QR code
        (x, y, w, h) = obj.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Display QR code data
        cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('QR Code Scanner', frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
