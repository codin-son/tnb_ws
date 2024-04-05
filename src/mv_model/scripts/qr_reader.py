import cv2
from pyzbar.pyzbar import decode

def read_qr(img):

    # Convert frame to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    qr_data = "N/A"

    # Decode QR codes
    decoded_objects = decode(gray)

    # Loop over detected objects
    for obj in decoded_objects:
        # Extract QR code data
        qr_data = obj.data.decode('utf-8')
        
        # Draw rectangle around the QR code
        (x, y, w, h) = obj.rect
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Display QR code data
        cv2.putText(img, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return img,qr_data