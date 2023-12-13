from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(sensor={"output_size": (2028, 1520), "bit_depth": 12})
picam2.configure(preview_config)
picam2.start_preview(Preview.QTGL)
#picam2.start_preview(Preview.QTGL)
picam2.start()
            
while(True):
    print("Capturing")
picam2.stop_preview()