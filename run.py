from pprint import pprint

import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture(2)
cap.set(3, 640)
cap.set(4, 480)
# Default resolutions of the frame are obtained.The default resolutions are system dependent.
# We convert the resolutions from float to integer.
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
out = cv2.VideoWriter(
    "outpy.mp4",
    cv2.VideoWriter_fourcc(*'MP4V'),
    10,
    (frame_width, frame_height),
)

while True:
    ret, frame = cap.read()

    if ret == True:

        # Write the frame into the file 'output.avi'
        out.write(frame)

        # Display the resulting frame
        cv2.imshow("frame", frame)

        # Press Q on keyboard to stop recording
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Break the loop
    else:
        break

# When everything done, release the video capture and video write objects
cap.release()
out.release()
print("done")
cv2.destroyAllWindows()

