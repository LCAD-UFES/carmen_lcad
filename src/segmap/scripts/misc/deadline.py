
import cv2
import time
import datetime
import numpy as np
from dateutil import tz

timezone = tz.gettz("America/Tijuana") # Pacific US (https://en.wikipedia.org/wiki/List_of_tz_database_time_zones)
deadline = datetime.datetime(2019, 8, 1, 23, 59, 0, tzinfo=timezone)

while True:
    left = deadline - datetime.datetime.now(tz=timezone)
    d = left.days
    h = left.seconds // 3600
    m = (left.seconds % 3600) // 60
    s = (left.seconds % 3600) % 60
    img = np.zeros((70, 750, 3))
    months = d / 30
    d = d % 30
    cv2.putText(img, "%2dm %02dd %02dh %02d:%02d" % (months, d, h, m, s), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 3)
    cv2.imshow("Deadline", img)
    cv2.waitKey(1)
    time.sleep(1)    


