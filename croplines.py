import numpy as np
import cv2
import math

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
idx = 5;
for i in range(1,9):
    invo = cv2.imread(str(i)+'.png')
    img21 = cv2.cvtColor(invo, cv2.COLOR_BGR2GRAY)
    img22 = cv2.medianBlur(img21,3)
    print(str(i))
    cl1 = clahe.apply(img22)
    #dst1 = cv2.Sobel(img22,cv2.CV_64F,1,1,ksize=5)
    low_threshold = 10
    high_threshold = 150
    edges = cv2.Canny(cl1, low_threshold, high_threshold)
    #dst1 = cv2.Laplacian(img22, -1)
    rho = 1 # distance resolution in pixels of the Hough grid
    theta = np.pi / 180 # angular resolution in radians of the Hough grid
    threshold = 15 # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 50 # minimum number of pixels making up a line
    max_line_gap = 20 # maximum gap in pixels between connectable line segments
    line_image = np.copy(invo) * 0 # creating a blank to draw lines on
    cv2.imshow('imggg',line_image)

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
    min_line_length, max_line_gap)
    try:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
            print(str(math.degrees(math.atan2(y2-y1,x2-x1))))
        cv2.imshow(lines_edges = cv2.addWeighted(invo, 0.8, line_image, 1, 0))
        cv2.imshow('imgg',lines_edges)
        cv2.imshow('imggg',line_image)
       # cv2.imwrite(str(i) + '.png',lines_edges)
    except:
        pass
      
