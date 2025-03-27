import cv2
import dlib
import pygame
import time
from scipy.spatial import distance
from collections import OrderedDict
from imutils import face_utils

pygame.mixer.init()
pygame.mixer.music.load('warning.mp3')

detection = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')

video_capture = cv2.VideoCapture(0)
time.sleep(2)

def calculate_EAR(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    ear_aspect_ratio = (A + B) / (2.0 * C)
    return ear_aspect_ratio

eyes_blink = 0.3
eyes_ratio = 25
count = 0
SHAPE_68_INDEX = OrderedDict([
    ("left_eye", (42, 48)),
    ("right_eye", (36, 42))
])
(lstart,lend) = SHAPE_68_INDEX['left_eye']
(rstart,rend) = SHAPE_68_INDEX['right_eye']

t = 0
while (True):
    ret, frame = video_capture.read()
    frame = cv2.flip(frame,1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detection(gray,0)
    for face in faces:
        shape = predictor(gray, face)
        shape = face_utils.shape_to_np(shape)
        leftEye = shape[lstart:lend]
        rightEye = shape[rstart:rend]

        leftEyeDistance = calculate_EAR(leftEye)
        rightEyeDistance = calculate_EAR(rightEye)
        ER = (leftEyeDistance+rightEyeDistance) / 2

        leftEyeHull = cv2.convexHull(leftEye)
        rightEyeHull = cv2.convexHull(rightEye)
        cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
        cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)

        t+=1
        if (ER < eyes_blink):
            count += 1
            if count >= eyes_ratio:
                pygame.mixer.music.play(-1)
                t = 0
                cv2.putText(frame, "you are tired", (150, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)
        else:
            if( t>50 ):
                pygame.mixer.music.stop()
            count = 0

    cv2.imshow('Video', frame)
    if(cv2.waitKey(1) & 0xFF == ord('q')):
        break

video_capture.release()
cv2.destroyAllWindows()
