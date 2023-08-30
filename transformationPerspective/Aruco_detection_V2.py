#!/usr/bin/env python
# %%

import time
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
from ArucoDetection_definitions import *
import keyboard

start_time = time.time()
 
desired_aruco_dictionary1 = "DICT_4X4_50"
desired_aruco_dictionary2 = "DICT_4X4_50"

robot_id = 10

#Map associating ids with corners
table_ids = {23:1, 22:2, 20:3, 21:4}


# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}


def get_markers(vid_frame, aruco_dictionary, aruco_parameters):
    detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_parameters)
    bboxs, ids, rejected = detector.detectMarkers(vid_frame)
    if ids is not None:
        ids_sorted=[]
        for id_number in ids:
            ids_sorted.append(id_number[0])
    else:
        ids_sorted=ids
    return bboxs,ids_sorted

init_loc_1=[10,400]
init_loc_2=[400,400]
init_loc_3=[400,10]
init_loc_4=[10,10]

#initiaize locations
current_square_points=[init_loc_1,init_loc_2,init_loc_3,init_loc_4]
current_center_Corner=[0,0]


def camera_compensation(x_coordinate, y_coordinate):
    h_robot = 300  # fam heigth of 80mm
    camera_position = [480, 150, 144]  # x,y,z coordinate from origin in mm
    # add 300 to move orgin to under the camera
    x_coordinate = x_coordinate + camera_position[0]
    y_coordinate = y_coordinate + camera_position[1]

    # perform compensation
    x_compensated = x_coordinate - (h_robot / (camera_position[2] / x_coordinate))
    y_compensated = y_coordinate - (h_robot / (camera_position[2] / y_coordinate))

    return int(x_compensated), int(y_compensated)

def main():
   
    # Load the ArUco dictionary
    print("[INFO] detecting '{}' markers...".format(desired_aruco_dictionary1))
    this_aruco_dictionary1 = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary1])   #for 4x4 markers
    this_aruco_parameters1 = cv2.aruco.DetectorParameters()  #for 4x4 markers
    this_aruco_dictionary2 = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary1])
    this_aruco_parameters2 = cv2.aruco.DetectorParameters()
    
    # Start the video stream
    #url = 'PXL_20230607_190612712.mp4'
    #Video chez fred aruco au sol
    url = 'WIN_20230727_21_53_08_Pro.mp4'
    cap = cv2.VideoCapture(url)


    while True:

        ret, frame = cap.read()

        # Detect 4x4 ArUco markers in the video frame
        markers,ids=get_markers(frame, this_aruco_dictionary1, this_aruco_parameters1)

        #create copy of te initial 'clean frame'
        frame_clean=frame.copy()

        #get info over the different markers and display info
        left_corners,corner_ids=getMarkerCoordinates(markers,ids,0)


        #update the markers positions when a markers is found. When no marker is found, use previous location
        if corner_ids is not None:
            count=0
            for id in corner_ids:
                if id in table_ids:
                    current_square_points[table_ids[id]-1]=left_corners[count]
                count=count+1
        left_corners=current_square_points
        corner_ids=[1,2,3,4]


        cv2.aruco.drawDetectedMarkers(frame, markers) #built in open cv function
        draw_corners(frame,left_corners)
        draw_numbers(frame,left_corners,corner_ids)
        show_spec(frame,left_corners)
       
        frame_with_square,squareFound=draw_field(frame,left_corners,corner_ids)

        if squareFound:
            square_points=left_corners
            img_wrapped=four_point_transform(frame_clean, np.array(square_points))
            # look for foam, Detect 6x6 ArUco markers in the video frame
            h, w, c = img_wrapped.shape
            marker_foam,ids_foam=get_markers(img_wrapped, this_aruco_dictionary2, this_aruco_parameters2)
            if ids_foam is not None and robot_id in ids_foam:
                left_corner_foam,corner_id_foam=getMarkerCoordinates(marker_foam,ids_foam,0)
                centerCorner=getMarkerCenter_foam([marker_foam[ids_foam.index(robot_id)]])

            #update the markers positions when a markers is found. When no marker is found, use previous location
            if corner_id_foam is not None:
                #only one piece of foam
                current_center_Corner[0]=centerCorner[0]
            centerCorner[0]=current_center_Corner[0]


        print(centerCorner)
        draw_corners(img_wrapped,centerCorner)
        #draw cross over frame
        img_wrapped=cv2.line(img_wrapped,(centerCorner[0][0],0), (centerCorner[0][0],h), (0,0,255), 2)
        img_wrapped=cv2.line(img_wrapped,(0,(centerCorner[0][1])), (w,(centerCorner[0][1])), (0,0,255), 2)

        draw_numbers(img_wrapped,left_corner_foam,corner_id_foam)
        cv2.imshow('img_wrapped',img_wrapped)

        # Display the resulting frame
        cv2.imshow('frame_with_square',frame_with_square)
        #cv2.imshow('img_cropped',img_cropped)


        """  
        x_coordinate=int((centerCorner[0][1]/h)*600)-300
        y_coordinate=int((centerCorner[0][0]/w)*300)
        print("Optical position: ",x_coordinate,", ",y_coordinate)
        # print(x_coordinate,y_coordinate)
        #camera compensation
        x_coordinate_comp,y_coordinate_comp=camera_compensation(x_coordinate,y_coordinate)
        print("Position after compensation: ",x_coordinate_comp,", ",y_coordinate_comp)
        # print(x_coordinate_comp,y_coordinate_comp)
        """
        if cv2.waitKey(1) == ord('q'):
            break
            
    # Close down the video stream
    cap.release()
    cv2.destroyAllWindows()
    return centerCorner 
   

if __name__ == '__main__':
    foam_center=main()  #pull foam location from markers

