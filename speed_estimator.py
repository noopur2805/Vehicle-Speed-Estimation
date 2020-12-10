#!/usr/bin/env python
# coding: utf-8

# In[1]:
## To run:
## python speed_estimator.py -i <input_filename> -o <output_filename>

import cv2
import imutils
import pandas as pd
import numpy as np

from PIL import Image


# In[2]:


# Took the vanishing points along the horizon

vp_left_tuple = (-3111, -463)
vp_right_tuple = (2270, -268)


# In[3]:



def find_intersection(x1,y1,x2,y2,x3,y3,x4,y4):
    """
    Returns intersection point of 2 lines
    """
    px= ( (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) )
    py= ( (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) )
    
    return [int(px), int(py)]

def get_calibration_lines(first_frame_name, vp_tuple):

    """
    first_frame_name: First blank frame with no vehicles on road
    vp_tuple: Vanishing point
    
    Returns perpendicular lines used for calibration
    """
    first_frame_name = 'frame0.jpg'
    img = cv2.imread(first_frame_name)

    # Filter lane boundaries
    blur_kernel = 5
    img_hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
#     img_lab = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)

    img_hls = cv2.medianBlur(img_hls, blur_kernel)
#     img_lab = cv2.medianBlur(img_lab, blur_kernel)

    white_lower = np.array([np.round(  0 / 2), np.round(0.5 * 255), np.round(0.00 * 255)])
    white_upper = np.array([np.round(360 / 1), np.round(0.8 * 255), np.round(0.30 * 255)])
    hls_filter1 = cv2.inRange(img_hls, white_lower, white_upper)

    kernel = np.ones((9,9),np.uint8)
    dilation = cv2.dilate(hls_filter1,kernel,iterations = 1)
    erosion = cv2.erode(dilation,kernel,iterations = 20)

    white_lower = np.array([0, np.round(0.65 * 255), np.round(0.0 * 255)])
    white_upper = np.array([360, np.round(0.8 * 255), np.round(0.3 * 255)])
    hls_filter2 = cv2.inRange(img_hls, white_lower, white_upper)

    mask = cv2.bitwise_and(erosion, hls_filter2)
    mask = cv2.dilate(mask,kernel)
    # Image.fromarray(mask)
    
    
    cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)

    hull = [cv2.convexHull(cnts[i], False) for i in range(len(cnts))]    

    for i in range(len(cnts)):
        if i<3:
            img = cv2.drawContours(mask, hull, i, [255, 0, 0], 3, 8)
        else:
            cv2.fillPoly(img, [hull[i]], [0, 0, 0])

    contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # contours = sorted(cnts, key = cv2.contourArea, reverse = True)
    # vp_tuple = (-3111, -463)


    stencil = np.zeros(img.shape).astype(img.dtype)
#     lane_perpendicular_lines = []
    
    perpendicular_left_intersection = []
    perpendicular_right_intersection = []

    for c in contours[0]:

        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect, c)

        box = np.int0(box)
        cv2.drawContours(img, [box], 0, (255,0,0), 3)
        
        '''
#         cv2.line(img, vp_tuple, tuple(box[1]), [255,0,0], 2)
        pt1 = find_intersection(vp_tuple[0],vp_tuple[1],box[1][0],box[1][1],img.shape[1],0,img.shape[1],img.shape[0])
        cv2.line(img, tuple(pt1), vp_tuple, [255,0,0], 2)
        lane_perpendicular_lines.append(pt1)

#         cv2.line(img, vp_tuple, tuple(box[2]), [255,0,0], 2)
        pt2 = find_intersection(vp_tuple[0],vp_tuple[1],box[2][0],box[2][1],img.shape[1],0,img.shape[1],img.shape[0])
        cv2.line(img, tuple(pt2), vp_tuple, [255,0,0], 2)
        lane_perpendicular_lines.append(pt2)
        '''
        
        
    #     cv2.line(img, vp_tuple, tuple(box[1]), [255,0,0], 2)
        # Intersection point at image end (right-side)
        pt1 = find_intersection(vp_tuple[0],vp_tuple[1],box[1][0],box[1][1],img.shape[1],0,img.shape[1],img.shape[0])
        # Intersection point at image start (left-side)
        pt2 = find_intersection(vp_tuple[0],vp_tuple[1],box[1][0],box[1][1],0,0,0,img.shape[0])

        cv2.line(img, tuple(pt1), tuple(pt2), [255,0,0], 2)
        perpendicular_right_intersection.append(pt1)
        perpendicular_left_intersection.append(pt2)

    #     cv2.line(img, vp_tuple, tuple(box[2]), [255,0,0], 2)
        # Intersection point at image end (right-side)
        pt3 = find_intersection(vp_tuple[0],vp_tuple[1],box[2][0],box[2][1],img.shape[1],0,img.shape[1],img.shape[0])
        # Intersection point at image start (left-side)
        pt4 = find_intersection(vp_tuple[0],vp_tuple[1],box[2][0],box[2][1],0,0,0,img.shape[0])

        cv2.line(img, tuple(pt3), tuple(pt4), [255,0,0], 2)
        perpendicular_right_intersection.append(pt3)
        perpendicular_left_intersection.append(pt4)

    return img, perpendicular_left_intersection, perpendicular_right_intersection


# In[4]:



img, perpendicular_left_intersection, perpendicular_right_intersection = get_calibration_lines('frame0.jpg', 
                                                                                              vp_left_tuple)


# In[5]:


def get_lane_contours(first_frame_name):
    
    #first_frame_name = 'frame0.jpg'
    img = cv2.imread(first_frame_name)

    # Filter lane boundaries
    blur_kernel = 5
    img_hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    #     img_lab = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)

    img_hls = cv2.medianBlur(img_hls, blur_kernel)
    #     img_lab = cv2.medianBlur(img_lab, blur_kernel)

    white_lower = np.array([0, np.round(0.5 * 255), np.round(0.00 * 255)])
    white_upper = np.array([360, np.round(0.8 * 255), np.round(0.30 * 255)])
    hls_filter1 = cv2.inRange(img_hls, white_lower, white_upper)


    kernel = np.ones((9,9),np.uint8)
    dilation = cv2.dilate(hls_filter1,kernel,iterations = 1)
    erosion = cv2.erode(dilation,kernel,iterations = 10)

    mask = cv2.bitwise_and(erosion, hls_filter1)
    mask = cv2.dilate(mask,kernel)

    cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    hull = [cv2.convexHull(cnts[i], False) for i in range(len(cnts))]    
    img = cv2.drawContours(mask, hull, 0, [255, 0, 0], 3, 8)
    cv2.fillPoly(img, [hull[0]], [0, 0, 0])

    contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    lane_contours = []
    flag = 1
    for c in contours[0]:

        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect, c)
        box = np.int0(box)
        cv2.line(img, tuple(box[0]), tuple(box[3]), (255,0,0), 3)
        if flag == 1:
            lane_contours.append(list(box[1]))
            lane_contours.append(list(box[2]))
            flag = 0
        else:
            lane_contours.append(list(box[2]))
            lane_contours.append(list(box[1]))
    
    return img, lane_contours


# In[6]:


_, lane_contours = get_lane_contours('frame0.jpg')
# lane_contours


# In[7]:


perpendicular_left_intersection.sort(key=lambda x: x[1])
perpendicular_right_intersection.sort(key=lambda x: x[1])


# In[8]:


x1, y1 = perpendicular_left_intersection[0]
x2, y2 = perpendicular_right_intersection[0]
x3, y3 = vp_right_tuple
x4, y4 = lane_contours[2]
p1 = find_intersection(x1,y1,x2,y2,x3,y3,x4,y4)

x1, y1 = perpendicular_left_intersection[-1]
x2, y2 = perpendicular_right_intersection[-1]
x3, y3 = vp_right_tuple
x4, y4 = lane_contours[3]
p2 = find_intersection(x1,y1,x2,y2,x3,y3,x4,y4)

x1, y1 = perpendicular_left_intersection[-1]
x2, y2 = perpendicular_right_intersection[-1]
x3, y3 = vp_right_tuple
x4, y4 = lane_contours[0]
p3 = find_intersection(x1,y1,x2,y2,x3,y3,x4,y4)

x1, y1 = perpendicular_left_intersection[0]
x2, y2 = perpendicular_right_intersection[0]
x3, y3 = vp_right_tuple
x4, y4 = lane_contours[1]
p4 = find_intersection(x1,y1,x2,y2,x3,y3,x4,y4)


# In[9]:


# Image.fromarray(img)


# In[10]:


# Image.fromarray(cv2.bitwise_or(img,_))


# In[11]:


# p1 = lane_contours[2]
# p2 = lane_contours[3]
# p3 = lane_contours[0]
# p4 = lane_contours[1]


# In[12]:


pts1 = np.float32([p1, p2, p3, p4]) 
pts2 = np.float32([[0, 0], [0, img.shape[0]], [img.shape[1], img.shape[0]], [img.shape[1], 0]])


# In[13]:


matrix = cv2.getPerspectiveTransform(pts1, pts2) 
img_transformed = cv2.warpPerspective(img, matrix, (img.shape[1]+10, img.shape[0]+10))

# Image.fromarray(img_transformed)

horizontal_line_idx = []
flag = 0
for i, p in enumerate(img_transformed):
    if ((flag==0) & (p[5]==255)):
        horizontal_line_idx.append(i)
        flag = 1
    if p[5]==0:
        flag=0
        
horizontal_line_idx = list(reversed(horizontal_line_idx))


# In[14]:


# horizontal_line_idx #[0, 182, 458, 632, 920, 1080]


# In[15]:


# # Equations of lines
# x1 = vp_left[0]
# y1 = vp_left[1]
# x2 = box[1][0]
# y2 = box[1][1]

# #Point coordinates
# xp = 0
# yp = 0

# solution = (yp-y1)*(x2-x1) - (y2-y1)*(xp-x1)
# if solution > 0:
#     print ('above the line')
# elif solution < 0:
#     print ('below the line')
# else:
#     print ('on the line')


# In[16]:


# video.get(cv2.CAP_PROP_POS_FRAMES), video.get(cv2.CAP_PROP_FRAME_COUNT)


# In[50]:


import argparse

if __name__ == '__main__':
    
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", required=True, help="path to input video file")
    ap.add_argument("-o", "--output", required=True, help="path to output video file")
    args = vars(ap.parse_args())

    input_video = args['input']
    output_video = args['output']

    first_frame = cv2.imread('frame0.jpg')
    first_frame = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)

    input_video = cv2.VideoCapture('video.mp4')
    fps = input_video.get(cv2.CAP_PROP_FPS)

    vw = first_frame.shape[1]
    vh = first_frame.shape[0]
    outvideo = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'XVID'), 20, (vw,vh))

    current_frame_number_list = {}
    bottom_position_of_detected_vehicle = [0]
    scale_param = .7 # Since camera calibration isn't being performed

    while True:

        _, frame = input_video.read()
        if _ == True:

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            current_frame_num = input_video.get(cv2.CAP_PROP_POS_FRAMES)

            frame_copy = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY) #cv2.imread('frame0.jpg')
            delta_frame = cv2.absdiff(frame_copy, first_frame)

            kernel = np.ones((3,3),np.uint8)

            th_delta = cv2.threshold(delta_frame, 10, 255, cv2.THRESH_BINARY)[1]
            th_delta = cv2.dilate(th_delta, kernel, iterations=2)
            th_delta = cv2.erode(th_delta,kernel,iterations = 4)

            img_transformed_th = cv2.warpPerspective(th_delta, matrix, (img.shape[1]+10, img.shape[0]+10))

            dilation = cv2.dilate(img_transformed_th,kernel,iterations = 1)
            erosion = cv2.erode(dilation,kernel,iterations = 25)

            img_transformed_frame = cv2.warpPerspective(frame_copy, matrix, (img.shape[1]+10, img.shape[0]+10))
            cnts = cv2.findContours(erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
            hull = [cv2.convexHull(cnts[i], False) for i in range(len(cnts))]    

            bottom = img_transformed_frame.shape[0]
            current_frame_num = input_video.get(cv2.CAP_PROP_POS_FRAMES)

            if len(cnts) == len(hull):
                # Num of contours = Num of vehicles 
                for i in range(len(cnts)):
        #             if len(cnts)==1:
        #                 cnts_area = cv2.contourArea(cnts[0])
        #             else:

                    cnts_area = cv2.contourArea(cnts[i])

                    if cnts_area > 10000:
        #                 try:
                        img_transformed_frame = cv2.drawContours(img_transformed_frame, hull, i, [255, 0, 0], 3, 8)
                        (x, y, w, h) = cv2.boundingRect(hull[i])
                        cv2.rectangle(img_transformed_frame, (x, y), (x+w, y+h), (255, 0, 255), 3)


                        if ((horizontal_line_idx[0] > (y+h) > horizontal_line_idx[1])
                            or (horizontal_line_idx[2] > (y+h) > horizontal_line_idx[3])
                            or (horizontal_line_idx[4] > (y+h) > horizontal_line_idx[5])):

                            if i not in current_frame_number_list.keys():
                                current_frame_number_list[i] = input_video.get(cv2.CAP_PROP_POS_FRAMES)
                            else:

                                pixel_length = bottom - (y+h) # tracking bottom position of vehicle
                                scaled_real_length = pixel_length * 3  # multiplied by 3 to convert pixel length to real length in meters 
                                total_time_passed = current_frame_num - current_frame_number_list[i]
                                current_frame_number_list.pop(i)
                                scaled_real_time_passed = total_time_passed * fps  # total time for vehicle to pass through ROI 
                                if scaled_real_time_passed != 0:
                                    speed = int(scaled_real_length / scaled_real_time_passed * scale_param)
                                    speed = speed * 3600 / 1000  # m/s ---> km/hr

                                    ## Put speed into frames

                                    matrix_opp = cv2.getPerspectiveTransform(pts2, pts1) 
                                    wp = cv2.warpPerspective(img_transformed_th.copy(), matrix_opp, (img.shape[1]-10, img.shape[0]-10), 
                                                           flags=cv2.INTER_LINEAR)

                                    dilation = cv2.dilate(wp,kernel,iterations = 1)
                                    erosion = cv2.erode(dilation,kernel,iterations = 25)

                                    # img_transformed_frame = cv2.warpPerspective(frame_copy, matrix, (img.shape[1]+10, img.shape[0]+10))
                                    cnts2 = cv2.findContours(erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                                    cnts2 = imutils.grab_contours(cnts2)
                                    cnts2 = sorted(cnts2, key = cv2.contourArea, reverse = True)[:10]

                                    for i in range(len(cnts2)):
                                        if cv2.contourArea(cnts2[i]) > 10000:
                                            (x, y, w, h) = cv2.boundingRect(cnts2[i])
                                            frame_ = cv2.rectangle(frame.copy(), (x, y), (x+w, y+h), (255, 0, 255), 3)
                                            frame_ = cv2.flip(frame_, 0)
                                            frame_ = cv2.putText(frame_, str(speed)+' km/hr', (x,img.shape[0]-y), 
                                                                 cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                                                 (0,0,0), 2, cv2.LINE_AA, True)  
                                            frame_ = cv2.flip(frame_, 0)
                                            outvideo.write(frame_)
                                            # Image.fromarray(frame_)
                                            # print(speed)
                                            cv2.imshow('Capturing', frame_)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    # for i in range(0, len(times), 2):
    #     df = df.append({"Start": times[i], "End": times[i+1]}, ignore_index=True)

    # df.to_csv("Times_"+datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")+".csv")
    input_video.release()
    cv2.destroyAllWindows()


# In[ ]:





# In[ ]:




