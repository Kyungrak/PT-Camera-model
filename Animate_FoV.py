import cv2
import numpy as np

img_array = []
ref_img = cv2.imread('FOV_image0.jpg')
height, width, layers = ref_img.shape
size = (width,height)

for i in range(0,180):
    img_array.append(cv2.imread('FOV_image'+str(i)+'.jpg'))


out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 20, size)
 
for j in range(len(img_array)):
    out.write(img_array[j])
out.release()
