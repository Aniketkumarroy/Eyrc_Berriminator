import cv2
import numpy as np



img=cv2.imread('test_images\\test_image_14.png')

gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
edged = cv2.Canny(gray,30,200)
#cv2.imshow('',edged)
contours, hierarchy = cv2.findContours(edged,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

i=0
    

detected_shapes = [] 





for contour in contours:
	#if i==0:
	#	i=1
	#	continue
		
	approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

	cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)
	

	x=0
	y=0
	colour=''
	s=[0,0,0]

	M = cv2.moments(contour)
	if M['m00'] != 0.0:
		x = int(M['m10']/M['m00'])
		y=int(M['m01']/M['m00'])
		
		
	    

	

	s=img[y,x]

	if(s[0]>180 and s[1]<100 and s[2]<100 ):
	    colour = 'Blue'
	if(s[1]>180 and s[0]<100 ):
		colour='Green'
	if(s[2]>200 and s[0]<100 and s[1]<100):
		colour='Red'
	if(s[0]<100 and s[1]>210 and s[2]>210):
		colour='Yellow'
	if(s[0]<100 and s[1]>130 and s[2]>210 and s[1]<200):
	    colour='Orange'
	

    

	k=0
	#print(img.shape)
	




	if len(approx) == 3:
		detected_shapes.append([colour,'Triangle',(x,y)])
	elif len(approx) == 4 :
		x,y,w,h=cv2.boundingRect(approx)
		aspectRatio=float(w/h)
		if aspectRatio>=0.95 and aspectRatio<=1.05:
			detected_shapes.append([colour,'Square',(x,y)])
		else:
			detected_shapes.append([colour,'Rectangle',(x,y)])

	elif len(approx) == 5:
		detected_shapes.append([colour,'Pentagon',(x,y)])
	elif len(approx) == 6:
		detected_shapes.append([colour,'Hexagon',(x,y)])
	else:
		detected_shapes.append([colour,'Circle',(x,y)])
	
print(detected_shapes)
		
cv2.imshow('shapes', img)
	
cv2.waitKey(20000)