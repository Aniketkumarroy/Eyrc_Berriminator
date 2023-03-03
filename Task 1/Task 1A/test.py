import cv2
import numpy as np
def sqr_rec(p):
    t=sorted((np.linalg.norm(p[0,0]-p[1,0]),np.linalg.norm(p[0,0]-p[2,0]),np.linalg.norm(p[2,0]-p[1,0])))
    print(t)
    print(t[0],t[1],0.05*t[1])
    return 'Square' if t[1]-t[0]<=0.05*t[1] else 'Rectangle'
inp=1
while inp!=0:
    img=cv2.imread("test_images\\test_image_{}.png".format(inp))
    blank=np.zeros(img.shape,np.uint8)
    img_gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # img_canny=cv2.Canny(img,200,300)
    # cv2.imshow("img_gray",img_gray)
    ret,thresh=cv2.threshold(img_gray,200,255,cv2.THRESH_BINARY_INV)
    thresh=cv2.erode(thresh,(3,3),iterations=3)
    cv2.imshow("thresh",thresh)
    cont,hier=cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    print(len(cont))
    for cnt in cont:
        peri=cv2.arcLength(cnt,True)
        approx=cv2.approxPolyDP(cnt,0.04*peri,True)
        # for i in approx:
        #     cv2.circle(blank,i[0],1,(255,255,255),-1)
        M,l=cv2.moments(approx),len(approx)
        cX,cY=int(M["m10"] / M["m00"]),int(M["m01"] / M["m00"])
        t=tuple(img[cY,cX,:])
        str1='Red' if t==(0,0,255) else ('Blue' if t==(255,0,0) else ('Green' if t==(0,255,0) else 'Orange'))
        str2='Triangle' if l==3 else (sqr_rec(approx) if l==4 else ('Pentagon' if l==5 else 'Circle'))
        print(str1,str2,(cX,cY))
        cv2.circle(img,(cX,cY),2,(255,255,255),-1)
    # cv2.imshow("contours",blank)
    cv2.imshow("original",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    inp=int(input("no:-"))
# arr=np.array([[[1,2]],[[3,4]],[[5,6]],[7,8]])
# p1,p2,p3=arr[-3:]
# print(p1)
# print(arr.sum(axis=0))