import cv2 
import sys

# path 
path = sys.path[0]
print(path)


image = cv2.imread(path+'/../tpl/0.png')
window_name = 'image'
cv2.imshow(window_name, image)
cv2.waitKey(0) 
cv2.destroyAllWindows() 
