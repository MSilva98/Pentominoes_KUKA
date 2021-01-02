import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

templates = ['p_90.png', 'p_270.png', 'p_180.png', 'p.png']

img_rgb = cv.imread('p_test_2.png')
img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)

for p in templates:
    template = cv.imread(p,0)
    w, h = template.shape[::-1]
    res = cv.matchTemplate(img_gray,template,cv.TM_CCOEFF_NORMED)
    threshold = 0.8
    loc = np.where( res >= threshold)
    print(loc)
    if loc[0] != [] and loc[1] != []:
        break

for pt in zip(*loc[::-1]):
    cv.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)

cv.imshow('res', img_rgb)
cv.waitKey()




# import cv2 as cv
# import numpy as np
# from matplotlib import pyplot as plt
# img = cv.imread('p_test_2.png',0)
# img2 = img.copy()
# template = cv.imread('p.png',0)
# w, h = template.shape[::-1]
# # All the 6 methods for comparison in a list
# methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
#             'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']
# for meth in methods:
#     img = img2.copy()
#     method = eval(meth)
#     # Apply template Matching
#     res = cv.matchTemplate(img,template,method)
#     min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
#     # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
#     if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
#         top_left = min_loc
#     else:
#         top_left = max_loc
#     bottom_right = (top_left[0] + w, top_left[1] + h)
#     cv.rectangle(img,top_left, bottom_right, 255, 2)

#     plt.subplot(121),plt.imshow(res)
#     plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
#     plt.subplot(122),plt.imshow(img)
#     plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
#     plt.suptitle(meth)
#     plt.show()









# import numpy as np
# import cv2
# from matplotlib import pyplot as plt

# img=cv2.imread("p_ours.png")
# img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# # lower mask (0-10)
# lower_red = np.array([0,50,50])
# upper_red = np.array([10,255,255])
# mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

# # upper mask (170-180)
# lower_red = np.array([170,50,50])
# upper_red = np.array([180,255,255])
# mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

# # join my masks
# mask = mask0+mask1

# # set my output img to zero everywhere except my mask
# output_img = img.copy()
# output_img[np.where(mask==0)] = 0

# # or your HSV image, which I *believe* is what you want
# output_hsv = img_hsv.copy()
# output_hsv[np.where(mask==0)] = 0

# cv2.imshow('piece', output_img)
# cv2.imshow('mask', mask)
# cv2.waitKey()