import numpy as np
import cv2
import copy

## https://stackoverflow.com/a/30418912/5008845
## Find largest rectangle containing only zeros in an N×N binary matrix
def find_min_rect(src, threshold=100):
    # return: top left corner and bottom right corner
    # [x0, y0, x1, y1]

    rows = src.shape[0]
    cols = src.shape[1]
    W = np.zeros([rows, cols])
    H = np.zeros([rows, cols])

    maxRect = [0,0,0,0]
    maxArea = .0

    for r in range(rows):
        for c in range(cols):
            if src[r, c] > threshold:
                i = 0
                if r > 0:
                    i = H[r-1, c]

                H[r, c] = 1.0 + i

                j = 0
                if c > 0:
                    j = W[r, c-1]

                W[r, c] = 1.0 +j

            minw = W[r,c];
            for i in range(int(H[r,c])):
                minw = min(minw, W[r-i, c])
                area = (i+1) * minw;
                if area > maxArea:
                    maxArea = area;
                    maxRect = [c - minw + 1, r - i, c+1, r+1]

    return [int(i) for i in maxRect]

def area(box):
    return (box[2]-box[0])*(box[3]-box[1])

def find_rect_w_rotation(src, angle):
    maxdim = src.shape[0]//2
    ## Rotate the image
    m = cv2.getRotationMatrix2D((maxdim,maxdim), angle, 1)
    # Mat1b rotated
    rotated = cv2.warpAffine(src, m, src.shape)

    ## Keep the crop with the polygon
    pts = cv2.findNonZero(rotated)
    box = cv2.boundingRect(pts)
    x = box[0]
    y = box[1]
    w = box[2]
    h = box[3]
    crop = rotated[y:y+h,x:x+w]

    ## Solve the problem: "Find largest rectangle containing only zeros in an binary matrix"
    ## https://stackoverflow.com/questions/2478447/find-largest-rectangle-containing-only-zeros-in-an-n%C3%97n-binary-matrix
    p = find_min_rect(crop);

    return [p[0]+x, p[1]+y, p[2]+x, p[3]+y]


def find_best_angle(src, size=-1):
    # The input should be a square image
    if size == -1:
        work = src
        maxdim = work.shape[0]
        ...
    else:
        maxdim = size*2
        # resize image
        work =  cv2.resize(src, (maxdim, maxdim), interpolation = cv2.INTER_AREA)

    # ## Store best data
    bestRect = [0,0,0,0]
    bestAngle = 0 # int

    ## For each angle
    # for angle in range(90):
    for angle in range(-30, 30):
        p = find_rect_w_rotation(work, angle)
        ## If best, save result
        if (area(p) > area(bestRect)):
            # Correct the crop displacement
            # bestRect = r + box.tl();
            bestRect = [p[0], p[1], p[2], p[3]]
            bestAngle = angle

    return bestAngle

## https://stackoverflow.com/questions/32674256
## How to adapt or resize a rectangle inside an object without including (or with a few numbers) of background pixels?
def largest_rect_in_non_convex_poly(src, thumbnail_size = -1):
    ## Create a matrix big enough to not lose points during rotation
    ptz = cv2.findNonZero(src)
    bbox = cv2.boundingRect(ptz) #bbox is [x, y, w, h]
    x = bbox[0]
    y = bbox[1] 
    width = bbox[2]
    height = bbox[3]

    if width%2==1:
        width -= 1
    ## height += 1 may leads to a y+height > 720 case 
    if height%2==1:
        height -= 1

    maxdim = max(width, height)
    work = np.zeros([2*maxdim, 2*maxdim])
    work[maxdim-height//2:maxdim+height//2,maxdim-width//2:maxdim+width//2] \
        = copy.deepcopy(src[y:y+height,x:x+width])

    # use a thumbnail to accelerate finding the best angle first
    # or set the size = -1 to use the original image to find the best angle
    bestAngle = find_best_angle(work, thumbnail_size)
    # use the original image and the found best angle to find the largest rectangle
    bestRect = find_rect_w_rotation(work, bestAngle)

    ## Apply the inverse rotation
    m_inv = cv2.getRotationMatrix2D((maxdim, maxdim), -bestAngle, 1)
    x0 = bestRect[0]
    y0 = bestRect[1]
    x1 = bestRect[2]
    y1 = bestRect[3]
    ## OpenCV on Python often wants points in the form
    ## np.array([ [[x1, y1]], ..., [[xn, yn]] ])
    rectPoints = np.array([ [[x0, y0]], [[x1, y0]], [[x1, y1]], [[x0, y1]] ])
    rotatedRectPoints = cv2.transform(rectPoints, m_inv)

    ## Apply the reverse translations
    for i in range(len(rotatedRectPoints)):
        ## rotatedRectPoints += bbox.tl() - Point(maxdim - bbox.width / 2, maxdim - bbox.height / 2)
        rotatedRectPoints[i][0][0] += x - maxdim + width / 2
        rotatedRectPoints[i][0][1] += y - maxdim + height / 2

    ## Get the rotated rect
    ## (center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(points)
    rrect = cv2.minAreaRect(rotatedRectPoints)

    return rrect


if __name__ == '__main__':
    img = cv2.imread("./mask.png", cv2.IMREAD_GRAYSCALE)

    # Compute largest rect inside polygon
    rect = largest_rect_in_non_convex_poly(img, thumbnail_size=100)
    print(rect)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    res = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(res,[box],0,(0,0,255),2)
    cv2.imshow("Result", res)
    cv2.waitKey()

    # p = find_min_rect(img)
    # print(p)
    # r = [[p[0],p[1]],[p[0],p[3]],[p[2],p[3]],[p[2],p[1]]]
    # res = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # for i in range(len(r)):
    #     cv2.line(res, r[i], r[(i+1)%4], (0, 0, 255), 2)

    # cv2.imshow("Result", res)
    # cv2.waitKey()
