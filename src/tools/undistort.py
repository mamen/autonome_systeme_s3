import cv2
import numpy as np
import time

# You should replace these 3 lines with the output in calibration step
DIM = (2688, 1520)
K = np.array(
    [[1230.0629104529546, 0.0, 1326.6835270392235], [0.0, 1230.9039925270206, 744.4096992151756], [0.0, 0.0, 1.0]])
D = np.array([[-0.020696262841677884], [0.014206600062335005], [-0.005580871305728541], [-0.00043358515472548357]])


def undistort(img_path, balance=0.55, dim2=None, dim3=None):
    img = cv2.imread(img_path)
    start = time.time()
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort

    assert dim1[0] / dim1[1] == DIM[0] / DIM[
        1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1

    scaled_k = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_k[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_k, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_k, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_k, D, np.eye(3), new_k, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    end = time.time()
    print(end - start)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    undistort('test.jpg')