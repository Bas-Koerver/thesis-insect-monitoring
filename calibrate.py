import cv2
import json
import numpy as np
import os
import pathlib
from glob import glob


def splitfn(fn):
    path, fn = os.path.split(fn)
    name, ext = os.path.splitext(fn)
    return path, name, ext


def load_calibration(calibration_file, key):
    with open(calibration_file, 'r') as f:
        data = json.load(f)

    return np.array(data['cams'][key]['calibration']['cameraMatrix']), np.array(data['cams'][key]['calibration']['distCoeffs'])

def load_stereo_calibration(calibration_file):
    with open(calibration_file, 'r') as f:
        data = json.load(f)

    return (np.array(data['stereoCalib']['0_1']['rotationMatrix']),
            np.array(data['stereoCalib']['0_1']['translationMatrix']),
            np.array(data['stereoCalib']['0_1']['essentialMatrix']),
            np.array(data['stereoCalib']['0_1']['fundamentalMatrix']))


def stereo_calibrate(debug_dir, mtx1, dist1, mtx2, dist2, data_dir_left, data_dir_right):
    height = 5
    width = 7
    square_size = 0.0298
    marker_size = 0.0208
    aruco_dict_name = cv2.aruco.DICT_6X6_50

    pattern_size = (width, height)

    # read the synched frames
    image_names_left = sorted(glob(data_dir_left))
    image_names_right = sorted(glob(data_dir_right))

    cl_images = []
    cr_images = []

    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)
    board = cv2.aruco.CharucoBoard(pattern_size, square_size, marker_size, aruco_dict)
    charuco_detector = cv2.aruco.CharucoDetector(board)

    for iml, imr in zip(image_names_left, image_names_right):
        _im = cv2.imread(iml, 1)
        cl_images.append(_im)

        _im = cv2.imread(imr, 1)
        cr_images.append(_im)

    # change this if stereo calibration not good.
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

    # objp = np.zeros((pattern_size[1]*pattern_size[0],3), np.float32)
    # objp[:,:2] = np.mgrid[0:pattern_size[1],0:pattern_size[0]].T.reshape(-1,2)

    # frame dimensions. Frames should be the same size.
    width = cr_images[0].shape[1]
    height = cr_images[0].shape[0]

    # Pixel coordinates of checkerboards
    object_points = []
    imgpoints_left = []  # 2d points in image plane.
    imgpoints_right = []

    allCorners = {'left': [], 'right': []}
    allIds = {'left': [], 'right': []}

    # coordinates of the checkerboard in checkerboard world space.
    # objpoints = []  # 3d point in real world space

    for frame_l, frame_r in zip(cl_images, cr_images):
        grayl = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
        grayr = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

        charuco_corners_l, charuco_ids_l, _, _ = charuco_detector.detectBoard(grayl)
        charuco_corners_r, charuco_ids_r, _, _ = charuco_detector.detectBoard(grayr)

        if len(np.intersect1d(charuco_ids_r, charuco_ids_l)) > 3:
            allCorners['left'].append(charuco_corners_l)
            allCorners['right'].append(charuco_corners_r)
            allIds['left'].append(charuco_ids_l)
            allIds['right'].append(charuco_ids_r)

    for i in range(len(allCorners['left'])):
        # common_ids = np.intersect1d(allIds['left'][i], allIds['right'][i])
        #
        # if len(common_ids) > 0:
        #     indices_left = np.isin(allIds['left'][i], common_ids).flatten()
        #     indices_right = np.isin(allIds['right'][i], common_ids).flatten()
        #
        #     frame_obj_points_l, frame_img_points_l = board.matchImagePoints(allCorners['left'][i][indices_left],
        #                                                                     common_ids)
        #     frame_obj_points_r, frame_img_points_r = board.matchImagePoints(allCorners['right'][i][indices_right],
        #                                                                     common_ids)
        #     object_points.append(frame_obj_points_l)
        #     imgpoints_left.append(frame_img_points_l)
        #     imgpoints_right.append(frame_img_points_r)

        ids_l = allIds['left'][i]
        ids_r = allIds['right'][i]
        if ids_l is None or ids_r is None:
            continue

        ids_l = ids_l.flatten()
        ids_r = ids_r.flatten()

        common = np.intersect1d(ids_l, ids_r)
        if len(common) < 4:
            continue

        mask_l = np.isin(ids_l, common)
        mask_r = np.isin(ids_r, common)

        corners_l = allCorners['left'][i][mask_l]
        ids_l_f = ids_l[mask_l].reshape(-1, 1).astype(np.int32)

        corners_r = allCorners['right'][i][mask_r]
        ids_r_f = ids_r[mask_r].reshape(-1, 1).astype(np.int32)

        obj_l, img_l = board.matchImagePoints(corners_l, ids_l_f)
        obj_r, img_r = board.matchImagePoints(corners_r, ids_r_f)

        # obj_l and obj_r should be identical (same board points); keep one.
        object_points.append(obj_l)
        imgpoints_left.append(img_l)
        imgpoints_right.append(img_r)

    stereocalibration_flags = cv2.CALIB_FIX_INTRINSIC
    ret, CM1, dist1, CM2, dist2, R, T, E, F = cv2.stereoCalibrate(object_points, imgpoints_left, imgpoints_right, mtx1,
                                                                 dist1,
                                                                 mtx2, dist2, (width, height), criteria=criteria, flags=stereocalibration_flags)

    data = {
        "rotation_matrix": R.tolist(),
        "translation_vector": T.tolist(),
        "essential_matrix": E.tolist(),
        "fundamental_matrix": F.tolist(),
        "stereo_rms_reprojection_error": ret,

    }
    with open(f"{debug_dir}/calibration.json", 'w') as outfile:
        json.dump(data, outfile)

    print(ret)
    return R, T


def calibrate(img_name, debug_dir):
    img_names = sorted(glob(img_name))
    if len(img_names) < 1:
        print("No images found")
        return None

    pathlib.Path(debug_dir).mkdir(parents=True, exist_ok=True)

    width = 7
    height = 5
    square_size = 0.0295
    marker_size = 0.0206
    aruco_dict_name = cv2.aruco.DICT_6X6_50

    pattern_size = (width, height)

    obj_points = []
    img_points = []
    # h, w = cv2.imread(img_names[0], cv2.IMREAD_GRAYSCALE).shape[:2]
    image_size = cv2.imread(img_names[0], cv2.IMREAD_GRAYSCALE).shape[::-1]

    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)
    board = cv2.aruco.CharucoBoard(pattern_size, square_size, marker_size, aruco_dict)
    charuco_detector = cv2.aruco.CharucoDetector(board)

    def processImage(fn):
        print('processing %s... ' % fn)
        img_c = cv2.imread(fn)
        img = cv2.cvtColor(img_c, cv2.COLOR_BGR2GRAY)
        if img is None:
            print("Failed to load", fn)
            return None

        assert image_size[0] == img.shape[1] and image_size[1] == img.shape[0], (
                "size: %d x %d ... " % (img.shape[1], img.shape[0]))
        found = False
        corners = 0

        corners, charucoIds, _, _ = charuco_detector.detectBoard(img)

        if (len(corners) > 3):
            frame_obj_points, frame_img_points = board.matchImagePoints(corners, charucoIds)
            found = True
        else:
            found = False

        img1 = img_c.copy()
        img2 = img_c.copy()
        img3 = img_c.copy()

        if frame_img_points is None or frame_img_points.size == 0 or frame_img_points.shape[0] < 3:
            return  # or just skip drawing

        x0 = int(frame_img_points[:, 0, 0].min())
        y0 = int(frame_img_points[:, 0, 1].min())
        x1 = int(frame_img_points[:, 0, 0].max())
        y1 = int(frame_img_points[:, 0, 1].max())

        # Guard: no NaNs/inf
        frame_img_points2 = frame_img_points.reshape(-1, 2)
        if not np.isfinite(frame_img_points2).all():
            return  # skip this frame


        if debug_dir:
            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            cv2.aruco.drawDetectedCornersCharuco(vis, corners, charucoIds=charucoIds)
            _path, name, _ext = splitfn(fn)
            outfile = os.path.join(debug_dir, name + '_board.png')
            cv2.imwrite(outfile, vis)

        if not found:
            print('pattern not found')
            return None

        print('           %s... OK' % fn)
        return (frame_img_points, frame_obj_points)

    chessboards = [processImage(fn) for fn in img_names]

    chessboards = [x for x in chessboards if x is not None]
    for (corners, pattern_points) in chessboards:
        img_points.append(corners)
        obj_points.append(pattern_points)

    # calculate camera distortion
    # criteria = (cv2.TERM_CRITERIA_EPS, 0, 1e-18)
    rms, camera_matrix, dist_coefs, _rvecs, _tvecs = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)

    data = {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coefs": dist_coefs.tolist(),
        "rms": rms,
    }
    with open(f"{debug_dir}/calibration.json", 'w') as outfile:
        json.dump(data, outfile)

    print("\nRMS:", rms)
    print("camera matrix:\n", camera_matrix)
    print("distortion coefficients: ", dist_coefs.ravel())

    mean_error = 0
    for i in range(len(obj_points)):
        img_points_2, _ = cv2.projectPoints(obj_points[i], _rvecs[i], _tvecs[i], camera_matrix, dist_coefs)
        error = cv2.norm(img_points[i], img_points_2, cv2.NORM_L2) / len(img_points_2)
        mean_error += error

    print("total error: {}".format(mean_error / len(obj_points)))

    # undistort the image with the calibration
    print('')
    for fn in img_names if debug_dir else []:
        _path, name, _ext = splitfn(fn)
        img_found = os.path.join(debug_dir, name + '_board.png')
        outfile = os.path.join(debug_dir, name + '_undistorted.png')

        img = cv2.imread(img_found)
        if img is None:
            continue

        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (w, h), 1, (w, h))

        dst = cv2.undistort(img, camera_matrix, dist_coefs, None, newcameramtx)

        # crop and save the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]

        # print('Undistorted image written to: %s' % outfile)
        cv2.imwrite(outfile, dst)

    print('Done')
    return rms, camera_matrix, dist_coefs, _rvecs, _tvecs


mouse_pos = (-1, -1)

def on_mouse(event, x, y, flags, userdata):
    global mouse_pos
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_pos = (x, y)

if __name__ == '__main__':
    # Left camera is Prophesee, Right camera is Basler
    data_dir_left = r"C:\Users\Bas_K\source\repos\Thesis\job_2025-12-28_18-54-04\images\verified\cam_0/*.png"
    debug_dir_left = r"C:\Users\Bas_K\source\repos\Thesis\job_2025-12-28_18-54-04\images\output\cam_0/"


    data_dir_right = r"C:\Users\Bas_K\source\repos\Thesis\job_2025-12-28_18-54-04\images\verified\cam_1/*.png"
    debug_dir_right = r"C:\Users\Bas_K\source\repos\Thesis\job_2025-12-28_18-54-04\images\output\cam_1/"

    calib_json = r"D:\datasets\calibration_bees_fake_1-4_human\job_data.json"

    # debug_dir = r"C:\Users\Bas_K\source\repos\Thesis\job_2025-12-28_18-54-04\images\output/"

    # rms_l, camera_matrix_l, dist_coefs_l, _rvecs_l, _tvecs_l = calibrate(data_dir_left, debug_dir_left)
    # rms_r, camera_matrix_r, dist_coefs_r, _rvecs_r, _tvecs_r = calibrate(data_dir_right, debug_dir_right)

    camera_matrix_l, dist_coefs_l = load_calibration(calib_json, "cam_0")
    camera_matrix_r, dist_coefs_r = load_calibration(calib_json, "cam_1")

    # R, T = stereo_calibrate(debug_dir, camera_matrix_l, dist_coefs_l, camera_matrix_r, dist_coefs_r, data_dir_left, data_dir_right)


    # rawL = cv2.imread("C:/Users/Bas_K/source/repos/Thesis/data/output/prophesee_dvs/frame_2057_board.png")
    # rawR = cv2.imread("C:/Users/Bas_K/source/repos/Thesis/data/output/basler_rgb/frame_2057_board.png")

    R, T, E, F = load_stereo_calibration(r"D:\datasets\calibration_bees_fake_1-4_human\job_data.json")

    cam_0 = cv2.imread(r"D:\datasets\bees_fake_1\processed\cam_0\frame_650.png")
    cam_1 = cv2.imread(r"D:\datasets\bees_fake_1\processed\cam_1\frame_650.png")

    win_name_0 = "left"
    cv2.namedWindow(win_name_0)
    cv2.setMouseCallback(win_name_0, on_mouse)

    win_name_1 = "right"
    cv2.namedWindow(win_name_1)
    cv2.setMouseCallback(win_name_1, on_mouse)

    ## Box cam_1
    # Box coordinates: Absolute:
    # 78, 528, 216, 663 width: 138 height: 135 size: (138x135)
    point1_0 = (78, 528)
    point1_1 = (216, 663)

    cv2.circle(cam_1, point1_0, 5, (0, 255, 0), -1)
    cv2.circle(cam_1, point1_1, 5, (0, 255, 0), -1)

    ## Point left
    fx = camera_matrix_r[0, 0]
    fy = camera_matrix_r[1, 1]
    cx = camera_matrix_r[0, 2]
    cy = camera_matrix_r[1, 2]
    Z = 398/1000  # in meters

    pt = np.array([[[point1_0[0], point1_0[1]]]], dtype=np.float32)
    pt_ud = cv2.undistortPoints(pt, camera_matrix_r, dist_coefs_r, P=camera_matrix_r)
    u_ud, v_ud = pt_ud.reshape(2)

    X = (u_ud - cx) * Z / fx
    Y = (v_ud - cy) * Z / fy

    X_R = np.array([[X], [Y], [Z]], dtype=np.float64)  # (3,1)
    T = T.reshape(3, 1).astype(np.float64)
    R = R.astype(np.float64)

    X_L = R.T @ (X_R - T)

    img_pt, _ = cv2.projectPoints(
        X_L.T,
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        camera_matrix_l,
        dist_coefs_l
    )

    c_l, v_l = img_pt.reshape(2)
    cv2.circle(cam_0, (int(c_l), int(v_l)), 5, (0, 255, 0), -1)

    pt = np.array([[[point1_1[0], point1_1[1]]]], dtype=np.float32)
    pt_ud = cv2.undistortPoints(pt, camera_matrix_r, dist_coefs_r, P=camera_matrix_r)
    u_ud, v_ud = pt_ud.reshape(2)

    X = (u_ud - cx) * Z / fx
    Y = (v_ud - cy) * Z / fy

    X_R = np.array([[X], [Y], [Z]], dtype=np.float64)  # (3,1)
    T = T.reshape(3, 1).astype(np.float64)
    R = R.astype(np.float64)

    X_L = R.T @ (X_R - T)

    img_pt, _ = cv2.projectPoints(
        X_L.T,
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        camera_matrix_l,
        dist_coefs_l
    )

    c_l, v_l = img_pt.reshape(2)
    cv2.circle(cam_0, (int(c_l), int(v_l)), 5, (0, 255, 0), -1)

    # # epipolar line right
    # line_1_0 = cv2.computeCorrespondEpilines(pts0_0, 1, F)
    # a, b, c = line_1_0[0, 0]
    # h, w = cam_1.shape[:2]
    # x0, x1 = 0, w - 1
    # y0 = int(round((-c - a * x0) / b))
    # y1 = int(round((-c - a * x1) / b))
    # cv2.line(cam_1, (x0, y0), (x1, y1), (0, 255, 255), 2)

    while True:
        display_0 = cam_0.copy()
        display_1 = cam_1.copy()

        if mouse_pos[0] >= 0 and mouse_pos[1] >= 0:
            text_0 = f"x={mouse_pos[0]}, y={mouse_pos[1]}"
            cv2.putText(
                display_0,
                text_0,
                (10, 25),  # top-left corner
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if mouse_pos[0] >= 0 and mouse_pos[1] >= 0:
            text_1 = f"x={mouse_pos[0]}, y={mouse_pos[1]}"
            cv2.putText(
                display_1,
                text_1,
                (10, 25),  # top-left corner
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow(win_name_0, display_0)
        cv2.imshow(win_name_1, display_1)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

    cv2.destroyAllWindows()
