import TY as ty
import numpy as np
from ctypes import *
import cv2
import matplotlib.pyplot as plt

buffer = create_string_buffer(1024*1024*5)

ty._TYInitLib()
dn = pointer(c_int32())
ty.TYGetDeviceNumber(dn)
DBI = ty.TY_DEVICE_BASE_INFO()
pDBI = pointer(DBI)
ty.TYGetDeviceList(pDBI, 100, dn)

hh = ty.TY_DEV_HANDLE()
did = pDBI[0].id.decode('utf-8')
didStr = ty.ReturnString(c_char_p(pDBI[0].id))

ty.TYOpenDevice(didStr, pointer(hh))

allComps = c_int32()
ty.TYGetComponentIDs(hh, pointer(allComps))
componentIDs = ty.TY_COMPONENT_POINT3D_CAM | ty.TY_COMPONENT_RGB_CAM | ty.TY_COMPONENT_DEPTH_CAM
ty.TYEnableComponents(hh, componentIDs)

frameSize = c_int32()
ty.TYGetFrameBufferSize(hh, pointer(frameSize))
frameBuffer = [create_string_buffer(frameSize.value),
               create_string_buffer(frameSize.value), ]

ty.TYEnqueueBuffer(hh, frameBuffer[0], frameSize.value)
ty.TYEnqueueBuffer(hh, frameBuffer[1], frameSize.value)

ty.TYSetBool(hh, ty.TY_COMPONENT_DEVICE, ty.TY_BOOL_TRIGGER_MODE, c_bool(False))

ty.TYStartCapture(hh)

frame = ty.TY_FRAME_DATA()
userdata = ty.TY_FRAME_DATA()

err = ty.TYFetchFrame(hh, pointer(frame), -1)
print(err)

for i in range(0, frame.validCount):
    image = frame.image[i]
    height = image.height
    width = image.width

    if image.componentID == ty.TY_COMPONENT_RGB_CAM:

        if image.pixelFormat == ty.TY_PIXEL_FORMAT_YVYU:
            print(image.pixelFormat)
            pass
        elif image.pixelFormat == ty.TY_PIXEL_FORMAT_YUYV:
            rgbBuf = cast(image.buffer, POINTER(c_uint8 * (height * width * 2))).contents
            rgbBuf = np.ctypeslib.as_array(rgbBuf).reshape((height, width, 2))
            img = cv2.cvtColor(rgbBuf, cv2.COLOR_YUV2BGR_YUYV)
            # cv2.imwrite("wt.png", img)
            pass
        elif image.pixelFormat == ty.TY_PIXEL_FORMAT_RGB:
            print(image.pixelFormat)
            pass
        elif image.pixelFormat == ty.TY_PIXEL_FORMAT_MONO:
            print(image.pixelFormat)
            pass
        pass
    elif image.componentID == ty.TY_COMPONENT_DEPTH_CAM:
        depthBuf = cast(image.buffer, POINTER(c_uint16 * (height * width))).contents
        depthBuf = np.ctypeslib.as_array(depthBuf)
        depthBuf = depthBuf.reshape((height, width))
        np.savetxt('depth.out', depthBuf, delimiter=',', fmt='%.2i')
        #img = cv2.cvtColor(depthBuf, cv2.COLOR_GRAY2BGR)
        #cv2.imwrite("depth.png", img)
        import matplotlib.pyplot as plt

        plt.imshow(depthBuf)  # Needs to be in row,col order
        plt.savefig('depth.png')
        pass

    elif image.componentID == ty.TY_COMPONENT_POINT3D_CAM:
        pointBuf = cast(image.buffer, POINTER(c_float * (height * width * 3))).contents
        pointBuf = np.ctypeslib.as_array(pointBuf).reshape((height*width, 3))
        import pcl
        p = pcl.PointCloud(pointBuf)
        pcl.save(p, 'pointcloud.pcd')

vectors = (ty.TY_VECT_3F * (height*width))()
for i,p in enumerate(pointBuf):
    vectors[i].x = p[0]
    vectors[i].y = p[1]
    vectors[i].z = p[2]

ty.TYRegisterWorldToColor(hh, vectors, 0, height*width, buffer, 1024*1024*5)
colorBuffer = cast(buffer, POINTER(c_uint16 * (1024 * 512 * 5))).contents
colorBuffer = np.ctypeslib.as_array(colorBuffer)[:rgbBuf.shape[0]*rgbBuf.shape[1]].reshape((rgbBuf.shape[0], rgbBuf.shape[1], 1))

import matplotlib.pyplot as plt

color = colorBuffer/100 + rgbBuf[:, :, 0:1]/2
plt.imshow(colorBuffer.reshape((rgbBuf.shape[0], rgbBuf.shape[1])))  # Needs to be in row,col order
plt.imshow(rgbBuf[:, :, 0:1].reshape((rgbBuf.shape[0], rgbBuf.shape[1])))  # Needs to be in row,col order
#plt.imshow(color.reshape(rgbBuf.shape[0], rgbBuf.shape[1]))  # Needs to be in row,col order
plt.show()
plt.savefig('depth.png')


'''
    import cv2.aruco as aruco
    import yaml

    cornerPoints = np.array([[[-87.5, 87.5, 0.0], [-57.5, 87.5, 0.0], [-57.5, 57.5, 0.0], [-87.5, 57.5, 0.0]],
                             [[57.5, 87.5, 0.0], [87.5, 87.5, 0.0], [87.5, 57.5, 0.0], [57.5, 57.5, 0.0]],
                             [[-87.5, -57.5, 0.0], [-57.5, -57.5, 0.0], [-57.5, -87.5, 0.0], [-87.5, -87.5, 0.0]],
                             [[57.5, -57.5, 0.0], [87.5, -57.5, 0.0], [87.5, -87.5, 0.0], [57.5, -87.5, 0.0]]])

    dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    dImg = img
    # dImg = cv2.GaussianBlur(dImg, (5, 5), 0)
    # _, dImg = cv2.threshold(dImg, 200, 255, cv2.THRESH_BINARY)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(dImg, dict)
    dImgd = aruco.drawDetectedMarkers(img, corners, ids)

    with open('calibration.yaml') as f:
        loadeddict = yaml.load(f)
    camera_matrix = np.array(loadeddict.get('camera_matrix'))
    dist_coeff = np.array(loadeddict.get('dist_coeff'))

    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 30, camera_matrix, dist_coeff)
    for i in range(0, len(corners)):
        aruco.drawAxis(dImgd, camera_matrix, dist_coeff, rvecs[i], tvecs[i], 40)

        data = {'rvecs': np.asarray(rvecs[i]).tolist(), 'tvecs': np.asarray(tvecs[i]).tolist()}
        with open("rt{}.yaml".format(i + 1), "w") as f:
            yaml.dump(data, f)

    imgPoints = np.zeros(shape=(0, 2))
    objectPoints = np.zeros(shape=(0, 3))
    for i in range(0, len(corners)):
        id = ids[i]
        imgPoints = np.append(imgPoints, corners[i][0], axis=0)
        objectPoints = np.append(objectPoints, cornerPoints[id - 1][0], axis=0)
    _retval, _rvec, _tvec = cv2.solvePnP(objectPoints, imgPoints, camera_matrix, dist_coeff)
    aruco.drawAxis(dImgd, camera_matrix, dist_coeff, _rvec, _tvec, 10)
    data = {'rvec': np.asarray(_rvec).tolist(), 'tvec': np.asarray(_tvec).tolist()}
    with open("rt_center.yaml", "w") as f:
        yaml.dump(data, f)

    cv2.imwrite('aruco_d.jpg', dImgd)
'''
