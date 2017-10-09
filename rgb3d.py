import TY as ty
import numpy as np
from ctypes import *
import cv2
import matplotlib.pyplot as plt
import pcl
import pcl.pcl_visualization
import yaml

buffer = create_string_buffer(1024 * 1024 * 5)

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

ty.TYSetEnum(hh, ty.TY_COMPONENT_DEPTH_CAM, ty.TY_ENUM_IMAGE_MODE, ty.TY_IMAGE_MODE_1280x960)
ty.TYSetEnum(hh, ty.TY_COMPONENT_POINT3D_CAM, ty.TY_ENUM_IMAGE_MODE, ty.TY_IMAGE_MODE_1280x960)
#ty.TYSetEnum(hh, ty.TY_COMPONENT_RGB_CAM, ty.TY_ENUM_PIXEL_FORMAT, ty.TY_PIXEL_FORMAT_RGB)


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
pointBufLen = 0

with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)
camera_matrix = np.array(loadeddict.get('camera_matrix'))
dist_coeff = np.array(loadeddict.get('dist_coeff'))

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
            img = cv2.cvtColor(rgbBuf, cv2.COLOR_YUV2RGB_YUYV)
            #cv::undistort(color, u, pData->colorM, pData->colorD, pData->colorM);
            img = cv2.undistort(img,camera_matrix, dist_coeff)
            cv2.imwrite('img.png',img)
            pass
        elif image.pixelFormat == ty.TY_PIXEL_FORMAT_RGB:
            #cv::Mat rgb(frame.image[i].height, frame.image[i].width, CV_8UC3, frame.image[i].buffer);
            #cv::cvtColor(rgb, *pColor, cv::COLOR_RGB2BGR);
            rgbBuf = cast(image.buffer, POINTER(c_uint8 * (height * width * 3))).contents
            rgbBuf = np.ctypeslib.as_array(rgbBuf).reshape((height, width, 3))
            img = cv2.cvtColor(rgbBuf, cv2.COLOR_RGB2BGR)
            cv2.imwrite('img_rgb.png',img)
            pass
        elif image.pixelFormat == ty.TY_PIXEL_FORMAT_MONO:
            print(image.pixelFormat)
            pass
        pass
    elif image.componentID == ty.TY_COMPONENT_DEPTH_CAM:
        depthBuf = cast(image.buffer, POINTER(c_uint16 * (height * width))).contents
        depthBuf = np.ctypeslib.as_array(depthBuf)
        depthBuf = depthBuf.reshape((height, width))
        #np.savetxt('depth.out', depthBuf, delimiter=',', fmt='%.2i')
        # img = cv2.cvtColor(depthBuf, cv2.COLOR_GRAY2BGR)
        # cv2.imwrite("depth.png", img)
        import matplotlib.pyplot as plt

        #plt.imshow(depthBuf)  # Needs to be in row,col order
        #plt.savefig('depth.png')
        pass

    elif image.componentID == ty.TY_COMPONENT_POINT3D_CAM:
        pointBuf = cast(image.buffer, POINTER(c_float * (height * width * 3))).contents
        pointBuf = np.ctypeslib.as_array(pointBuf).reshape((height * width, 3))
        pointBufLen = height * width

vectors = (ty.TY_VECT_3F * pointBufLen)()
for i, p in enumerate(pointBuf):
    vectors[i].x = p[0]
    vectors[i].y = p[1]
    vectors[i].z = p[2]

ty.TYRegisterWorldToColor(hh, vectors, 0, pointBufLen, buffer, 1024 * 1024 * 5)
colorBuffer = cast(buffer, POINTER(c_uint16 * (1024 * 512 * 5))).contents
colorBuffer = np.ctypeslib.as_array(colorBuffer)[:rgbBuf.shape[0] * rgbBuf.shape[1]].reshape((rgbBuf.shape[0], rgbBuf.shape[1]))

colorPoint= []

for x in range(rgbBuf.shape[1]):
    for y in range(rgbBuf.shape[0]):
        z = colorBuffer[y][x]
        color = int(img[y][x][0]) *256*256 + int(img[y][x][1]) *256 + int(img[y][x][2])
        if not np.isnan(z):
            #float fx = m_oldIntrinsic.data[0];
            #float fy = m_oldIntrinsic.data[4];
            #float cx = m_oldIntrinsic.data[2];
            #float cy = m_oldIntrinsic.data[5];
            fx = camera_matrix[0][0]
            fy = camera_matrix[1][1]
            cx = camera_matrix[0][2]
            cy = camera_matrix[1][2]
            _x = (x - cx) *z / fx;
            _y = (y - cy) *z / fy;
            colorPoint.append(( _y, _x,  z, color))



#rgbBuf = img.reshape((960*1280, 3))

#colorPoint= []
#for i in range(pointBufLen):
#    if not np.isnan(pointBuf[i][0]):
#        colorPoint.append((pointBuf[i][0], pointBuf[i][1], pointBuf[i][2], int(rgbBuf[i][0]) *256*256 + int(rgbBuf[i][1]) *256 + int(rgbBuf[i][2])))
#        #colorPoint.append((pointBuf[i][0], pointBuf[i][1], pointBuf[i][2], 90))



cloud = pcl.PointCloud_PointXYZRGB()
cloud.from_list(colorPoint)

visual = pcl.pcl_visualization.CloudViewing()
visual.ShowColorCloud(cloud, b'cloud')
while True:
    visual.WasStopped()
end

print(cloud)


















