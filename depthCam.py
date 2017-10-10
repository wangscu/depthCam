import TY as ty
import numpy as np
from ctypes import *
import cv2
import matplotlib.pyplot as plt
import vtk
import pcl
from vtk.util import numpy_support
import yaml
from transformations import *
from uf.wrapper.swift_api import SwiftAPI
from uf.utils.log import *
from time import sleep
logger_init(logging.VERBOSE)

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
            cv2.imwrite("wt.png", img)
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
        import pcl

        p = pcl.PointCloud(pointBuf)
        pcl.save(p, 'pointcloud.pcd')

vectors = (ty.TY_VECT_3F * pointBufLen)()
for i, p in enumerate(pointBuf):
    vectors[i].x = p[0]
    vectors[i].y = p[1]
    vectors[i].z = p[2]

with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)
camera_matrix = np.array(loadeddict.get('camera_matrix'))
dist_coeff = np.array(loadeddict.get('dist_coeff'))

ty.TYRegisterWorldToColor(hh, vectors, 0, pointBufLen, buffer, 1024 * 1024 * 5)
colorBuffer = cast(buffer, POINTER(c_uint16 * (1024 * 512 * 5))).contents
colorBuffer = np.ctypeslib.as_array(colorBuffer)[:rgbBuf.shape[0] * rgbBuf.shape[1]].reshape((rgbBuf.shape[0], rgbBuf.shape[1]))

word = np.zeros((rgbBuf.shape[0], rgbBuf.shape[1], 3), dtype=np.float64)

for x in range(rgbBuf.shape[1]):
    for y in range(rgbBuf.shape[0]):
        z = colorBuffer[y][x]
        fx = camera_matrix[0][0]
        fy = camera_matrix[1][1]
        cx = camera_matrix[0][2]
        cy = camera_matrix[1][2]
        _x = (x - cx) *z / fx
        _y = (y - cy) *z / fy
        word[y][x][0] = _x
        word[y][x][1] = _y
        word[y][x][2] = z

#p = pcl.PointCloud(word.reshape(rgbBuf.shape[0]*rgbBuf.shape[1], 3).astype(float))
#pcl.save(p, 'word.pcd')

#import matplotlib.pyplot as plt
#color = colorBuffer / 100 + rgbBuf[:, :, 0:1] / 2
#plt.imshow(colorBuffer.reshape((rgbBuf.shape[0], rgbBuf.shape[1])))  # Needs to be in row,col order
# plt.imshow(rgbBuf[:, :, 0:1].reshape((rgbBuf.shape[0], rgbBuf.shape[1])))  # Needs to be in row,col order
# plt.imshow(color.reshape(rgbBuf.shape[0], rgbBuf.shape[1]))  # Needs to be in row,col order
#plt.show()
#plt.savefig('depth.png')

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

arm_corners = []
arm_ids = []

for corner, id in zip(corners, ids):
    if id[0] > 20:
        drug_corner = corner[0]
    else:
        arm_corners.append(corner[0])
        arm_ids.append(id[0])


rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(np.array(arm_corners), 30, camera_matrix, dist_coeff)
for i in range(0, len(arm_corners)):
    aruco.drawAxis(dImgd, camera_matrix, dist_coeff, rvecs[i], tvecs[i], 40)

if len(drug_corner) > 0:
    drug_rvecs, drug_tvecs, drug_objPoints = aruco.estimatePoseSingleMarkers(np.array([drug_corner]), 40, camera_matrix,
                                                                             dist_coeff)
    aruco.drawAxis(dImgd, camera_matrix, dist_coeff, drug_rvecs, drug_tvecs, 60)

data = {'rvec': np.asarray(drug_rvecs[0]).tolist(), 'tvec': np.asarray(drug_tvecs[0]).tolist()}
with open("drug_center.yaml", "w") as f:
    yaml.dump(data, f)

width_array = np.sort(drug_corner[:, 0:1].reshape([4]).astype(int))
height_array = np.sort(drug_corner[:, 1:2].reshape([4]).astype(int))

drug_point = word[height_array[0]:height_array[3], width_array[0]:width_array[3]]
drug_depth = colorBuffer[height_array[0]:height_array[3], width_array[0]:width_array[3]]

#colorBuffer[ height_array[0]:height_array[3], width_array[0]:width_array[3]] = 100
#plt.imshow(colorBuffer)  # Needs to be in row,col order
# plt.imshow(rgbBuf[:, :, 0:1].reshape((rgbBuf.shape[0], rgbBuf.shape[1])))  # Needs to be in row,col order
# plt.imshow(color.reshape(rgbBuf.shape[0], rgbBuf.shape[1]))  # Needs to be in row,col order
#plt.show()

drug_point_effect = []
drug_depth_effect = []
for x, y in np.ndindex(drug_point.shape[0], drug_point.shape[1]):
    if drug_point[x][y][2] > 0:
        drug_point_effect.append(drug_point[x][y])
        drug_depth_effect.append(drug_depth[x][y])

data = {'drug_point_effect': np.asarray(drug_point_effect).tolist()}
with open("drug_point_effect.yaml", "w") as f:
    yaml.dump(data, f)

data = {'drug_depth_effect': np.asarray(drug_depth_effect).tolist()}
with open("drug_depth_effect.yaml", "w") as f:
    yaml.dump(data, f)

imgPoints = np.zeros(shape=(0, 2))
objectPoints = np.zeros(shape=(0, 3))
for i in range(0, len(arm_corners)):
    id = arm_ids[i]
    imgPoints = np.append(imgPoints, np.array(arm_corners[i]), axis=0)
    objectPoints = np.append(objectPoints, cornerPoints[id - 1], axis=0)
_retval, _rvec, _tvec = cv2.solvePnP(objectPoints, imgPoints, camera_matrix, dist_coeff)
aruco.drawAxis(dImgd, camera_matrix, dist_coeff, _rvec, _tvec, 1000)
data = {'rvec': np.asarray(_rvec).tolist(), 'tvec': np.asarray(_tvec).tolist()}
with open("rt_center.yaml", "w") as f:
    yaml.dump(data, f)

axis = np.float32([[0,0,0]]).reshape(-1,3)
imgpts, _ = cv2.projectPoints(axis, _rvec, _tvec, camera_matrix, dist_coeff)

cp = imgpts[0][0].astype(int)
center_depth = colorBuffer[(cp[1]-10):(cp[1]+10), (cp[0]-10):(cp[0]+10)]

rgbBuf[height_array[0]:height_array[3], width_array[0]:width_array[3]] = 100
rgbBuf[(cp[1]-10):(cp[1]+10), (cp[0]-10):(cp[0]+10), 0:1] = 200
img = cv2.cvtColor(rgbBuf, cv2.COLOR_YUV2BGR_YUYV)
cv2.imwrite("wt_1.png", img)
#plt.imshow(rgbBuf)
#plt.show()

center_point = word[(cp[1]-10):(cp[1]+10), (cp[0]-10):(cp[0]+10)]
center_point_effect = []
center_depth_effect = []
for x, y in np.ndindex(center_point.shape[0], center_point.shape[1]):
    if center_point[x][y][2] > 0:
        center_point_effect.append(center_point[x][y])
        center_depth_effect.append(center_depth[x][y])

data = {'center_point_effect': np.asarray(center_point_effect).tolist()}
with open("center_point_effect.yaml", "w") as f:
    yaml.dump(data, f)

data = {'center_depth_effect': np.asarray(center_depth_effect).tolist()}
with open("center_depth_effect.yaml", "w") as f:
    yaml.dump(data, f)

'''
word[height_array[0]:height_array[3], width_array[0]:width_array[3]] = (0, 0, 0)
word[(cp[1]-10):(cp[1]+10), (cp[0]-40):(cp[0]+40)] = (0, 0, 0)
points = vtk.vtkPoints()
# Create the topology of the point (a vertex)
vertices = vtk.vtkCellArray()
for x, y in np.ndindex(word.shape[0], word.shape[1]):
    id = points.InsertNextPoint(word[x][y])
    vertices.InsertNextCell(1)
    vertices.InsertCellPoint(id)

# Create a polydata object
polydata = vtk.vtkPolyData()

# Set the points and vertices we created as the geometry and topology of the polydata
polydata.SetPoints(points)
polydata.SetVerts(vertices)
#polydata.GetPointData().SetScalars(Colors)

# Visualize
mapper = vtk.vtkPolyDataMapper()
mapper.SetInputData(polydata)

actor = vtk.vtkActor()
actor.SetMapper(mapper)
actor.GetProperty().SetPointSize(1)

renderer = vtk.vtkRenderer()
renderWindow = vtk.vtkRenderWindow()
renderWindow.SetSize(1280, 960)
renderWindow.AddRenderer(renderer)
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)

renderer.AddActor(actor)

renderWindow.Render()
renderWindowInteractor.Start()
'''
cv2.imwrite('aruco_d.jpg', dImgd)

I = np.identity(4, np.float64)
rv = np.array(_rvec)
tv = np.array(_tvec)
rm, _ = cv2.Rodrigues(rv)

I[0:3, 0:3] = rm
I[0:3, 3:4] = tv.reshape(3, 1)

IR = inverse_matrix(I)

p = [0, 0, 0 , 1]
p[:3] = drug_tvecs[0][0]
point = np.dot(IR, np.array(p))

position = [int(point[1]), int(-point[0]), int(point[2]+50)]

data = {'matrix': np.asarray(I).tolist(), 'drug_point':np.asarray(point).tolist(), 'position':np.asarray(position).tolist()}
with open("matrix.yaml", "w") as f:
    yaml.dump(data, f)

print(I)

exit(0)


swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})


print(position)
sleep(10)
swift.set_position(position[0], position[1], 200, speed=1500)
sleep(10)
swift.set_position(position[0], position[1], position[2], speed=1500)
sleep(10)
swift.set_pump(True)
sleep(2)
swift.set_position(position[0], position[1], 200, speed=1500)
sleep(10)
swift.set_position(100, 200, 100, speed=1500)
sleep(10)
swift.set_pump(False)