import vtk
import pcl
from vtk.util import numpy_support

clouds = pcl.load("pointcloud.pcd")

# Create the geometry of a point (the coordinate)
points = vtk.vtkPoints()
p = [1.0, 2.0, 3.0]

# Create the topology of the point (a vertex)
vertices = vtk.vtkCellArray()

for pi in clouds.to_array():
    id = points.InsertNextPoint(pi.tolist())
    vertices.InsertNextCell(1)
    vertices.InsertCellPoint(id)

clouds_array = clouds.to_array()

Colors = vtk.vtkUnsignedCharArray()
Colors.SetNumberOfComponents(3)
Colors.SetName("Colors")
for pi in clouds.to_array():
    Colors.InsertNextTuple3(255,0,0)

# Create a polydata object
polydata = vtk.vtkPolyData()

# Set the points and vertices we created as the geometry and topology of the polydata
polydata.SetPoints(points)
polydata.SetVerts(vertices)
polydata.GetPointData().SetScalars(Colors)

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