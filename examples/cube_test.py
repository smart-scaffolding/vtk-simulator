#!/usr/bin/env python

import vtk


def main():
    # Basic stuff setup
    # Set up the renderer, window, and interactor
    colors = vtk.vtkNamedColors()

    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    renWin.SetSize(640, 480)
    iRen = vtk.vtkRenderWindowInteractor()
    iRen.SetRenderWindow(renWin)


    # Make a lookup table using a color series.
    # colorSeries = vtk.vtkColorSeries()
    # colorSeries.SetColorScheme(vtk.vtkColorSeries.BREWER_DIVERGING_SPECTRAL_11)
    #
    # lut = vtk.vtkLookupTable()
    # colorSeries.BuildLookupTable(lut, vtk.vtkColorSeries.ORDINAL)



    # Set up the Orientation Marker Widget.
    prop_assembly = MakeAnnotatedCubeActor(colors)
    om1 = vtk.vtkOrientationMarkerWidget()
    om1.SetOrientationMarker(prop_assembly)
    om1.SetInteractor(iRen)
    om1.SetDefaultRenderer(ren)
    om1.On()
    om1.InteractiveOn()

    xyzLabels = ['X', 'Y', 'Z']
    scale = [1.0, 1.0, 1.0]
    axes = MakeAxesActor(scale, xyzLabels)

    om2 = vtk.vtkOrientationMarkerWidget()
    om2.SetOrientationMarker(axes)
    # Position lower right in the viewport.
    om2.SetViewport(0.8, 0, 1.0, 0.2)
    om2.SetInteractor(iRen)
    om2.EnabledOn()
    # om2.InteractiveOn()

    # ren.AddActor(coneActor)
    # ren.AddActor(contourLineActor)
    # ren.SetBackground2(colors.GetColor3d('RoyalBlue'))
    # ren.SetBackground(colors.GetColor3d('MistyRose'))
    # ren.GradientBackgroundOn()
    ren.GetActiveCamera().Azimuth(45)
    ren.GetActiveCamera().Pitch(-22.5)
    ren.ResetCamera()

    renWin.SetSize(600, 600)
    renWin.Render()
    renWin.SetWindowName('ColoredAnnotatedCube')
    renWin.Render()
    iRen.Start()


def MakeAnnotatedCubeActor(colors):
    # Colored faces cube setup
    cube_source = vtk.vtkCubeSource()
    cube_source.Update()

    face_colors = vtk.vtkUnsignedCharArray()
    face_colors.SetNumberOfComponents(3)
    face_x_plus = colors.GetColor3ub('Red')
    face_x_minus = colors.GetColor3ub('Red')
    face_y_plus = colors.GetColor3ub('Red')
    face_y_minus = colors.GetColor3ub('Red')
    face_z_plus = colors.GetColor3ub('Cyan')
    face_z_minus = colors.GetColor3ub('Red')
    face_colors.InsertNextTypedTuple(face_x_minus)
    face_colors.InsertNextTypedTuple(face_x_plus)
    face_colors.InsertNextTypedTuple(face_y_minus)
    face_colors.InsertNextTypedTuple(face_y_plus)
    face_colors.InsertNextTypedTuple(face_z_minus)
    face_colors.InsertNextTypedTuple(face_z_plus)

    cube_source.GetOutput().GetCellData().SetScalars(face_colors)
    cube_source.Update()

    cube_mapper = vtk.vtkPolyDataMapper()
    cube_mapper.SetInputData(cube_source.GetOutput())
    cube_mapper.Update()

    cube_actor = vtk.vtkActor()
    cube_actor.SetMapper(cube_mapper)

    # Assemble the colored cube and annotated cube texts into a composite prop.
    prop_assembly = vtk.vtkPropAssembly()
    # prop_assembly.AddPart(annotated_cube)
    prop_assembly.AddPart(cube_actor)
    return prop_assembly


def MakeAxesActor(scale, xyzLabels):
    axes = vtk.vtkAxesActor()
    axes.SetScale(scale[0], scale[1], scale[2])
    axes.SetShaftTypeToCylinder()
    axes.SetXAxisLabelText(xyzLabels[0])
    axes.SetYAxisLabelText(xyzLabels[1])
    axes.SetZAxisLabelText(xyzLabels[2])
    axes.SetCylinderRadius(0.5 * axes.GetCylinderRadius())
    axes.SetConeRadius(1.025 * axes.GetConeRadius())
    axes.SetSphereRadius(1.5 * axes.GetSphereRadius())
    tprop = axes.GetXAxisCaptionActor2D().GetCaptionTextProperty()
    tprop.ItalicOn()
    tprop.ShadowOn()
    tprop.SetFontFamilyToTimes()
    # Use the same text properties on the other two axes.
    axes.GetYAxisCaptionActor2D().GetCaptionTextProperty().ShallowCopy(tprop)
    axes.GetZAxisCaptionActor2D().GetCaptionTextProperty().ShallowCopy(tprop)
    return axes


if __name__ == '__main__':
    main()