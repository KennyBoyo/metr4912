#!/usr/bin/env python
import vtk
import os, sys

def vtk2vtp(invtkfile, outvtpfile, binary=False):
    """What it says on the label"""
    reader = vtk.vtkXMLPolyDataReader()
    reader.SetFileName(invtkfile)
    reader.Update()

    writer = vtk.vtkSTLWriter()
    writer.SetFileName(outvtpfile)
    if binary:
        writer.SetFileTypeToBinary()
    writer.SetInputConnection(reader.GetOutputPort())
    writer.Update()

if __name__ == '__main__':
    args = sys.argv
    binary = False
    if '-b' in args:
        args.remove('-b')
        binary = True
    if len(args) < 2:
        print('Batch converts vtk files to vtp files.\nUsage:\n    vtk2vtp.py model1.vtk model2.vtk ...')
        print('    [-b] causes output to be in binary format, much smaller vtp file size, if it happens to work')
        sys.exit()
    dir = args[1]
    # print(os.listdir(dir))
    for name in os.listdir(dir):
        # if name[-11:] == "_mirror.vtp":
            fullPath = dir + '/' + name
            fullPath2 = dir + '_stl/' + name
            print(fullPath)
            vtk2vtp(fullPath, fullPath2[:-4]+'.stl', binary=binary)
    # for vtkfile in infiles:
    #     vtk2vtp(vtkfile, vtkfile[:-4]+'.vtp', binary=binary)