from __future__ import print_function

import random
import os
import os.path
import sys

from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Display.SimpleGui import init_display

from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Extend.DataExchange import read_step_file

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepGProp import brepgprop_VolumeProperties, brepgprop_SurfaceProperties, BRepGProp_Face
import OCC.Core.BRepGProp as BRP
from OCC.Core.TopoDS import topods
import OCC.Core.TopExp
from OCC.Core.GProp import GProp_GProps, GProp_CenterMassX, GProp_CenterMassY

"""
def import_as_one_shape(event=None):
    #shp = read_step_file(os.path.join("..", "assets", "models", "as1_pe_203.stp"))
    shp = read_step_file(os.path.join("Antriebswelle.STEP"))
    

    # Convert the shape to a TopoDS_Shape
    #topo_shape = topods.TopoDS_Shape(shp)

    # Create a GProp_GProps object to store the properties
    gprops = GProp_GProps()

    # Compute properties
    #BRep_Tool.SurfaceProperties(topo_shape, gprops)
    brepgprop_VolumeProperties(shp, gprops)

    # Get the center of mass
    print('+++++++++++++++++++++++++++++++++++++++++++++++++++++')
    print(gprops.Mass())
    cog = gprops.CentreOfMass()
    cog_x, cog_y, cog_z = cog.Coord()
    print("Center of mass: x = %f;y = %f;z = %f;" % (cog_x, cog_y, cog_z))
    print('+++++++++++++++++++++++++++++++++++++++++++++++++++++')
    print('Faces')


    # then loop over faces
    t = TopologyExplorer(shp)
    props = GProp_GProps()
    shp_idx = 1
    for face in t.faces():
        BRepGProp_Face(face)
        brepgprop_SurfaceProperties(face, props)
        face_surf = props.Mass()
        #normal_vector = props.PrincipalProperties()
        #print('Normal Vektor:',normal_vector)
        print("Surface for face nbr %i : %f" % (shp_idx, face_surf))
        shp_idx += 1


    display.EraseAll()
    display.DisplayShape(shp, update=True)


def import_as_multiple_shapes(event=None):
    compound = read_step_file(os.path.join("..", "assets", "models", "as1_pe_203.stp"))
    t = TopologyExplorer(compound)
    display.EraseAll()
    for solid in t.solids():
        color = Quantity_Color(
            random.random(), random.random(), random.random(), Quantity_TOC_RGB
        )
        display.DisplayColoredShape(solid, color)
    display.FitAll()


def exit(event=None):
    sys.exit()


if __name__ == "__main__":
    display, start_display, add_menu, add_function_to_menu = init_display()
    add_menu("STEP import")
    add_function_to_menu("STEP import", import_as_one_shape)
    add_function_to_menu("STEP import", import_as_multiple_shapes)
    start_display()
""" 

#shp = read_step_file(os.path.join("..", "assets", "models", "as1_pe_203.stp"))
shp = read_step_file(os.path.join("Antriebswelle.STEP"))


# Convert the shape to a TopoDS_Shape
#topo_shape = topods.TopoDS_Shape(shp)

# Create a GProp_GProps object to store the properties
gprops = GProp_GProps()

# Compute properties
#BRep_Tool.SurfaceProperties(topo_shape, gprops)
brepgprop_VolumeProperties(shp, gprops)

# Get the center of mass
print('+++++++++++++++++++++++++++++++++++++++++++++++++++++')
print(gprops.Mass())
cog = gprops.CentreOfMass()
cog_x, cog_y, cog_z = cog.Coord()
print("Center of mass: x = %f;y = %f;z = %f;" % (cog_x, cog_y, cog_z))
print('+++++++++++++++++++++++++++++++++++++++++++++++++++++')
print('Faces')


# then loop over faces
t = TopologyExplorer(shp)
props = GProp_GProps()
shp_idx = 1
for face in t.faces():
    Face = BRepGProp_Face(face)
    Normal = Face.Normal()
    #brepgprop_SurfaceProperties(face, props)
    #face_surf = props.Mass()
    #normal_vector = props.PrincipalProperties()
    #print('Normal Vektor:',normal_vector)
    print("Surface for face nbr %i : %f" % (shp_idx, face_surf))
    shp_idx += 1