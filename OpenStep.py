from __future__ import print_function
import os
import os.path
import time
from os import listdir
from os.path import isfile, join


from OCC.Extend.TopologyUtils import TopologyExplorer, TopoDS_Face
from OCC.Extend.DataExchange import read_step_file

from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Dir

# Topologie Pakete -----------------------------------------------------------------------------------
# Ein Modell besteht in aller regel aus mehreen primitven formen. Eine Welle z.B. aus verschiedenen 
# Zylindern und Kegeln und auch Endflächen. Die Topologie beschreibt dann wie diese Primitven Formen 
# zusammengeschnürt werden müssen also wie diese Formen relativ zueinander stehen müssen
# um das gesamte Modell darzustellen.
from OCC.Core.TopoDS import topods, TopoDS_Face, TopoDS_Edge, TopoDS_Shape # DS == Data Struktur
from OCC.Core import TopLoc # paket bietet Ressourcen für den Umgang mit lokalen 3D-Koordinatensystemen
# topologie pakete um die topologien zu bearbeiten 
import OCC.Core.TopTools
from OCC.Core.TopExp import TopExp_Explorer # Um Topologie eines gegenstand zu untersuchen
from OCC.Core import BRepTools, TopAbs


# Bearbeitungs Pakete ---------------------------------------------------------------------------------
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepGProp import brepgprop_SurfaceProperties, brepgprop_VolumeProperties, BRepGProp_Face, BRepGProp_Vinert
from OCC.Core.GeomLProp import GeomLProp_SLProps # Um die krümmung einer Fläche zu bestimmen
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere

# Visu Pakete --------------------------------------------------------------------------------------------
from OCC.Display.SimpleGui import init_display
from OCC.Core.AIS import AIS_ColoredShape
from OCC.Display.OCCViewer import rgb_color

import tkinter as tk
from tkinter import filedialog


class Part():
    def __init__(self,dir):
        print('Part init')
        self.shape = read_step_file(os.path.join(dir))
        self.view_shape = AIS_ColoredShape(self.shape) # shape as ais for displaying
        self.topologie = TopologyExplorer(self.shape)
        self.shapeProps = GProp_GProps()
        brepgprop_VolumeProperties(self.shape, self.shapeProps)

        self.handling_face = None

    def get_center_of_mass(self):
        return self.shapeProps.CentreOfMass().Coord()
    
    def get_mass(self):
        mass = self.shapeProps.Mass()
        print('Mass:',mass)

    def define_handling_face(self):
        # loop trough all faces of the part and find the ones that 
        # are ither zylindrikal or flat
        # from the found faces return the one closest to the center of mass
        shp_idx = 1
        cog = self.shapeProps.CentreOfMass()
        mindistanz = 999999999999
        for face in self.topologie.faces():


            # get face properties            
            Surface = BRep_Tool.Surface(face)
            surface = BRepAdaptor_Surface(face)
            # coordinates on face u == x, v == y
            u = 0.0 
            v = 0.0
            PointOnSurface = GeomLProp_SLProps(Surface, u, v, 1, 1e-6)
            
            location = PointOnSurface.Value()
            
            # zwei richtungen (wie einheits vektoren) in die dann die 
            # krümung in u und v richtung der Fläche angegeben wird
            dir1 = gp_Dir(1,0,0)
            dir2 = gp_Dir(1,0,0)
            PointOnSurface.CurvatureDirections(dir1,dir2)
            x1 = dir1.X()
            y1 = dir1.Y()
            z1 = dir1.Z()

            if x1 < 1e-6:
                x1 = 0
            if y1 < 1e-6:
                y1 = 0
            if z1 < 1e-6:
                z1 = 0
            
            # ist x == 1 und y == 0 und z == 0 dann handelt es sich um einen Zylinder
            # Check if the surface type is cylindrical
            if surface.GetType() == GeomAbs_Cylinder:
                print('Fläche ist Zylinder')
                pos = PointOnSurface.Value()


                
                
                print('mittelpunkt:',round(pos.X(),2),',',round(pos.Y(),2),',',round(pos.Z(),2))
                # Distanz zwischen Schwerpunkt vom teil und dem punkt auf der Oberfläche ermitteln
                vector_between = gp_Vec(location,cog)
                distance = vector_between.Magnitude()
                print('Distnaz zum schwepunkt:',distance)
                
                if mindistanz > distance:
                    print('set handling face')
                    mindistanz = distance
                    self.handling_face = face
                
                #time.sleep(2)

            elif surface.GetType() == GeomAbs_Plane:
                print('Is a Plane')

            else:
                print('Other surface type')

            shp_idx += 1


        
    def display(self,display):
        #display, start_display, add_menu, add_function_to_menu = init_display()
        #add_menu("STEP import")
        #add_function_to_menu("STEP import", import_as_one_shape)
        #start_display()
        #time.sleep(1)
        display.EraseAll()
        # set face color of handlingface to green
        self.view_shape.SetColor(rgb_color(0.2, 0.2, 0.2))
        if self.handling_face != None:
            self.view_shape.SetCustomColor(self.handling_face, rgb_color(0, 1, 0))
        else:
            print('No handling face detirmand !!!')
        display.Context.Display(self.view_shape, True)
        display.FitAll()




def open_parts(event=None):
     # select part directory
    root = tk.Tk()
    root.withdraw()

    dir_path = filedialog.askdirectory()
    onlyfiles = []
    for f in listdir(dir_path):
        if isfile(join(dir_path, f)):
            onlyfiles.append(dir_path+'/'+f)

    # open parts

    parts = []
    for partdir in onlyfiles:
        newpart = Part(partdir)
        newpart.define_handling_face()
        newpart.display(display)
        time.sleep(2)
        parts.append(newpart)

    return parts

def face_algo_example(event=None):
    display.EraseAll()
    # STEP datei einlesen. Wird als shape gespeichert
    shp = read_step_file(os.path.join("C:/Users/theja/Documents/GIT/OpenOCC/Parts/Antriebswelle.STEP"))
    
    ais_shp = AIS_ColoredShape(shp)

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

    mindistanz = 999999999
    mindistanzface = False

    for face in t.faces():   
        #print('---Face '+str(shp_idx)+'---')
        # für visu
        ais_face = AIS_ColoredShape(face)
        
        # Get the surface of the face
        surface = BRep_Tool.Surface(face)

        

        # Create a surface adaptor
        surface_adaptor = BRepAdaptor_Surface(face)

        # Find the center parameter of the surface
        umin, umax, vmin, vmax = surface.Bounds()
        u_center = (umin + umax) / 2
        v_center = (vmin + vmax) / 2

        # Evaluate the surface at the center parameter to get the center coordinate
        center_coordinate = surface_adaptor.Value(u_center, v_center)
        
        # Check if the surface type is cylindrical
        if surface_adaptor.GetType() == GeomAbs_Cylinder:
            print('Is a cylinder')
            
            
            print('mittelpunkt:',round(center_coordinate.X(),2),',',round(center_coordinate.Y(),2),',',round(center_coordinate.Z(),2))
            ais_shp.SetCustomColor(face, rgb_color(1, 0, 0))
            ais_face.SetCustomColor(face, rgb_color(1, 0, 0))
            # Distanz zwischen Schwerpunkt vom teil und dem punkt auf der Oberfläche ermitteln
            vector_between = gp_Vec(center_coordinate,cog)
            distance = vector_between.Magnitude()
            print('Distnaz zum schwepunkt:',distance)
            if mindistanz > distance:
                mindistanz = distance
                mindistanzface = face

            # Create a sphere at the specified coordinates
            sphere_builder = BRepPrimAPI_MakeSphere(center_coordinate, 2.0)
            sphere_shape = sphere_builder.Shape()
            ais_sphere = AIS_ColoredShape(sphere_shape)
            display.Context.Display(ais_sphere, True)

        # Check if the surface type is plane
        elif surface_adaptor.GetType() == GeomAbs_Plane:
            print('Is a Plane')
            ais_shp.SetCustomColor(face, rgb_color(0, 0, 1))
            ais_face.SetCustomColor(face, rgb_color(0, 0, 1))

        else:
            ais_shp.SetCustomColor(face, rgb_color(0.135, 0.135, 0.135))
            ais_face.SetCustomColor(face, rgb_color(0.135, 0.135, 0.135))

        display.Context.Display(ais_face, True)
        display.FitAll()
        #display.DisplayShape(ais_face, update=True)
        time.sleep(0.2)

        
        shp_idx += 1
        #time.sleep(2)
    
    # Fläche mit kleinster distanz grün makieren
    ais_shp.SetCustomColor(mindistanzface, rgb_color(0, 1, 0))


    display.EraseAll()
    display.Context.Display(ais_shp, True)
    display.FitAll()


if __name__ == "__main__":
    print('Start')
    display, start_display, add_menu, add_function_to_menu = init_display()
    add_menu("STEP import")
    add_function_to_menu("STEP import", open_parts)
    add_function_to_menu("STEP import", face_algo_example)
    start_display()
    print('END')
    
   

