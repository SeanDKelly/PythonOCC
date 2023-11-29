from __future__ import print_function
import os
import os.path
import time


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

# Visu Pakete --------------------------------------------------------------------------------------------
from OCC.Display.SimpleGui import init_display
from OCC.Core.AIS import AIS_ColoredShape
from OCC.Display.OCCViewer import rgb_color

def import_as_one_shape(event=None):
    display.EraseAll()
    # STEP datei einlesen. Wird als shape gespeichert
    shp = read_step_file(os.path.join("Antriebswelle.STEP"))

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
        
        h_srf = BRep_Tool.Surface(face)
        u = 0.0
        v = 0.0
        curvature = GeomLProp_SLProps(h_srf, u, v, 1, 1e-6)
        
        loc = curvature.Value()
        #print('Location:',loc.X(),',',loc.Y(),',',loc.Z())
        
        # zwei richtungen (wie einheits vektoren) in die dann die 
        # krümung in u und v richtung der Fläche angegeben wird
        dir1 = gp_Dir(1,0,0)
        dir2 = gp_Dir(1,0,0)
        curvature.CurvatureDirections(dir1,dir2)
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
        if x1 == 1 and y1 == 0 and z1 == 0:
            print('Fläche ist Zylinder')
            pos = curvature.Value()
            print('mittelpunkt:',round(pos.X(),2),',',round(pos.Y(),2),',',round(pos.Z(),2))
            ais_shp.SetCustomColor(face, rgb_color(1, 0, 0))
            ais_face.SetCustomColor(face, rgb_color(1, 0, 0))
            # Distanz zwischen Schwerpunkt vom teil und dem punkt auf der Oberfläche ermitteln
            vector_between = gp_Vec(loc,cog)
            distance = vector_between.Magnitude()
            print('Distnaz zum schwepunkt:',distance)
            if mindistanz > distance:
                mindistanz = distance
                mindistanzface = face


        else:
            ais_shp.SetCustomColor(face, rgb_color(0.135, 0.135, 0.135))
            ais_face.SetCustomColor(face, rgb_color(0.135, 0.135, 0.135))

        
        
        display.Context.Display(ais_face, True)
        #display.DisplayShape(ais_face, update=True)
        time.sleep(0.1)

        
        shp_idx += 1
        #time.sleep(2)
    
    # Fläche mit kleinster distanz grün makieren
    ais_shp.SetCustomColor(mindistanzface, rgb_color(0, 1, 0))


    display.EraseAll()
    display.Context.Display(ais_shp, True)
    display.FitAll()


if __name__ == "__main__":
    display, start_display, add_menu, add_function_to_menu = init_display()
    add_menu("STEP import")
    add_function_to_menu("STEP import", import_as_one_shape)
    #add_function_to_menu("STEP import", import_as_multiple_shapes)
    start_display()
