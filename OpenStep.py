from __future__ import print_function
import os
import os.path
import time
from os import listdir
from os.path import isfile, join
# Trigonometrische Funktionen importieren
from math import radians , degrees, sin, asin, cos, acos, tan, atan, pow, sqrt, pi

from OCC.Extend.TopologyUtils import TopologyExplorer, TopoDS_Face
from OCC.Extend.DataExchange import read_step_file

from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Dir, gp_Ax1, gp_Ax2, gp_OX2d


# Topologie Pakete -----------------------------------------------------------------------------------
# Ein Modell besteht in aller regel aus mehreen primitven formen. Eine Welle z.B. aus verschiedenen 
# Zylindern und Kegeln und auch Endflächen. Die Topologie beschreibt dann wie diese Primitven Formen 
# zusammengeschnürt werden müssen also wie diese Formen relativ zueinander stehen müssen
# um das gesamte Modell darzustellen.
from OCC.Core.TopoDS import topods, TopoDS_Face, TopoDS_Edge, TopoDS_Shape, TopoDS_Wire, TopoDS_Edge # DS == Data Struktur
from OCC.Core import TopLoc # paket bietet Ressourcen für den Umgang mit lokalen 3D-Koordinatensystemen
# topologie pakete um die topologien zu bearbeiten 
import OCC.Core.TopTools
from OCC.Core.TopExp import TopExp_Explorer # Um Topologie eines gegenstand zu untersuchen
from OCC.Core import BRepTools, TopAbs
from OCC.Extend.TopologyUtils import WireExplorer


# Bearbeitungs Pakete ---------------------------------------------------------------------------------
from OCC.Core.BRep import BRep_Tool
#from OCC.Core.BRepGProp import brepgprop_SurfaceProperties, brepgprop_VolumeProperties, BRepGProp_Face, BRepGProp_Vinert

from OCC.Core.GeomLProp import GeomLProp_SLProps # Um die krümmung einer Fläche zu bestimmen
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Circle

from OCC.Core.Geom import Geom_Circle, Geom_BezierCurve, Geom_BSplineCurve, Geom_Plane
from OCC.Core.GeomAPI import GeomAPI_PointsToBSpline, geomapi
from OCC.Core.GeomConvert import GeomConvert_CompCurveToBSplineCurve

from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere, BRepPrimAPI_MakePrism

#from OCC.Core.Geom2d import Geom2d_TrimmedCurve
from OCC.Core.GCE2d import GCE2d_MakeCircle, GCE2d_MakeArcOfCircle, GCE2d_MakeLine
import OCC.Core.Geom2dConvert as Converter2d
from OCC.Core.Convert import Convert_TgtThetaOver2,  Convert_CircleToBSplineCurve



from OCC.Core.TColgp import TColgp_Array1OfPnt

from OCC.Core.gp import gp_Pnt2d, gp_Ax2d, gp_Dir2d, gp_Ax3, gp_Trsf, gp_Ax2, gp_Ax1, gp_Pln, gp_Circ

# Visu Pakete --------------------------------------------------------------------------------------------
from OCC.Display.SimpleGui import init_display
from OCC.Core.AIS import AIS_ColoredShape
from OCC.Display.OCCViewer import rgb_color

import tkinter as tk
from tkinter import filedialog

from OCC.Extend.ShapeFactory import make_vertex, make_edge2d


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

class Gripper():
    def __init__(self):
        pass

    def create(self,min_radius,max_radius,zero_gap,flansch,width):
        
        WidthTip = width # 10mm Breite an dem bogen der dünnsten stelle des Fingers
        WidthFlansch = flansch # 15mm Höhe des Flansch

        # 1.    Punkte berechnen -------------------------------------------------------
        #       
        #       Im ersten Schritt werden alle Punkte der Greiferkontur berechnet
        
        
        Punkte = [] # Liste in der alle Punkte gespeichert werden

        # P1 Tangenten Punkt 1 berechnen
        Alpha = degrees(acos(min_radius/max_radius))
        Beta = 90 - Alpha
        Y1 = sin(radians(Alpha)) * min_radius
        X1 = cos(radians(Alpha)) * -min_radius 
        P1 = gp_Pnt2d(*(X1,Y1))

        Punkte.append(P1)

        print('Punkt 1: X:',X1,' Y:',Y1)

        # P2  Mittelpunkt bestimmen
        X2 = 0
        Y2 = min_radius
        P2 = gp_Pnt2d(*(X2,Y2))

        Punkte.append(P2)

        # P3 Tangenten Punkt 2 berechnen (Liegen symetrisch gegenüber Punkt 1)
        Y3 = Y1
        X3 = X1*-1
        
        P3 = gp_Pnt2d(*(X3,Y3))

        Punkte.append(P3)

        # P4 Endpunkt Wange links
        Y4 = zero_gap
        X4 = -(max_radius - (Y4 / tan(radians(Beta))))

        P4 = gp_Pnt2d(X4,Y4)
        Punkte.append(P4)

        # P5 Spitze Wange rechts

        X5 = min_radius
        Y5 = (max_radius - min_radius) * tan(radians(Beta))

        P5 = gp_Pnt2d(X5,Y5)
        Punkte.append(P5)

        # P6 Flansch unten

        X6 = -max_radius
        Y6 = zero_gap

        P6 = gp_Pnt2d(X6,Y6)
        Punkte.append(P6)

        # P7 Flansch oben

        # Hier müsste die höhe des Flansch anhand der Kräfte und des Materials berechnet werden
        # wir gehen der einfach heit von 20mm aus
        
        Y7 = zero_gap + WidthFlansch
        X7 = - max_radius

        P7 = gp_Pnt2d(X7,Y7)
        Punkte.append(P7)

        # P8 Linkes Seite des oberen Bogens
        # Auch hier müsste man die anhand der Kräfte die Stärke berechnen wir setzen einfach 10mm ein
        r2 = min_radius + WidthTip # r2 ist der Radius des Ausenbogens

        # Betrag des vektors zu P7 berechen
        P7_ = sqrt(pow(X7,2)+pow(Y7,2))
        gamma = degrees(acos(r2/P7_))
        omega = degrees(atan(Y7/-X7))

        theta = -(90 - (gamma + omega))
        print('------------------------------------------')
        print('gamme:',gamma)
        print('omega',omega)
        print('theta:',theta)
        print('r2:',r2)
        print('Alpha:',Alpha)
        print('Beta:', Beta)
        print('------------------------------------------')


        
        X8 = sin(radians(theta)) * r2
        Y8 = cos(radians(theta)) * r2

        P8 = gp_Pnt2d(X8,Y8)
        Punkte.append(P8)

        # P9 Mitte ausenbogen
        
        # P9 muss als dirtter Punkt zwischen P8 und P10 auf dem Bogen liegen.
        # Wenn P8 eine positiven X wert hat dann darf P9 natrürlich nicht mit X = 0 dahinter stehen
        # Darum muss P9 wie P8 auf dem Bogen berechnet werden
        
        if X8 > 0:
            MiddleAngle = (Beta - theta)/2
            print('MiddleAngle:',MiddleAngle)
            X9 = sin(radians(MiddleAngle)) * r2
            Y9 = cos(radians(MiddleAngle)) * r2


        else:
            Y9 = r2
            X9 = 0
        P9 = gp_Pnt2d(X9,Y9)
        Punkte.append(P9)

        # P10 Bogen ausen rechter Punkt

        X10 = sin(radians(Beta)) * r2
        Y10 = cos(radians(Beta)) * r2

        P10 = gp_Pnt2d(X10,Y10)
        Punkte.append(P10)

        # P11 Spitze oben
        X11 = min_radius
        Y11 = Y5 + (WidthTip / cos(radians(Beta)))

        P11 = gp_Pnt2d(X11,Y11)
        Punkte.append(P11)


        # 2.  Edges erstellen -----------------------------------------------------------
        #
        # Im zweiten Schritt werden Linien aus den Punkten erzeugt. Die meisten Linien sind
        # Geraden womit immer zwischen zwei punkten eine Linie gezogen wird.
        # Nachtrag: Es stellt sich raus das die aktuelle art und weise wie man bögen erzeugt
        # so ist, das man zunächst einen Kreis an der gewnünschten Position mit dem gewünschten 
        # Radius erzeugt und dann ahand eines Start und Stop winkels in einem weitern Schritt
        # das gewünschte  Segment rausschneidet.
        Edges = []
        Points3D = []
        Verts = []

        # da die MakeEdge funktion eine funktion für dreidimensionale objekte ist müssen
        # die Punkte zu Vertecis erweitert werden also dreidimensionale punkte.
        # Die Z koordinate ist hier einfach 0
        for p in Punkte:
            Points3D.append(gp_Pnt(p.X(), p.Y(), 0))
            Verts.append(make_vertex(gp_Pnt(p.X(), p.Y(), 0)))


        # Innen Bogen erzeugen
            
        circle = gp_Circ(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), min_radius)
        Edges.append(BRepBuilderAPI_MakeEdge(circle, -radians(Beta)+pi/2, radians(Beta)+pi/2))
        
        # Wange links erzeugen
        Edges.append(BRepBuilderAPI_MakeEdge(Verts[3],Verts[0]))

        # Wange rechts
        Edges.append(BRepBuilderAPI_MakeEdge(Verts[2],Verts[4]))
        
        # Strecke zu Flansch 
        Edges.append(BRepBuilderAPI_MakeEdge(Verts[5],Verts[3]))

        # Flansch
        Edges.append(BRepBuilderAPI_MakeEdge(Verts[6],Verts[5]))

        # Rücken flansch zu Bogen
        Edges.append(BRepBuilderAPI_MakeEdge(Verts[7],Verts[6]))

        # Bogen ausen

        circle = gp_Circ(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), r2)
        Edges.append(BRepBuilderAPI_MakeEdge(circle, -radians(Beta)+pi/2, -radians(theta)+pi/2))
        
        # Ende Ausenbogen zu Spitze
        Edges.append(BRepBuilderAPI_MakeEdge(Verts[10],Verts[9]))

        # Front Spitze
        Edges.append(BRepBuilderAPI_MakeEdge(Verts[4],Verts[10]))
        


        # 3. Wire aus Edges erzeugen -----------------------------------------------------
        # Jetzt werden die einzelnen Linien zu einem sogenannten Wire verbunden.
        # Der Wire bildet dann unsere Kontur des Greifers.

        
        WireBuilder = BRepBuilderAPI_MakeWire()
        
        for edge in Edges:
            WireBuilder.Add(edge.Edge())
        
        Wire = WireBuilder.Wire()

        print('Wire closed:',Wire.Closed())
         
        # 4. Face aus Wire erzeugen ------------------------------------------------------
        # Aus dem Wire also der Kontur kann dann eine Fläche erzeugt werden, die die Kontur ausfüllt.
        
        FaceBuilder = BRepBuilderAPI_MakeFace(Wire)
        Face = FaceBuilder.Face()
        
        
        # 5. Poligon aus Face durch extrusion erzeugen -----------------------------------
        # Als letztes wird wie man es aus jedem anderen CAD kennt. die Fläche extrudiert um einen
        # Körper daraus zu Bilden.

        PrismBuilder = BRepPrimAPI_MakePrism(Face, gp_Vec(gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(0.0, 0.0, 15.0)))
        Prism = PrismBuilder.Shape()
        AISPrism = AIS_ColoredShape(Prism)
        AISPrism.SetColor(rgb_color(0.2, 0.2, 0.2))
  
        # Display -----------------------------------------------------------------------
        # Hier geben wir das ganze einfach nur auf dem 3D view port aus das wir das ding sehen können

        display, start_display, add_menu, add_function_to_menu = init_display()

        # Punkte anzeigen
        print('----------- Punkte -------------')
        for i, p in enumerate(Punkte):
            
            print('Punkt',str(i+1),'X:',p.X(),' Y:',p.Y())
            display.DisplayShape(p, update=True)
            display.DisplayMessage(p, "P"+str(i+1))

        print('--------------------------------')

        # display edges
        
        for edge in Edges:
            display.DisplayShape(edge.Edge(), update=True)
        
       
        # display prism
        display.Context.Display(AISPrism, True)
        
        # Viewport so ausrichten das alles zu sehen ist
        display.FitAll()
        start_display()


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
    """
    display, start_display, add_menu, add_function_to_menu = init_display()
    add_menu("STEP import")
    add_function_to_menu("STEP import", open_parts)
    add_function_to_menu("STEP import", face_algo_example)
    start_display()
    """
    
    MyGripper = Gripper()
    # Create the arc
    MyGripper.create(min_radius=35,max_radius=50,zero_gap=3,flansch=25,width=10)

    print('END')
    
   

