
# PythonOCC 

Installation------------------------------

Basiert auf dieser Anleitung: http://analysissitus.org/forum/index.php?threads/pythonocc-getting-started-guide.19/

1. Anaconda installieren

  https://www.anaconda.com/download

2. Conda pakete installieren die OCC und die dazugehörige python lib installieren.
    Dafür die Conda promt öffnen. Ist ein Komandozeilen programm das mit conda installiert wird und
   kann wie jedes ander Programm in Windows gesucht werden.

    <code>conda create -n pyoccenv python=3.8</code>
   
    <code>conda activate pyoccenv</code>
   
    <code>conda install -c conda-forge pythonocc-core</code>

Starten ------------------------------------

Der einfachste und schnellste weg das Programm auszuführen ist das ganze mit
der conda prompt zu machen. Dafür in windows unter dem windows symbol nach conda prompt suche.
In der Komandozeile dann mit "cd C:\xx\xx" in das verzeichnis wechseln wo sich das Programm befindet.
Dort muss dann noch in die Virtual enviroment gewechselt werden die wir in der Installation
angelegt haben und in der wir occ installiert haben.
dafür  "conda activate pyoccenv " in der Komandozeile ausführen.
Dann kann ganznormal mit "python xxxx.py" das programm gestartet werden

Wenn nicht schon aktiv erst die python umgebung in der wir occ installiert haben aktivieren
<code>conda activate pyoccenv</code>

In dem Order wo dieses repo runtergeladen wurde dann die entsprechende python datei ausführen
aktuell ist das die OpenStep.py

<code>python OpenStep.py</code>

Dokumentation-------------------------------

Eine Dokumentation zu den verfügbaren python bibliotheken ist im repo unter "PythonOCC_Doc/0.18.1" dabei.
Das ganze ist als website gestaltet. Deshalb in dem Verzeichnis die index.html öffnen.

Die Doku für OpenCasCade ist unter : https://dev.opencascade.org/doc/overview/html/index.html
