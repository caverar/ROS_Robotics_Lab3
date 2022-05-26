from numpy import array
import numpy as np
import math

def getInvKin(T, l):
    """
    Cinematica inversa para robot Phantom X
    Por: Maria Alejandra Arias Frontanilla
    Entradas:
    T: Matriz 4x4 con posición y orientación de la herramienta
    l: Longitud de eslabones
    Salida:
    q: Matriz 2x4 con valores de las 4 articulaciones en grados. 
        Row 1: codo abajo
        Row 2: codo arriba
        [q1d q2d q3d q4d]
        [q1u q2u q3u q4u]
    """
    try:
        q = np.double([[0, 0, 0, 0], [0, 0, 0, 0]])

        #%Cálculo de la primera articulación en radianes
        q[0,0] = math.atan2(T[1,3], T[0,3]) #Codo abajo
        q[1,0] = q[0,0] #Codo arriba

        
        #Desacople de muñeca:
        #Cálculo de la posición de la muñeca W
        Pos_w = T[0:3, 3] - l[3]*T[0:3, 2]
        #print("Pos_w: ", Pos_w)
        #Solución de mecanismo 2R para q2 y q3
        h = round(Pos_w[2] - l[0],3)
        r = round(math.sqrt(Pos_w[0]**2 + Pos_w[1]**2),3)
        #print("h: ", h)
        #print("r: ", r)
        #Codo abajo:
        
        #Tercera articulación en radianes   
        
        q[0,2] = math.acos(round((r**2+h**2-l[1]**2-l[2]**2)/(2*l[1]*l[2]), 2))
        #Segunda articulación en radianes sin offset
        q[0,1] = math.atan2(h,r) - math.atan2(l[2]*math.sin(q[0,2]), l[1]+l[2]*math.cos(q[0,2]))
        #Teniendo en cuenta offset en la segunda articulación
        q[0,1] = q[0,1] - math.pi/2

        #Codo arriba: 

        #Tercera articulación en radianes: negativo de codo abajo para q3
        q[1,2]= -q[0,2]
        #Segunda articulación en radianes sin offset
        q[1,1] = math.atan2(h,r) + math.atan2(l[2]*math.sin(q[0,2]), l[1]+l[2]*math.cos(q[0,2]))
        #Teniendo en cuenta offset en la segunda articulación
        q[1,1] = q[1,1] - math.pi/2;   

        #Obtención de valor de cuarta articulación usando vector |a| de T
        phi = math.atan2(T[2,2], math.sqrt(T[0,2]**2 +T[1,2]**2)) - math.pi/2
        q[0,3] = phi - q[0,1] -q[0,2]
        q[1,3] = phi - q[1,1] -q[1,2]

        # print("Codo abajo: ")
        # print("1: ", q[0,0])
        # print("2: ", q[0,1])
        # print("3: ", q[0,2])
        # print("4: ", q[0,3])

        # print("Codo arriba: ")
        # print("1: ", q[1,0])
        # print("2: ", q[1,1])
        # print("3: ", q[1,2])
        # print("4: ", q[1,3])
        return q
        
    except ValueError:
        print("That position can't be reached with this configuration. Try another movement or configuration")
    
