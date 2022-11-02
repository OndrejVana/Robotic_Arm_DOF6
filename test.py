import math


DH = [[0.01, -90, 220, 3,5],[-90, 0, 0, 500],[0, 90, 0, 0],[0.01, -90, -360, 0],[0.01, 90, 0, 0],[0.01, 0, -44, 0]]

def InverseKinematics(Xcor, Ycor, Zcor, p_cor, y_cor, r_cor):
    #QVADRANT
    if Xcor > 0 and Ycor > 0:
        Q = 1
    elif Xcor < 0 and Ycor > 0:
        Q = 2
    elif Xcor < 0 and Ycor < 0:
        Q = 3
    elif Xcor > 0 and Ycor < 0:
        Q = 4

    #ARM FORWARD
    if Xcor > DH[0][3]:
        angle_J1 = math.atan(Ycor / Xcor)
        angle_k = math.atan((Zcor - DH[0][2]) / (Xcor - DH[0][3]))
        k = math.sqrt(math.pow((Xcor - DH[0][3]),2) +  math.pow((Zcor - DH[0][2],2)))
        angle_k2 = math.acos((DH[1][3] * DH[1][3] + k * k - DH[3][2]) / (2 * DH[1][3] * k))
        angle_J2 = angle_k + angle_k2
        angle_J3 = math.acos((DH[3][2] * DH[3][2] + DH[1][3] * DH[1][3] - k * k) / (2 * DH[3][2] * DH[1][2]))

    #ARM MID    
    elif Xcor < DH[0][3] and Xcor > 0:
        angle_J1 = math.atan(Ycor / Xcor)
        l = DH[0][3] - Xcor
        m = Zcor - DH[0][2]
        km = math.sqrt(m * m + l * l)
        angle_km1 = math.atan(l / m)
        #angle_km2 = 90 - angle_km1
        angle_km = math.acos((DH[1][3] * DH[1][3] + km * km - DH[3][2] * DH[3][2]) / (2 * DH[1][3] * km))
        angle_J2 = -90 - (angle_km1 - angle_km)
        angle_J3 = math.acos((DH[3][2] * DH[3][2] + DH[1][3] * DH[1][3] - k * k) / (2 * DH[3][2] * DH[1][2]))

    #ARM BACK and ELBOW BACK -- mozna potom

    #correction of 2 and 3 qvadrant
    if Q == 2:
        angle_J1 = angle_J1 * (-1) + 90
    elif Q == 3:
        angle_J1 = angle_J1 * (-1) - 90

    #DH table update
    DH[0][0] = angle_J1
    DH[1][0] = angle_J2
    DH[2][0] = angle_J3

    #J1 Matrix

    #J2 Matrix

    #J3 Matrix

    #R_0_3 Matrix

    #Inverse R_0_3 Matrix

    #ROT Matrix

    #R_0_6r Matrix

    #R_0_5 Matrix

    #R_3_6 Matrix

    #J4, J5 and J6 calculation 

    #DH update


