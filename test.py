import math


DH = [[0.01, -90, 220, 3,5],[-90, 0, 0, 500],[0, 90, 0, 0],[0.01, -90, -360, 0],[0.01, 90, 0, 0],[0.01, 0, -44, 0]]

def InverseKinematics(Xcor, Ycor, Zcor, p_cor, y_cor, r_cor):
    
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

    DH[0][0] = angle_J1
    DH[1][0] = angle_J2
    DH[2][0] = angle_J3
