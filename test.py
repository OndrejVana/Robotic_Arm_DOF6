from math import *
import time


time_dur = 0

def RotMat(angle, alpha, d, a):
    M = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    #------
    M[0][0] = cos(radians(angle))
    M[0][1] = -sin(radians(angle) * cos(radians(alpha)))
    M[0][2] = sin(radians(angle)) * sin(radians(alpha))
    M[0][3] = a * cos(radians(angle))
    #------
    M[1][0] = sin(radians(angle))
    M[1][1] = cos(radians(angle)) *  cos(radians(alpha))
    M[1][2] = -cos(radians(angle)) * sin(radians(alpha))
    M[1][3] = a * sin(radians(angle))
    #------
    M[2][1] = sin(radians(alpha))
    M[2][2] = cos(radians(alpha))
    M[2][3] = d
    #------
    M[3][3] = 1
    #------
    return M

def MatMul(A, B, size):
    C = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    for i in range(0, size):
        for j in range(0, size):
            for k in range(0, size):
                C[i][j] += A[i][k] * B[k][j]
    
    return C

def MatInv(X):
    L = [[X[0][0], X[1][0], X[2][0]], [X[0][1], X[1][1], X[2][1]], [X[0][2], X[1][2], X[2][2]]]
    return L


DH = [[0.01, -90, 220, 3,5],[-90, 0, 0, 500],[0.01, 90, 0, 0],[0.01, -90, 360, 0],[0.01, 90, 0, 0],[0.01, 0, 44, 0]]

def InverseKinematics(Xcor, Ycor, Zcor, y_cor, p_cor, r_cor):
    start = time.time()
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
        angle_J1 = degrees(atan(Ycor / Xcor))
        angle_k = atan((Zcor - DH[0][2]) / (Xcor - DH[0][3]))
        k = sqrt(pow((Xcor - DH[0][3]),2) +  pow((Zcor - DH[0][2]),2))
        angle_k2 = acos(((DH[1][3] * DH[1][3]) + (k * k) - (DH[3][2] * DH[3][2])) / (2 * DH[1][3] * k))
        angle_J2 = -degrees(angle_k + angle_k2)
        angle_J3 =180 - degrees(acos((DH[3][2] * DH[3][2] + DH[1][3] * DH[1][3] - k * k) / (2 * DH[3][2] * DH[1][3])))

    #ARM MID    
    elif Xcor < DH[0][3] and Xcor > 0:
        angle_J1 = atan(Ycor / Xcor)
        l = DH[0][3] - Xcor
        m = Zcor - DH[0][2]
        km = sqrt(m * m + l * l)
        angle_km1 = atan(l / m)
        #angle_km2 = 90 - angle_km1
        angle_km = acos((DH[1][3] * DH[1][3] + km * km - DH[3][2] * DH[3][2]) / (2 * DH[1][3] * km))
        angle_J2 = -90 - (angle_km1 - angle_km)
        angle_J3 = acos((DH[3][2] * DH[3][2] + DH[1][3] * DH[1][3] - k * k) / (2 * DH[3][2] * DH[1][2]))

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
    J_1 = RotMat(DH[0][0], DH[0][1], DH[0][2], DH[0][3])

    #J2 Matrix
    J_2 = RotMat(DH[1][0], DH[1][1], DH[1][2], DH[1][3])
    #J3 Matrix
    J_3 = RotMat(DH[2][0], DH[2][1], DH[2][2], DH[2][3])

    #R_0_3 Matrix
    R_0_2 = MatMul(J_1, J_2, 4)
    
    R_0_3 = MatMul(R_0_2, J_3, 4)

    #Inverse R_0_3 Matrix
    R_0_3_INV = MatInv(R_0_3)
    for i in range(0, 3):
        for j in range(0, 3):
            print(R_0_3_INV[i][j])
        print(" ")
    #ROT Matrix
    R_O_T = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    R_O_T[0][0] = -((cos(radians(y_cor)) * cos(radians(r_cor))) - (cos(radians(p_cor)) * sin(radians(y_cor)) * sin(radians(r_cor))))
    R_O_T[0][1] = cos(radians(r_cor)) * sin(radians(y_cor)) + cos(radians(y_cor)) * cos(radians(p_cor)) * sin(radians(r_cor))
    R_O_T[0][2] = sin(radians(p_cor)) * sin(radians(r_cor))
    R_O_T[0][3] = Xcor
    #------
    R_O_T[1][0] = cos(radians(p_cor)) * cos(radians(r_cor)) * sin(radians(y_cor)) + cos(radians(y_cor)) * sin(radians(r_cor))
    R_O_T[1][1] = cos(radians(y_cor)) * cos(radians(p_cor)) * cos(radians(r_cor)) - sin(radians(y_cor)) * sin(radians(r_cor))
    R_O_T[1][2] = cos(radians(r_cor)) * sin(radians(p_cor))
    R_O_T[1][3] = Ycor
    #------
    R_O_T[2][0] = sin(radians(y_cor)) * sin(radians(p_cor))
    R_O_T[2][1] = cos(radians(y_cor)) * sin(radians(p_cor))
    R_O_T[2][2] = -cos(radians(p_cor))
    R_O_T[2][3] = Zcor
    #------
    R_O_T[3][3] = 1
    
    #R_0_6r Matrix
    R_0_6R = [[-1, 0.0000001, 0, 0], [-0.00001, -1, 0, 0], [0, 0, 1, -44], [0, 0, 0, 1]]

    #R_0_5 Matrix
    R_0_5 = MatMul(R_O_T, R_0_6R, 4)
    
    #R_3_6 Matrix
    R_3_6 = MatMul(R_0_3_INV, R_0_5, 3)
    for i in range(0, 4):
        for j in range(0, 4):
            print(R_3_6[i][j])
        print(" ")
    #J4, J5 and J6 calculation 
    print(" " + str(DH[3][0]))
    if DH[3][0] > 0:
        angle_J5 = atan2(R_3_6[2][2], sqrt(1 - R_3_6[2][2]*R_3_6[2][2]))
        angle_J4 = atan2(R_3_6[0][2], R_3_6[1][2])
        angle_J6 = atan2(-R_3_6[2][0], R_3_6[2][1])
    elif DH[3][0] < 0:
        angle_J5 = atan2(R_3_6[2][2], - sqrt(1 - R_3_6[2][2]*R_3_6[2][2]))
        angle_J4 = atan2(-R_3_6[0][2], -R_3_6[1][2])
        angle_J6 = atan2(R_3_6[2][0], -R_3_6[2][1])

    #DH update
    DH[3][0] = degrees(angle_J4)
    DH[4][0] = degrees(angle_J5)
    DH[5][0] = degrees(angle_J6)
    end = time.time()
    #print(end - start)

InverseKinematics(400, 200, 300, 10, 20, 30)
for i in range(0, 6):
    print(str(DH[i][0]))