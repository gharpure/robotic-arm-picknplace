# Inverse kinematic for all 6 joints
import numpy as np
def ur5Inverse(p60, R60):
    A = [0, -0.425, -0.3922, 0, 0, 0]
    D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
    
    T60 = np.array([[R60[0][0], R60[0][1], R60[0][2], p60[0]],
            [R60[1][0], R60[1][1], R60[1][2], p60[1]],
            [R60[2][0], R60[2][1], R60[2][2], p60[2]],
            [0, 0, 0, 1]])

    T10f = lambda th1: np.array([[np.cos(th1), -np.sin(th1), 0, 0],
                        [np.sin(th1), np.cos(th1), 0, 0],
                        [0, 0, 1, D[0]],
                        [0, 0, 0, 1]])
    
    T21f = lambda th2: np.array([[np.cos(th2), -np.sin(th2), 0, 0],
                        [0, 0, -1, 0],
                        [np.sin(th2), np.cos(th2), 0, 0],
                        [0, 0, 0, 1]])

    T32f = lambda th3: np.array([[np.cos(th3), -np.sin(th3), 0, A[1]],
                        [np.sin(th3), np.cos(th3), 0, 0],
                        [0, 0, 1, D[2]],
                        [0, 0, 0, 1]])

    T43f = lambda th4: np.array([[np.cos(th4), -np.sin(th4), 0, A[2]],
                        [np.sin(th4), np.cos(th4), 0, 0],
                        [0, 0, 1, D[3]],
                        [0, 0, 0, 1]])

    T54f = lambda th5: np.array([[np.cos(th5), -np.sin(th5), 0, 0],
                        [0, 0, -1, -D[4]],
                        [np.sin(th5), np.cos(th5), 0, 0],
                        [0, 0, 0, 1]])

    T65f = lambda th6: np.array([[np.cos(th6), -np.sin(th6), 0, 0],
                        [0, 0, 1, D[5]],
                        [-np.sin(th6), -np.cos(th6), 0, 0],
                        [0, 0, 0, 1]])
    
    # Finding Th1
    x = [0, 0, -D[5], 1]
    x = np.transpose(x)
    p50 = T60*x
    p50 = np.dot(T60, x)

    th1_1= np.real(np.arctan2(p50[1], p50[0]) + np.arccos(D[3] / np.hypot(p50[1], p50[0]))) + (np.pi/2)
    th1_2= np.real(np.arctan2(p50[1], p50[0]) - np.arccos(D[3] / np.hypot(p50[1], p50[0]))) + (np.pi/2)

    # Finding Th5
    th5_1= +np.real(np.arccos((p60[0]*np.sin(th1_1) - p60[1]*np.cos(th1_1)-D[3]) / D[5]))
    th5_2= -np.real(np.arccos((p60[0]*np.sin(th1_1) - p60[1]*np.cos(th1_1)-D[3]) / D[5]))
    th5_3= +np.real(np.arccos((p60[0]*np.sin(th1_2) - p60[1]*np.cos(th1_2)-D[3]) / D[5]))
    th5_4= -np.real(np.arccos((p60[0]*np.sin(th1_2) - p60[1]*np.cos(th1_2)-D[3]) / D[5]))

    T06 = np.linalg.inv(T60)
    Xhat = T06[0:3,0]
    Yhat = T06[0:3,1]

    th6_1 = np.real(np.arctan2(((-Xhat[1]*np.sin(th1_1)+Yhat[1]*np.cos(th1_1)))/np.sin(th5_1), ((Xhat[0]*np.sin(th1_1)-Yhat[0]*np.cos(th1_1)))/np.sin(th5_1)))
    th6_2 = np.real(np.arctan2(((-Xhat[1]*np.sin(th1_1)+Yhat[1]*np.cos(th1_1)))/np.sin(th5_2), ((Xhat[0]*np.sin(th1_1)-Yhat[0]*np.cos(th1_1)))/np.sin(th5_2)))
    th6_3 = np.real(np.arctan2(((-Xhat[1]*np.sin(th1_2)+Yhat[1]*np.cos(th1_2)))/np.sin(th5_3), ((Xhat[0]*np.sin(th1_2)-Yhat[0]*np.cos(th1_2)))/np.sin(th5_3)))
    th6_4 = np.real(np.arctan2(((-Xhat[1]*np.sin(th1_2)+Yhat[1]*np.cos(th1_2)))/np.sin(th5_4), ((Xhat[0]*np.sin(th1_2)-Yhat[0]*np.cos(th1_2)))/np.sin(th5_4)))
    
    T41m = np.dot(np.dot(np.linalg.inv(T10f(th1_1)),T60),np.dot(np.linalg.inv(T65f(th6_1)),np.linalg.inv(T54f(th5_1))))
    p41_1 = T41m[0:3, 3]
    p41xz_1 = np.hypot(p41_1[0], p41_1[2])

    T41m = np.dot(np.dot(np.linalg.inv(T10f(th1_1)),T60),np.dot(np.linalg.inv(T65f(th6_2)),np.linalg.inv(T54f(th5_2))))
    p41_2 = T41m[0:3, 3]
    p41xz_2 = np.hypot(p41_2[0], p41_2[2])

    T41m = np.dot(np.dot(np.linalg.inv(T10f(th1_2)),T60),np.dot(np.linalg.inv(T65f(th6_3)),np.linalg.inv(T54f(th5_3))))
    p41_3 = T41m[0:3, 3]
    p41xz_3 = np.hypot(p41_3[0], p41_3[2])

    T41m = np.dot(np.dot(np.linalg.inv(T10f(th1_2)),T60),np.dot(np.linalg.inv(T65f(th6_4)),np.linalg.inv(T54f(th5_4))))
    p41_4 = T41m[0:3, 3]
    p41xz_4 = np.hypot(p41_4[0], p41_4[2])
    
    th3_1 = np.real(np.emath.arccos((p41xz_1**2-A[1]**2-A[2]**2)/(2*A[1]*A[2])))
    th3_2 = np.real(np.emath.arccos((p41xz_2**2-A[1]**2-A[2]**2)/(2*A[1]*A[2])))
    th3_3 = np.real(np.emath.arccos((p41xz_3**2-A[1]**2-A[2]**2)/(2*A[1]*A[2])))
    th3_4 = np.real(np.emath.arccos((p41xz_4**2-A[1]**2-A[2]**2)/(2*A[1]*A[2])))

    th3_5 = -th3_1
    th3_6 = -th3_2
    th3_7 = -th3_3
    th3_8 = -th3_4

    th2_1 = np.real(np.arctan2(-p41_1[2], -p41_1[0])-np.arcsin((-A[2]*np.sin(th3_1))/p41xz_1))
    th2_2 = np.real(np.arctan2(-p41_2[2], -p41_2[0])-np.arcsin((-A[2]*np.sin(th3_2))/p41xz_2))
    th2_3 = np.real(np.arctan2(-p41_3[2], -p41_3[0])-np.arcsin((-A[2]*np.sin(th3_3))/p41xz_3))
    th2_4 = np.real(np.arctan2(-p41_4[2], -p41_4[0])-np.arcsin((-A[2]*np.sin(th3_4))/p41xz_4))

    th2_5 = np.real(np.arctan2(-p41_1[2], -p41_1[0])-np.arcsin((A[2]*np.sin(th3_1))/p41xz_1))
    th2_6 = np.real(np.arctan2(-p41_2[2], -p41_2[0])-np.arcsin((A[2]*np.sin(th3_2))/p41xz_2))
    th2_7 = np.real(np.arctan2(-p41_3[2], -p41_3[0])-np.arcsin((A[2]*np.sin(th3_3))/p41xz_3))
    th2_8 = np.real(np.arctan2(-p41_4[2], -p41_4[0])-np.arcsin((A[2]*np.sin(th3_4))/p41xz_4))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_1)),np.linalg.inv(T21f(th2_1))),np.dot(np.linalg.inv(T10f(th1_1)),T60)),np.dot(np.linalg.inv(T65f(th6_1)),np.linalg.inv(T54f(th5_1))))
    Xhat43 = T43m[0:3,0]
    th4_1 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_2)),np.linalg.inv(T21f(th2_2))),np.dot(np.linalg.inv(T10f(th1_1)),T60)),np.dot(np.linalg.inv(T65f(th6_2)),np.linalg.inv(T54f(th5_2))))
    Xhat43 = T43m[0:3,0]
    th4_2 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_3)),np.linalg.inv(T21f(th2_3))),np.dot(np.linalg.inv(T10f(th1_2)),T60)),np.dot(np.linalg.inv(T65f(th6_3)),np.linalg.inv(T54f(th5_3))))
    Xhat43 = T43m[0:3,0]
    th4_3 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_4)),np.linalg.inv(T21f(th2_4))),np.dot(np.linalg.inv(T10f(th1_2)),T60)),np.dot(np.linalg.inv(T65f(th6_4)),np.linalg.inv(T54f(th5_4))))
    Xhat43 = T43m[0:3,0]
    th4_4 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_5)),np.linalg.inv(T21f(th2_5))),np.dot(np.linalg.inv(T10f(th1_1)),T60)),np.dot(np.linalg.inv(T65f(th6_1)),np.linalg.inv(T54f(th5_1))))
    Xhat43 = T43m[0:3,0]
    th4_5 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_6)),np.linalg.inv(T21f(th2_6))),np.dot(np.linalg.inv(T10f(th1_1)),T60)),np.dot(np.linalg.inv(T65f(th6_2)),np.linalg.inv(T54f(th5_2))))
    Xhat43 = T43m[0:3,0]
    th4_6 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_7)),np.linalg.inv(T21f(th2_7))),np.dot(np.linalg.inv(T10f(th1_2)),T60)),np.dot(np.linalg.inv(T65f(th6_3)),np.linalg.inv(T54f(th5_3))))
    Xhat43 = T43m[0:3,0]
    th4_7 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_8)),np.linalg.inv(T21f(th2_8))),np.dot(np.linalg.inv(T10f(th1_2)),T60)),np.dot(np.linalg.inv(T65f(th6_4)),np.linalg.inv(T54f(th5_4))))
    Xhat43 = T43m[0:3,0]
    th4_8 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))
    
    
    Th = [[th1_1, th2_1, th3_1, th4_1, th5_1, th6_1],
          [th1_1, th2_2, th3_2, th4_2, th5_2, th6_2],
          [th1_2, th2_3, th3_3, th4_3, th5_3, th6_3],
          [th1_2, th2_4, th3_4, th4_4, th5_4, th6_4],
          [th1_1, th2_5, th3_5, th4_5, th5_1, th6_1],
          [th1_1, th2_6, th3_6, th4_6, th5_2, th6_2],
          [th1_2, th2_7, th3_7, th4_7, th5_3, th6_3],
          [th1_2, th2_8, th3_8, th4_8, th5_4, th6_4]]

    return Th
