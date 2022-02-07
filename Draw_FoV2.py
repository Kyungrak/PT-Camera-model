from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import matplotlib.pyplot as plt
import math

from numpy.core.shape_base import atleast_1d
from numpy.lib.shape_base import expand_dims

xI = np.array([6.54, 6.20, 3.18])
a = 0.036 
b = 0.027
lamda = 0.05
TIME_STEP = 0.05
x = np.linspace(12,0,240)
angle_psi = np.linspace(0,360,180)
Pt = np.array([1, 1])
error_last_x = 0
error_last_y = 0
Pt_x = []
Pt_y = []

def FindVertice(psi, phi):
        # Orthonormal matrices Hpsi and Hphi
        Hpsi = np.array([[math.cos(psi), -math.sin(psi), 0],
                        [math.sin(psi), math.cos(psi), 0],
                        [0, 0, 1]])

        Hphi = np.array([[1, 0, 0],
                        [0, math.cos(phi), -math.sin(phi)],
                        [0, math.sin(phi), math.cos(phi)]])

        # Find the position of target wrt camera frame
        qt = Hphi.transpose()@Hpsi.transpose()@(np.array([x[i],y,0])-xI)

        # projection of the target state xT onto the virtual image plane
        Pt = lamda*np.array([qt[0]/qt[2], qt[1]/qt[2]])

        # Find Virtual image plane
        q1V = np.array([[a/2], [b/2], [lamda]])
        q2V = np.array([[a/2], [-b/2], [lamda]])
        q3V = np.array([[-a/2], [-b/2], [lamda]])
        q4V = np.array([[-a/2], [b/2], [lamda]])

        # Find position relative to the inertial frame
        q1I = Hpsi @ Hphi @ q1V
        q2I = Hpsi @ Hphi @ q2V
        q3I = Hpsi @ Hphi @ q3V
        q4I = Hpsi @ Hphi @ q4V

        p14 = (-xI[2]/((b/2)*math.sin(phi)+lamda*math.cos(phi)))
        p23 = (-xI[2]/((-b/2)*math.sin(phi)+lamda*math.cos(phi)))

        q1Ip = p14*q1I
        q2Ip = p23*q2I
        q3Ip = p23*q3I
        q4Ip = p14*q4I

        # Find FOV vertices in the inertial frame
        vertice1 = xI + q1Ip.transpose()
        vertice2 = xI + q2Ip.transpose()
        vertice3 = xI + q3Ip.transpose()
        vertice4 = xI + q4Ip.transpose()

        X1 = np.array([vertice1[0][0], vertice2[0][0], vertice3[0][0], vertice4[0][0]])
        Y1 = np.array([vertice1[0][1], vertice2[0][1], vertice3[0][1], vertice4[0][1]])
        Z1 = np.array([vertice1[0][2], vertice2[0][2], vertice3[0][2], vertice4[0][2]])

        X2 = np.array([xI[0], vertice1[0][0], vertice2[0][0]])
        Y2 = np.array([xI[1], vertice1[0][1], vertice2[0][1]])
        Z2 = np.array([xI[2], vertice1[0][2], vertice2[0][2]])

        X3 = np.array([xI[0], vertice2[0][0], vertice3[0][0]])
        Y3 = np.array([xI[1], vertice2[0][1], vertice3[0][1]])
        Z3 = np.array([xI[2], vertice2[0][2], vertice3[0][2]])

        X4 = np.array([xI[0], vertice3[0][0], vertice4[0][0]])
        Y4 = np.array([xI[1], vertice3[0][1], vertice4[0][1]])
        Z4 = np.array([xI[2], vertice3[0][2], vertice4[0][2]])

        X5 = np.array([xI[0], vertice4[0][0], vertice1[0][0]])
        Y5 = np.array([xI[1], vertice4[0][1], vertice1[0][1]])
        Z5 = np.array([xI[2], vertice4[0][2], vertice1[0][2]])

        vertices1 = [list(zip(X1,Y1,Z1))]
        vertices2 = [list(zip(X2,Y2,Z2))]
        vertices3 = [list(zip(X3,Y3,Z3))]
        vertices4 = [list(zip(X4,Y4,Z4))]
        vertices5 = [list(zip(X5,Y5,Z5))]

        return vertices1, vertices2, vertices3, vertices4, vertices5, Pt

# Simulate the images
def Image(vertices1, vertices2, vertices3, vertices4, vertices5):
        fig = plt.figure()

        ax = fig.add_subplot(111, projection='3d')

        ax.set_xlim(0,12)
        ax.set_ylim(0,12)
        ax.set_zlim(0,10)

        poly1 = Poly3DCollection(vertices1, facecolors='red', alpha=0.7)
        poly2 = Poly3DCollection(vertices2, edgecolors='b', alpha=0.5)
        poly3 = Poly3DCollection(vertices3, edgecolors='b', alpha=0.5)
        poly4 = Poly3DCollection(vertices4, edgecolors='b', alpha=0.5)
        poly5 = Poly3DCollection(vertices5, edgecolors='b', alpha=0.5)

        ax.scatter(xI[0],xI[1],xI[2], marker='o', c='green') # Draw position of the camera
        ax.scatter(x[i],y,0, marker='o', c='black') # draw position of the target
        ax.add_collection3d(poly1)
        ax.add_collection3d(poly2)
        ax.add_collection3d(poly3)
        ax.add_collection3d(poly4)
        ax.add_collection3d(poly5)

        filename = "FOV_image"+str(i)+".jpg"
        plt.savefig(filename)
        plt.close(fig)        
        return

# Design a controller
def control(deltax, deltay, psi, phi, error_last_x, error_last_y):
        S = np.array([[psi], [phi], [0.05], [0.04]])
        A = np.array([[1, 0, TIME_STEP, 0], [0, 1, 0, TIME_STEP], [0, 0, 1, 0], [0, 0, 0, 1]])
        B = np.array([[0, 0], [0, 0], [1*math.pi/180, 0], [0, 1*math.pi/180]])
        error_x = deltax
        error_y = deltay
        derivative_error_x = (error_x - error_last_x) / TIME_STEP
        derivative_error_y = (error_y - error_last_y) / TIME_STEP
        output1 = 0.3 * error_x + 0.02 * derivative_error_x
        output2 = 0.3 * error_y + 0.02 * derivative_error_y
        U = np.array([[output1],[output2]])

        S_1 = A@S+B@U

        return S_1[2], S_1[3], error_x, error_y

for i in range(0,180):
        y = -x[i] + 15

        # Find intersection 
        if -a/2<Pt[0]<a/2 and -b/2<Pt[1]<b/2:
                
                Pt_x = np.append(Pt_x, Pt[0])
                Pt_y = np.append(Pt_y, Pt[1])

                # Find the desired angles
                val = control(Pt[0], Pt[1], psi, phi, error_last_x, error_last_y)
                error_last_x = val[2]
                error_last_y = val[3]
                if val[2] < 0: # Adjust pan angle
                        desired_psi = psi - val[0]*0.6
                        if val[3] < 0: # Adjust tilt angle
                                desired_phi = phi + val[1]*0.3
                        elif val[3] > 0:
                                desired_phi = phi - val[1]*0.3                               
                elif val[2] > 0: # Adjust pan angle
                        desired_psi = psi + val[0]*0.6
                        if val[3] < 0: # Adjust tilt angle
                                desired_phi = phi + val[1]*0.3
                        elif val[3] > 0:
                                desired_phi = phi - val[1]*0.3
                # Minimum boundary of the tilt angle
                if desired_phi < 120*math.pi/180:
                        desired_phi = 120*math.pi/180

                VERTICES = FindVertice(desired_psi, desired_phi)
                Image(VERTICES[0], VERTICES[1], VERTICES[2], VERTICES[3], VERTICES[4])
                psi = desired_psi
                phi = desired_phi
                Pt = VERTICES[5]
                continue

        psi = (angle_psi[i]*math.pi)/180
        phi = (145*math.pi)/180
        VERTICES = FindVertice(psi, phi)
        Image(VERTICES[0], VERTICES[1], VERTICES[2], VERTICES[3], VERTICES[4])

        Pt = VERTICES[5]
        
TIME = np.linspace(0,8,len(Pt_x))

plt.figure(1)
plt.plot(TIME, Pt_x)
plt.axis([0, 8, -0.1, 0.1])
plt.xlabel('time(t)')
plt.ylabel('error 1')
plt.grid()
plt.show()

plt.figure(2)
plt.plot(TIME, Pt_y)
plt.axis([0, 8, -0.1, 0.1])
plt.xlabel('time(t)')
plt.ylabel('error 2')
plt.grid()
plt.show()