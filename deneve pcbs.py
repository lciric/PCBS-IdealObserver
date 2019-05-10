# -*- coding: utf-8 -*-
"""
Created on Mon Mar  4 00:25:11 2019

@author: Marco Huarancca
"""

import  numpy as np
import random
import math
import cmath
import matplotlib.pyplot as plt

n_trials = 1000
n_iterations = 3
print('n_iterations = ', n_iterations)
K = 74.0
nu = 3.7
sigma_th = 0.38
sigma_lamb = 0.38
P_th = 20
P_lamb = 20
C= 0.5
mu = 0.002
S = 1
delta_wth = 0.38
theta_grid = [2*math.pi*i/P_th for i in range(1,P_th+1)]
lambda_grid = [2*math.pi*i/P_lamb for i in range(1,P_lamb+1)]


#Cette fonction retourne la matrice des fij(theta_p, lambda_p) avec theta_p et lambda_p en radians 

def input_tuning_curve(theta_p, lambda_p) : 
    F=np.zeros((P_th,P_lamb))
    C1 = list(map(lambda x: (math.cos(theta_p - x) -1)/sigma_th**2, theta_grid))
    C2 = list(map(lambda x: (math.cos(lambda_p - x) -1)/sigma_lamb**2, lambda_grid))
    for i in range(P_th):
        for j in range(P_lamb):    
            F[i][j] = K*C*math.exp(C1[i] + C2[j]) + nu     
    return F


#Cette fonction retourne la matrice d'activité totale en entrée du réseau

def input_activity(ITC) :
    A = np.zeros((P_th,P_lamb))
    for i in range(P_th):
        for j in range(P_lamb):
            variance = ITC[i][j] #25.0
            bruit = random.gauss(0,variance)
            A[i][j] = ITC[i][j] + bruit
    return A

K_w = 1.0

#Cette fonction retourne la matrice des poids synaptiques

def filtering_weights() :
    W=np.zeros((P_th,P_lamb))
    for m in range(P_th):
        for n in range(P_lamb): 
            W[m][n] = K_w * math.exp((math.cos(2*math.pi*m/P_th) - 1)/(delta_wth**2) + (math.cos(2*math.pi*n/P_lamb)- 1)/(delta_wth**2))
    return W

           
#Cette fonction retourne l'activité du réseau au bout de n_iterations mises à jour  

def output(input_activity, W, n_iterations) :
#Condition initiale 
    O = input_activity.copy()
    U = np.zeros((P_th,P_lamb))
#Iterations 
    for t in range(1, n_iterations +1):
        sum_u = 0.0
        for i in range(P_th) :
            for j in range(P_lamb) :   
                for k in range(P_th) :
                    for l in range(P_lamb) :
                        U[i][j] += O[k][l]*W[i-k][j-l]
                sum_u += U[i][j]**2
        O[:,:] = (U[:,:]**2)/(S + mu * sum_u )
    return O

#Cette fonction retourne l'estimation de l'orientation du stimulus sur un essai
#calculée à partir de l'activité du réseau 

def theta_estimator(output) :
    Z = 0.0 + 0.0*1j
    C = list(map(lambda x: math.cos(x), theta_grid))
    S = list(map(lambda x: math.sin(x), theta_grid))
    C = np.array(C)
    S = np.array(S)
    for k in range(P_th):
        for j in range(P_lamb):
            Z += output[k][j]*C[j] + output[k][j]*S[j]*1j
    phase = cmath.phase(Z)
    if phase < 0.0:
        phase = phase + 2*math.pi # pour que l'angle calculé soit dans l'intervalle [0;2π] comme theta_p  
    return phase

#Cette fonction retourne l'estimation de la fréquence spatiale du stimulus sur
# un essai calculée à partir de l'activité du réseau

def lambda_estimator(output) :
    Z = 0.0 + 0.0*1j
    C = list(map(lambda x: math.cos(x), lambda_grid))
    S = list(map(lambda x: math.sin(x), lambda_grid))
    C = np.array(C)
    S = np.array(S)
    for k in range(P_th):
        for j in range(P_lamb):
            Z += output[k][j]*C[j] + output[k][j]*S[j]*1j
    phase = cmath.phase(Z)
    if phase < 0.0:
        phase = phase + 2*math.pi # pour que la fréquence calculée soit dans l'intervalle [0;2π] comme lambda_p      
    return phase


#Cette fonction calcule la fréquence spatiale moyenne estimée par le réseau 
#et la variance de la fréquence spatiale estimée sur n_trials essais différents
    
def stats_net(theta_p, lambda_p, ITC, W, n_iter):
    m = 0.0
    v = 0.0
    for i in range(n_trials):
        #Initialisation de la matrice d'activité totale en entrée du réseau    
        A = input_activity(ITC)  
        #Calcul de l'activité en sortie du réseau au bout de n_iter mises à jour
        O = output(A, W, n_iter)
        #Calcul de l'estimation de l'orientation du stimulus par le réseau
        lambda_estimated = lambda_estimator(O)
        m += lambda_estimated
        v += (lambda_estimated - theta_p)**2    
    m = m/float(n_trials)
    v = v/(float(n_trials)-1)
    return m, v    


###############################################################################################

theta_p = 4*math.pi/3 #Orientation du stimulus
lambda_p = 4*math.pi/3 #Fréquence spatiale du stimulus

#Calcul de la matrice des poids de filtrage 
W = filtering_weights()

#Calcul de la matrice d'activité moyenne en entrée du réseau     
F = input_tuning_curve(theta_p, lambda_p)

###############################################################################################

########### Essai unique avec θ = 4π/3, λ = 4π/3 #####################################

###############################################################################################
#Initialisation de la matrice d'activité totale en entrée du réseau    
A = input_activity(F)

######## Visualisation de l'activité du réseau imédiatement après initialisation ##############
initial_activity = output(A, W, 0)

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
#On convertit la matrice de l'activité initiale du réseau en array à deux dimensions
Z0=[]
for i in range(P_th) :
    for j in range(P_lamb) :                
        Z0.append(initial_activity[i][j])
Z0 = np.array(Z0)
#On convertit la liste lambda_grid en array 
Y = []
for i in range(P_th):
    for j in range(P_lamb):
        Y.append(lambda_grid[j])
Y = np.array(Y)  
#On convertit la liste theta_grid en array     
X = []
for i in range(P_th):
    for j in range(P_lamb):
        X.append(180*theta_grid[i]/math.pi) #on convertit les orientations préférentielles theta_grid[i] en degrés
X = np.array(X)
    
xs = X.reshape((-1, P_th))
ys = Y.reshape((-1, P_th))
zs = Z0.reshape((-1, P_th))  
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')  
ax.plot_surface(xs,ys,zs, rstride=1, cstride=1)
plt.xlabel('θ')
plt.ylabel('λ')
ax.set_zlabel('Activity', fontsize=10)
plt.show()

######## Visualisation de l'activité du réseau mise à jour après 3 iterations ################

#Calcul de l'activité en sortie du réseau au bout de n_iterations = 3 mises à jour
output_activity = output(A, W, n_iterations)

#On convertit la matrice de l'activité en sortie du réseau en array à deux dimensions
Z=[]
for i in range(P_th) :
    for j in range(P_lamb) :                
        Z.append(output_activity[i][j])
Z = np.array(Z)

zs = Z.reshape((-1, P_th))  
fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')  
ax.plot_surface(xs,ys,zs, rstride=1, cstride=1)
plt.xlabel('θ')
plt.ylabel('λ')
ax.set_zlabel('Activity', fontsize=10)
plt.show()


######################################################################################################

#### Calcul de la moyenne et de la variance de l'estimation de la fréquence spatiale du stimulus  ####

######################################################################################################

print('Calcul de la moyenne et de la variance de l estimation de la frequence spatiale du stimulus en cours...')
mean_lambda_est, variance_lambda = stats_net(theta_p, lambda_p, F, W, n_iterations)
print('Fréquence spatiale présentée égale à', lambda_p, 'radians' )
print ('Fréquence spatiale moyenne estimée par le réseau égale à ', mean_lambda_est, 'radians')
print('Variance de la fréquence spatiale estimée par le réseau égale à ', variance_lambda)
