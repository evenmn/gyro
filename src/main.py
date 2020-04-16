""" Gyroscope

@author: Even Marius Nordhagen
"""

import numpy as np
import matplotlib.pyplot as plt

# declare system quantities
m = 0.5   # mass rotor, kg
r0 = 0.3   # radius rotor, m
M = 0     # mass rack, kg (no mass)
R0 = 1     # radius rack, m
g = -9.81 # gravitational acceleration

# declare time quantities
dt = 0.001  # s
T = 50       # s
N = int(T/dt)

# variables
i = 0.5*m*r0**2
I = m*R0**3
F = np.array([0, 0, m*g])

# declare rotor arrays
θ = np.zeros((N+1, 3))     # angle
ω = np.zeros((N+1, 3))     # angular velocity
α = np.zeros((N+1, 3))     # angular acceleration
τ = np.zeros((N+1, 3))     # torque
λ = np.zeros((N+1, 3))     # angular momentum
r = np.zeros((N+1, 3))     # array from rotor center to rotor

# declare gyroscope rack arrays
Θ = np.zeros((N+1, 3))     # angle
Ω = np.zeros((N+1, 3))     # angular velocity
A = np.zeros((N+1, 3))     # angular acceleration
T = np.zeros((N+1, 3))     # torque
Λ = np.zeros((N+1, 3))     # angular momentum
R = np.zeros((N+1, 3))     # array from rack center to gyroscope

# initialize arrays
ω[0] = [10, 0, 0]
λ[0] = i * ω[0]
r[0] = [0, r0, 0]
R[0] = [R0, 0, 0]

# integration loop
for i in range(N):
    τ = np.cross(R[i], F)
    λ[i+1] = λ[i] + τ * dt
    #θ[i+1] = θ[i] + ω[i+1] * dt
    R[i+1] = R0 * λ[i+1]/np.linalg.norm(λ[i+1]) 
    
plt.plot(R[:,0], R[:,1])
plt.axis('equal')
plt.show()

plt.plot(np.linalg.norm(λ, axis=1))
plt.show()
