import numpy as np
from scipy.optimize import minimize

# Sudut dari masing-masing roda dalam derajat
a1, a2, a3, a4 = 49.7334, 133.8309, 223.3317, 313.6028

def inv_kin(Vx, Vy, Wr):
    V1 = -Vx*np.sin(np.deg2rad(a1)) + Vy*np.cos(np.deg2rad(a1)) + Wr*(np.cos(np.deg2rad(45-a1)))
    V2 = -Vx*np.sin(np.deg2rad(a2)) + Vy*np.cos(np.deg2rad(a2)) + Wr*(np.cos(np.deg2rad(135-a2)))
    V3 = -Vx*np.sin(np.deg2rad(a3)) + Vy*np.cos(np.deg2rad(a3)) + Wr*(np.cos(np.deg2rad(225-a3)))
    V4 = -Vx*np.sin(np.deg2rad(a4)) + Vy*np.cos(np.deg2rad(a4)) + Wr*(np.cos(np.deg2rad(315-a4)))
    
    return V1, V2, V3, V4

def for_kin(v1, v2, v3, v4, weights):
    w1, w2, w3, w4 = weights
    x = (-v1*np.sin(np.deg2rad(a1))*w1 - v2*np.sin(np.deg2rad(a2))*w2 - v3*np.sin(np.deg2rad(a3))*w3 - v4*np.sin(np.deg2rad(a4))*w4) / 2
    y = (v1*np.cos(np.deg2rad(a1))*w1 + v2*np.cos(np.deg2rad(a2))*w2 + v3*np.cos(np.deg2rad(a3))*w3 + v4*np.cos(np.deg2rad(a4))*w4) / 2
    w = (v1*np.cos(np.deg2rad(45-a1))*w1 + v2*np.cos(np.deg2rad(135-a2))*w2 + v3*np.cos(np.deg2rad(225-a3))*w3 + v4*np.cos(np.deg2rad(315-a4)))*w4 / 4
    
    return x, y, w

def loss_function(weights):
    v1, v2, v3, v4 = inv_kin(0, 100, 0)
    x, y, w = for_kin(v1, v2, v3, v4, weights)
    loss1 = (x - 0)**2 + (y - 100)**2 + (w - 0)**2
    
    v1, v2, v3, v4 = inv_kin(100, 0, 0)
    x, y, w = for_kin(v1, v2, v3, v4, weights)
    loss2 = (x - 100)**2 + (y - 0)**2 + (w - 0)**2
    
    return loss1 + loss2

# Optimasi bobot
initial_weights = [1.0, 1.0, 1.0, 1.0]
result = minimize(loss_function, initial_weights, method='Powell')
optimized_weights = result.x

# Pengujian
print("Optimized weights:", optimized_weights)
print(inv_kin(100, 0, 0))
v1, v2, v3, v4 = inv_kin(100, 0, 0)
print(for_kin(v1, v2, v3, v4, optimized_weights))
