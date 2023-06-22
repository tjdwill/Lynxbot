"""
Author: Terrance Williams
Date: 23 November 2022
Description: Python functions for calculating inverse and forward kinematics for the Lynxmotion 3DoF Robot Arm.
"""

import numpy as np

# Variables (Lengths (in cm) of link 1, link 2, and ground-to-base height)
l1, l2, b = 13.5, 19, 5.5 

def lynxmotion_fk(servo1, servo2, servo3, mode='rad'):
  """ Returns "universe-frame" coordinates for the Lynxmotion 3DoF Robot Arm"""
    if mode == 'deg':
        servo1 = np.radians(servo1)
        servo2 = np.radians(servo2)
        servo3 = np.radians(servo3)
    x = (l1*np.sin(servo3) + l2*np.cos(servo2))*np.cos(servo1)
    y = (-l1*np.sin(servo3) + l2*np.cos(servo2))*-np.sin(servo1)
    z = (l1*np.cos(servo3) + l2*np.sin(servo2))

    return np.array([x, y, z+b])

  
def lynxmotion_ik(px, py, pz):
  """Input desired position: outputs the requisite LSS servo angles (in radians)
      NOTE: Angle sense (+ or -) is determined based on how the LSS moves the robot. 
      For example, theta 3 is inverted to - to result in proper movement.  
      
      Calculations based on Dr. Rainer Hessmer's work on the Lynxmotion 3DoF Kinematics
  """
    d = np.sqrt(px**2 + py**2)
    z_prime = pz - b
    cos_a = ((d**2 + z_prime**2 - l1**2 - l2**2)/(2*l1*l2)).round(5)
    theta_a = np.arctan2(np.sqrt(1-(cos_a**2)), cos_a)
    theta_1 = -np.arctan2(py, px)

    k1 = l2 + l1*cos_a
    k2 = l1*np.sin(theta_a)

    theta_2 = np.arctan2(z_prime, d) - np.arctan2(k2, k1)
    theta_3 = theta_a + theta_2 - (np.pi/2)

    return np.array([theta_1, theta_2, -theta_3])
  
if __name__ == "__main__":
  """Example code that outputs servo vals for multiple positions"""
  
    k = 5.5  # desired z-value

    # output angles for various positions
    for i in range(11, 29, 1):
        for j in range(-12, 12, 1):
            print(f'({i}, {j}, {k})cm: {np.degrees(lynxmotion_ik(i, j, k)).round(3)*10}')


    # For positions that lie along a circle of determined radius.
    def y_calc(magnitude: float, x: float) -> float:
        val = magnitude**2 - x**2
        return np.sqrt(val)

    # Print values in Lynxmotion form (tenths of a degree)
    for x_val in range(11, 29, 1):
        print(f'Circle def ({x_val}, {y_calc(21, a).round(2)}, {k}): {np.degrees(lynxmotion_ik(x_val, y_calc(21, a), k)).round(2)*10}')
