# -*- coding: utf-8 -*-
"""
Created on Sun May  3 13:35:10 2020

@author: Marica
"""
import numpy as np
import errno
import logging
import time
import threading

from Src.Math import IMUcalc

from Src.Management.thread_communication import llc_rec

def jump_block(value, value_last, r):
    jump = value - value_last
    if jump > r:
        sig = value_last
    else:
        sig = value
    return sig

def filter_1(value, value_last, x):
    filtered = (1-x)*value_last + x*value; 
    return filtered

def calc_angle_m(acc_1_x, acc_1_y, acc_0_x, acc_0_y):
    phib = 90;              #in degrees
    phi1 = np.arctan2(acc_0_y, acc_0_x)
    acc_1_xy = np.array([[acc_1_x], [acc_1_y]])
    theta = -phi1 + phib*(np.pi/180)
    R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    #acc_1_xy_t = np.transpose(acc_1_xy)
    tilde = np.dot(R, acc_1_xy)
    alpha_acc_help = np.arctan2(tilde[1], tilde[0])*(180/np.pi)
    alpha_acc = -alpha_acc_help - phib
    return alpha_acc

def calc_tangent(alpha, b, s):
    s_vec = [None, None]
    s_vec_diff = [None, None]
    alpha_rad = alpha*(np.pi/180)
    s_vec[0]= s*np.cos(alpha_rad/2)            #x-Komponente
    s_vec[1] = s*np.sin(alpha_rad/2)            #y-komponente
    s_vec_diff[0] = (b*(((alpha_rad*(np.cos(alpha_rad)))-np.sin(alpha_rad))))/(alpha_rad**2) #x
    s_vec_diff[1] = (2*b*(alpha_rad*np.cos(alpha_rad/2)-np.sin(alpha_rad/2))*np.sin(alpha_rad/2))/(alpha_rad**2) #y
    direction = s_vec + s_vec_diff
    gamma = np.arctan((s_vec[1]-direction[0])/(direction[1]-s_vec[1])) #Winkel von der Senkrechten gesehen
    if gamma <= 0:
        tilde = (np.pi/2)+gamma                                        #Winkel von senrechter positiv oben nach unten zwischen 0 und pi
        gamma = ((np.pi/2)+tilde)*(180/np.pi)                             #gamma in degrees
    else: 
        gamma = gamma*(180/np.pi)
    return gamma
    
def calc_s(alpha, b):
    alpha_rad = alpha*(np.pi/180)
    if abs(alpha)<2:
        s = b
    else:    
        s = ((2*b)/alpha_rad)*np.sin(alpha_rad/2)
    return s    
     
def calc_J(alpha, s, b):
    alpha_rad = alpha*(np.pi/180);
    if abs(alpha)<2:
        J = 0.0048                                      #Stab mit länge b
    else:    
        J = ((s**2/4)+(((2*b**2)-(2*b*s))/alpha_rad**2))   #dünnwandiger Hohlzylinder    
    return J    
    
def calc_acc_dyn(alpha, gyro_0_z, gyro_1_z, delta_t, b):
    acc_dyn = [0, 0]
    s = calc_s(alpha, b)
    J = calc_J(alpha, s, b)
    gamma = calc_tangent(alpha, b, s)*(np.pi/180)          #in rad
    SCALE_t = 1000
    omega_punkt = (gyro_1_z - gyro_0_z)/(delta_t*SCALE_t)
    if abs(alpha)<2:                                 #hat aber nix gebracht
        acc_dyn_amount = (J*omega_punkt)/b
    else:    
        acc_dyn_amount = (J*omega_punkt)/s
    acc_dyn[0] = acc_dyn_amount*np.sin(gamma)
    acc_dyn[1] = acc_dyn_amount*np.cos(gamma)
    return acc_dyn

def calc_angle_2D(vec1, vec2, rotate_angle=0., delta_out=False, jump=np.pi*.5):
    theta = np.radians(rotate_angle)
    vec1 = IMUcalc.rotate(vec1, theta)
    x1, y1, z1 = normalize_3D(vec1)
    x2, y2 = normalize_2D(vec2)
    phi1 = np.arctan2(y1, x1)
    vec2 = rotate([x2, y2, 0], -phi1+jump)
    phi2 = np.degrees(np.arctan2(vec2[1], vec2[0]) - jump)

    alpha_IMU = -phi2

    if delta_out:
        z = np.mean([z1])
        delta = np.degrees(np.arccos(z))

    return alpha_IMU if not delta_out else (alpha_IMU, delta)   

def normalize_2D(vec):
    x, y = vec
    l = np.sqrt(x**2 + y**2)
    return x/l, y/l

def normalize_3D(vec):
    x, y, z = vec
    l = np.sqrt(x**2 + y**2 + z**2)
    return x/l, y/l, z/l

def rotate(vec, theta):
    c, s = np.cos(theta), np.sin(theta)
    return (c*vec[0]-s*vec[1], s*vec[0]+c*vec[1], vec[2])
 
def calc_alpha_J(packet, packet_last, rot_angle):
    acc_static = [0, 0]
    b = 0.12        #length actuator [m]
    x = 0.3         #filter
    r = 20          #jump block magnitude of allowed jump (alpha)
    delta_t = packet[4] - packet_last[4]
    alpha_last = packet_last[5]
    acc_dyn = calc_acc_dyn(alpha_last, packet[2], packet[3], delta_t, b)
    acc_static[0] = packet[1][0] - acc_dyn[1]                      #x
    acc_static[1] = packet[1][1] - acc_dyn[1]                      #y
    #alpha_unf = calc_angle_m(acc_static[0], acc_static[1], packet[0][0], packet[0][1])
    alpha_unf = calc_angle_2D(packet[0], acc_static, rot_angle)
    alpha_f = filter_1(alpha_unf, alpha_last, x)
    alpha_f = jump_block(alpha_f, alpha_last, r)
    return alpha_f