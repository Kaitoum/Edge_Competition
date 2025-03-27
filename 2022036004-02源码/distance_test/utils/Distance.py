import os
import urllib
import traceback
import time
import sys
import numpy as np
import cv2
import pandas as pd
import math
from rknn.api import RKNN

def camera_parameters( excel_path ):
    df_internal  = pd.read_excel( excel_path,sheet_name='内参矩阵',header=None )
    df_external = pd.read_excel(excel_path, sheet_name='外参矩阵', header=None)

    return df_external.values,df_internal.values


def get_coordinates( top,left,right,bottom ,p ,k ,W,H):
    u = ( top + right ) / 2
    v = bottom
    fx = k[0,0]
    fy = k[1,1]

    #相机高度
    #关键参数，不准会导致结果不对
    h =0.4
    #相机与水平线夹角，默认为0：相机正对前方，无倾斜
    #关键参数，不准会导致结果不对
    angle_a = 0
    angle_b = math.atan( (v-H/2) / fy )
    angle_c = angle_b + angle_a

    depth =( h/np.sin(angle_c) * math.cos(angle_b) )

    k_inv = np.linalg.inv( k )
    p_inv = np.linalg.inv( p )

    point_c = np.array( [u,v,1] )
    point_c = np.transpose( point_c )

    #相机坐标系下的关键点位置
    c_position = np.matmul( k_inv , depth * point_c )

    #世界坐标系下的关键点位置
    c_position = np.append( c_position,1 )
    c_position = np.transpose( c_position )
    c_position = np.matmul( p_inv,c_position )
    d = np.array( ( c_position[0] , c_position[1] ) , dtype = float )

    return d


def get_distance( top,left,right,bottom , excel_path , W, H ):
    p,k = camera_parameters( excel_path )
    d = get_coordinates( top,left,right,bottom ,p ,k ,W,H )
    distance = 0
    if( d[0]<=0 ):
        d[:] = 0
    else:
        distance = math.sqrt( math.pow( d[0],2 ) + math.pow( d[1],2 ) )
    return distance