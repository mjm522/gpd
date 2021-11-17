#!/usr/bin/env python

from gpd_py import GPDPy
import time
import pickle
import os
import numpy as np
import open3d as o3d


def _pcd_xyz_to_o3d(pcd_xyz):
    h, w, _ = pcd_xyz.shape
    pcd_xyz = pcd_xyz.reshape(h*w, 3)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_xyz)
    return pcd


def show_pcd(pcd):
    o3d.visualization.draw_geometries([pcd], width=640, height=480)

gpd = GPDPy("/home/arm/devenvironment/gpd/gpd/cfg/eigen_params.cfg")

pcd = pickle.load(open(f"{os.environ['ARM_ROOT']}/data/gpd_py/pcd.pkl", 'rb'))
# show_pcd(_pcd_xyz_to_o3d(pcd))

modi = np.hstack([np.reshape(pcd[:,:,0], (480*640, 1), order='C'),
                  np.reshape(pcd[:,:,1], (480*640, 1), order='C'),
                  np.reshape(pcd[:,:,2], (480*640, 1), order='C')]).astype(np.float32)
modi -= modi.mean(0)


# modi_recast = np.zeros(pcd.shape)
# modi_recast[:, :, 0] = modi[:, 0].reshape(480, 640)
# modi_recast[:, :, 1] = modi[:, 1].reshape(480, 640)
# modi_recast[:, :, 2] = modi[:, 2].reshape(480, 640)
# show_pcd(_pcd_xyz_to_o3d(modi_recast))

grasps = gpd.detect_grasps(np.ascontiguousarray(modi))
print(grasps.shape)
