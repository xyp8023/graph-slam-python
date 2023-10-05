import matplotlib.pyplot as plt

import os
import sys
import argparse

from collections import defaultdict, namedtuple
import g2o
import numpy as np

from utils.pose_graph_opt import PoseGraphOptimization
from utils.icp import icp
import scipy

if __name__ == "__main__":


    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit() if event.key == 'escape' else None])
    plt.gcf().gca().set_aspect('equal')
    plt.gcf().canvas.manager.set_window_title('Graph SLAM')
    plt.gcf().tight_layout(pad=0)

    max_x = -float('inf')
    max_y = -float('inf')
    min_x = float('inf')
    min_y = float('inf')

    lc_num = 0
    name = sys.argv[1] # default intel
    thres = float(sys.argv[2]) # default 0.1

    # odoms_laser = np.load("./datasets"+os.sep+name+"_odoms_laser.npy")
    # lasers = np.load("./datasets"+os.sep+name+"_lasers.npy")
    # ipc_ts = np.load("./datasets"+os.sep+name+"_ipc_ts.npy")
    # logger_ts = np.load("./datasets"+os.sep+name+"_logger_ts.npy")

    root_path = os.getcwd()
            
    odoms_laser_file = root_path + os.sep + "datasets" + os.sep + name + "_odoms_laser.npy"
    lasers_file = root_path + os.sep + "datasets" + os.sep + name + "_lasers.npy"
    ipc_ts_file = root_path + os.sep + "datasets" + os.sep + name + "_ipc_ts.npy"
    logger_ts_file = root_path + os.sep + "datasets" + os.sep + name + "_logger_ts.npy"
    odoms_laser = np.load(odoms_laser_file)
    lasers = np.load(lasers_file)
    ipc_ts = np.load(ipc_ts_file)
    logger_ts = np.load(logger_ts_file)

    graph_optimizer = PoseGraphOptimization()
    pose = np.eye(3)
    id = 0
    graph_optimizer.add_vertex(id, g2o.SE2(g2o.Isometry2d(pose)), fixed=True)

    init_pose = np.eye(3)
    vertex_idx = 1
    registered_lasers = []
    registered_lasers.append(lasers[0])
    vertex_id_odom_idx = []

    # add odom to graph
    for odom_idx, odom in enumerate(odoms_laser):
        if odom_idx==0:
            prev_odom = odom.copy()
            prev_idx = 0
            continue

        do = odom - prev_odom
        if np.linalg.norm(do[:2])>0.4 or abs(do[2])>0.2:
            
            # (2, 180)
            A = lasers[prev_idx]
            B = lasers[odom_idx]
            registered_lasers.append(B)
            dx, dy, dtheta = do[0], do[1], do[2]
            init_pose = np.array([[np.cos(dtheta), -np.sin(dtheta), dx], [np.sin(dtheta), np.cos(dtheta), dy],[0, 0, 1]])
            with np.errstate(all='raise'):
                try:
                    # T, distances, iterations = icp(B.T, A.T, init_pose, max_iterations=80, tolerance=0.0001)
                    T, distances, iterations,information = icp(B.T, A.T, init_pose, max_iterations=80, tolerance=0.0001)

                    
                except Exception as e:
                    print(odom_idx, e, A.shape, B.shape)
                    assert 1==0
                    continue

            pose = np.matmul(pose, T)
            graph_optimizer.add_vertex(vertex_idx, g2o.SE2(g2o.Isometry2d(pose)))
            vertex_id_odom_idx.append(odom_idx)
            

            # information = np.eye(3)
            rk = g2o.RobustKernelDCS()
            
            graph_optimizer.add_edge([vertex_idx-1, vertex_idx],
                            g2o.SE2(g2o.Isometry2d(T)),
                            information, robust_kernel=rk)

            prev_odom = odom
            prev_idx = odom_idx  
            
            # loop closure
            if vertex_idx > 1:
                poses = [graph_optimizer.get_pose(idx).to_vector()[:2] for idx in range(vertex_idx-1)]
                
                kd = scipy.spatial.cKDTree(poses)
                x, y, theta = graph_optimizer.get_pose(vertex_idx).to_vector()
                direction = np.array([np.cos(theta), np.sin(theta)])
                idxs = kd.query_ball_point(np.array([x,y]), r=4.25)
                for idx in idxs:
                    A = registered_lasers[idx]
                    with np.errstate(all='raise'):
                        try:
                            # T, distances, iterations = icp(A.T, B.T, np.eye(3), max_iterations=80, tolerance=0.0001)
                            T, distances, iterations, information = icp(A.T, B.T, np.eye(3), max_iterations=80, tolerance=0.0001)
                    
                        except Exception as e:
                            print(odom_idx, e, A.shape, B.shape)
                            continue
            
                    if np.mean(distances) < thres:
                        dist = np.linalg.norm(T[:2,2])
                        print(odom_idx, vertex_idx, lc_num, dist, "added an edge")
                        lc_num+=1
                        
                        

                        # information = np.eye(3)
                        rk = g2o.RobustKernelDCS()
                        graph_optimizer.add_edge([vertex_idx, idx], g2o.SE2(g2o.Isometry2d(T)), information, robust_kernel=rk)

                graph_optimizer.optimize()
                pose = graph_optimizer.get_pose(vertex_idx).to_isometry().matrix()

            # Draw trajectory and map
            traj = []
            point_cloud = []
            draw_last = float("inf")

            for idx in range(max(0, vertex_idx-draw_last), vertex_idx):
                x = graph_optimizer.get_pose(idx)
                r = x.to_isometry().R
                t = x.to_isometry().t
                filtered = registered_lasers[idx].T
                filtered = filtered[np.linalg.norm(filtered, axis=1) < 80]
                point_cloud.append((r @ filtered.T + t[:, np.newaxis]).T)
                traj.append(x.to_vector()[0:2])
            point_cloud = np.vstack(point_cloud)

            # Map resolution (m)
            xyreso = 0.01 
            point_cloud = (point_cloud / xyreso).astype('int')
            point_cloud = np.unique(point_cloud, axis=0)
            point_cloud = point_cloud * xyreso

            current_max = np.max(point_cloud, axis=0)
            current_min = np.min(point_cloud, axis=0)
            max_x = max(max_x, current_max[0])
            max_y = max(max_y, current_max[1])
            min_x = min(min_x, current_min[0])
            min_y = min(min_y, current_min[1])

            plt.cla()
            plt.axis([min_x, max_x, min_y, max_y])

            traj = np.array(traj)
            # plt.plot(traj[:, 0], traj[:, 1], '-g', markersize=0.001)
            plt.scatter(traj[:, 0], traj[:, 1],s=0.1,c='g')

            plt.plot(point_cloud[:, 0], point_cloud[:, 1], '.b', markersize=0.01)
            plt.pause(0.0001)


            vertex_idx+=1

    plt.savefig("./map_and_pose"+str(vertex_idx)+"_name_"+str(name)+".png")




