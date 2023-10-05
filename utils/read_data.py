"""
This script is to parse the raw data and extract useful information for later.
The datasets used in this slam demo are from http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php
"""

import os, sys
import numpy as np
from pathlib import Path


def read_data(name):
    root_path = os.getcwd()

    clf_file = root_path + os.sep + "datasets" + os.sep + name + ".clf"
    # print(clf_file)
    # check if the dataset is existed
    assert os.path.exists(clf_file)

    # FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta

    # odom_laser = [x, y, theta]
    odoms_laser = []

    # list of converted_readings
    lasers = []

    # list of ipc_timestamps
    ipc_ts = []

    # list of logger_timestamp
    logger_ts = []

    # parse the dataset
    with open(clf_file, "r") as f:
        for line in f:
            if line.startswith("FLASER"):
                read_line = line.split()

                # number of range readings, should be 180
                num_readings = int(read_line[1])

                range_readings = np.array(read_line[2 : 2 + num_readings], dtype=float)
                if num_readings == 180:
                    index = np.arange(-90, 91)
                elif num_readings == 360:
                    index = np.arange(-180, 181)
                else:
                    raise ValueError("Laser reading {} invalid", num_readings)
                index = np.delete(index, num_readings // 2)
                angles = np.radians(index)

                converted_readings = (
                    np.array([np.cos(angles), np.sin(angles)]) * range_readings
                )
                lasers.append(converted_readings)

                x = read_line[2 + num_readings]
                y = read_line[3 + num_readings]
                theta = read_line[4 + num_readings]
                odoms_laser.append([float(x), float(y), float(theta)])

                ipc_timestamp = read_line[8 + num_readings]
                logger_timestamp = read_line[10 + num_readings]
                ipc_ts.append(ipc_timestamp)
                logger_ts.append(logger_timestamp)

    odoms_laser = np.array(odoms_laser)
    lasers = np.array((lasers))
    ipc_ts = np.array(ipc_ts)
    logger_ts = np.array(logger_ts)

    odoms_laser_file = (
        root_path + os.sep + "datasets" + os.sep + name + "_odoms_laser.npy"
    )
    if not os.path.exists(odoms_laser_file):
        np.save(odoms_laser_file, odoms_laser)

    lasers_file = root_path + os.sep + "datasets" + os.sep + name + "_lasers.npy"
    if not os.path.exists(lasers_file):
        np.save(lasers_file, lasers)

    ipc_ts_file = root_path + os.sep + "datasets" + os.sep + name + "_ipc_ts.npy"
    if not os.path.exists(ipc_ts_file):
        np.save(ipc_ts_file, ipc_ts)

    logger_ts_file = root_path + os.sep + "datasets" + os.sep + name + "_logger_ts.npy"
    if not os.path.exists(logger_ts_file):
        np.save(logger_ts_file, logger_ts)


if __name__ == "__main__":
    name = "aces"
    if sys.argv[1] != "":
        name = sys.argv[1]
    read_data(name=name)
