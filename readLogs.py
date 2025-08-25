import os
import pickle
import pandas as pd
import ast
import numpy as np
from scipy.interpolate import interp1d
import rosbag
from std_msgs.msg import Float64MultiArray, Header
import rospy

# read mavlink log file
def read_mavlink_log(log_file):
    dataSet = {}
    
    fin = open(log_file, 'r')
    data = fin.readlines()
    for line in data:
        line = line.replace('nan', '"nan"')
        
        try:
            jline = ast.literal_eval(line)
        except:
            continue
        
        if jline['mavpackettype'] not in dataSet.keys():
            dataSet[jline['mavpackettype']] = {}
            for key in jline.keys():
                dataSet[jline['mavpackettype']][key] = []
            
        for key in jline.keys():
            dataSet[jline['mavpackettype']][key].append(jline[key])
             
    fin.close()        
    return dataSet 

def export_to_rosbag(dataSet, timeLine_boot_ms, output_filename):
    """Export data to ROS bag format for optimal PlotJuggler integration"""
    bag = rosbag.Bag(output_filename, 'w')
    
    try:
        for i, timestamp in enumerate(timeLine_boot_ms):
            # Convert timestamp to ROS time
            ros_time = rospy.Time.from_sec(timestamp / 1000.0)  # Convert ms to seconds
            
            for groupName in dataSet:
                group = dataSet[groupName]
                for key in group.keys():
                    if key in ['mavpackettype', 'mode_string', 'text']:
                        continue
                    
                    # Create topic name
                    topic_name = f"/{groupName}/{key}"
                    
                    # Create message
                    msg = Float64MultiArray()                
                    
                    if len(group[key].shape) > 1:
                        msg.data = group[key][:, i].tolist()
                    else:
                        msg.data = [group[key][i]]
                    
                    # Write to bag
                    bag.write(topic_name, msg, ros_time)
    finally:
        bag.close()


if __name__ == "__main__":
    log_file = "logs/1/mavlink20250817_124118.txt"
    dataSet = read_mavlink_log(log_file)
    
    timeLine_boot_ms=np.array(dataSet['HIGHRES_IMU']['time_usec'])/1e3
    timeLine_local_ms=np.array(dataSet['HIGHRES_IMU']['local-ts'])
    for groupName in dataSet:
        group = dataSet[groupName]
        if 'time_usec' in group.keys():
            time_boot_ms = np.array(group['time_usec'])/1e3
            group.pop('time_usec')
            group['time_boot_ms'] = time_boot_ms
                        
        for key in group.keys():
            if key == 'mavpackettype' or key == 'time_boot_ms' or key == 'mode_string' or key == 'text':
                continue            
            data = np.array(group[key])
            if 'time_boot_ms' in group.keys():
                try:
                    if len(data.shape) == 1:
                        interpolator = interp1d(group['time_boot_ms'], data, fill_value="extrapolate")
                    elif len(data.shape) == 2:
                        interpolator = interp1d(group['time_boot_ms'], data.T, fill_value="extrapolate")
                    group[key] = interpolator(timeLine_boot_ms)

                except:
                    pass
            elif 'local-ts' in group.keys():
                try:
                    if len(data.shape) == 1:
                        interpolator = interp1d(group['local-ts'], data, fill_value="extrapolate")
                    elif len(data.shape) == 2:
                        interpolator = interp1d(group['local-ts'], data.T, fill_value="extrapolate")
                    group[key] = interpolator(timeLine_local_ms)
                except:
                    pass
            else:
                pass
        group['time_boot_ms'] = timeLine_boot_ms
        group['local-ts'] = timeLine_local_ms
                        
    out_log_filename = log_file.replace('.txt', '_interpolated.csv')
    fout = open(out_log_filename, 'w')
    
    export_to_rosbag(dataSet, timeLine_boot_ms, out_log_filename)
    # # write header
    # header = []
    # numberOfColumns = 0
    # for groupName in dataSet:
    #     group = dataSet[groupName]
    #     for key in group.keys():
    #         if key == 'mavpackettype' or key == 'mode_string' or key == 'text':
    #             continue
    #         if len(group[key].shape) > 1:
    #             for i in range(group[key].shape[0]):
    #                 header.append(f"{groupName}.{key}.{i}")
    #                 numberOfColumns += 1
    #         else:
    #             header.append(f"{groupName}.{key}")
    #             numberOfColumns += 1
    # fout.write(",".join(header))
    # fout.write("\n")
    
    # # write data to a string variable
    # data_matrix = np.zeros((len(timeLine_boot_ms), numberOfColumns))
    # columnCounter = 0
    # for groupName in dataSet:
    #     group = dataSet[groupName]
    #     for key in group.keys():
    #         if key == 'mavpackettype' or key == 'mode_string' or key == 'text':
    #             continue
    #         if len(group[key].shape) > 1:
    #             for i in range(group[key].shape[0]):
    #                 data_matrix[:, columnCounter] = group[key][i]
    #                 columnCounter += 1
    #         else:
    #             data_matrix[:, columnCounter] = group[key]
    #             columnCounter += 1

    # np.savetxt(fout, data_matrix, delimiter=",", fmt="%s")
    # fout.close()
    
    pass







