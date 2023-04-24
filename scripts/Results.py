from matplotlib import pyplot as plt
from matplotlib import transforms
import numpy as np
import pandas as pd
import os, math
import json, yaml

MapYamlName = "lab_map.yaml"                                                                             
WaypointsJsonName = "waypoints.json"  
TestRunCSVName = "PPTest1mps.csv"

def rmsValue(arr, n):
    square = 0
    mean = 0.0
    root = 0.0
     
    #Calculate square
    for i in range(0,n):
        square += (arr[i]**2)
     
    #Calculate Mean
    mean = (square / (float)(n))
     
    #Calculate Root
    root = math.sqrt(mean)
     
    return root
    
def main():
    FMLRootDir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),".."))
     #------------------------------Map Loading-----------------------------------------------------------------------#
    FMLMapDir = os.path.join(FMLRootDir,"src","car_slam","config")
    MapYaml = os.path.join(FMLMapDir,MapYamlName)
    with open(MapYaml,"r") as read_file:
        yaml_params = yaml.load(read_file, Loader=yaml.SafeLoader)
        resolution = yaml_params["resolution"]
        origin = yaml_params["origin"]
        MapName = yaml_params["image"]

    MapFile = os.path.join(FMLMapDir,MapName)
    map_image = plt.imread(MapFile)
    map_graph_height, map_graph_width = map_image.shape
    map_height = map_graph_height * resolution
    map_width = map_graph_width * resolution
    figscalefactor = 0.5
    _, ax = plt.subplots(figsize=(map_height*figscalefactor, map_width*figscalefactor+0.25)) #0.25 
    tr = transforms.Affine2D().rotate_deg(90)
    ax.imshow(map_image,cmap="gray",extent=[origin[0],map_width+origin[0],origin[1],map_height+origin[1]],transform=tr+ax.transData)

    #------------------------------Fetching Waypoints----------------------------------------------------------------#
    WaypointsName = os.path.join(FMLRootDir,"src","ros2_car_control","config")
    WaypointsJson = os.path.join(WaypointsName,WaypointsJsonName)
    with open(WaypointsJson,"r") as read_file:
        waypointsfile = json.load(read_file)
        untranslated_waypoints = waypointsfile['smoothed_wpts']
        x = np.array([x[0] for x in untranslated_waypoints])
        y = np.array([y[1] for y in untranslated_waypoints])
        
        x = x - origin[0] + 1.3   #1.3, -1.4 are hardcoded offsets, due to path planner transcription
        y = -y + origin[1] - 1.4 + 2*(map_height)
    #------------------------------Test Run Data Extraction---------------------------------------------------------#
    TestRunCSV = os.path.join(FMLRootDir,"Testing",TestRunCSVName)
    ResultsTable = pd.read_csv(TestRunCSV)
    AMCLTable = ResultsTable.query("ROSTOPIC == 'amcl_pose'")
    pose_hist_x = AMCLTable['X'].to_numpy()
    pose_hist_y = AMCLTable['Y'].to_numpy()

    #------------------------------Plotting Results----------------------------------------------------------------#
    plt.plot(-y,x,label="Path",color="black")
    plt.plot(-pose_hist_y,pose_hist_x,label="Vehicle Trajectory",color="blue")
    plt.xlabel("X Coordinates [m]")
    plt.ylabel("Y Coordinates [m]")
    plt.ylim(origin[0],-origin[1])
    plt.xlim(-map_height-origin[1],-origin[1])
    plt.legend()

    SteerAngleTable = ResultsTable.query("ROSTOPIC == 'cmd_ackermann'")
    cmd_time = SteerAngleTable["TIME_SEC"].to_numpy()+SteerAngleTable["TIME_NSEC"].to_numpy()*10**-9
    cmd_time = cmd_time - cmd_time[0]
    cmds = SteerAngleTable["STEER_CMD"].to_numpy()
    plt.figure()
    plt.plot(cmd_time,cmds)
    plt.xlabel("Time [sec]")
    plt.ylabel("Steering Angle [rad]")
    plt.axhline(y=0, color="k", linestyle="dotted")




    error_array = []
    windupcount=0
    for i in range(len(pose_hist_x)):
        distance_to_waypoint = []
        for j in range(len(x)):
            distance_to_waypoint.append(np.sqrt((pose_hist_x[i] - x[j])**2 + (pose_hist_y[i] - y[j])**2))
        
        error = min(distance_to_waypoint)
        if distance_to_waypoint.index(error) != len(distance_to_waypoint)-1:
            error_array.append(abs(min(distance_to_waypoint)))
        else:
            windupcount+=1
        
    print("RMS Lateral Error [m]: {}".format(rmsValue(error_array,len(error_array))))

        
    plt.show()

if __name__ == '__main__':
    main()