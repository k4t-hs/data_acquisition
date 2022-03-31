# data_acquisition

This project's ros2 node 'eval_data' implements the data acquisition process for multiple benchmarks to evaluate the two visual fiducial systems AprilTag and ArUco and some optimisation methods.    
To run it you need ros2 (foxy) and the [AprilTag](https://github.com/christianrauch/apriltag_ros) and [ArUco](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco) nodes. It records the marker localistaion data published on the `/tf` topic by the AprilTag node and published on the `/aruco_markers` topic by the ArUco node.  
The following optimisation methods are considered in addition to storing the original localisation data:  
  - calculating the localisation mean over multiple frames containing one fiducial marker detection (mean over time, mot) for 3, 5 and 10 frames
  - calculating the localisation mean over a grid of multiple markers per frame (mean over grid, mog) for a grid of 3 and 5 markers
The suitable optimisation method according to the parameter `grid_size` (more information on parameters in the text below) will be selected automatically.  

You need to keep in mind that the coordinate system of a detected fiducial marker differs for AprilTag and ArUco so the rotational localisation values differ from each other as well. With a marker being positioned parallel the the camera's coordinate system, AprilTag's x-, y- and z-axes point in the same direction as the camera's. The ArUco coordinate system (on the left) in comparison to the AprilTag one (on the right) is shown in the image below. The ArUco z-axis points to the viewer out of the marker and the AprilTag z-axis points into the marker.  

![Coordinate Systems](/images/coordinate_systems.png)

## Launching the node:
`ros2 run data_acquisition topics2df_v2`   

## Parameters:  
Ground truth data for translation along x, y, z in metres and rotation about x, y, z in deg.  
`vals_gt`				  default: `[0.0, 0.0, 0.3, 0.0, 0.0, 0.0]`  

Number of markers in the marker grid. Set to 1 if there is no grid but only one marker.  
`grid_size`			  default: `1`  

Determine whether the AprilTag data and / or the ArUco data should be recorded and saved in an excel file. It is advised to only record one of both at a time, because during the saving process the other system's data will not be recorded.  
`record_apriltag`	default: `False`  
`record_aruco`		default: `False`  

Path realitve to the home directory and file name to store the acquired data. For grid_size > 1 a suffix containing the specified grid_size and the optimisation method mog will be appended automatically to the filename.  
`name_apriltag`		default: `'apriltag_data.xlsx'`  
`name_aruco`			default: `'aruco_data.xlsx'`  

## Example launching with all parameters being set manually:  
1.0 m translation along z, 30.0 deg rotation about y, a grid of 5 markers, only record AprilTag data and save it in the excel file at
'/home/username/data/apriltag.xlsx'  
`ros2 run data_acquisition topics2df_v2 --ros-args -p vals_gt:="[0.0, 0.0, 1.0, 0.0, 30.0, 0.0]" -p grid_size:=5 -p record_apriltag:=True -p 
name_apriltag:="data/apriltag.xlsx"`  

The parameters can be changed in another terminal while running the node.   
E. g.:  
`ros2 param set /eval_data record_aruco True` will start recording ArUco data as well.  
