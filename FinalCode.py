import open3d as o3d
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import glob


class CSV_Data:
    def __init__(self, filepath, path):
        self.filepath = filepath
        self.path = path
        self.point_cloud = None

    # Read X, Y, Z coordinates from a CSV file and check for errors
    def read_csv(self):
        try:
            df = pd.read_csv(self.filepath)
            self.point_cloud = np.array(df[['Point_X', 'Point_Y', 'Point_Z']])

        except FileNotFoundError:
            print(f"Error: File '{self.filepath}' not found.")
        except pd.errors.EmptyDataError:
            print(f"Error: File '{self.filepath}' is empty.")
        except pd.errors.ParserError:
            print(f"Error: Unable to parse file '{self.filepath}'.")
        except KeyError:
            print(f"Error: Required columns 'Point_X', 'Point_Y', and 'Point_Z' not found in file '{self.filepath}'.")

        #Check for file content:

        #print(df)
        #print(self.point_cloud)

    # Go through and check for errors in all the CSV files
    def check_all_csv(self):

        all_files = glob.glob(self.path + "/*.csv")
        all_files

        li = []
        if not all_files:
            print(f"No CSV files found in directory '{self.path}'.")
            return False
        for filename in all_files:

            try:
                df = pd.read_csv(filename)
                # li.append(df)
                self.point_cloud = np.array(df[['Point_X', 'Point_Y', 'Point_Z']])

                print(f"There is no error in '{filename}'.")

            except FileNotFoundError:
                print(f"Error: File '{filename}' not found.")
            except pd.errors.EmptyDataError:
                print(f"Error: File '{filename}' is empty.")
            except pd.errors.ParserError:
                print(f"Error: Unable to parse file '{filename}'.")
            except KeyError:
                print(
                    f"Error: Required columns 'Point_X', 'Point_Y', and 'Point_Z' not found in file '{filename}'.")

#Read any Sensor data
class SensorData(CSV_Data):
    def __init__(self, filepath):
        super().__init__(filepath, path)

    def read_csv(self):
        super().read_csv()
        # additional processing specific to sensor data can be added here

#Visualize the sensor data using Open3D
class SensorVisualizer(SensorData):
    def __init__(self, data):
        self.data = data

    def visualize(self):
        if self.data.point_cloud is None:
            print("Error: No point-cloud data to visualize.")
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.data.point_cloud)
        o3d.visualization.draw_geometries([pcd])
        # additional visualization/information specific to LiDAR data can be added here:
        #print(pcd)
        #print(np.asarray(pcd.points))

#Selecting the wall for analysis
class Wall(SensorVisualizer):
    def __init__(self, data):
        super().__init__(data)

    def analyze(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.data.point_cloud)
        # additional visualization specific to LiDAR data can be added here
        # o3d.visualization.draw_geometries([pcd])

        # Fit a plane to the points in the point cloud using RANSAC
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.045, ransac_n=3, num_iterations=10000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")


        # Extract the points that belong to the wall
        wall_points = pcd.select_by_index(inliers)
        non_wall_points = pcd.select_by_index(inliers, invert=True)

        # Visualize the point cloud with the wall points in blue and the non-wall points in red
        wall_points.paint_uniform_color([0.0, 0.0, 1.0])  # blue
        non_wall_points.paint_uniform_color([1.0, 0.0, 0.0])  # red
        plane = np.asarray(plane_model)

        o3d.visualization.draw_geometries([non_wall_points, wall_points])
        o3d.visualization.draw_geometries([wall_points])

        # Calculate the average distance between the wall plane and the wall points (not working)
        # pcd = o3d.geometry.PointCloud()
        # pcd.plane = o3d.utility.Vector3dVector(plane_model)
        # distances = wall_points.compute_point_cloud_distance(plane_model)
        # avg_distance = np.mean(distances)
        # print("Average distance between the wall plane and the wall points: ", avg_distance)

#Measuring the cylinder object
class Cylinder(SensorVisualizer):
    def __init__(self, data):
        super().__init__(data)
        self.filepath = 'DataFiles/hazi_feladat (Frame 0000).csv'

    def measure_height(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.data.point_cloud)

        print("Pick the points for the cylinder object in this order: maximum point, minimum point, left and right (for diameter)!")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Pick the maximum point, then the minimum point of the cylinder object!")
        vis.add_geometry(pcd)
        vis.run()  # user picks points
        vis.destroy_window()
        picked_points = vis.get_picked_points()
        print(picked_points)

        df = pd.read_csv(self.filepath)
        self.point_cloud = np.array(df[['Point_Z']])
        max_picked_point_coordinates = self.point_cloud[picked_points[0]]
        min_picked_point_coordinates = self.point_cloud[picked_points[1]]
        height_of_object = max_picked_point_coordinates-min_picked_point_coordinates

        print("The selected points:")
        print(self.point_cloud[picked_points[0]])
        print(self.point_cloud[picked_points[1]])
        print("The height of the object is:", height_of_object)



        #Visualize the selected points:
        #Cylinder_points = pcd.select_by_index(picked_points)
        #o3d.visualization.draw_geometries([Cylinder_points])

    def measure_diameter(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.data.point_cloud)

        print("Pick the points for the cylinder object in this order: left and right (for diameter)!")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Pick the points for the cylinder object in this order: left and right (for diameter)!")
        vis.add_geometry(pcd)
        vis.run()  # user picks points
        vis.destroy_window()
        picked_points = vis.get_picked_points()
        print(picked_points)

        df = pd.read_csv(self.filepath)
        self.point_cloud = np.array(df[['Point_X']])
        left_picked_point_coordinates = self.point_cloud[picked_points[0]]
        right_picked_point_coordinates = self.point_cloud[picked_points[1]]
        diameter_of_object = right_picked_point_coordinates - left_picked_point_coordinates

        print("The selected points:")
        print(self.point_cloud[picked_points[0]])
        print(self.point_cloud[picked_points[1]])
        print("The diameter of the object is:", diameter_of_object)


# Creating objects:


filepath = 'DataFiles/hazi_feladat (Frame 0000).csv'
path = r'C:\Users\User\Documents\aiMotive\hazi_feladat\hazi_feladat'

# Check the content and errors of current csv file:
check_current_csv = CSV_Data(filepath,path)
check_current_csv.read_csv()

# Look for an error in all csv files:
csv_error = CSV_Data(filepath, path)
csv_error.check_all_csv()

# Read in sensor data:
lidar_data = SensorData(filepath)
lidar_data.read_csv()

# Visualize sensor data:
lidar_visualizer = SensorVisualizer(lidar_data)
lidar_visualizer.visualize()

# Analyze the wall from the lidar data:
wall_analyzer = Wall(lidar_data)
wall_analyzer.analyze()


#Select points to measure the cylinder object:

cylinder_analyzer = Cylinder(lidar_data)
cylinder_analyzer.measure_height()
cylinder_analyzer.measure_diameter()
