#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

boxes = {}
arms = {}
normals = []
crop = 50

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    cloud = get_normals_prox(cloud).cluster
    # cloud = cloud.cropImage(0, 0, crop, 0, crop);
    return cloud

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = '' + object_name.data
    # yaml_dict["pick_pose"] = pick_pose
    # yaml_dict["place_pose"] = place_pose
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"yaml dict": dict_list}
    print(data_dict)
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    
    # TODO: Statistical Outlier Filtering

    # TODO: Voxel Grid Downsampling

    # TODO: PassThrough Filter

    # TODO: RANSAC Plane Segmentation

    # TODO: Extract inliers and outliers

    # TODO: Euclidean Clustering

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster

        # Compute the associated feature vector

        # Make the prediction

        # Publish a label into RViz

        # Add the detected object to the list of detected objects.

    # Publish the list of detected objects

    # TODO: Convert ROS msg to PCL data
    cloud = pointXYZRGB = ros_to_pcl(pcl_msg)
    # cloud = cloud.cropImage(0, 0, crop, 0, crop);

    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.007
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    max_distance = 0.001
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    outlier_filter = cloud_objects.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(20)
    x = 1.0
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_objects = outlier_filter.filter()

    # TODO: Euclidean Clustering
    white_cloud = xyz = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(30)
    ec.set_MaxClusterSize(3000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Convert PCL data to ROS messages
    pcl_table = pcl_to_ros(cloud_table)
    pcl_objects = pcl_to_ros(cloud_objects)

    # TODO: Publish ROS messages
    # pcl_objects_pub.publish(pcl_objects)
    # pcl_table_pub.publish(pcl_table)
    # pcl_cluster_pub.publish(ros_cluster_cloud)


    # Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)
    # Classify the clusters!
    detected_objects_labels = []
    detected_objects = []
    do_idx = {}
    
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        sample_cloud = ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # print('color hist')
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = []
        if normals:
            # print('normal hist')
            nhists = compute_normal_histograms(normals)
        # print('concat')
        # print(len(nhists))
        if len(nhists) > 0:
            feature = np.concatenate((chists, nhists))
            # detected_objects.append([feature, ''])
            # Compute the associated feature vector
            # Make the prediction
            # Publish a label into RViz
            # Add the detected object to the list of detected objects.

            # Make the prediction, retrieve the label for the result
            # and add it to detected_objects_labels list
            prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
            label = encoder.inverse_transform(prediction)[0]
            detected_objects_labels.append(label)
            # print(            detected_objects_labels)

            # Publish a label into RViz
            label_pos = list(white_cloud[pts_list[0]])
            label_pos[2] += .4
            object_markers_pub.publish(make_label(label,label_pos, index))
            
            # Add the detected object to the list of detected objects.
            do = DetectedObject()
            do.label = label
            do.cloud = ros_cluster
            detected_objects.append(do)
            do_idx[label] = do

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels),
                                                   detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)
    # detected_objects_list = []

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        # print('pr2 call:', len(detected_objects_labels))
        pr2_mover(detected_objects, detected_objects_labels)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(objects, dlabels):
    print(dlabels)
    # TODO: Initialize variables

    # TODO: Get/Read parameters
    # TODO: Parse parameters into individual variable
    # get parameters
    dropboxinfo = rospy.get_param('/dropbox')
    # print(dropboxinfo)
    for drop in dropboxinfo:
        print('dropbox:', drop)
        boxes[drop["group"]] = drop["position"]
        # green -> right
        arms[drop["group"]] = drop["name"]

    print('arms:', arms)
    print('boxes:', boxes)
    # red = boxes['red']
    # green = boxes['green']

    object_list_param = rospy.get_param('/object_list')
    oidx = {}

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    print('list:', object_list_param)
    for i in range(0, len(object_list_param)):
        object_name = object_list_param[i]['name']
        object_group = object_list_param[i]['group']
        print('remembering:', object_name)
        oidx[object_name] = {"group": object_group}
        
    # TODO: Loop through the pick list
    # TODO: Get the PointCloud for a given object and obtain it's centroid
    dict_list = []
    # for i in range(0, len(object_list_param)):
    # Populate various ROS messages
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    do_centroids = {}
    for o in range(0,len(objects)):
        ob = objects[o]
        label = dlabels[o]
        print(o, label)
        labels.append(label)
        points_arr = ros_to_pcl(ob.cloud).to_array()
        # print('points:', points_arr)
        # print('np.mean:', np.mean(points_arr, axis=0)[:3])
        centroid = np.mean(points_arr, axis=0)[:3]
        # np.asscalar(
        # print(np.mean(points_arr, axis=0)[:3])
        print(centroid)
        centroids.append(centroid)
        do_centroids[label] = centroid

        # TODO: Create 'place_pose' for the object
        if label in oidx:
            test_scene_num = Int32()
            test_scene_num.data = 3
            object_name = String()
            # print('ob:', ob)
            print('ob.label:', ob.label)
            object_name.data = ob.label #_list_param[i]['name']
            # TODO: Assign the arm to be used for pick_place
            group = oidx[label]["group"]
            arm_name = String()
            arm_name.data = arms[group]
            pick_pose = Pose()
            pick_pose.position.x = np.asscalar(centroid[0])
            pick_pose.position.y = np.asscalar(centroid[1] + 0.01)
            pick_pose.position.z = np.asscalar(0.0 + centroid[2])
            pick_pose.orientation.x = np.asscalar(0.0 + centroid[0])
            pick_pose.orientation.y = np.asscalar(0.0 + centroid[1] + 0.01)
            pick_pose.orientation.z = np.asscalar(0.0 + centroid[2])
            pick_pose.orientation.w = .1
            p=boxes[group]
            place_pose = Pose()
            place_pose.position.x = p[0]
            place_pose.position.y = p[1]
            place_pose.position.z = p[2]
            print('pick_pose:', pick_pose)
            print('place_pose:', place_pose)

            yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name,
                                       pick_pose, place_pose)
            # TODO: Create a list of dictionaries (made with make_yaml_dict())
            # for later output to yaml format
            dict_list.append(yaml_dict)
    
            move_items = True
            if move_items:
                # Wait for 'pick_place_routine' service to come up
                rospy.wait_for_service('pick_place_routine')

                try:
                    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
                    # TODO: Insert your message variables to be sent as a service request
                    resp = pick_place_routine(test_scene_num, object_name, arm_name,
                                              pick_pose, place_pose)
                    print ("Response: ",resp.success)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    yamls = dict_list
    send_to_yaml('out.yaml', yamls)


if __name__ == '__main__':

    # TODO: ROS node initialization
    # TODO: Create Subscribers
    # TODO: Create Publishers
    # TODO: Load Model From disk
    # TODO: Spin while node is not shutdown

    # Initialize color_list
    get_color_list.color_list = []

    rospy.init_node('project_template')
    rate = rospy.Rate(5000)
    start_time = 0
    while not start_time:
        start_time = rospy.Time.now().to_sec()

    # Create Publishers
    # TODO: here you need to create two publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
    # print('registering publishers')
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=10)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray,
                                           queue_size=10)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']


    # TODO: Create Subscribers
    # pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2,
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # TODO: Create Publishers

    # TODO: Load Model From disk

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time
        rate.sleep()
