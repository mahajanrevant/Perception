[//]: # (Image References)
[pr2_robot]:./Pictures/simulation.png
[Voxel_Downsampling]:./Pictures/Voxel_Downsampling.png
[orignal_table]:./Pictures/orignal_table.png
[Pass_Through]:./Pictures/Pass_Through.png
[Inlier]:./Pictures/Inlier.png
[Outlier]:./Pictures/Outlier.png
[cluster]:./Pictures/cluster.png
[world_1]:./Pictures/world_1.png
[world_2]:./Pictures/world_2.png
[world_3]:./Pictures/world_3.png
[world1]:./Pictures/world1.png
[world2]:./Pictures/world2.png
[world3]:./Pictures/world3.png

# Project: Perception Pick & Place
---

![pr2 robot][pr2_robot]

## Goals and requirements of the project:
The project focuses on 3D perception and object recognition with the aid of an RGB-D camera mounted on a PR2 robot. The end goal is to recognize all the objects present in front of the robot on a table in different scenarios with a minimum predetermined accuracy as mentioned further below. The trained model is tested in three scenarios- world_1, world_2, world_3. Three main parts of the implemented pipeline are- Point Cloud filtering, segmentation and object recognition .All of these tasks are explained in detail below.


## Contents


* [Filtering](Filtering)
* Clustering
* Object Recognition
* Simulation
* Improvements


## Filtering

![orignal_table][orignal_table]

This step of the pipeline concerns itself with the cleaning and proper formatting of the input data(the above picture) to feed as output to the next step of the pipeline. The steps performed are as follows:-

* __Outlier Removal Filter__ - Every sensor has a certain noise element to it, called outliers. These outliers can be internal or external to the sensor. Such outliers lead to complications in the estimation of point cloud characteristics like curvature, gradients, etc. leading to erroneous values, which in turn might cause failures at various stages in our perception pipeline. An statistical outlier filter with a mean of 8 and a threshold of 0.3 is used to get rid of the outliers.

* __VoxelGrid Downsampling Filter__ - The raw point cloud will often have more details than required, causing excess processing power use when analyzing it. The input point cloud data is downsampled using this filter to increase the computational efficiency. A leaf size of 0.01 is used to perform downsampling using make_voxel_grid_filter() function.

![Voxel_Downsampling][Voxel_Downsampling]

* __Pass Through Filter__ - All the objects of interest are placed on a table and the table is a static object itself. The region of interest is known and so a pass through filer is used to just extract table and the object from the input image. A filter along the z axis with a minimum values of 0.6 and a maximum value of 1.1 is used to get extract the objects and table. Another filter along the x axis with a minimum values of 0.3 and a maximum value of 0,6 is used to get rid of the collection bins from the region of interest.

![Pass_Through][Pass_Through]

* __RANSAC Plane Segmentation__ - Random Sample Consensus (RANSAC) is used to identify points in the dataset that belong to a particular model. It assumes that all of the data in a dataset is composed of both inliers and outliers, where inliers can be defined by a particular model with a specific set of parameters, and outliers don't. The particular model in this case is the top plane of the table. A max distance of 0.01 is used to get the outliers and inliers. In this case, the outliers are the objects and the inlier is the top plane of the table.

![Inlier][Inlier]

![Outlier][Outlier]


## Clustering


* __DBSCAN(Density-based spatial cluster of applications with noise)__ - The DBSCAN algorithm creates clusters by grouping data points that are within some threshold distance from the nearest other point in the data. The decision of whether to place a point in a particular cluster is based upon the “Euclidean distance” between that point and other cluster members. The XYZRGB point cloud data was converted to XYZ and DBSCAN algorithm was applied with a cluster_tolerance of 0.05, min_cluster_size of 30 and max_cluster_size of 3000. The min_cluster size is set small in order to incorporate for the segmentation of glue in world_3 which has very few voxel leafs.

![cluster][cluster]

## Object Recognition


The object recognition code allows each object within the object cluster to be identified. In order to do this, the system first needs to train a model to learn what each object looks like. Once it has this model, the system will be able to make predictions as to which object it sees.

* __Capture Object Features__ - Color Histograms in HSV format along with normal histograms are used to extract features. This particular approach is used as the training objects and the test objects have the same shape and colors. The goal is to get the model used to the traing features as much as possible. This approach would be considered really bad as training and test objects have various difference in the real world. The code for building the histograms is in features.py. The capture_features_pr2.py script saves the object features to a file named training_set_pr2.sav. Data for each object in 100 random orientations was collected using the HSV color space and 32 bins when creating the image histograms.

* __Training Model__ - A SVM classifier with a linear kernel for scikit-learn is used to make predictions. Before training the input data is scaled. A 5 fold cross validation is performed as well to remove bias and to check models accuracy on unseen data. The model is stored as model.sav. The model gives an accuracy of 93.6%. The following confusion matrices are generated:-

### World 1:

![world_1][world_1]

### World 2:

![world_2][world_2]

### World 3:

![world_3][world_3]

## Simulation


The performance of the model is checked in 3 different scenarios.

### World 1: Accuracy 3/3

![world1][world1]

### World 2: Accuracy 5/5

![world2][world2]

### World 3: Accuracy 8/8

![world3][world3]

## Improvements


The model is unable to distinguish between soap and soap2 sometimes. More training data should be generated to improve those results. Also the pick and place instruction should be given to the pr2 as well to drop all the objects in determined boxes.

---
