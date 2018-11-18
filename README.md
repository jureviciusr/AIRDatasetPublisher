# AIRDatasetPublisher

ROS Node for publishing images from AIR Dataset
AIR Dataset can be found:
https://zenodo.org/record/1211730

# Usage

Clone into your catkin workspace.
To publish images from the dataset, download one of the ZIP files from link above containing flight images.

Run 
```
rosrun air_dataset_publisher PublishDataset.py -d /path/to/unzipped/dataset/directory
```
Or you can edit `launch/preview.launch` file to get instant image preview, just 
replace /path/to/dataset with an actual path in line 3 and run `roslaunch air_dataset_publisher preview.launch`
