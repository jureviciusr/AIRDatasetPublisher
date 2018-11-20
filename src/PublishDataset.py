import argparse
import csv

# Ros libraries
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped, QuaternionStamped, PointStamped

parser = argparse.ArgumentParser(description='Publish images from AIR dataset.')
parser.add_argument('--dataset', '-d', dest='dataset', required=True,
                    metavar="/path/to/dataset", help='Path to the dataset directory')
parser.add_argument('--rate', '-r', dest='rate', default=50,
                    help='Rate of image publishing in Hz')

args, unknown = parser.parse_known_args()


def talker():
    rospy.init_node('dataset_publisher', anonymous=True)
    rate = rospy.Rate(args.rate)
    image_pub = rospy.Publisher("/camera/image/compressed", CompressedImage, queue_size=args.rate)
    gps_publisher = rospy.Publisher("/gps", NavSatFix, queue_size=args.rate)
    imu_publisher = rospy.Publisher("/imu", QuaternionStamped, queue_size=args.rate)
    gt_pose_pub = rospy.Publisher("/gt_pose", PoseStamped, queue_size=args.rate)
    map_position_pub = rospy.Publisher("/map_location", PointStamped, queue_size=args.rate)
    svo_position_pub = rospy.Publisher("/svo_location", PointStamped, queue_size=args.rate)

    metafile = open(args.dataset + "/metadata.csv", "r")
    spamreader = csv.reader(metafile)
    header = next(spamreader)
    while not rospy.is_shutdown():
        # Read the metadata until the end of file and raise a graceful exit
        try:
            line = next(spamreader)
        except StopIteration:
            raise rospy.ROSInterruptException

        # Image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        with open(args.dataset + "/" + line[0], 'rb') as myfile:
            msg.data = myfile.read()
        # Publish new image
        image_pub.publish(msg)

        # GPS coordinates
        gps_msg = NavSatFix()
        gps_msg.header.stamp = msg.header.stamp
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        gps_msg.latitude = float(line[1])
        gps_msg.longitude = float(line[2])
        gps_msg.altitude = float(line[3])
        gps_publisher.publish(gps_msg)

        # IMU orientation
        imu_msg = QuaternionStamped()
        imu_msg.header.stamp = msg.header.stamp
        imu_msg.quaternion.x = float(line[4])
        imu_msg.quaternion.y = float(line[5])
        imu_msg.quaternion.z = float(line[6])
        imu_msg.quaternion.w = float(line[7])
        imu_publisher.publish(imu_msg)

        # Ground truth location from Gazebo
        gt_pose = PoseStamped()
        gt_pose.header.stamp = msg.header.stamp
        gt_pose.pose.position.x = float(line[8])
        gt_pose.pose.position.y = float(line[9])
        gt_pose.pose.position.z = float(line[10])
        gt_pose.pose.orientation.x = float(line[11])
        gt_pose.pose.orientation.y = float(line[12])
        gt_pose.pose.orientation.z = float(line[13])
        gt_pose.pose.orientation.w = float(line[14])
        gt_pose_pub.publish(gt_pose)

        # Pixel location on map
        map_position = PointStamped()
        map_position.header.stamp = msg.header.stamp
        map_position.point.x = float(line[15])
        map_position.point.y = float(line[16])
        map_position.point.z = 0.0
        map_position_pub.publish(map_position)

        # SVO estimated location
        svo_position = PointStamped()
        svo_position.header.stamp = msg.header.stamp
        svo_position.point.x = float(line[17])
        svo_position.point.y = float(line[18])
        svo_position.point.z = float(line[19])
        svo_position_pub.publish(svo_position)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
