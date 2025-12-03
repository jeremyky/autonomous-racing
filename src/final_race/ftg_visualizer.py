
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import math

class FTGVisualizer:
    def __init__(self):
        self.lidar_pub = rospy.Publisher("/car_8/lidar/processed", PointCloud2, queue_size=2)
        self.marker1_pub = rospy.Publisher('marker1', Marker, queue_size=10)
        self.marker2_pub = rospy.Publisher('marker2', Marker, queue_size=10)

    def publish_lidar(self, data, ranges):
        """
        publish LIDAR data as PointCloud2, add color to borders of gap
        """
        # Setup header for PointCloud2
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = data.header.frame_id
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        angle = data.angle_min
        points = []
        for r, r_ext in zip(data.ranges, ranges):
            if r == r_ext:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0

                point = [x, y, z, 0XFFFFFF]  # white
                points.append(point)
            else:  # this point was disparity extended
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0

                point = [x, y, z, 0XFFFFFF]  # white
                points.append(point)

                x_ext = r_ext * math.cos(angle)
                y_ext = r_ext * math.sin(angle)
                z_ext = 0

                point_ext = [x_ext, y_ext, z_ext, 0XFF0000]  # red
                points.append(point_ext)

            angle += data.angle_increment

        # create PointCloud2 message
        pc2_msg = point_cloud2.create_cloud(header, fields, points)

        # resize = 200
        # pc2_msg.height = resize
        # pc2_msg.width = resize
        # pc2_msg.row_step = resize * pc2_msg.point_step

        # new_size = pc2_msg * resize * pc2_msg.point_step
        # pc2_msg.data = bytearray(new_size)
        self.lidar_pub.publish(pc2_msg)

    def publish_gap_points(self, data, start_i, end_i):
        # add all laser scan points, but set start_i and end_i to red (edge of gap)
        r1 = data.ranges[start_i]
        angle = data.angle_min + start_i * data.angle_increment
        x1 = r1 * math.cos(angle)
        y1 = r1 * math.sin(angle)
        z1 = 0
        m1 = self.generate_marker(x1, y1, z1, [0.0, 1.0, 0.0, 1.0])
        self.marker1_pub.publish(m1)

        r2 = data.ranges[end_i]
        angle = data.angle_min + end_i * data.angle_increment
        x2 = r2 * math.cos(angle)
        y2 = r2 * math.sin(angle)
        z2 = 0
        m2 = self.generate_marker(x2, y2, z2, [1.0, 0.0, 0.0, 1.0])
        self.marker2_pub.publish(m2)

    def generate_marker(self, x, y, z, color):
        marker = Marker()
        marker.header.frame_id = "car_8_laser"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker


