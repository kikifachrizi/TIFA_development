import rospy
import yaml
from geometry_msgs.msg import PoseArray, Pose

# Path file YAML
trajectory_file = "/home/deli/catkin_ws/src/deli_navigation/data/trajectory.yaml"

# Fungsi untuk membaca trajectory dari file YAML
def load_trajectory(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    return data['poses']

# Fungsi utama
def publish_pose_array():
    rospy.init_node('pose_array_publisher')
    pub = rospy.Publisher('/visualization_pose_array', PoseArray, queue_size=10)

    # Baca trajectory
    poses = load_trajectory(trajectory_file)

    # Buat pesan PoseArray
    pose_array = PoseArray()
    pose_array.header.frame_id = "map"  # Pastikan sesuai frame yang digunakan
    pose_array.header.stamp = rospy.Time.now()

    for pose in poses:
        pose_msg = Pose()
        pose_msg.position.x = pose['position']['x']
        pose_msg.position.y = pose['position']['y']
        pose_msg.position.z = pose['position']['z']
        pose_msg.orientation.x = pose['orientation']['x']
        pose_msg.orientation.y = pose['orientation']['y']
        pose_msg.orientation.z = pose['orientation']['z']
        pose_msg.orientation.w = pose['orientation']['w']
        pose_array.poses.append(pose_msg)

    rate = rospy.Rate(1)  # Frekuensi 1 Hz
    while not rospy.is_shutdown():
        pub.publish(pose_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose_array()
    except rospy.ROSInterruptException:
        pass
