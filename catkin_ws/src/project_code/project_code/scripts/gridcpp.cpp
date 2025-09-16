#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"

ros::Publisher marker_pub;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    int width = msg->info.width;    // Lebar peta dalam grid cell
    int height = msg->info.height;  // Tinggi peta dalam grid cell
    float resolution = msg->info.resolution;  // Ukuran satu grid cell (meter)
    
    // Iterasi pada data occupancy grid
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int index = i * width + j;
            int value = msg->data[index]; // Nilai occupancy, -1 (unknown), 0 (free), 100 (occupied)

            if (value == 100) {  // Jika grid tersebut occupied
                // Hitung posisi grid dalam koordinat dunia
                float x = j * resolution + msg->info.origin.position.x;
                float y = i * resolution + msg->info.origin.position.y;

                // Membuat marker untuk visualisasi di RViz
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";  // Atur frame ke peta
                marker.header.stamp = ros::Time();
                marker.ns = "grid_labels";
                marker.id = i * width + j;  // Unik ID per marker
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = x;  // Posisi X grid di peta
                marker.pose.position.y = y;  // Posisi Y grid di peta
                marker.pose.position.z = 0.5;  // Ketinggian text
                marker.scale.z = 0.1;  // Ukuran text
                marker.color.a = 1.0;  // Transparansi text
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;

                // Teks yang akan ditampilkan
                std::stringstream ss;
                ss << "Row: " << i << ", Col: " << j;
                marker.text = ss.str();

                // Publikasikan marker ke RViz
                marker_pub.publish(marker);
            }
        }
    }
    ROS_INFO("Map resolution: %f", resolution);
    ROS_INFO("Width: %d", width);
    ROS_INFO("Height: %d", height);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_grid_node");  // Menginisialisasi node ROS
    ros::NodeHandle n;

    // Subscribe ke topik /map
    ros::Subscriber sub = n.subscribe("/map", 0, mapCallback);

    // Publisher untuk marker
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::spin();  // Menjalankan loop ROS

    return 0;
}

