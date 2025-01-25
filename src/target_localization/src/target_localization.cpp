#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TargetLocalization : public rclcpp::Node
{
public:
    TargetLocalization()
        : Node("target_localization"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", 10, std::bind(&TargetLocalization::pointCloudCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/block_position", 10);

        RCLCPP_INFO(this->get_logger(), "Target Localization Node Initialized");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Calculate bounds of the incoming point cloud
        double min_x = std::numeric_limits<double>::infinity();
double max_x = -std::numeric_limits<double>::infinity();
double min_y = std::numeric_limits<double>::infinity();
double max_y = -std::numeric_limits<double>::infinity();
double min_z = std::numeric_limits<double>::infinity();
double max_z = -std::numeric_limits<double>::infinity();

for (const auto &point : cloud->points)
{
    min_x = std::min(min_x, static_cast<double>(point.x));
    max_x = std::max(max_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_y = std::max(max_y, static_cast<double>(point.y));
    min_z = std::min(min_z, static_cast<double>(point.z));
    max_z = std::max(max_z, static_cast<double>(point.z));
}

RCLCPP_INFO(this->get_logger(),
            "Point cloud bounds - X: [%.2f, %.2f], Y: [%.2f, %.2f], Z: [%.2f, %.2f]",
            min_x, max_x, min_y, max_y, min_z, max_z);

        // Filter points within workspace bounds
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto &point : cloud->points)
        {
            if (point.x >= min_x_ && point.x <= max_x_ &&
                point.y >= min_y_ && point.y <= max_y_ &&
                point.z >= min_z_ && point.z <= max_z_)
            {
                filtered_cloud->points.push_back(point);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Filtered cloud contains %zu points.", filtered_cloud->size());

        if (filtered_cloud->points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points found in the defined workspace bounds.");
            return;
        }

        // Perform clustering to detect block
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(filtered_cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.01); 
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(100);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered_cloud);
        ec.extract(cluster_indices);

        RCLCPP_INFO(this->get_logger(), "Number of clusters found: %zu", cluster_indices.size());

        if (cluster_indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No clusters found.");
            return;
        }

        // Find the largest cluster (assuming it's the block)
        auto largest_cluster = std::max_element(
            cluster_indices.begin(), cluster_indices.end(),
            [](const pcl::PointIndices &a, const pcl::PointIndices &b) {
                return a.indices.size() < b.indices.size();
            });

        // Compute the centroid of the largest cluster
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*filtered_cloud, largest_cluster->indices, centroid);

        RCLCPP_INFO(this->get_logger(), "Centroid of largest cluster: [%.2f, %.2f, %.2f]",
                    centroid[0], centroid[1], centroid[2]);

        // Create a PointStamped message
        geometry_msgs::msg::PointStamped block_position;
        block_position.header.stamp = this->now();
        block_position.header.frame_id = "lidar_link";
        block_position.point.x = centroid[0];
        block_position.point.y = centroid[1];
        block_position.point.z = centroid[2];

        // Transform the position to the world frame
        try
        {
            geometry_msgs::msg::PointStamped transformed_position = tf_buffer_->transform(
                block_position, "world", tf2::durationFromSec(1.0));

            // Publish the transformed position
            publisher_->publish(transformed_position);

            RCLCPP_INFO(this->get_logger(), "Block detected at: [%.2f, %.2f, %.2f]",
                        transformed_position.point.x,
                        transformed_position.point.y,
                        transformed_position.point.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Workspace bounds
    const double min_x_ = -5.0, max_x_ = 5.0;
    const double min_y_ = -5.0, max_y_ = 5.0;
    const double min_z_ = -.04, max_z_ = 0.5;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetLocalization>());
    rclcpp::shutdown();
    return 0;
}
