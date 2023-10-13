#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

ros::Publisher pub;
ros::Subscriber sub;

double minLineLength = 1.05;  // Minimum line length threshold
double maxLineLength = 1.15;  // Maximum line length threshold
double maxDeviation = 0.3;  // Maximum allowed deviation from a straight line
double maxNearestNeighborDistance = 0.2;  // Maximum pairwise point distance threshold


bool arePointsOnStraightLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->size() < 2) {
        // You need at least two points to form a line.
        return false;
    }

    double x1 = cloud->points[0].x;
    double y1 = cloud->points[0].y;
    double x2 = cloud->points[1].x;
    double y2 = cloud->points[1].y;

    double m = (y2 - y1) / (x2 - x1);
    double b = y1 - m * x1;

    for (size_t i = 2; i < cloud->size(); ++i) {
        double xi = cloud->points[i].x;
        double yi = cloud->points[i].y;

        double expectedY = m * xi + b;
        
        //print std::fabs(expectedY - yi)
        ROS_INFO("std::fabs(expectedY - yi): %f", std::fabs(expectedY - yi));

        if (std::fabs(expectedY - yi) > maxDeviation) {
            return false;
        }
    }

    return true;
}



void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    //check if the laser scan ranges that are not inf are all lower than the 2.0
    for (size_t i = 0; i < laser_msg->ranges.size(); ++i)
    {
        double range = laser_msg->ranges[i];
        if (!std::isinf(range))
        {
            if(range > 2.0)
            {
                ROS_INFO("The laser scan ranges that are not inf are not all lower than the 2.0. Skipping.");
                return;
            }
        }
    }

    // Convert LaserScan to a PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < laser_msg->ranges.size(); ++i)
    {
        double range = laser_msg->ranges[i];
        if (!std::isinf(range))
        {
            pcl::PointXYZ point;
            point.x = range * cos(laser_msg->angle_min + i * laser_msg->angle_increment);
            point.y = range * sin(laser_msg->angle_min + i * laser_msg->angle_increment);
            point.z = 0.0;
            cloud->push_back(point);
        }
    }

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Set the model you want to fit (LINE)
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1); // Adjust this threshold as needed

    // Call segment function to obtain set of inlier indices and model coefficients
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Check line length (euclidian distance between first and last point) before extracting
    // double x1 = cloud->points[inliers->indices[0]].x;
    // double y1 = cloud->points[inliers->indices[0]].y;
    // double x2 = cloud->points[inliers->indices[inliers->indices.size() - 1]].x;
    // double y2 = cloud->points[inliers->indices[inliers->indices.size() - 1]].y;
    // double lineLength = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    // ROS_INFO("Line length: %f", lineLength);
    // if(lineLength < minLineLength || lineLength > maxLineLength)
    // {
    //     ROS_INFO("Line length is not within the specified range (length=%.3f). Skipping.", lineLength);
    //     return;
    // }
    

    // Extract the inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*extracted_cloud);


    // Check line length (euclidian distance between first and last point) before extracting
    double x1 = extracted_cloud->points[0].x;
    double y1 = extracted_cloud->points[0].y;
    double x2 = extracted_cloud->points[extracted_cloud->size() - 1].x;
    double y2 = extracted_cloud->points[extracted_cloud->size() - 1].y;
    double lineLength = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    ROS_INFO("Line length : %f", lineLength);
    if(lineLength < minLineLength || lineLength > maxLineLength)
    {
        ROS_INFO("Line length is not within the specified range (length=%.3f). Skipping.", lineLength);
        return;
    }


    // Check if the extracted line is straight
    // double deviation = 0.0;
    // for (size_t i = 0; i < extracted_cloud->size(); ++i)
    // {
    //     double x = extracted_cloud->points[i].x;
    //     double y = extracted_cloud->points[i].y;
    //     double distance = fabs(coefficients->values[0] * x + coefficients->values[1] * y + coefficients->values[2]) /
    //                         sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2));

    //     if (distance > deviation)
    //     {
    //         deviation = distance;
    //     }
    // }
    // //print deviation
    // ROS_INFO("Deviation: %f", deviation);

    // if (deviation > maxDeviation)
    // {
    //     ROS_INFO("Detected line is not straight (deviation=%.3f). Skipping.", deviation);
    //     return;
    // }

    // Check if the extracted line is straight or curved
    // double maxDistanceToLine = 0.0;
    // for (size_t i = 0; i < extracted_cloud->size(); ++i)
    // {
    //     double x = extracted_cloud->points[i].x;
    //     double y = extracted_cloud->points[i].y;
    //     double distanceToLine = fabs(coefficients->values[0] * x + coefficients->values[1] * y + coefficients->values[2]) /
    //                             sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2));

    //     if (distanceToLine > maxDistanceToLine)
    //     {
    //         maxDistanceToLine = distanceToLine;
    //     }
    // }

    // //print max distance to line
    // ROS_INFO("Max distance to line: %f", maxDistanceToLine);

    // if (maxDistanceToLine < maxDeviation)
    // {
    //     ROS_INFO("Detected line is not straight (max deviation=%.3f). Skipping.", maxDistanceToLine);
    //     return;
    // }
    
    //check if the points are on a straight line
    if(!arePointsOnStraightLine(extracted_cloud))
    {
        ROS_INFO("Detected line is not straight. Skipping.");
        return;
    }

    // Find the maximum distance between each point and its nearest neighbor in the extracted line
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(extracted_cloud);

    double maxNearestNeighborDist = 0.0;
    for (size_t i = 0; i < extracted_cloud->size(); ++i)
    {
        std::vector<int> pointIdx;
        std::vector<float> pointSquaredDistance;

        if (kdtree.nearestKSearch(extracted_cloud->points[i], 2, pointIdx, pointSquaredDistance) > 1)
        {
            double nearestNeighborDist = sqrt(pointSquaredDistance[1]); // Distance to the second nearest neighbor (first is the point itself)

            if (nearestNeighborDist > maxNearestNeighborDist)
            {
                maxNearestNeighborDist = nearestNeighborDist;
            }
        }
    }

    if (maxNearestNeighborDist > maxNearestNeighborDistance)
    {
        ROS_INFO("Maximum nearest neighbor distance in the line is too large (distance=%0.3f). Skipping.", maxNearestNeighborDist);
        return;
    }

    //calculate the angle_min and angle_max
    double angle_min = atan2(y1, x1);
    double angle_max = atan2(y2, x2);


    // Convert the extracted PCL PointCloud back to LaserScan
    sensor_msgs::LaserScan output;
    output.header = laser_msg->header;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.angle_increment = laser_msg->angle_increment;
    output.time_increment = laser_msg->time_increment;
    output.scan_time = laser_msg->scan_time;
    output.range_min = laser_msg->range_min;
    output.range_max = laser_msg->range_max;
    output.ranges.resize(extracted_cloud->size());

    for (size_t i = 0; i < extracted_cloud->size(); ++i)
    {
        double x = extracted_cloud->points[i].x;
        double y = extracted_cloud->points[i].y;
        double range = sqrt(x * x + y * y);
        output.ranges[i] = range;
    }

    // Publish the extracted line as a LaserScan message
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_extraction");
    ros::NodeHandle nh;

    sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
    pub = nh.advertise<sensor_msgs::LaserScan>("/extracted_line", 1);

    ros::spin();

    return 0;
}