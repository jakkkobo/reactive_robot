#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include <dynamic_tt/RobotMissionConfig.h>


using namespace Eigen;


ros::Publisher pub;
ros::Publisher pub_laser_right;
ros::Publisher pub_laser_front;
ros::Publisher pub_laser_left;
ros::Publisher pub_laser_back;
ros::Publisher pub_line;
ros::Publisher pub_end;
ros::Subscriber sub;
ros::Subscriber sub_line;

bool corrected = false;
float max_range = 2.0;
bool stop_state = false;
bool end_state = false;
float side = 1.0;
float minimum_distance_to_wall = 0.7;
// P controller to correct the error angle and compute the angular velocity
double Kp_angular = 4.0; //1.5
double Kp_wall = 0.2; //0.2
// P controller to compute the linear velocity
double Kp_linear = 0.3; //0.1
double Kp_wall_linear = 0.1; //0.1


//struct for the x and y data of the curve fit
struct curve_fit_data
{
    std::vector<double> x_data;
    std::vector<double> y_data;
};


void dynamic_callback(dynamic_tt::RobotMissionConfig &config, uint32_t level)
{
    //print the config parameters
    ROS_INFO("max_range: %f", config.max_laser_range_to_wall);
    ROS_INFO("side: %f", config.map_side);
    ROS_INFO("minimum_distance_to_wall: %f", config.minimum_distance_to_wall);
    ROS_INFO("Kp_linear: %f", config.kp_linear);
    ROS_INFO("Stop: %d", config.STOP);

    // stop simulation
    if(config.STOP)
        stop_state = true;
    else
        stop_state = false;

    //set the max_range
    max_range = config.max_laser_range_to_wall;
    //set the side
    if (side != config.map_side)
    {
        corrected = false;
        side = config.map_side;
    }
    
    //set the minimum_distance_to_wall
    minimum_distance_to_wall = config.minimum_distance_to_wall;
    //set the Kp_linear
    Kp_linear = config.kp_linear;
    //set the Kp_angular
    Kp_angular = config.kp_angular;
    //set the Kp_wall
    Kp_wall = config.kp_wall_distance;
    //set the Kp_wall_linear
    Kp_wall_linear = config.kp_wall_linear;

    return;
}

curve_fit_data curve_fit(std::vector<float> x, std::vector<float> y)
{
    //convert x and y to std::vector<double> x_data and y_data also verify if exists inf in the x_and y and save correct values in x_data and y_data
    std::vector<double> x_data, y_data;
    for(int i = 0; i < x.size(); i++)
    {
        if(x[i] != INFINITY && y[i] != INFINITY && x[i] != -INFINITY && y[i] != -INFINITY)
        {
            x_data.push_back(x[i]);
            y_data.push_back(y[i]);
        }
        // ROS_INFO("inf detected");
    }

    //check if the x_data and y_data are empty
    if(x_data.empty() || y_data.empty())
    {
        ROS_INFO("No data to fit");
        return curve_fit_data();
    }

    // Convert data to Eigen::Map for efficient computation
    Map<VectorXd> x_vec(x_data.data(), x_data.size());
    Map<VectorXd> y_vec(y_data.data(), y_data.size());

    // Create the Vandermonde matrix (A matrix) for polynomial fitting
    int degree = 1;  // 1 for a straight line
    MatrixXd A(x_data.size(), degree + 1);
    for (int i = 0; i < x_data.size(); i++) {
        for (int j = 0; j <= degree; j++) {
            A(i, j) = pow(x_data[i], j);
        }
    }

    // Solve the linear system to find coefficients
    VectorXd coefficients = A.householderQr().solve(y_vec);

    // Generate x values for the fitted line
    VectorXd x_fit = VectorXd::LinSpaced(100, x_vec.minCoeff(), x_vec.maxCoeff());
    VectorXd y_fit(x_fit.size());
    for (int i = 0; i < x_fit.size(); i++) {
        y_fit(i) = 0.0;
        for (int j = 0; j <= degree; j++) {
            y_fit(i) += coefficients(j) * pow(x_fit(i), j);
        }
    }

    //publish a maker line on "base_scan" frame with the x_fit and y_fit data
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "base_scan";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "curve_fit";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.01;
    line_strip.color.b = 1.0;   
    line_strip.color.a = 1.0;
    geometry_msgs::Point p;
    for(int i = 0; i < x_fit.size(); i++)
    {
        p.x = x_fit(i);
        p.y = y_fit(i);
        p.z = 0;
        line_strip.points.push_back(p);
    }
    pub_line.publish(line_strip);

    //convert the x_fit and y_fit data to the struct curve_fit_data
    curve_fit_data line;
    for(int i = 0; i < x_fit.size(); i++)
    {
        line.x_data.push_back(x_fit(i));
        line.y_data.push_back(y_fit(i));
    }

    return line;
    
}

double compute_angle(std::vector<double> x_fit, std::vector<double> y_fit)
{
    //compute the angle between the line and the paralele to the x axis
    curve_fit_data line;
    line.x_data = x_fit;
    line.y_data = y_fit;
    double angle = atan((line.y_data[line.y_data.size()/2] - line.y_data[0])/(line.x_data[line.x_data.size()/2] - line.x_data[0]));

    //publish a maker line on "base_scan" frame equal to the paralele of the x axis from the middle of the line
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "base_scan";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "heading_line";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.01;
    line_strip.color.r = 0.5;
    line_strip.color.a = 5.0;
    geometry_msgs::Point p;
    p.x = line.x_data[line.x_data.size()/2];
    p.y = line.y_data[line.y_data.size()/2];
    p.z = 0;
    line_strip.points.push_back(p);
    p.x = line.x_data[line.x_data.size()/2] + 0.5;
    p.y = line.y_data[line.y_data.size()/2];
    p.z = 0;
    line_strip.points.push_back(p);
    pub_line.publish(line_strip);
    
    return angle;
}

bool heading_correction(std::vector<float> work_part)
{
    //define alignement to all state
    bool inf_state;
    //turn while inf data exists in the work_part part vector else stop
    for(int i = 0; i < work_part.size(); i++)
    {
        if(work_part[i] == INFINITY)
        {
            inf_state = true;
            break;
        }
        else
        {
            inf_state = false;
        }
    }
    geometry_msgs::Twist msg;
    if(inf_state)
    {
        msg.linear.x = 0.0;
        msg.angular.z = 0.5*side;
        pub.publish(msg);
        return false;
    }
    else
    {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        pub.publish(msg);
        return true;
    }
    
}

void lineCallback(const sensor_msgs::LaserScan::ConstPtr& line_msg)
{
    Kp_linear = 0.1;
    //print end point of the line
    double middle_point = line_msg->ranges.size()/2;
    ROS_INFO("End point of the line: %f", line_msg->ranges[middle_point]);

    //compute the distance from robot to the middle point
    double distance_to_middle_point = line_msg->ranges[middle_point];

    //get the line from laser scan
    std::vector<float> x_data, y_data;
    for(int i = 0; i < line_msg->ranges.size(); i++)
    {
        if(line_msg->ranges[i] != INFINITY && line_msg->ranges[i] != NAN)
        {
            x_data.push_back(line_msg->ranges[i]*cos(line_msg->angle_min + i*line_msg->angle_increment));
            y_data.push_back(line_msg->ranges[i]*sin(line_msg->angle_min + i*line_msg->angle_increment));
        }
    }

    //compute the angle between line and the perpendicular to the robot x axis on middle point
    double angle_to_middle_point = atan(y_data[y_data.size()/2]/x_data[x_data.size()/2]);

    //print the angle to the middle point in degrees
    ROS_INFO("Angle to middle point: %f", angle_to_middle_point*180/M_PI);
    
    //check if angle is bigger or equal to 80 degrees and stop robot
    if(angle_to_middle_point >= 80*M_PI/180)
    {
        end_state = true;
        ROS_INFO("End goal");
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        pub.publish(msg);
        sleep(1);
        std_msgs::Bool end_msg;
        end_msg.data = true;
        pub_end.publish(end_msg);
        return;
    }

}

void publish_scans(const sensor_msgs::LaserScan::ConstPtr& laser_msg, std::vector<float> left, std::vector<float> right, std::vector<float> front, std::vector<float> back)
{
        //publish a laser scan with only the back part
    sensor_msgs::LaserScan laser_msg_5;
    laser_msg_5.header = laser_msg->header;
    laser_msg_5.angle_min = laser_msg->angle_min + 3*M_PI/4;
    laser_msg_5.angle_max = laser_msg->angle_max + 3*M_PI/4;
    laser_msg_5.angle_increment = laser_msg->angle_increment;
    laser_msg_5.time_increment = laser_msg->time_increment;
    laser_msg_5.scan_time = laser_msg->scan_time;
    laser_msg_5.range_min = laser_msg->range_min;
    laser_msg_5.range_max = laser_msg->range_max;
    laser_msg_5.ranges = back;
    pub_laser_back.publish(laser_msg_5);

    //publish a laser scan with only the right part
    sensor_msgs::LaserScan laser_msg_2;
    laser_msg_2.header = laser_msg->header;
    laser_msg_2.angle_min = laser_msg->angle_min + 5*M_PI/4;
    laser_msg_2.angle_max = laser_msg->angle_max + 5*M_PI/4;
    laser_msg_2.angle_increment = laser_msg->angle_increment;
    laser_msg_2.time_increment = laser_msg->time_increment;
    laser_msg_2.scan_time = laser_msg->scan_time;
    laser_msg_2.range_min = laser_msg->range_min;
    laser_msg_2.range_max = laser_msg->range_max;
    laser_msg_2.ranges = right;
    pub_laser_right.publish(laser_msg_2);

    //publish a laser scan with only the front part
    sensor_msgs::LaserScan laser_msg_3;
    laser_msg_3.header = laser_msg->header;
    laser_msg_3.angle_min = laser_msg->angle_min + 7*M_PI/4;
    laser_msg_3.angle_max = laser_msg->angle_max + 7*M_PI/4;
    laser_msg_3.angle_increment = laser_msg->angle_increment;
    laser_msg_3.time_increment = laser_msg->time_increment;
    laser_msg_3.scan_time = laser_msg->scan_time;
    laser_msg_3.range_min = laser_msg->range_min;
    laser_msg_3.range_max = laser_msg->range_max;
    laser_msg_3.ranges = front;
    pub_laser_front.publish(laser_msg_3);

    //publish a laser scan with only the left part
    sensor_msgs::LaserScan laser_msg_4;
    laser_msg_4.header = laser_msg->header;
    laser_msg_4.angle_min = laser_msg->angle_min + 9*M_PI/4;
    laser_msg_4.angle_max = laser_msg->angle_max + 9*M_PI/4;
    laser_msg_4.angle_increment = laser_msg->angle_increment;
    laser_msg_4.time_increment = laser_msg->time_increment;
    laser_msg_4.scan_time = laser_msg->scan_time;
    laser_msg_4.range_min = laser_msg->range_min;
    laser_msg_4.range_max = laser_msg->range_max;
    laser_msg_4.ranges = left;
    pub_laser_left.publish(laser_msg_4);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    //check if stop state is true
    if(stop_state)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        pub.publish(msg);
        return;
    }
    
    //check if the laser scan data is empty or all inf or nan
    if(laser_msg->ranges.empty())
    {
        ROS_INFO("No laser scan data");
        return;
    }
    else
    {
        bool state;
        for(int i = 0; i < laser_msg->ranges.size(); i++)
        {
            if(laser_msg->ranges[i] == INFINITY || laser_msg->ranges[i] == NAN)
            {
                state = true;
            }
            else
            {
                state = false;
                break;
            }
        }
        if(state)
        {
            ROS_INFO("All laser scan data is inf or nan");
            return;
        }
    }

    //dividing the laser scan data into 4 parts, front, left, right, back, but the parts are rotated by 45 degrees
    std::vector<float> front, left, right, back, s_front, e_front;
    for(int i = 0; i < laser_msg->ranges.size(); i++)
    {
        if(i > laser_msg->ranges.size()/8 && i < laser_msg->ranges.size()*3/8)
        {
            left.push_back(laser_msg->ranges[i]);
        }
        else if(i > laser_msg->ranges.size()*3/8 && i < laser_msg->ranges.size()*5/8)
        {
            back.push_back(laser_msg->ranges[i]);
        }
        else if(i > laser_msg->ranges.size()*5/8 && i < laser_msg->ranges.size()*7/8)
        {
            right.push_back(laser_msg->ranges[i]);
        }
        else if (i > laser_msg->ranges.size()*7/8)
        {
            s_front.push_back(laser_msg->ranges[i]);
        }
        else if (i < laser_msg->ranges.size()/8)
        {
            e_front.push_back(laser_msg->ranges[i]);
        }
    }

    //sum the front parts
    for(int i = 0; i < s_front.size(); i++)
    {
        front.push_back(s_front[i]);
    }
    for(int i = 0; i < e_front.size(); i++)
    {
        front.push_back(e_front[i]);
    }

    //define the work_part
    std::vector<float> work_part;
    float skew;
    if (side == 1.0)
    {
        work_part = left;
        skew = 9*M_PI/4;
    }
    else
    {
        work_part = right;
        skew = 5*M_PI/4;
    }

    //publish the laser scan data
    publish_scans(laser_msg, left, right, front, back);



    //remove ranges bigger than max_range from the work_part
    for(int i = 0; i < work_part.size(); i++)
    {
        if(work_part[i] > max_range)
        {
            work_part[i] = INFINITY;
        }
    }

    //convert the range from work_part data to x and y data
    std::vector<float> x_data_work_part, y_data_work_part;
    for (int i = 0; i < work_part.size(); i++) {
        x_data_work_part.push_back(work_part[i]*cos(laser_msg->angle_min + skew + i*laser_msg->angle_increment));
        y_data_work_part.push_back(work_part[i]*sin(laser_msg->angle_min + skew + i*laser_msg->angle_increment));
    }

    //check opening
    if(!corrected)
        corrected = heading_correction(work_part);

    //break callback if not corrected
    if(!corrected)
    {
        return;
    }

    //compute the middle line of the work_part part scan curve data
    curve_fit_data line_work_part = curve_fit(x_data_work_part, y_data_work_part);
    double error_angle = compute_angle(line_work_part.x_data, line_work_part.y_data);

    //print the error angle
    ROS_INFO("Error angle: %f", error_angle);


    //compute the distance to the wall
    double distance_to_wall = sqrt(pow(line_work_part.x_data[line_work_part.x_data.size()/2], 2) + pow(line_work_part.y_data[line_work_part.y_data.size()/2], 2));

    //print the distance to the wall
    ROS_INFO("Distance to wall: %f", distance_to_wall);


    if(end_state)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        pub.publish(msg);
        sleep(1);
        //reset ros node
        system("rosnode kill /follow_wall");
        sleep(0.1);
        system("rosrun  turtlebot3_control follow_wall");
        return;
    }
    else
    {
        //mantain minimum distance to wall
        if(distance_to_wall < minimum_distance_to_wall)
        {
            Kp_wall = -Kp_wall;
        }
        
        //compute the angular velocity
        double angular_velocity = Kp_angular*error_angle + Kp_wall*distance_to_wall*side;
        
        // //calculate the line_work_part length
        double line_work_part_length = sqrt(pow(line_work_part.x_data[line_work_part.x_data.size()-1] - line_work_part.x_data[0], 2) + pow(line_work_part.y_data[line_work_part.y_data.size()-1] - line_work_part.y_data[0], 2));

        //print the line_work_part length
        ROS_INFO("Line work_part length: %f", line_work_part_length);

        //compute the linear velocity with a P controller to the length of the line_work_par
        double linear_velocity = Kp_linear - Kp_wall_linear/distance_to_wall;
        // double linear_velocity = Kp_linear*line_work_part_length;


        //publish the angular and linear velocity
        geometry_msgs::Twist msg;
        msg.linear.x = linear_velocity;
        msg.angular.z = angular_velocity;
        pub.publish(msg);

    }
   
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_wall");
    ros::NodeHandle nh;

    sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
    sub_line = nh.subscribe<sensor_msgs::LaserScan>("/extracted_line", 1, lineCallback);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_laser_back = nh.advertise<sensor_msgs::LaserScan>("/wall_tracking/back", 1);
    pub_laser_right = nh.advertise<sensor_msgs::LaserScan>("/wall_tracking/right", 1);
    pub_laser_front = nh.advertise<sensor_msgs::LaserScan>("/wall_tracking/front", 1);
    pub_laser_left = nh.advertise<sensor_msgs::LaserScan>("/wall_tracking/left", 1);

    pub_line = nh.advertise<visualization_msgs::Marker>("/wall_tracking/line", 1);
    pub_end = nh.advertise<std_msgs::Bool>("/end_goal", 1);

    //init dynamic reconfigure
    dynamic_reconfigure::Server<dynamic_tt::RobotMissionConfig> server;
    dynamic_reconfigure::Server<dynamic_tt::RobotMissionConfig>::CallbackType f;
    f = boost::bind(&dynamic_callback, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}