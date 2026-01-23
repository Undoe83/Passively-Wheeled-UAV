#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sample_waypoints
{

    nav_msgs::msg::Path point()
    {
        double h = 0.0;
        nav_msgs::msg::Path waypoints;
        geometry_msgs::msg::PoseStamped pt;

        pt.pose.position.x = 0.0;
        pt.pose.position.y = 0.0;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = -100.0;
        pt.pose.position.y = 0;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = 0.0;
        pt.pose.position.y = 0.0;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        return waypoints;
    }

    nav_msgs::msg::Path circle()
    {
        double h = 1.0;
        double scale = 5.0;
        nav_msgs::msg::Path waypoints;
        geometry_msgs::msg::PoseStamped pt;
        tf2::Quaternion q;

        q.setRPY(0, 0, 0); // Roll, Pitch, Yaw
        pt.pose.orientation = tf2::toMsg(q);

        pt.pose.position.x = scale * 2.5;
        pt.pose.position.y = scale * -1.2;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 5.0;
        pt.pose.position.y = scale * -2.4;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 5.0;
        pt.pose.position.y = scale * 0.0;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 2.5;
        pt.pose.position.y = scale * -1.2;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 0.0;
        pt.pose.position.y = scale * -2.4;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 0.0;
        pt.pose.position.y = scale * 0.0;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 2.5;
        pt.pose.position.y = scale * -1.2;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 5.0;
        pt.pose.position.y = scale * -2.4;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 5.0;
        pt.pose.position.y = scale * 0.0;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 2.5;
        pt.pose.position.y = scale * -1.2;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 0.0;
        pt.pose.position.y = scale * -2.4;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        pt.pose.position.x = scale * 0.0;
        pt.pose.position.y = scale * 0.0;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);

        return waypoints;
    }

    nav_msgs::msg::Path eight()
    {
        double r = 3.0;
        double h = 0.0;
        int segments = 30;
        
        nav_msgs::msg::Path waypoints;
        geometry_msgs::msg::PoseStamped pt;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        pt.pose.orientation = tf2::toMsg(q);
        
        double center1_x = 0;
        double center1_y = r;

        // pt.pose.position.x = 0.0;
        // pt.pose.position.y = 0.0;
        // pt.pose.position.z = 0.1;
        // waypoints.poses.push_back(pt);
        
        for (int i = 0; i <= segments; i++)
        {
            double angle = -M_PI / 2 - i * 2 * M_PI / segments;
            pt.pose.position.x = center1_x + r * cos(angle);
            pt.pose.position.y = center1_y + r * sin(angle);
            // pt.pose.position.z = h;
            pt.pose.position.z = h - (h / 2) * cos(2 * M_PI * i / segments);
            waypoints.poses.push_back(pt);
        }
        
        double center2_x = 0;
        double center2_y = -r;
        
        for (int i = 0; i <= segments; i++)
        {
            double angle = M_PI / 2 - i * 2 * M_PI / segments;
            pt.pose.position.x = center2_x + -r * cos(angle);
            pt.pose.position.y = center2_y + r * sin(angle);
            // pt.pose.position.z = h;
            pt.pose.position.z = h - (h / 2) * cos(2 * M_PI * i / segments);
            waypoints.poses.push_back(pt);
        }
        
        return waypoints;   
    }
} // namespace sample_waypoints

#endif // SAMPLE_WAYPOINTS_H