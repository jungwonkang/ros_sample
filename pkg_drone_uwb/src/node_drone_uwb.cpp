#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "cls_display.h"


using namespace std;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
    ros::init(argc, argv, "node_drone_uwb");
    ros::NodeHandle handle_node;
    tf::TransformBroadcaster broadcaster;

    ros::Rate loop_rate(30);

    DisplayCLS  obj_disp;


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// setting - common
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    string frame_id = "odom";


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// setting - station
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double pos_x_station[4];
    double pos_y_station[4];
    double pos_z_station[4];

    double clr_r_station[4];
    double clr_g_station[4];
    double clr_b_station[4];

    string name_station [4];


    int totnum_station = 4;

    pos_x_station[0] = 0.0;     pos_y_station[0] = 0.0;     pos_z_station[0] = 0.5;
    pos_x_station[1] = 5.0;     pos_y_station[1] = 0.0;     pos_z_station[1] = 1.0;
    pos_x_station[2] = 0.0;     pos_y_station[2] = 5.0;     pos_z_station[2] = 1.5;
    pos_x_station[3] = 5.0;     pos_y_station[3] = 5.0;     pos_z_station[3] = 2.0;

    name_station[0] = "TD101";
    name_station[1] = "TD102";
    name_station[2] = "TD103";
    name_station[3] = "TD104";




    //// set info station for display
    obj_disp.set_info_station(totnum_station,
                              pos_x_station, pos_y_station, pos_z_station,
                              name_station);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    double pos_x_receiver[1];
    double pos_y_receiver[1];
    double pos_z_receiver[1];
    float color_r_receiver[1];
    float color_g_receiver[1];
    float color_b_receiver[1];


    const double degree = M_PI/180;

    // robot state
    double angle=0;






    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// set
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Publisher marker_pub = handle_node.advertise<visualization_msgs::Marker>("visualization_marker", 100);

    visualization_msgs::Marker marker_station_a[4];
    visualization_msgs::Marker marker_station_b[4];
    visualization_msgs::Marker marker_mea_line [4];



    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //// set marker for station (a)
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    obj_disp.set_marker_station_a(marker_station_a[0], frame_id,  0, 0);
    obj_disp.set_marker_station_a(marker_station_a[1], frame_id, 10, 1);
    obj_disp.set_marker_station_a(marker_station_a[2], frame_id, 20, 2);
    obj_disp.set_marker_station_a(marker_station_a[3], frame_id, 30, 3);


    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //// set marker for station (b)
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    obj_disp.set_marker_station_b(marker_station_b[0], frame_id,  1, 0);
    obj_disp.set_marker_station_b(marker_station_b[1], frame_id, 11, 1);
    obj_disp.set_marker_station_b(marker_station_b[2], frame_id, 21, 2);
    obj_disp.set_marker_station_b(marker_station_b[3], frame_id, 31, 3);


    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //// set marker for measurement line
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    obj_disp.set_marker_measurement_line(marker_mea_line[0], frame_id,  2);
    obj_disp.set_marker_measurement_line(marker_mea_line[1], frame_id, 12);
    obj_disp.set_marker_measurement_line(marker_mea_line[2], frame_id, 22);
    obj_disp.set_marker_measurement_line(marker_mea_line[3], frame_id, 32);


    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //// set tf
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = "base_link";


    geometry_msgs::Point pnt_a, pnt_b;


    while (ros::ok()) 
    {

        pos_x_receiver[0] = cos(angle)*2;
        pos_y_receiver[0] = sin(angle)*2;
        pos_z_receiver[0] = 3.0;


        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //// update marker for measurement line
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        //// clear
        marker_mea_line[0].points.clear();
        marker_mea_line[1].points.clear();
        marker_mea_line[2].points.clear();
        marker_mea_line[3].points.clear();


        pnt_a.x = pos_x_station[0];     pnt_a.y = pos_y_station[0];     pnt_a.z = pos_z_station[0];
        pnt_b.x = pos_x_receiver[0];    pnt_b.y = pos_y_receiver[0];    pnt_b.z = pos_z_receiver[0];
        marker_mea_line[0].points.push_back(pnt_a);
        marker_mea_line[0].points.push_back(pnt_b);


        pnt_a.x = pos_x_station[1];     pnt_a.y = pos_y_station[1];     pnt_a.z = pos_z_station[1];
        pnt_b.x = pos_x_receiver[0];    pnt_b.y = pos_y_receiver[0];    pnt_b.z = pos_z_receiver[0];
        marker_mea_line[1].points.push_back(pnt_a);
        marker_mea_line[1].points.push_back(pnt_b);


        pnt_a.x = pos_x_station[2];     pnt_a.y = pos_y_station[2];     pnt_a.z = pos_z_station[2];
        pnt_b.x = pos_x_receiver[0];    pnt_b.y = pos_y_receiver[0];    pnt_b.z = pos_z_receiver[0];
        marker_mea_line[2].points.push_back(pnt_a);
        marker_mea_line[2].points.push_back(pnt_b);


        pnt_a.x = pos_x_station[3];     pnt_a.y = pos_y_station[3];     pnt_a.z = pos_z_station[3];
        pnt_b.x = pos_x_receiver[0];    pnt_b.y = pos_y_receiver[0];    pnt_b.z = pos_z_receiver[0];
        marker_mea_line[3].points.push_back(pnt_a);
        marker_mea_line[3].points.push_back(pnt_b);




        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //// publish the marker
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        /*
        while(marker_pub.getNumSubscribers() < 1)
        {
            if( !ros::ok() ) { return -1; }

            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        */

        marker_pub.publish(marker_station_a[0]);
        marker_pub.publish(marker_station_a[1]);
        marker_pub.publish(marker_station_a[2]);
        marker_pub.publish(marker_station_a[3]);

        marker_pub.publish(marker_station_b[0]);
        marker_pub.publish(marker_station_b[1]);
        marker_pub.publish(marker_station_b[2]);
        marker_pub.publish(marker_station_b[3]);

        marker_pub.publish(marker_mea_line[0]);
        marker_pub.publish(marker_mea_line[1]);
        marker_pub.publish(marker_mea_line[2]);
        marker_pub.publish(marker_mea_line[3]);


        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //// update tf
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        odom_trans.header.stamp            = ros::Time::now();
        odom_trans.transform.translation.x = pos_x_receiver[0];
        odom_trans.transform.translation.y = pos_y_receiver[0];
        odom_trans.transform.translation.z = pos_z_receiver[0];
        odom_trans.transform.rotation      = tf::createQuaternionMsgFromYaw(angle+M_PI/2);


        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //// send tf
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        broadcaster.sendTransform(odom_trans);


        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //// sleep
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        loop_rate.sleep();


        //// temp - create new robot state
        angle += degree/4;

    }


    return 0;
}

