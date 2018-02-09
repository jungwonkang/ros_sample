// Jungwon Kang
// Aug 12 2017

#ifndef DISPLAYCLS_H
#define DISPLAYCLS_H

#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;


class DisplayCLS
{
  public:
      DisplayCLS();
      ~DisplayCLS();

  public:
      void  set_marker_station_a(visualization_msgs::Marker &Marker, string Frame_id, int Marker_id, int IDX_station);
      void  set_marker_station_b(visualization_msgs::Marker &Marker, string Frame_id, int Marker_id, int IDX_station);
      void  set_marker_measurement_line(visualization_msgs::Marker &Marker, string Frame_id, int Marker_id);

      void  set_info_station(int totnum, double* pos_x, double* pos_y, double* pos_z, string* name);



  private:
      int       m_totnum_station;

      double    m_pos_x_station[20];
      double    m_pos_y_station[20];
      double    m_pos_z_station[20];

      double    m_clr_r_station[20];   // 0.0 ~ 1.0
      double    m_clr_g_station[20];
      double    m_clr_b_station[20];

      string    m_name_station[20];

      uint32_t  m_shape_station_a;
      uint32_t  m_shape_station_b;


};

#endif // DISPLAYCLS_H
