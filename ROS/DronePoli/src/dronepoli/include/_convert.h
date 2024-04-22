#pragma once

#include <math.h>
#include <tf/tf.h>

#define PI 3.1415926535

#include "GeographicLib/Geoid.hpp"
GeographicLib::Geoid _egm96("egm96-5");  // WARNING: not thread safe

double calc_geoid_height(double lat, double lon) {
    return _egm96(lat, lon);
}
double amsl_to_ellipsoid_height(double lat, double lon, double amsl) {
  return amsl + GeographicLib::Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon, double ellipsoid_height) {
  return ellipsoid_height + GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}

double compute_bearing(double P1_latitude, double P1_longitude, double P2_latitude, double P2_longitude)
{
  double Cur_Lat_radian = P1_latitude * (3.141592 / 180);
  double Cur_Lon_radian = P1_longitude * (3.141592 / 180);
  double Dest_Lat_radian = P2_latitude * (3.141592 / 180);
  double Dest_Lon_radian = P2_longitude * (3.141592 / 180);
  double radian_distance = 0;
  radian_distance = acos(sin(Cur_Lat_radian)
      * sin(Dest_Lat_radian) + cos(Cur_Lat_radian)
      * cos(Dest_Lat_radian) * cos(Cur_Lon_radian - Dest_Lon_radian));
  double radian_bearing = acos((sin(Dest_Lat_radian) - sin(Cur_Lat_radian)
      * cos(radian_distance)) /  (cos(Cur_Lat_radian) * sin(radian_distance)) );
  double true_bearing = 0;
  if (sin(Dest_Lon_radian - Cur_Lon_radian) < 0){
      true_bearing = radian_bearing * (180 / 3.141592);
      true_bearing = 360 - true_bearing;
  }
  else{
      true_bearing = radian_bearing * (180 / 3.141592);
  }
  return true_bearing;
}


QUAT convert_euler_to_quaternion(double radian){
  QUAT q;
  
  double yaw = radian;
  double pitch = 0, roll = 0;

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  q.w = cy * cp * cr + sy * sp * sr;

  return q;
}
double convert_quaternion_to_euler(double x, double y, double z, double w) // 쿼터니언 to 오일러
{
  double t0 = 2.0 * (w*x+y*z);
  double t1 = 1.0 - 2.0*(x*x+y*y);
  //roll_curr = atan2(t0,t1);
  double t2 = 2.0*(w*y-z*x);
  if(t2>1.0) t2=1.0;
  if(t2<-1.0) t2=-1.0;
  //pitch_curr = asin(t2);
  double t3 = 2.0*(w*z+x*y);
  double t4 = 1.0 - 2.0*(y*y+z*z);
  return atan2(t3,t4);
}