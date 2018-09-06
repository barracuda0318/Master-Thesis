 //Lidar + ROS Publisher + servo, work!
#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>

LIDARLite myLidarLite;
Servo myservo;

const int degree_increment = 16; //degree
const int measureTime_increment = degree_increment * 10; //ms
const int num_readings = 160/degree_increment;
const double pi = 3.141592;
// variable to store the servo position
int pos = degree_increment; 

//ROS node handle
ros::NodeHandle  nh;

// ROS Serial Laser scan message definition
sensor_msgs::LaserScan scan;
// definition of the ROS publisher for the laser scan data
ros::Publisher scan_pub("scan", &scan);
// Frame ID used in the ROS topics
char frameid[] = "/laser_lite_v3";
float ranges[num_readings];

void setup()
{
  Serial.begin(57600);
  // Set configuration to default and I2C to 400 kHz  
  myLidarLite.begin(0, true); 
  // Change this number to try out alternate configurations
  myLidarLite.configure(0); 
  myservo.attach(9);
  /* ROS related */
  nh.initNode();
  nh.advertise(scan_pub);
  
  scan.angle_min = degree_increment*pi/180.0;//2 degree
  scan.angle_max = 160.0*pi/180.0;//160 degree
  scan.angle_increment = degree_increment*pi/180.0;
  scan.time_increment = measureTime_increment/1000.0;
  scan.scan_time = num_readings*measureTime_increment/1000.0;
  scan.range_min = 0.01;
  scan.range_max = 40.0;
  scan.ranges_length = num_readings;
}

void loop()
{
  scan.header.stamp = nh.now();
  scan.header.frame_id = frameid;
  
  // goes from 2 degrees to 160 degrees
  // in steps of 2 degree
  for(pos = degree_increment; pos <= 160; pos += degree_increment)
  {  
      myservo.write(pos);   
      delay(measureTime_increment);                    
      ranges[pos/degree_increment-1] = myLidarLite.distance()/100.0;
      nh.spinOnce(); 
  }  
  scan.ranges = ranges;
  scan_pub.publish(&scan);
  
  nh.spinOnce(); 
  
  // goes from 160 degrees to 2 degrees
  // in steps of 2 degree
  for(pos = 160; pos > 0; pos -= degree_increment)
  {   
      myservo.write(pos);          
      delay(measureTime_increment);                    
      ranges[pos/degree_increment-1] = myLidarLite.distance()/100.0;
      nh.spinOnce(); 
  }
  scan.ranges = ranges;
  scan_pub.publish(&scan);
  
  nh.spinOnce();  
}


