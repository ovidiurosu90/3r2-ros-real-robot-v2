//NOTE Check README.md for wheel reduction calculations

// Serial communication between ODrive and Teensy (Serial1)
// Teensy 4.1 - ODrive 3.6
// GND        - GND      (J3)
// GPIO0 RX   - GPIO1 TX (J3)
// GPIO1 TX   - GPIO2 RX (J3)

// Push button 1 - for initialising ODrive
// Teensy 4.1 - PBS-33B Tactile Push Button
// GND        - Pin 1
// GPIO5      - Pin 2

// #define LED_BUILTIN 13

#ifndef BUTTON1_PIN
#define BUTTON1_PIN 5
#endif

#include <ODriveArduino.h>

#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

ODriveArduino odrive1(Serial1);

//TODO Make it consistent with http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom and keep same units (meters over millimeters)

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

// ODrive Motor Axis configuration
int MOTOR_LEFT  = 1;
int MOTOR_RIGHT = 0;
int MOTOR_LEFT_SIGN  = -1;
int MOTOR_RIGHT_SIGN =  1;

// tf variables to be broadcast
double x = 0;
double y = 0;
double theta = 0;
double angularZ = 0;

char base_footprint[] = "base_footprint";
char odom[] = "odom";

// cmd_vel variables to be received to drive with
double demandx;
double demandz;

// timers for the sub-main loop
unsigned long currentMillis;
unsigned long previousMillis = 0; // set up timers
unsigned short int loopTime = 78; // 12.8 Hz, because /scan has 12.725 Hz (loopTime was 20 <=> 50 Hz)
double dt; // time since last execution in seconds

// ODrive init stuff
int button1Value;
int requested_state;

// output variables to drive the ODrive
double demandx_turns;
double demandz_turns;

// positions read from the ODrive
double posLeft;
double posRight;

// variables to work out the difference on each cycle
double posLeft_old;
double posRight_old;
double posLeft_diff;
double posRight_diff;
double posLeft_mm_diff;
double posRight_mm_diff;
double pos_average_mm_diff;
double pos_average_diff; // in m

// ** ROS callback & subscriber **
void velCallback(const geometry_msgs::Twist& vel)
{
     demandx = vel.linear.x;
     demandz = vel.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);


void ODriveInit()
{
    //Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 60000.0f << '\n';
    //Serial1 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
    //Serial1 << "w axis" << axis << ".motor.config.calibration_current " << 10.0f << '\n';
    // *** current limit and other parameers are set on the ODrive using the Odrive tool as above. ***


    // axis 1 = left motor
    //NOTE I commented out the following 2 commands as I pre-configured the motors in the odrivetool
    // requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
    // odrive1.run_state(MOTOR_LEFT, requested_state, true);

    // requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    // odrive1.run_state(MOTOR_LEFT, requested_state, true);

    requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
    odrive1.run_state(MOTOR_LEFT, requested_state, false); // don't wait


    // axis 0 = right motor
    //NOTE I commented out the following 2 commands as I pre-configured the motors in the odrivetool
    // requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
    // odrive1.run_state(MOTOR_RIGHT, requested_state, true);

    // requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    // odrive1.run_state(MOTOR_RIGHT, requested_state, true);

    requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
    odrive1.run_state(MOTOR_RIGHT, requested_state, false); // don't wait
}


void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUTTON1_PIN, INPUT_PULLUP); // ODrive init switch

    nh.getHardware()->setBaud(115200);
    nh.initNode();            // init ROS
    nh.subscribe(sub);        // subscribe to cmd_vel
    nh.advertise(odom_pub);
    broadcaster.init(nh);     // set up broadcaster

    Serial1.begin(115200);    // ODrive
    // Serial6.begin(115200);    // debug port using a USB-serial adapter (Serial-zero is in use by ros_serial)
}

void loop()
{
    nh.spinOnce(); // make sure we listen for ROS messages and activate the callback if there is one

    currentMillis = millis();
    if (currentMillis - previousMillis < loopTime) {  // run a loop every 10ms
        return;
    }
    dt = (currentMillis - previousMillis) / 1000.0;   // time since last execution in seconds
    previousMillis = currentMillis;                   // reset the clock to time it


    // -------------------------------- Initialise ODrive (set both axis in closed loop control)
    button1Value = digitalRead(BUTTON1_PIN);
    if (button1Value == LOW) {
        digitalWrite(LED_BUILTIN, HIGH);
        ODriveInit();
        digitalWrite(LED_BUILTIN, LOW);
    }


    // -------------------------------- Calculate velocities to be sent to ODrive, in order to drive the wheels
    // (James) forward0 = demandx * 83466; // convert m/s into counts/s
    // (James) forward1 = demandx * 83466; // convert m/s into counts/s
    demandx_turns = demandx * 10.185916;   // convert m/s into turns/s
    // (1 * 1000 / 628.318530 * 6.4 = 10.185916)

    // (James) turn0 = demandz * 15091;    // convert rads/s into counts/s
    // (James) turn1 = demandz * 15091;    // convert rads/s into counts/s
    demandz_turns = demandz * 1.762163;    // convert rads/s into turns/s
    // (1 * (346 * 3.141592 / 2) / 628.318530 * 6.4 / 3.141592 = 1.762163)
    // (used to be 1.833464 based on 3d model measurements)

    //NOTE One motor-encoder pair (left) is mounted facing the other way
    odrive1.SetVelocity(MOTOR_LEFT,  demandx_turns * MOTOR_LEFT_SIGN  + demandz_turns);
    odrive1.SetVelocity(MOTOR_RIGHT, demandx_turns * MOTOR_RIGHT_SIGN + demandz_turns);



    // -------------------------------- Get positions from Odrive, in order to compute /odom and /tf messages
    posLeft  = odrive1.GetPosition(MOTOR_LEFT)  * MOTOR_LEFT_SIGN;
    posRight = odrive1.GetPosition(MOTOR_RIGHT) * MOTOR_RIGHT_SIGN;

    // work out the difference on each loop, and bookmark the old value
    posLeft_diff = posLeft - posLeft_old;
    posLeft_old = posLeft;
    posRight_diff = posRight - posRight_old;
    posRight_old = posRight;

    // calc mm from encoder counts
    // (James) pos0_mm_diff = pos0_diff / 83.44;
    // (James) pos1_mm_diff = pos1_diff / 83.44;

    // calc mm from turns
    posLeft_mm_diff  = posLeft_diff  / 0.010185;
    posRight_mm_diff = posRight_diff / 0.010185;
    // (6.4 / 628.318530 = 0.010185)
    // (used to be 0.010185 based on 3d model measurements)

    // calc distance travelled based on average of both wheels
    pos_average_mm_diff = (posLeft_mm_diff + posRight_mm_diff) / 2; // difference in each cycle
    pos_average_diff = pos_average_mm_diff / 1000; // convert mm to m

    // calc angle or rotation to broadcast with tf
    //NOTE It's okay to use mm here as both parts are in the same unit
    angularZ = (posRight_mm_diff - posLeft_mm_diff) / 346; //NOTE The distance between wheels

    //NOTE Some sources recommend adding '* dt', but it won't work well for us (the scan readings would be delayed)
    //NOTE One source recommends adding '/ 2', but it won't work well for us (the scan readings would be delayed)
    //NOTE One source recommenda adding 'asin(', but it won't work at all for us (invalid quaternion in the transform)
    theta += angularZ;

    // Keep theta between -PI and PI
    //NOTE I tried multiple variations for the next area though it doens't seem to help with the scan readings being delayed
    if (theta > PI) {
        theta -= TWO_PI;
    }
    if (theta < -PI) {
        theta += TWO_PI;
    }

    x += pos_average_diff * cos(theta);
    y += pos_average_diff * sin(theta);


    // *** broadcast odom->base_footprint transform with tf ***
    geometry_msgs::TransformStamped t;
    t.header.stamp = nh.now();
    t.header.frame_id = odom;
    t.child_frame_id = base_footprint;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0;

    t.transform.rotation = tf::createQuaternionFromYaw(theta);

    broadcaster.sendTransform(t);


    // *** broadcast odom message ***
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    odom_msg.child_frame_id = base_footprint;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

    odom_msg.twist.twist.linear.x = pos_average_diff / dt;         // should match forward linear velocity
    odom_msg.twist.twist.linear.y = 0.0;                           // robot does not move sideways
    odom_msg.twist.twist.angular.z = angularZ / dt;                // should match angular velocity

    //NOTE Adding covariance according to https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/
    //NOTE If we don't manually set these, the whole matrix is filled with 0s
    //NOTE This is needed for <node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf">
    //NOTE This helps with <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization" clear_params="true">
    //TODO Investigate how to do it properly
    for (int i = 0; i < 36; i++) {
        if (i == 0 || i == 7 || i == 14) {
            odom_msg.pose.covariance[i] = .01;
        } else if (i == 21 || i == 28 || i== 35) {
            odom_msg.pose.covariance[i] += 0.1;
        } else {
            odom_msg.pose.covariance[i] = 0;
        }
    }

    //NOTE For debugging
    //odom_msg.pose.pose.position.x = demandx; // vel.linear.x
    //odom_msg.pose.pose.position.z = demandz; // vel.angular.z
    //odom_msg.pose.pose.orientation.x = (demandx_turns + demandz_turns) * MOTOR_RIGHT_SIGN; // setVelocity(0)
    //odom_msg.pose.pose.orientation.y = (demandx_turns + demandz_turns) * MOTOR_LEFT_SIGN;  // setVelocity(1)
    //odom_msg.twist.twist.linear.x = posLeft; // getPosition(1) * -1
    //odom_msg.twist.twist.linear.y = posRight; // getPosition(0)
    //odom_msg.twist.twist.angular.x = posLeft_diff; // pos0 - pos0_old
    //odom_msg.twist.twist.angular.y = posRight_diff; // pos1 - pos1_old
    //odom_msg.pose.pose.orientation.z = theta; // adding or substracting two_pi
    //odom_msg.pose.pose.orientation.w = pos_average_mm_diff; // (posLeft_mm_diff + posRight_mm_diff) / 2
    //odom_msg.twist.twist.linear.z = dt; // time since last execution in seconds

    odom_pub.publish(&odom_msg);
} // end loop

