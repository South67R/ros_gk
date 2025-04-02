#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <map>
#define FIXED_SPEED 0.1
#define FIXED_TURN  0.3
const std::string msg =
"Reading from the keyboard and Publishing to Twist!\n"
"---------------------------\n"
"Moving around:\n"
"        W    \n"
"    A   S   D\n"
"        X    \n"
"CTRL-C to quit\n";
struct Move {
  double x;
  double y;
  double z;
  double th;
};
std::map<char, Move> moveBindings = {
    {'w', {1, 0, 0, 0}},
    {'a', {0, 0, 0, 1}},
    {'s', {0, 0, 0, 0}},
    {'d', {0, 0, 0, -1}},
    {'x', {-1, 0, 0, 0}}
};
int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
std::string vels(double speed, double turn)
{
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "currently:\tspeed %.2f\tturn %.2f", speed, turn);
    return std::string(buffer);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_keyboard");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    double speed = FIXED_SPEED;
    double turn  = FIXED_TURN;
    double x = 0, y = 0, z = 0, th = 0;
    std::cout << msg << std::endl;
    std::cout << vels(speed, turn) << std::endl;
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok())
    {
        int c = getch();
        std::cout << "Key pressed: " << c << " (" << static_cast<char>(c) << ")" << std::endl;
        if (moveBindings.find(c) != moveBindings.end())
        {
            x  = moveBindings[c].x;
            y  = moveBindings[c].y;
            z  = moveBindings[c].z;
            th = moveBindings[c].th;
        }
        else
        {
            x = y = z = th = 0;
            if (c == 3)
                break;
        }
        geometry_msgs::Twist twist;
        twist.linear.x  = x * speed;
        twist.linear.y  = y * speed;
        twist.linear.z  = z * speed;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;
        pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }
    geometry_msgs::Twist twist;
    twist.linear.x  = 0;
    twist.linear.y  = 0;
    twist.linear.z  = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    pub.publish(twist);
    return 0;
}
