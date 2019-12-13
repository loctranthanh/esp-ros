#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/String.h"

int getch()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);               // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

  int c = getchar(); // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return c;
}

void send_command(ros::Publisher &control, const char* command)
{
  std_msgs::String msg;
  msg.data = command;
  control.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle n;
  ros::Publisher controler_pub = n.advertise<std_msgs::String>("controler", 1000);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    int c = getch(); // call your non-blocking input function
    if (c == 'w') {
      send_command(controler_pub, "UP");
    } else if (c == 's') {
      send_command(controler_pub, "DOWN");
    } else if (c == 'd') {
      send_command(controler_pub, "RIGHT");
    } else if (c == 'a') {
      send_command(controler_pub, "LEFT");
    } else if (c == 'q') {
      send_command(controler_pub, "ROTATE_LEFT");
    } else if (c == 'e') {
      send_command(controler_pub, "ROTATE_RIGHT");
    } if (c == ' ') {
      send_command(controler_pub, "STOP");
    } else if (c == 'p') {
      break;
    }
  }
  return 0;
}
