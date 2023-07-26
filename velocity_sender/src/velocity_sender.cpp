#include <velocity_sender/velocity_sender.hpp>
int main (int argc, char **argv)
{
ros::init(argc,argv,"velocity_sender");
ros::NodeHandle n;
velocity_node talker(n);
talker.driving();

return 0;
}