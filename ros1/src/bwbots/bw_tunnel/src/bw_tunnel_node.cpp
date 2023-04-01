
#include "bw_tunnel.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bw_tunnel");
    ros::NodeHandle nh;

    BwTunnel broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
