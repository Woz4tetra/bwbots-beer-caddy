
#include "bw_description/bw_description.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bw_description");
    ros::NodeHandle nh;

    BwDescription broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
