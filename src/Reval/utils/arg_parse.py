import argparse

class command:
    parser = argparse.ArgumentParser(description='Reval is an open-source framework to evaluate the performace of Robotics platforms. Currently it only supports Husky platform. The useres can evalute the performance of a mission for a given gazebo envirnoment (or on their own gazebo envirnment) for different configurations in an automated fashion and log the results. Reveal records the rosbag and evalutes all ros topics from the rosbag file.',
                                    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-v', '-viz', help='Turn on/off visualization of gazebo and rviz', default='On', metavar='')
    parser.add_argument('-e', '-epoch', help='Number of data-points to be recorded', type=int, default=1, metavar='')
    args = parser.parse_args()   