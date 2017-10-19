"""
This tool filters a rosbag resulting from a Udacity submission,
to keep only the topics that are required to run our code on.

The benefit is that it removes the code output from the previous submission,
thus preventing data clash when running our code while replaying the rosbag.
"""
import os
import argparse
import subprocess

def filter_bag(input_bag, output_bag):
    """
    Filters the input_bag to keep only the necessary data.

    Inputs: input_bag - full path to the original rosbag
            output_bag - full path to the output rosbag
    """
    # Get directory where the rosbag is located
    rosbag_dir = os.path.dirname(input_bag)

    # Define topics to keep
    topics_to_keep = ['/image_color', '/current_velocity', '/current_pose',
                      '/vehicle/dbw_enabled']

    # Create filter expression
    filter_expression = ''
    for topic in topics_to_keep:
        if topic == topics_to_keep[-1]:
            filter_expression += 'topic == \'{}\''.format(topic)
        else:
            filter_expression += 'topic == \'{}\' or '.format(topic)

    # Create run command
    cmd = ['docker', 'run', '--rm=true', '--tty=true', '--interactive=true',
           '--user={}:{}'.format(os.getuid(), os.getgid()),
           '--volume={}:{}'.format(rosbag_dir, rosbag_dir),
           'eurobots/carnd_capstone:latest',
           '/bin/bash',
           '-c',
           'source /opt/ros/kinetic/setup.bash;'
           'rosbag filter {} {} "{}"'.format(input_bag, output_bag,
                                           filter_expression)]
    # Run it
    subprocess.check_call(cmd)

def parse_arguments():
    """
    Parses the arguments sent from command line
    Returns: a struct with the value of the arguments
    """
    parser = argparse.ArgumentParser()

    parser.add_argument('-i', '--input', dest='input_bag')
    parser.add_argument('-o', '--output', dest='output_bag')

    return parser.parse_args()


def main():
    """
    Runs main functionality
    """
    args = parse_arguments()
    input_bag = os.path.abspath(args.input_bag)
    output_bag = os.path.abspath(args.output_bag)

    filter_bag(input_bag, output_bag)

if __name__ == '__main__':
    main()
