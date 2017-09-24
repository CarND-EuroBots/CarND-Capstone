#!/usr/bin/env python
from __future__ import print_function
import os
import argparse
import cv2

def parse_arguments():
    """ Parses the arguments sent from command line
        Returns: a struct with the value of the arguments
    """
    parser = argparse.ArgumentParser()

    parser.add_argument('-s', '--src', dest='src_path')
    parser.add_argument('-d', '--dest', dest='dest_path')

    return parser.parse_args()

def main():
    """ Main function """
    # Parse arguments
    args = parse_arguments()

    if not os.path.exists(args.dest_path):
        os.mkdir(args.dest_path)
    if not os.path.exists(os.path.join(args.dest_path, "red")):
        os.mkdir(os.path.join(args.dest_path, "red"))
    if not os.path.exists(os.path.join(args.dest_path, "green")):
        os.mkdir(os.path.join(args.dest_path, "green"))
    if not os.path.exists(os.path.join(args.dest_path, "none")):
        os.mkdir(os.path.join(args.dest_path, "none"))
    if not os.path.exists(os.path.join(args.dest_path, "yellow")):
        os.mkdir(os.path.join(args.dest_path, "yellow"))

    src_files = os.listdir(args.src_path)
    for filename in src_files:
        if not filename.startswith("."):
            path = os.path.join(args.src_path, filename)
            image = cv2.imread(path)
            cv2.imshow(filename, image)
            key = cv2.waitKey(0)
            if key == ord('r'):
                w = os.path.join(args.dest_path, "red", filename)
                os.rename(path, os.path.join(args.dest_path, "red", filename))
            elif key == ord('g'):
                os.rename(path, os.path.join(args.dest_path, "green", filename))
            elif key == ord('y'):
                os.rename(path, os.path.join(args.dest_path, "yellow", filename))
            elif key == ord('n'):
                os.rename(path, os.path.join(args.dest_path, "none", filename))
            elif key == ord('q'):
                return 0
            cv2.destroyAllWindows()
    return 0

if __name__ == "__main__":
    exit(main())
