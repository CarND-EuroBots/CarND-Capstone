import os
import subprocess

ROOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
ROS_SRC_DIR = os.path.realpath(os.path.join(ROOT_DIR, 'ros', 'src'))
PYJOBS_DIR = os.path.realpath(os.path.join(ROOT_DIR, 'pyjobs'))

MAX_LINE_LENGTH = 80


def main():
    cmd = ['docker', 'run', '--rm=true',
           '--volume={}:{}'.format(ROOT_DIR, ROOT_DIR),
           'eurobots/carnd_capstone',
           'pycodestyle', '--max-line-length={}'.format(MAX_LINE_LENGTH),
           ROS_SRC_DIR, PYJOBS_DIR]

    try:
        subprocess.check_output(cmd, stderr=subprocess.STDOUT)
        output = 0

    except subprocess.CalledProcessError as e:
        print('>>> Please fix the following pycodestyle errors <<<')
        print(e.output)
        output = e.returncode

    finally:
        return output


if __name__ == '__main__':
    exit(main())
