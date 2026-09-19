#!/usr/bin/env python3
"""Host-only helper to run ROS humble xacro whose entry point metadata is broken."""
import sys
import xacro

if __name__ == '__main__':
    sys.argv = ['xacro'] + sys.argv[1:]
    xacro.main()
