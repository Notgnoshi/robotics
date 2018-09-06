#!/usr/bin/env python3
from multiprocessing import Process

from config_generator import ConfigGenerator
from kinematics_verifier import KinematicsVerifier
from workspace_generator import WorkspaceGenerator


def main():
    """A single main entry point for a collection of ROS nodes."""
    try:
        # Run each ROS node in its own process
        kv = Process(target=KinematicsVerifier.create_and_run, args=('kinematics_verifier',))
        cg = Process(target=ConfigGenerator.create_and_run, args=('config_generator',))
        wg = Process(target=WorkspaceGenerator.create_and_run, args=('workspace_generator',))

        # Make sure consumers are created before the producers
        kv.start()
        cg.start()
        wg.start()
    except KeyboardInterrupt:
        kv.join()
        cg.join()
        wg.join()

if __name__ == '__main__':
    main()
