#!/usr/bin/env python3
from multiprocessing import Process
from workspace_generator import WorkspaceGenerator
from config_generator import ConfigGenerator
from verifier import Verifier


def main():
    """A single main entry point for a collection of ROS nodes."""
    try:
        v = Process(target=Verifier.create_and_run)
        cg = Process(target=ConfigGenerator.create_and_run)
        wg = Process(target=WorkspaceGenerator.create_and_run,
                     kwargs=dict(loop=True),
                    )
        v.start()
        cg.start()
        wg.start()
    except KeyboardInterrupt:
        v.join()
        cg.join()
        wg.join()

if __name__ == '__main__':
    main()
