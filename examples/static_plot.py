#!/usr/bin/env python3
"""
A minimal example of plotting in a background process.
"""
import multiprocessing

import matplotlib.pyplot as plt


def plot_a_graph():
    fig, axis = plt.subplots(1)
    plt.plot(range(10))
    print(multiprocessing.current_process().name, "Starting plot")
    plt.show()
    print(multiprocessing.current_process().name, "Finished plot")


job = multiprocessing.Process(target=plot_a_graph, args=())
job.start()

print(multiprocessing.current_process().name, "Main process")
