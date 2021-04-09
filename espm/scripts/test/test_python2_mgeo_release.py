#!/usr/bin/env python

import os
import sys

current_path = os.path.dirname(os.path.realpath(__file__))
mgeo_lib_path = os.path.normpath(os.path.join(current_path, "../lib/mgeo/"))
# print('mgeo_lib_path: {}'.format(mgeo_lib_path))
sys.path.append(mgeo_lib_path)

import matplotlib.pyplot as plt
from class_defs import *


def test_load(test_data_path):
    test_case_name = "test_load"
    print("")
    print("Test starting: {}, arg: {}".format(test_case_name, test_data_path))

    mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(test_data_path)

    plt.figure()
    mgeo_planner_map.node_set.draw_plot(plt.gca())
    mgeo_planner_map.link_set.draw_plot(plt.gca())
    plt.show()

    print(">> Test Passed")


def test_main():

    test_data_path = []
    test_data_path.append(
        os.path.normpath(os.path.join(current_path, "test_data/kcity_prev/"))
    )
    test_data_path.append(
        os.path.normpath(os.path.join(current_path, "test_data/kcity_latest/"))
    )
    test_data_path.append(
        os.path.normpath(os.path.join(current_path, "test_data/katri/"))
    )
    test_data_path.append(
        os.path.normpath(os.path.join(current_path, "test_data/daejeon_kaist/"))
    )
    # test_data_path.append(
    #     os.path.normpath(os.path.join(current_path, 'test_data/kcity/')))
    # test_data_path.append(
    #     os.path.normpath(os.path.join(current_path, 'test_data/kcity/')))

    print("---------- Import Test START----------")
    for path in test_data_path:
        test_load(path)

    print("---------- Import Test ENDED----------")


test_main()