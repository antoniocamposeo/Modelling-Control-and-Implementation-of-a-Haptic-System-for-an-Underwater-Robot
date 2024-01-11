#!/usr/bin/python

""" PYHTON SCRIPT TO CONVERT DATA OF MUVIC POSE INTO TUM FORMAT """
import pandas as pd
import numpy as np
import os
import time
from bagpy import bagreader


def conv_bag_file(topic):
    """
    extract data from bag file refers to topic and convert data in pandas dataframe
    string :param path_file: path of file bag into project directory
    string :param topic: name of topic format: '/name_topic'
    dataframe :return: pondas daframe
    """

    path_of_csv_file = b.message_by_topic(topic=topic)

    # Conversion csv into dataframe
    dataframe = pd.read_csv(path_of_csv_file)
    return dataframe


if __name__ == "__main__":
    # Insert paths of folders
    name_folder = 'log_5_1.bag'
    path_of_bag_file = "/home/antonio/Desktop/bagfile_folder"
    b = bagreader(path_of_bag_file + '/' + name_folder)
    # Insert the name of topic
    topic = ['/opencm904/motor_actual_position_raw','/opencm904/sensor_ft_data', '/motor_position']

    topic_data = []
    for topic in topic:
        topic_data.append(conv_bag_file(topic))

    # Insert the name of file txt to be saved
    # name_txt_file = '/home/antonio/PycharmProjects/from_posegt_to_tum/trajectory_files/TUM_muvic.txt'

    # topic_data_1 = conv_bag_file(path_of_bag_file, topic_name_1)
    # topic_data_2 = conv_bag_file(path_of_bag_file, topic_name_2)
