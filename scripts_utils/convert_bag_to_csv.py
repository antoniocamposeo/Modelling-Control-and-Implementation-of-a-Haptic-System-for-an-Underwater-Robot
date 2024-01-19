#!/usr/bin/python

""" PYHTON SCRIPT TO CONVERT DATA OF MUVIC POSE INTO TUM FORMAT """

"""
Tips to use the code
- go in the folder of the file 
- run the code 
- python3 convert_bag_to_csv.py "folder-path-of-bag-files"
"""
import pandas as pd
import numpy as np
import os
import time
import argparse
import sys
from os import listdir
from os.path import isfile, join
from bagpy import bagreader


def create_arg_parser():
    # Creates and returns the ArgumentParser object
    parser = argparse.ArgumentParser(description='Insert the Path of bag files.')
    parser.add_argument('inputDirectory',
                    help='Path to the input directory.',type=str)

    return parser


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
    arg_parser = create_arg_parser()
    parsed_args = arg_parser.parse_args(sys.argv[1:])
    mypath = parsed_args.inputDirectory
    onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    print(onlyfiles)
    #print(parsed_args.inputDirectory)
    if os.path.exists(parsed_args.inputDirectory):
       print("File exist")
    # Insert the name of topic
    topic = ['/opencm904/motor_actual_position_raw','/opencm904/sensor_ft_data', '/motor_position']
    # Insert paths of folders
    path_of_bag_file = mypath
    for file in onlyfiles:
        name_folder = file
        b = bagreader(path_of_bag_file + '/' + name_folder)
        topic_data = []
        for topic in topic:
            topic_data.append(conv_bag_file(topic))