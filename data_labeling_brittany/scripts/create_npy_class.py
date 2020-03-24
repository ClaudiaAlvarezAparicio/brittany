#!/usr/bin/env python
# -*- coding: utf-8 -*-
# OS
import os
import sys
from os import listdir
from os.path import isfile, join
import time
# CV2
import cv2

import numpy as np
import math

LENGTH_MATRIX= 256

# Returns true if the directory exist, if not return false
def is_directory_correct(directory):
    return os.path.isdir(directory+"/")

# Returns two list with the files .npy containing in the directory received
def get_files_npy_class(directory):
    listNPY = []
    listNPY_class = []

    # For each item in the directory /raw
    for item in listdir(directory):
        # Test if item is a file
        if isfile(directory+item):
            # Test if the extension is .npy
            (nombreFichero, extension) = os.path.splitext(directory+item)
            if(extension == ".npy"):
                listNPY.append(directory+item)
                listNPY_class.append(directory+"/class/"+item)


    return listNPY, listNPY_class

# Returns tha class of the element
def get_element_class(fichero):
    clase = -1
    is_user0 = fichero.find("user0")
    is_user1 = fichero.find("user1")
    is_user2 = fichero.find("user2")
    is_user3 = fichero.find("user3")
    is_user4 = fichero.find("user4")
    if (is_user0 != -1):
        clase = 0
    elif (is_user1 != -1):
        clase = 1
    elif (is_user2 != -1):
        clase = 2
    elif(is_user3 != -1):
        clase = 3
    elif(is_user4!= -1):
        clase = 4

    return clase


def list_to_npy(longitud, element_class):
    npy_class = np.ndarray(longitud, dtype=np.uint8)

    for i in range(0, len(npy_class)):
        npy_class[i] = np.array([element_class])

    return npy_class

def class_npys(ruta):

    # Get path of npy files
    list_npys, list_npys_class = get_files_npy_class(ruta)

    for i in range(0, len(list_npys)):
        # Get class
        clase_npy = get_element_class(list_npys[i]);

        npy = np.load(list_npys[i])
        longitud=npy.shape[0]

        npy_class = list_to_npy(longitud, clase_npy)

        np.save(list_npys_class[i], npy_class)

if __name__ == '__main__':
    if (is_directory_correct(sys.argv[1])):
        class_npys(sys.argv[1])
    else:
        print("Not valid directory")