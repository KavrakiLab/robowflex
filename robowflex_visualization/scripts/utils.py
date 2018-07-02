#!/usr/bin/env python
''' 
Simple functions you shouldn't have to worry about.
Copied as much as possible from `robowflex_library`, and should stay updated with that.
'''

import logging
import os.path
import rospkg
import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

def resolvePackage(path):
    """
    Resolves `package://` URLs to their canonical form. The path does not need
    to exist, but the package does. Can be used to write new files in packages.
    Returns "" on failure.
    """
    if not path:
        return ''
    
    PREFIX = 'package://'
    if PREFIX in path:
        path = path[len(PREFIX):] # Remove 'package://'
        if '/' not in path:
            package_name = path
            path = ''
        else:   
            package_name = path[:path.find('/')]
            path = path[path.find('/'):]
        rospack = rospkg.RosPack()
        package_path1 = rospack.get_path(package_name)
    else:
        package_path1 = ''

    return os.path.realpath(package_path1 + path)

def resolvePath(path):
    """
    Resolves `package://` URLs and relative file paths to their canonical form.
    Returns "" on failure.
    """
    full_path = resolvePackage(path)
    if not os.path.exists(full_path):
        logging.warn('File {} does not exist'.format(full_path))
        return ''
    return full_path

        
def read_yaml_data(file_name):
    ''' Returns the yaml data structure of the data stored. '''
    full_name = resolvePath(file_name)
    if not full_name:
        logging.warn('Cannot open {}'.format(file_name))
        return None
    with open(full_name) as input_file:
        return yaml.load(input_file.read(), Loader=Loader)