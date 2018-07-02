#!/usr/bin/env python
''' 
Simple functions you shouldn't have to worry about.
Copied as much as possible from `robowflex_library`, and should stay updated with that.
'''

import logging
import os.path
import rospkg

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
        
