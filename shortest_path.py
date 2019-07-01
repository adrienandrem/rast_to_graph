#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 28 18:11:31 2019

@author: aandre
"""

import os
import sys
import logging

import argparse

from rast_to_graph import shortest_path


def main(start, end, elevation, neighborhood):
    """Main function"""

    path = shortest_path(start, end, elevation, neighborhood)

    print(path)


if __name__ == "__main__":
    # execute only if run as a script

    PARSER = argparse.ArgumentParser(prog='shortest_path', usage='%(prog)s [options]')
    PARSER.add_argument('start', type=int, help='Start cell')
    PARSER.add_argument('end', type=int, help='End cell')
    PARSER.add_argument('elevation', help='DEM raster')
    PARSER.add_argument('neighborhood', type=int, choices=[4, 8, 24, 48], help='Neighbor cell count')
    ARGS = PARSER.parse_args()

    START, END = ARGS.start, ARGS.end
    ELEVATION = ARGS.elevation
    NEIGHBORHOOD = ARGS.neighborhood

    # Check input data
    if START < 0 or END < 0:
        logging.error('Cells are designated by positive integers: %s',
                      {'Start': START, 'End': END})
        sys.exit(1)
    if not os.path.isfile(ELEVATION):
        logging.error('Error reading file: %s', ELEVATION)
        sys.exit(1)

    main(START, END, ELEVATION, NEIGHBORHOOD)
