#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 28 19:04:01 2019

@author: aandre
"""

from osgeo import gdal


def imp_raster(filename):
    """Load raster data"""

    # Open the input raster to retrieve values in an array
    data = gdal.Open(filename, 1)
    proj = data.GetProjection()
    scr = data.GetGeoTransform()
    resolution = scr[1]

    band = data.GetRasterBand(1)
    iArray = band.ReadAsArray()

    return iArray, scr, proj, resolution


def shortest_path(start, end, elevation, neighborhood):
    """Computes the shortest path between 2 cells"""

    path = [start, end]  # FIXME: Implement this

    return path
