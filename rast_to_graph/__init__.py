#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 28 19:04:01 2019

@author: aandre
"""

import re

from datetime import datetime

from osgeo import ogr
from osgeo import gdal


class Timer():
  """Timer to show processing time"""
  startTimes = dict()
  stopTimes = dict()

  @staticmethod
  def start(key=0):
    Timer.startTimes[key] = datetime.now()
    Timer.stopTimes[key] = None

  @staticmethod
  def stop(key=0):
    Timer.stopTimes[key] = datetime.now()

  @staticmethod
  def show(key=0):
    if key in Timer.startTimes:
      if Timer.startTimes[key] is not None:
        if key in Timer.stopTimes:
          if Timer.stopTimes[key] is not None:
            delta = Timer.stopTimes[key] - Timer.startTimes[key]
            print delta


def curvature_option(cli_option):
    """Extract curvature constraint from command line option"""
    method, threshold = None, None

    if re.match(r'^[A-Z]+=\d+(?:\.\d+)?$', cli_option, re.I):
        m_str, t_str = cli_option.split('=')

        method, threshold = m_str.upper(), float(t_str)

    return method, threshold


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


def imp_init_point(filename, transform):
    """Read points"""

    init_list = []

    # Open the init point shapefile to project each feature in pixel coordinates
    datasource = ogr.Open(filename)
    layer = datasource.GetLayer()
    for feat in layer:
        geom = feat.GetGeometryRef()
        mx, my = geom.GetX(), geom.GetY()

        # Convert from map to pixel coordinates.
        px = int(( my - transform[3] + transform[5]/2)/transform[5])
        py = int(((mx - transform[0] - transform[1]/2)/transform[1]))

        init_list.append((px, py))

    # return the list of init point with x, y pixel coordinates
    return init_list, layer.GetSpatialRef()


def imp_end_point(filepath, transform):
    """Read points"""

    end_list = []

    # Open the end point shapefile to project each feature in pixel coordinates
    datasource = ogr.Open(filepath)
    layer = datasource.GetLayer()
    for feat in layer:
        geom = feat.GetGeometryRef()
        mx, my = geom.GetX(), geom.GetY()

        # Convert from map to pixel coordinates.
        px = int((my - transform[3] + transform[5]/2)/transform[5])
        py = int((mx - transform[0] + transform[1]/2)/transform[1])

        end_list.append((px, py))

    # return the list of end point with x,y pixel coordinates + spatial ref to reproj point => id_to_coord()
    return end_list, layer.GetSpatialRef()


def shortest_path(start, end, elevation, neighborhood):
    """Computes the shortest path between 2 cells"""

    path = [start, end]  # FIXME: Implement this

    return path
