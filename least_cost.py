#!/usr/bin/env python
# -*- coding: utf-8 -*-
""""
/***************************************************************************
 least_cost.py

 Perform a least cost path with a raster conversion in graph

 Need : OsGeo library
                              -------------------
        begin                : 2017-07-07
        git sha              : 2017-07-07
        copyright            : (C) 2017 by Peillet Sebastien
        email                : peillet.seb@gmail.com
 ***************************************************************************/
"""

import os
import sys

import math
from datetime import datetime

from collections import defaultdict

import logging

import argparse

from osgeo import gdal
from osgeo import ogr
from osgeo import osr
from osgeo import gdal_array
from osgeo import gdalconst

from rast_to_graph import link_pattern


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


class Graph():

    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.weight = dict()

    def add_nodes(self, id):
        self.nodes.add(id)

    def add_edge(self, beg, end, w):
        self.edges[beg].append(end)
        self.weight[(beg, end)] = w

    def node_count(self):
        sum_nodes = 0
        for node in self.nodes:
            sum_nodes += len(self.edges[node])

        return sum_nodes


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


def imp_end_point(gt):

    print 'Enter input end point path:'
    iFile_name=raw_input()

    end_list = []

    # Open the end point shapefile to project each feature in pixel coordinates
    ds=ogr.Open(iFile_name)
    lyr=ds.GetLayer()
    src = lyr.GetSpatialRef()
    for feat in lyr:
        geom = feat.GetGeometryRef()
        mx,my=geom.GetX(), geom.GetY()

        #Convert from map to pixel coordinates.
        px = int(( my - gt[3] + gt[5]/2) / gt[5])
        py = int((mx - gt[0]+ gt[1]/2) / gt[1])
        end_list.append((px,py))

    # return the list of end point with x,y pixel coordinates + spatial ref to reproj point => id_to_coord()
    return end_list, src


def output_prep(filename, filename_wpt, src):
    """Initialize the output shapefile"""

    # Output path shapefile path
    oLineFile_name = filename
    oDriver = ogr.GetDriverByName("ESRI Shapefile")
    if os.path.exists(oLineFile_name):
        oDriver.DeleteDataSource(oLineFile_name)
    oDataSource = oDriver.CreateDataSource(oLineFile_name)

    # Create a LineString layer
    oLayer = oDataSource.CreateLayer("ridge", src, geom_type=ogr.wkbLineString)

    # Add two fields to store the col_id and the pic_id
    colID_field = ogr.FieldDefn("col_id", ogr.OFTString)
    picID_field = ogr.FieldDefn("pic_id", ogr.OFTString)
    oLayer.CreateField(colID_field)
    oLayer.CreateField(picID_field)

    # Output path shapefile path
    oPointFile_name = filename_wpt
    oDriver = ogr.GetDriverByName("ESRI Shapefile")
    if os.path.exists(oPointFile_name):
        oDriver.DeleteDataSource(oPointFile_name)
    oDataSource = oDriver.CreateDataSource(oPointFile_name)

    # Create a LineString layer
    oLayer = oDataSource.CreateLayer("point", src, geom_type=ogr.wkbPoint)
    ordreID_field = ogr.FieldDefn("ordre_id", ogr.OFTString)
    oLayer.CreateField(ordreID_field)

    return oLineFile_name, oPointFile_name


def rast_to_graph(rastArray, res):
    """Build a graph from raster"""
    G = Graph()

    [H,W] = rastArray.shape

    SHIFTS = link_pattern.SHIFTS16

    # Loop over each pixel to convert it into nodes
    for i in range(0,H):
        for j in range(0,W):
            # node id based on x and y pixel coordinates
            nodeName = "x"+str(i)+"y"+str(j)
            G.add_nodes(nodeName)

    # Loop over each pixel again to create every edges for each node
    for i in range(0,H):
        for j in range(0,W):
            nodeBeg = "x"+str(i)+"y"+str(j)
            nodeBegValue= rastArray[i,j]
            for index in range(1,17):
                x, y = SHIFTS[index]
                nodeEnd="x"+str(i+x)+"y"+str(j+y)
                try:
                    nodeEndValue= rastArray[i+x,j+y]
                    # Calculate cost on length + addcost based on slope percent
                    if index in [2,4,6,8]:
                        length = res
                    elif index in [1,3,5,7]:
                        length = res*math.sqrt(2)
                    else:
                        length = res*math.sqrt(res)
                    slope = math.fabs(nodeEndValue-nodeBegValue)/length*100
                    addcost=0
                    # max slope accepted in percent
                    max_slope_wanted= 10
                    if slope >max_slope_wanted:
                        # coeff to prevent steep path
                        coeff= 4
                        addcost=(slope-max_slope_wanted)*coeff
                    cost = length+addcost

                    G.add_edge(nodeBeg,nodeEnd,cost)
                except IndexError:
                    continue
    return G


def dijkstra(graph, init, end_list, out_point, scr):
    # Change the end point coordinates to graph id
    end_name=[]
    for end_point in end_list:
        x,y=end_point
        end_id = "x"+str(x)+"y"+str(y)
        if end_id != init:
            end_name.append(end_id)

    # dict to get visited nodes and path
    visited = {init: 0}
    path = defaultdict(list)

    nodes = set(graph.nodes)

    # Dijkstra algo
    min_node = None
    while nodes:
        if min_node not in end_name:
            min_node = None
            for node in nodes:
                if node in visited:
                    if node in end_name:
                        finish = node
                    if min_node is None:
                        min_node = node
                    elif visited[node] < visited[min_node]:
                        min_node = node


            createPoint(out_point,min_node,scr)
            nodes.remove(min_node)
            current_weight = visited[min_node]

            for edge in graph.edges[min_node]:
                if min_node in path:
                    pid,w = path[min_node][-1]
                    x1,y1 = id_to_coord(pid)
                    x2,y2 = id_to_coord(min_node)
                    x3,y3 = id_to_coord(edge)
                    az1 = math.degrees(math.atan2(x2 - x1, y2 - y1))
                    az2 = math.degrees(math.atan2(x3 - x2, y3 - y2))
                    if az1 < 0 and az2 > 0:
                        angle = math.fabs(az1)+az2
                    elif az1 > 0 and az2 < 0:
                        angle = math.fabs(az2)+az1
                    else:
                        angle = math.fabs(az1-az2)
                    if angle < -180:
                        angle = angle + 360
                    if angle > 180:
                        angle = angle - 360
                    if math.fabs(angle) <= 60:
                        weight = current_weight + graph.weight[(min_node, edge)]
                        if edge not in visited or weight < visited[edge]:
                            visited[edge] = weight
                            path[edge].append((min_node,weight))
                else:
                    weight = current_weight + graph.weight[(min_node, edge)]
                    if edge not in visited or weight < visited[edge]:
                        visited[edge] = weight
                        path[edge].append((min_node,weight))
        else:
            break
    return path, finish


def id_to_coord(id):
    """Coordinates from identifier"""
    id=id[1:]
    px,py=id.split('y')
    px,py=int(px),int(py)
    return px,py


def ids_to_coord(lcp, gt):
    # Reproj pixel coordinates to map coordinates
    coord_list = []
    for id in lcp:
        id=id[1:]
        px,py=id.split('y')
        px,py=int(px),int(py)

        # Convert from pixel to map coordinates.
        mx = py * gt[1] + gt[0] + gt[1]/2
        my = px * gt[5] + gt[3] + gt[5]/2

        coord_list.append((mx,my))
    # return the list of end point with x,y map coordinates
    return coord_list


def createPoint(oFile, node, gt):
    driver= ogr.GetDriverByName("ESRI Shapefile")

    # Open the output shapefile
    iDataSource = driver.Open(oFile,1)
    iLayer = iDataSource.GetLayer()
    featDefn = iLayer.GetLayerDefn()
    count = iLayer.GetFeatureCount()
    # Initiate feature
    feat = ogr.Feature(featDefn)
    px,py=id_to_coord(node)
    # Initiate feature geometry
    point = ogr.Geometry(ogr.wkbPoint)
    mx = py * gt[1] + gt[0] + gt[1]/2
    my = px * gt[5] + gt[3] + gt[5]/2
    point.AddPoint(mx,my)
    feat.SetGeometry(point)
    feat.SetField('ordre_id',count+1)
    iLayer.CreateFeature(feat)

    feature = None
    iDataSource = None


def create_ridge(oFile, lcp, col, pic):
    driver= ogr.GetDriverByName("ESRI Shapefile")

    # Open the output shapefile
    iDataSource = driver.Open(oFile,1)
    iLayer = iDataSource.GetLayer()
    featDefn = iLayer.GetLayerDefn()

    # Initiate feature
    feat = ogr.Feature(featDefn)

    # Initiate feature geometry
    line = ogr.Geometry(ogr.wkbLineString)
    for coord in lcp:
        x,y = coord
        # Add new vertice to the linestring
        line.AddPoint(x,y)
    feat.SetGeometry(line)

    # Update the data field
    feat.SetField("col_id",col)
    feat.SetField("pic_id",pic)
    iLayer.CreateFeature(feat)
    feature = None
    iDataSource = None


def main(points, elevation, path, waypoints):
    # Main function

    # Load elevation raster
    in_array, scr, proj, res = imp_raster(elevation)

    # Read points to link
    beg_list, scr_shp = imp_init_point(points, scr)
    logging.debug('%s points to link' % len(beg_list))

    # Prepare outputfiles
    out_line, out_point = output_prep(path, waypoints, scr_shp)

    time = Timer()
    time.start()

    logging.info('Build graph from raster...')
    G = rast_to_graph(in_array, res)
    logging.info('Built graph (%s nodes, %s edges).'
                 % (len(G.nodes), G.node_count()))

    # Begin to search least_cost path for each beg point
    i = 1
    for beg_point in beg_list:
        x, y = beg_point
        beg_id = "x" + str(x) + "y" + str(y)
        print 'Searching the least cost path for %s' % beg_id
        path, end_id = dijkstra(G, beg_id, beg_list, out_point, scr)
        print 'Searching the least cost path done'

        act = end_id
        leastCostPath = [end_id]
        print 'Create the least cost path as OGR LineString...'
        while act != beg_id:
            id, w = path[act][-1]
            act = id
            leastCostPath.append(id)
        filename = "lcp" + str(i) + ".txt"
        file = open(filename,"w")
        file.write(str(leastCostPath))
        file.close()
        i += 1
        coord_list = ids_to_coord(leastCostPath, scr)

        create_ridge(out_line, coord_list, beg_id, end_id)
        print 'Create the least cost path as OGR LineString done'

    time.stop()
    print 'Processing Time:'
    time.show()


if __name__ == '__main__':

    PARSER = argparse.ArgumentParser(prog='least_cost', usage='%(prog)s [options]')
    PARSER.add_argument('points', help='Shapefile of points to link')
    PARSER.add_argument('elevation', help='Elevation raster file')
    PARSER.add_argument('path', help='Output least cost path shapefile')
    PARSER.add_argument('waypoints',
                        help='Output least cost path points shapefile')
    ARGS = PARSER.parse_args()

    POINTS = ARGS.points
    ELEVATION = ARGS.elevation
    PATH = ARGS.path
    WAYPOINTS = ARGS.waypoints

    # Check input data
    if not os.path.isfile(POINTS):
        logging.error('Error reading file: %s', POINTS)
        sys.exit(1)
    if not os.path.isfile(ELEVATION):
        logging.error('Error reading file: %s', ELEVATION)
        sys.exit(1)

    main(POINTS, ELEVATION, PATH, WAYPOINTS)

    sys.exit(0)
