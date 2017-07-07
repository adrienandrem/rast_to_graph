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
from osgeo import gdal
from osgeo import ogr
from osgeo import osr
from osgeo import gdal_array
from osgeo import gdalconst
from collections import defaultdict
from datetime import datetime
import math

#Timer to show processing time
class Timer():
  startTimes=dict()
  stopTimes=dict()

  @staticmethod
  def start(key = 0):
    Timer.startTimes[key] = datetime.now()
    Timer.stopTimes[key] = None

  @staticmethod
  def stop(key = 0):
    Timer.stopTimes[key] = datetime.now()

  @staticmethod
  def show(key = 0):
    if key in Timer.startTimes:
      if Timer.startTimes[key] is not None:
        if key in Timer.stopTimes:
          if Timer.stopTimes[key] is not None:
            delta = Timer.stopTimes[key] - Timer.startTimes[key]
            print delta

class Graph ():
    def __init__(self):
        self.nodes=set()
        self.edges=defaultdict(list)
        self.weight = {}
    
    def add_nodes(self, id):
        self.nodes.add(id)
    
    def add_edge(self, beg, end, w):
        self.edges[beg].append(end)
        self.weight[(beg,end)] = w
    
def imp_raster():
    print 'ENTER input raster path'
    iFile_name=raw_input()

    # Open the input raster to retrieve values in an array
    data = gdal.Open(iFile_name,1)
    proj = data.GetProjection()
    scr = data.GetGeoTransform()
    resolution = scr[1]

    band=data.GetRasterBand(1)
    iArray=band.ReadAsArray()
    
    return iArray, scr, proj, resolution

def imp_init_point(gt):
    print 'ENTER input init point path'
    iFile_name=raw_input()
    
    init_list = []
    
    #Open the init point shapefile to project each feature in pixel coordinates
    ds=ogr.Open(iFile_name)
    lyr=ds.GetLayer()
    for feat in lyr:
        geom = feat.GetGeometryRef()
        mx,my=geom.GetX(), geom.GetY()

        #Convert from map to pixel coordinates.
        px = int((mx - gt[0]) / gt[1])
        py = int((my - gt[3]) / gt[5])
        init_list.append((px,py))
    
    #return the list of init point with x,y pixel coordinates
    return init_list

def imp_end_point(gt):
    print 'ENTER input end point path'
    iFile_name=raw_input()
    
    end_list = []
    
    #Open the end point shapefile to project each feature in pixel coordinates
    ds=ogr.Open(iFile_name)
    lyr=ds.GetLayer()
    src = lyr.GetSpatialRef()
    for feat in lyr:
        geom = feat.GetGeometryRef()
        mx,my=geom.GetX(), geom.GetY()

        #Convert from map to pixel coordinates.
        px = int((mx - gt[0]) / gt[1])
        py = int((my - gt[3]) / gt[5])
        end_list.append((px,py))
    
    #return the list of end point with x,y pixel coordinates + spatial ref to reproj point => id_to_coord()
    return end_list, src
    
    
def output_prep(src):
    #Initialize the shapefile output

    oFile_name=raw_input()
    oDriver=ogr.GetDriverByName("ESRI Shapefile")
    if os.path.exists(oFile_name):
        oDriver.DeleteDataSource(oFile_name)
    oDataSource=oDriver.CreateDataSource(oFile_name)
    
    #Create a LineString layer
    oLayer = oDataSource.CreateLayer("ridge",src,geom_type=ogr.wkbLineString)
    
    #Add two fields to store the col_id and the pic_id
    colID_field=ogr.FieldDefn("col_id",ogr.OFTString)
    picID_field=ogr.FieldDefn("pic_id",ogr.OFTString)
    oLayer.CreateField(colID_field)
    oLayer.CreateField(picID_field)
    
    return oFile_name
    
def rast_to_graph(rastArray, res) :
    G= Graph()
    
    
    [H,W] = rastArray.shape
    
    #Shifts to get every edges from each nodes. For now, based on 16 direction like :
    #     | 11|   | 10|
    #  ---|---|---|---|---
    #   12| 3 | 2 | 1 | 9
    #  ---|---|---|---|---
    #     | 4 | 0 | 8 |
    #  ---|---|---|---|---
    #   13| 5 | 6 | 7 | 16
    #  ---|---|---|---|---
    #     | 14|   | 15|
    
    shift = [( 0,  0),
             (-1,  1),
             (-1,  0),
             (-1, -1),
             ( 0, -1),
             ( 1, -1),
             ( 1,  0),
             ( 1,  1),
             ( 0,  1),
             (-1,  2),
             (-2,  1),
             (-2, -1),
             (-1, -2),
             ( 1, -2),
             ( 2, -1),
             ( 2,  1),
             ( 1,  2)]

    #Loop over each pixel to convert it into nodes
    for i in range(0,H) :
        for j in range(0,W) :
            #node id based on x and y pixel coordinates
            nodeName = "x"+str(i)+"y"+str(j)
            G.add_nodes(nodeName)

    #Loop over each pixel again to create every edges for each node    
    for i in range(0,H) :
        for j in range(0,W) :
            nodeBeg = "x"+str(i)+"y"+str(j)
            nodeBegValue= rastArray[i,j]
            for index in range(1,17) :
                x,y=shift[index]
                nodeEnd="x"+str(i+x)+"y"+str(j+y)
                try :
                    nodeEndValue= rastArray[i+x,j+y]
                    #Calculate cost on length + addcost based on slope percent
                    if index in [2,4,6,8] :
                        length = res
                    elif index in [1,3,5,7] :
                        length = res*math.sqrt(2)
                    else :
                        length = res*math.sqrt(res)
                    slope = math.fabs(nodeEndValue-nodeBegValue)/length*100
                    addcost=0
                    #max slope accepted in percent
                    max_slope_wanted= 10
                    if slope >10 :
                        #coeff to prevent steep path
                        coeff= 4
                        addcost=(slope-max_slope_wanted)*coeff
                    cost = length+addcost
                    
                    G.add_edge(nodeBeg,nodeEnd,cost)
                except IndexError :
                    continue
    return G
    
def dijkstra(graph, init, end_list):
    #change the end point coordinates to graph id
    end_name=[]
    for end_point in end_list :
        x,y=end_point
        end_name.append("x"+str(x)+"y"+str(y))
    
    #dict to get visited nodes and path
    visited = {init: 0}
    path = defaultdict(list)

    nodes = set(graph.nodes)

    #dijkstra algo
    while nodes: 
        min_node = None
        for node in nodes:
            if node in visited:
                if node in end_name :
                    finish = node
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node

        if min_node in end_name:
            break

        nodes.remove(min_node)
        current_weight = visited[min_node]

        for edge in graph.edges[min_node]:
            weight = current_weight + graph.weight[(min_node, edge)]
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge].append((min_node,weight))
    return path, finish

def id_to_coord(lcp,gt):
    #Reproj pixel coordinates to map coordinates
    coord_list = []
    for id in lcp :
        id=id[1:]
        px,py=id.split('y')
        px,py=int(px),int(py)
        
        #Convert from pixel to map coordinates.
        mx = px * gt[1] + gt[0] + gt[1]/2
        my = py * gt[5] + gt[3] + gt[5]/2
        
        coord_list.append((mx,my))
    #return the list of end point with x,y map coordinates
    return coord_list

def create_ridge(oFile,lcp, col, pic) :
    driver= ogr.GetDriverByName("ESRI Shapefile")
    
    #Open the output shapefile
    iDataSource = driver.Open(oFile,1)
    iLayer = iDataSource.GetLayer()
    featDefn = iLayer.GetLayerDefn()
    
    #Initiate feature
    feat = ogr.Feature(featDefn)
    
    #Initiate feature geometry
    line = ogr.Geometry(ogr.wkbLineString)
    for coord in lcp :
        x,y = coord
        #Add new vertice to the linestring
        line.AddPoint(x,y)
    feat.SetGeometry(line)
    
    #Update the data field
    feat.SetField("col_id",col)
    feat.SetField("pic_id",pic)
    iLayer.CreateFeature(feat)
    feature = None
    iDataSource = None
    
def main() :
    #Main function
    print 'Import raster...'
    in_array, scr, proj, res = imp_raster()
    print 'Import raster done'
    
    
    print 'Import vector ...'
    beg_list = imp_init_point(scr)
    print '%s beg_point feature(s)' % len(beg_list)
    end_list, scr_shp = imp_end_point(scr)
    print '%s end_point feature(s)' % len(end_list)
    print 'Import vector done'

    print 'Name vector output'
    out_name=output_prep(scr_shp)
    
    time=Timer()
    time.start()
    
    print 'Convert rast to graph...'
    G = rast_to_graph(in_array, res)
    print 'Convert rast to graph done'

    print '%s nodes in the graph' % len(G.nodes)
    sum_nodes=0
    for node in G.nodes :
        sum_nodes += len(G.edges[node])
    print '%s edges in the graph' % sum_nodes

    #Begin to search least_cost path for each beg point
    for beg_point in beg_list :
        print 'Searching the least cost path... '
        x,y = beg_point
        beg_id = "x"+str(x)+"y"+str(y)
        
        path, end_id = dijkstra(G,beg_id,end_list)
        print 'Searching the least cost path done'
        
        act=end_id
        leastCostPath=[end_id]
        print 'Create the least cost path as OGR LineString...'
        while act!=beg_id :
            id,w=path[act][-1]
            act=id
            leastCostPath.append(id)
            
        coord_list = id_to_coord(leastCostPath,scr)
        
        create_ridge(out_name,coord_list,beg_id,end_id)
        print 'Create the least cost path as OGR LineString done'
        
    time.stop()
    print 'processing Time :'
    time.show()
            
if __name__ == '__main__':
    sys.exit(main())