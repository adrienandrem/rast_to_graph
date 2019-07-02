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
from collections import defaultdict

import math

from osgeo import ogr

import logging

import argparse

from rast_to_graph import imp_raster, imp_init_point
from rast_to_graph import curvature_option, Timer


CURVATURE_OPTIONS = ['ANGLE', 'RADIUS']


class Graph():

    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.slope_info = defaultdict(list)
        self.length = dict()
        self.slope = dict()
        self.weight = dict()

    def add_nodes(self, id):
        self.nodes.add(id)

    def add_edge(self, beg, end, w):
        self.edges[beg].append(end)
        self.weight[(beg, end)] = w

    def add_info(self, beg, end, length, slope):
        self.slope_info[beg].append(end)
        self.length[(beg, end)] = length
        self.slope[(beg, end)] = slope


def output_prep(filename, src):
    """Initialize the shapefile output"""

    oDriver=ogr.GetDriverByName("ESRI Shapefile")
    if os.path.exists(filename):
        oDriver.DeleteDataSource(filename)
    oDataSource=oDriver.CreateDataSource(filename)

    # Create a LineString layer
    oLayer = oDataSource.CreateLayer("ridge",src,geom_type=ogr.wkbLineString)

    # Add two fields to store the col_id and the pic_id
    colID_field=ogr.FieldDefn("col_id",ogr.OFTString)
    picID_field=ogr.FieldDefn("pic_id",ogr.OFTString)
    weight_field=ogr.FieldDefn("weight",ogr.OFTReal)
    oLayer.CreateField(colID_field)
    oLayer.CreateField(picID_field)
    oLayer.CreateField(weight_field)

    return filename


def out_point_prep(filename, src):

    oDriver=ogr.GetDriverByName("ESRI Shapefile")
    if os.path.exists(filename):
        oDriver.DeleteDataSource(filename)
    oDataSource=oDriver.CreateDataSource(filename)

    # Create a LineString layer
    oLayer = oDataSource.CreateLayer("point",src,geom_type=ogr.wkbPoint)
    ordreID_field=ogr.FieldDefn("ordre_id",ogr.OFTString)
    nodeID_field=ogr.FieldDefn("node_id",ogr.OFTString)
    weightID_field=ogr.FieldDefn("weight",ogr.OFTReal)
    pathID_field=ogr.FieldDefn("path_id",ogr.OFTString)
    previous_field = ogr.FieldDefn("previous",ogr.OFTString)
    oLayer.CreateField(ordreID_field)
    oLayer.CreateField(nodeID_field)
    oLayer.CreateField(weightID_field)
    oLayer.CreateField(pathID_field)
    oLayer.CreateField(previous_field)

    return filename


def rast_to_graph(rastArray, res, nb_edge, max_slope):
    G = Graph()

    [H, W] = rastArray.shape

    # Shifts to get every edges from each nodes. For now, based on 48 direction like:
    #     |   |   |   | 43|   | 42|   |   |   |
    #  ---|---|---|---|---|---|---|---|---|---|---
    #     |   |   |   |   |   |   |   |   |   |
    #  ---|---|---|---|---|---|---|---|---|---|---
    #     |   |   | 30| 29|   | 28| 27|   |   |
    #  ---|---|---|---|---|---|---|---|---|---|---
    #     |   | 31| 14| 13| 12| 11| 10| 26|   |
    #  ---|---|---|---|---|---|---|---|---|---|---
    #   44|   | 32| 15| 3 | 2 | 1 | 9 | 25|   | 41
    #  ---|---|---|---|---|---|---|---|---|---|---
    #     |   |   | 16| 4 | 0 | 8 | 24|   |   |
    #  ---|---|---|---|---|---|---|---|---|---|---
    #   45|   | 33| 17| 5 | 6 | 7 | 23| 40|   | 48
    #  ---|---|---|---|---|---|---|---|---|---|---
    #     |   | 34| 18| 19| 20| 21| 22| 39|   |
    #  ---|---|---|---|---|---|---|---|---|---|---
    #     |   |   | 35| 36|   | 37| 38|   |   |
    #  ---|---|---|---|---|---|---|---|---|---|---
    #     |   |   |   |   |   |   |   |   |   |
    #  ---|---|---|---|---|---|---|---|---|---|---
    #     |   |   |   | 46|   | 47|   |   |   |



    #         px  py
    shift = [( 0,  0), #0
             (-1,  1), #1
             (-1,  0), #2
             (-1, -1), #3
             ( 0, -1), #4
             ( 1, -1), #5
             ( 1,  0), #6
             ( 1,  1), #7
             ( 0,  1), #8
             (-1,  2), #9
             (-2,  2), #10
             (-2,  1), #11
             (-2,  0), #12
             (-2, -1), #13
             (-2, -2), #14
             (-1, -2), #15
             ( 0, -2), #16
             ( 1, -2), #17
             ( 2, -2), #18
             ( 2, -1), #19
             ( 2,  0), #20
             ( 2,  1), #21
             ( 2,  2), #22
             ( 1,  2), #23
             ( 0,  2), #24
             (-1,  3), #25
             (-2,  3), #26
             (-3,  2), #27
             (-3,  1), #28
             (-3, -1), #29
             (-3, -2), #30
             (-2, -3), #31
             (-1, -3), #32
             ( 1, -3), #33
             ( 2, -3), #34
             ( 3, -2), #35
             ( 3, -1), #36
             ( 3,  1), #37
             ( 3,  2), #38
             ( 2,  3), #39
             ( 1,  3), #40
             (-1,  5), #41
             (-5,  1), #42
             (-5, -1), #43
             (-1, -5), #44
             ( 1, -5), #45
             ( 5, -1), #46
             ( 5,  1), #47
             ( 1,  5)  #48
             ]

    slope_calc_coord  =    [( 0,  0),                                                                                                       #0
                            ([ [shift[2]  ,  shift[8]] ]),                                                                                  #1
                            ([ [shift[4]  ,  shift[8]] , [shift[3]  ,  shift[1]] ]),                                                        #2
                            ([ [shift[4]  ,  shift[2]] ]),                                                                                  #3
                            ([ [shift[6]  ,  shift[2]] , [shift[5]  ,  shift[3]] ]),                                                        #4
                            ([ [shift[4]  ,  shift[6]] ]),                                                                                  #5
                            ([ [shift[8]  ,  shift[4]] , [shift[7]  ,  shift[5]] ]),                                                        #6
                            ([ [shift[8]  ,  shift[6]] ]),                                                                                  #7
                            ([ [shift[2]  ,  shift[6]] , [shift[1]  ,  shift[7]] ]),                                                        #8
                            ([ [shift[2]  ,  shift[7]] , [shift[11] ,  shift[24]] , [shift[12],  shift[8]]  , [shift[1]  , shift[23]] ]),   #9
                            ([ [shift[11] ,  shift[9]] , [shift[2]  ,  shift[8]] ]) ,                                                       #10
                            ([ [shift[3]  ,  shift[8]] , [shift[2]  ,  shift[24]] , [shift[12],  shift[9]]  , [shift[13] , shift[1]]  ]),   #11
                            ([ [shift[13] ,  shift[11]], [shift[3]  ,  shift[1]]  , [shift[4] ,  shift[8]] ]) ,                             #12
                            ([ [shift[4]  ,  shift[1]] , [shift[3]  ,  shift[11]] , [shift[16],  shift[2]]  , [shift[15] , shift[12]] ]),   #13
                            ([ [shift[4]  ,  shift[2]] , [shift[15] ,  shift[13]] ]),                                                       #14
                            ([ [shift[5]  ,  shift[2]] , [shift[4]  ,  shift[12]] , [shift[16],  shift[13]] , [shift[17] , shift[3]]  ]),   #15
                            ([ [shift[17] ,  shift[15]], [shift[5]  ,  shift[3]]  , [shift[6] ,  shift[2]] ]) ,                             #16
                            ([ [shift[6]  ,  shift[3]] , [shift[20] ,  shift[4]]  , [shift[5] ,  shift[15]] , [shift[19] , shift[16]] ]),   #17
                            ([ [shift[6]  ,  shift[4]] , [shift[19] ,  shift[17]] ]),                                                       #18
                            ([ [shift[7]  ,  shift[4]] , [shift[6]  ,  shift[16]] , [shift[21],  shift[5]]  , [shift[20] , shift[17]] ]),   #19
                            ([ [shift[8]  ,  shift[4]] , [shift[5]  ,  shift[7]]  , [shift[21],  shift[19]] ]),                             #20
                            ([ [shift[8]  ,  shift[5]] , [shift[24] ,  shift[6]]  , [shift[7] ,  shift[19]] , [shift[23] , shift[20]] ]),   #21
                            ([ [shift[8]  ,  shift[6]] , [shift[23] ,  shift[21]] ]),                                                       #22
                            ([ [shift[1]  ,  shift[6]] , [shift[8]  ,  shift[20]] , [shift[24],  shift[21]] , [shift[9]  , shift[7]]  ]),   #23
                            ([ [shift[2]  ,  shift[6]] , [shift[7]  ,  shift[1]]  , [shift[9] ,  shift[23]] ]),                             #24
                            ([ [shift[2]  ,  shift[21]] , [shift[12]  ,  shift[7]] , [shift[1],  shift[22]] , [shift[11]  , shift[23]]  ]),   #25
                            ([ [shift[2]  ,  shift[22]] , [shift[12]  ,  shift[23]] , [shift[1],  shift[39]] , [shift[13]  , shift[7]]  ]),   #26
                            ([ [shift[3]  ,  shift[23]] , [shift[2]  ,  shift[40]] , [shift[13],  shift[24]] , [shift[14]  , shift[8]]  ]),   #27
                            ([ [shift[3]  ,  shift[24]] , [shift[15]  ,  shift[8]] , [shift[13],  shift[9]] , [shift[14]  , shift[1]]  ]),    #28
                            ([ [shift[4]  ,  shift[9]] , [shift[16]  ,  shift[1]] , [shift[3],  shift[10]] , [shift[15]  , shift[11]]  ]),    #29
                            ([ [shift[4]  ,  shift[10]] , [shift[16]  ,  shift[11]] , [shift[3],  shift[27]] , [shift[17]  , shift[1]]  ]),   #30
                            ([ [shift[5]  ,  shift[11]] , [shift[4]  ,  shift[28]] , [shift[17],  shift[12]] , [shift[18]  , shift[2]]  ]),   #31
                            ([ [shift[5]  ,  shift[12]] , [shift[19]  ,  shift[2]] , [shift[17],  shift[13]] , [shift[18]  , shift[3]]  ]),   #32
                            ([ [shift[6]  ,  shift[13]] , [shift[20]  ,  shift[3]] , [shift[5],  shift[14]] , [shift[19]  , shift[15]]  ]),   #33
                            ([ [shift[6]  ,  shift[14]] , [shift[20]  ,  shift[15]] , [shift[5],  shift[31]] , [shift[21]  , shift[3]]  ]),   #34
                            ([ [shift[6]  ,  shift[32]] , [shift[7]  ,  shift[15]] , [shift[21],  shift[16]] , [shift[22]  , shift[4]]  ]),   #35
                            ([ [shift[7]  ,  shift[16]] , [shift[23]  ,  shift[4]] , [shift[21],  shift[17]] , [shift[22]  , shift[5]]  ]),   #36
                            ([ [shift[8]  ,  shift[17]] , [shift[24]  ,  shift[5]] , [shift[7],  shift[18]] , [shift[23]  , shift[19]]  ]),   #37
                            ([ [shift[8]  ,  shift[18]] , [shift[24]  ,  shift[19]] , [shift[7],  shift[35]] , [shift[23]  , shift[36]]  ]),  #38
                            ([ [shift[1]  ,  shift[19]] , [shift[9]  ,  shift[20]] , [shift[8],  shift[36]] , [shift[10]  , shift[6]]  ]),    #39
                            ([ [shift[1]  ,  shift[20]] , [shift[9]  ,  shift[21]] , [shift[24],  shift[37]] , [shift[11]  , shift[6]]  ]),   #40
                            ([ [shift[12]  ,  shift[37]] , [shift[28]  ,  shift[22]] , [shift[27],  shift[39]]  ]),                           #41
                            ([ [shift[14]  ,  shift[25]] , [shift[32]  ,  shift[24]] , [shift[30],  shift[26]]  ]),                           #42
                            ([ [shift[16]  ,  shift[25]] , [shift[32]  ,  shift[10]] , [shift[30],  shift[26]]  ]),                           #43
                            ([ [shift[18]  ,  shift[29]] , [shift[36]  ,  shift[12]] , [shift[34],  shift[30]]  ]),                           #44
                            ([ [shift[20]  ,  shift[29]] , [shift[36]  ,  shift[14]] , [shift[35],  shift[31]]  ]),                           #45
                            ([ [shift[22]  ,  shift[33]] , [shift[40]  ,  shift[16]] , [shift[38],  shift[34]]  ]),                           #46
                            ([ [shift[24]  ,  shift[33]] , [shift[40]  ,  shift[18]] , [shift[39],  shift[35]]  ]),                           #47
                            ([ [shift[10]  ,  shift[37]] , [shift[28]  ,  shift[20]] , [shift[26],  shift[38]]  ])                            #48
                            ]

    nb_edge+=1
    # Loop over each pixel to convert it into nodes
    for i in range(0,H):
        for j in range(0,W):
            # node id based on x and y pixel coordinates
            nodeName = "x"+str(i)+"y"+str(j)
            G.add_nodes(nodeName)

    # Loop over each pixel again to create slope and length dictionnary
    for i in range(0,H):
        for j in range(0,W):
            nodeBeg = "x"+str(i)+"y"+str(j)
            nodeBegValue= rastArray[i,j]
            for index in range(1,nb_edge):
                x,y=shift[index]
                nodeEnd="x"+str(i+x)+"y"+str(j+y)
                try:
                    nodeEndValue= rastArray[i+x,j+y]
                    # Calculate cost on length + addcost based on slope percent
                    if index in [2,4,6,8]:
                        length = res
                    elif index in [1,3,5,7]:
                        length = res*math.sqrt(2)
                    elif index in [9,11,13,15,17,19,21,23]:
                        length = res*math.sqrt(res)
                    elif index in [10,14,18,22]:
                        length = 2*res*math.sqrt(2)
                    elif index in [12,16,20,24]:
                        length = 2*res
                    elif index in [25,28,29,32,33,36,37,40]:
                        length = res*math.sqrt(10)
                    elif index in [26,27,30,31,34,35,38,39]:
                        length = res*math.sqrt(13)
                    else:
                        length = res*math.sqrt(26)
                    slope = math.fabs(nodeEndValue-nodeBegValue)/length*100
                    # #max slope accepted in percent
                    # max_slope_wanted= 12
                    # if slope <= max_slope_wanted:
                    G.add_info(nodeBeg,nodeEnd,length,slope)
                except IndexError:
                    continue

    for i in range(0,H):
        for j in range(0,W):
            nodeBeg = "x"+str(i)+"y"+str(j)
            for index in range(1,nb_edge):
                x,y=shift[index]
                nodeEnd="x"+str(i+x)+"y"+str(j+y)
                if (i+x) > 0 and (j+y) > 0 and (i+x) < H and (j+y) < W:
                    try:
                        length = G.length[(nodeBeg, nodeEnd)]
                        slope = G.slope[(nodeBeg, nodeEnd)]
                        if slope <= max_slope:
                            coords_list = slope_calc_coord[index]
                            c_slope_list=[]
                            c_slope = None
                            count = 0

                            for coords in coords_list:
                                lx,ly = coords[0]
                                nodeLeft="x"+str(i+lx)+"y"+str(j+ly)
                                rx,ry = coords[1]
                                nodeRight="x"+str(i+rx)+"y"+str(j+ry)
                                if (i+lx) > 0 and (j+ly) > 0 and (i+rx) > 0 and (j+ry) > 0 and\
                                    (i+lx) < H and (j+ly) < W and (i+rx) < H and (j+ry) < W:
                                    c_slope_list.append(G.slope[nodeLeft,nodeRight])
                                count+=1
                            if len(c_slope_list) == count and count != 0:
                                c_slope = sum(c_slope_list) / len(c_slope_list)

                                pmax = 25
                                pmin = 60
                                larg = 4

                                if c_slope < pmax:
                                    assise = larg/2
                                else:
                                    assise = min(round((larg / 2*(1 + ((c_slope - pmax)/(pmin - pmax))**2)),2),larg)
                                talus  = assise**2 *larg * (c_slope/100) / 2 /(larg - (c_slope/100))
                                addcost = talus

                                cost = length * addcost + length * 1
                                G.add_edge(nodeBeg, nodeEnd, cost)
                    except IndexError:
                        continue

    return G


def dijkstra(graph, init, end_list, scr, method, threshold, out_point, nb_path):
    # change the end point coordinates to graph id
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

    # dijkstra algo
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

            if min_node != None:
                current_weight = visited[min_node]
                if min_node in path:
                    pid,w = path[min_node][-1]
                else:
                    pid = ''
                if out_point != None:
                    createPoint(out_point, min_node, scr, current_weight, nb_path, pid)
                nodes.remove(min_node)


                for edge in graph.edges[min_node]:
                    if method == 'angle':
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
                            if math.fabs(angle) <= threshold:
                                weight = current_weight + graph.weight[(min_node, edge)]
                                if edge not in visited or weight < visited[edge]:
                                    visited[edge] = weight
                                    path[edge].append((min_node,weight))
                        else:
                            weight = current_weight + graph.weight[(min_node, edge)]
                            if edge not in visited or weight < visited[edge]:
                                visited[edge] = weight
                                path[edge].append((min_node,weight))

                    if method == 'radius':
                        if min_node in path:
                            pid,w = path[min_node][-1]
                            x1,y1 = id_to_coord(pid)
                            x2,y2 = id_to_coord(min_node)
                            x3,y3 = id_to_coord(edge)

                            if min(x1,x3) <= x2 <= max(x1,x3) and min(y1,y3) <= y2 <= max(y1,y3):

                                mag_v1 = math.sqrt((x1-x2)**2+(y1-y2)**2)
                                mag_v2 = math.sqrt((x3-x2)**2+(y3-y2)**2)


                                if mag_v1 < mag_v2:
                                    x_v2 , y_v2 = (x3 - x2, y3 - y2)
                                    x3,y3 = x2+x_v2/mag_v2*mag_v1 ,y2+y_v2/mag_v2*mag_v1
                                elif mag_v2 < mag_v1:
                                    x_v2 , y_v2 = (x1 - x2, y1 - y2)
                                    x1,y1 = x2+x_v2/mag_v1*mag_v2 ,y2+y_v2/mag_v1*mag_v2

                                x_v1 , y_v1 = (x2 - x1, y2 - y1)
                                x_v1_ort , y_v1_ort = y_v1 , -x_v1
                                x_v2 , y_v2 = (x3 - x2, y3 - y2)
                                x_v2_ort , y_v2_ort = y_v2 , -x_v2

                                c_v1_ort = y_v1_ort*x1+(-x_v1_ort)*y1
                                c_v1_ort = -c_v1_ort
                                c_v2_ort = y_v2_ort*x3+(-x_v2_ort)*y3
                                c_v2_ort = -c_v2_ort

                                e = [-y_v1_ort,x_v1_ort,c_v1_ort]
                                f = [-y_v2_ort,x_v2_ort,c_v2_ort]
                                x4 , y4, colineaire = equationResolve(e,f)

                                if (x4 != None and y4 != None):
                                    dist1 = math.sqrt((x1-x4)**2+(y1-y4)**2)*5
                                    dist2 = math.sqrt((x3-x4)**2+(y3-y4)**2)*5

                                    if dist1 >= threshold:
                                        weight = current_weight + graph.weight[(min_node, edge)]
                                        if edge not in visited or weight < visited[edge]:
                                            visited[edge] = weight
                                            path[edge].append((min_node,weight))
                                elif colineaire == True:
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
                print 'no solution'
                finish = None
                break
        else:
            break
    return path, finish, visited


def equationResolve(e1, e2):
    determinant=e1[0]*e2[1]-e1[1]*e2[0]
    x , y = None,None
    colineaire = False
    if determinant != 0:
        x=(e1[2]*e2[1]-e1[1]*e2[2])/determinant
        y=(e1[0]*e2[2]-e1[2]*e2[0])/determinant
    else:
        colineaire = True
    return x, y, colineaire


def id_to_coord(id):
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


def create_ridge(oFile, lcp, col, pic, weight):
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
    feat.SetField("weight",weight)
    iLayer.CreateFeature(feat)
    feature = None
    iDataSource = None


def createPoint(oFile, node, gt, weight, nb_path, previous):
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
    feat.SetField('node_id',node)
    feat.SetField('weight',weight)
    feat.SetField('path_id', nb_path)
    feat.SetField('previous', previous)
    iLayer.CreateFeature(feat)

    iDataSource = None


def main(points, elevation,
         links, curvature, slope_max,
         path, waypoints):
    """Main function"""

    # Load elevation raster
    in_array, scr, proj, res = imp_raster(elevation)

    # Read points to link
    beg_list, scr_shp = imp_init_point(points, scr)
    logging.debug('%s points to link', len(beg_list))

    # Prepare output files
    out_line = output_prep(path, scr_shp)

    if waypoints:
        out_point = out_point_prep(waypoints, scr_shp)
    else:
        out_point = None

    method, threshold = curvature

    # Processing
    time = Timer()
    time.start()

    print 'Convert rast to graph...'
    G = rast_to_graph(in_array, res, links, slope_max)
    print 'Convert rast to graph done'

    print '%s nodes in the graph' % len(G.nodes)
    sum_nodes = 0
    for node in G.nodes:
        sum_nodes += len(G.edges[node])
    print '%s edges in the graph' % sum_nodes

    # Begin to search least_cost path for each beg point
    i = 0
    for beg_point in beg_list:
        x, y = beg_point
        beg_id = "x" + str(x) + "y" + str(y)
        print 'Searching the least cost path for %s' % beg_id
        path, end_id, visited = dijkstra(G, beg_id, beg_list, scr, method, threshold, out_point, i)
        i += 1
        print 'Searching the least cost path done'

        if end_id ins not None:
            act = end_id
            leastCostPath = [end_id]
            print 'Create the least cost path as OGR LineString...'
            while act != beg_id:
                id, w = path[act][-1]
                act = id
                leastCostPath.append(id)

            filename = "lcp" + str(i) + ".txt"
            file = open(filename, 'w')
            file.write(str(leastCostPath))
            file.close()

            filename = "path" + str(i) + ".txt"
            file = open(filename, 'w')
            file.write(str(path))
            file.close()

            coord_list = ids_to_coord(leastCostPath, scr)
            id, w = path[end_id][-1]
            create_ridge(out_line, coord_list, beg_id, end_id, w)
            print 'Create the least cost path as OGR LineString done'

    time.stop()
    print 'Processing Time:'
    time.show()


if __name__ == '__main__':

    PARSER = argparse.ArgumentParser(prog='least_terr_cost', usage='%(prog)s [options]')
    PARSER.add_argument('links', type=int, choices=[8, 24, 40, 48])
    PARSER.add_argument('curvature',
                        help='Maximum slope (RADIUS=30, ANGLE=45)')
    PARSER.add_argument('slope', type=float, help='Maximum slope (%)')
    PARSER.add_argument('points', help='Shapefile of points to link')
    PARSER.add_argument('elevation', help='Elevation raster file')
    PARSER.add_argument('path', help='Output least cost path shapefile')
    PARSER.add_argument('waypoints',
                        help='Output least cost path points shapefile')
    ARGS = PARSER.parse_args()

    POINTS = ARGS.points
    ELEVATION = ARGS.elevation

    LINKS = ARGS.links
    CURVATURE = ARGS.curvature
    SLOPE = ARGS.slope

    PATH = ARGS.path
    WAYPOINTS = ARGS.waypoints

    # Check input data
    if not os.path.isfile(POINTS):
        logging.error('Error reading file: %s', POINTS)
        sys.exit(1)
    if not os.path.isfile(ELEVATION):
        logging.error('Error reading file: %s', ELEVATION)
        sys.exit(1)
    curvature = curvature_option(CURVATURE)
    if not curvature[0]:
        logging.error('Wrong curvature option: %s', CURVATURE)
        sys.exit(1)

    sys.exit(main(POINTS, ELEVATION,
                  LINKS, curvature, SLOPE,
                  PATH, WAYPOINTS))
