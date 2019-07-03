# -*- coding: utf-8 -*-
"""
Created on Wed Jul  3 08:10:52 2019

@author: aandre
"""

from rast_to_graph import imp_raster, imp_init_point


def test_point_to_pixel_0():
    """Geo to image coordinate transform"""
    filepath = 'test/unit/data/proj_points.shp'  # POINT(2.5 7.5)
    rasterpath = 'test/unit/data/2x2.asc'  # square 5m resolution
    expected = [(0, 0)]

    __, transform, __, __ = imp_raster(rasterpath)
    points, __ = imp_init_point(filepath, transform)

    assert points == expected
