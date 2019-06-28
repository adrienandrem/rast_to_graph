#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 28 17:58:10 2019

@author: aandre
"""

from aloe import step, world
from aloe.tools import guess_types

from rast_to_graph import shortest_path


@step(r'a start point (\d+)')
def point_start(self, n):
    world.start = int(n)


@step(r'an end point (\d+)')
def point_end(self, n):
    world.end = int(n)


@step(r'an elevation raster ([^\s]+)')
def elevation(self, filename):
    world.elevation = filename


@step(r'a neighborhood of (\d+)')
def neighborhood(self, n):
    world.neighborhood = int(n)


@step(r'I call ShortestPath')
def path(self):
    world.path = shortest_path(world.start, world.end,
                               world.elevation, world.neighborhood)


@step(r'I should see the path')
def analyse_result(self):
    path = [cell['Cell'] for cell in guess_types(self.hashes)]

    assert world.path == path
