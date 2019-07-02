# rast_to_graph

Some work inspired by research paper about graph:  
Stückelberger J.A., 2008,
[A Weighted-Graph Optimization Approach for Automatic Location of Forest Road Networks](https://www.research-collection.ethz.ch/handle/20.500.11850/9705)
([DOI](https://doi.org/10.3218/3217-8))
    
    
## Usage examples

Least cost path

    $ least_cost.py points.shp elevation.tif path.shp path-points.shp

Least cost path with earthwork consideration

    $ least_terr_cost.py points.shp elevation.tif path.shp path-points.shp
