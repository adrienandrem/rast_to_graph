Feature: Trace path

    As a user
    I want to see the shortest path

    Scenario: Get shortest path
        Given a start point 65
          And an end point 66
          And an elevation raster data/dem-12x11.asc
          And a neighborhood of 8
        When I call ShortestPath
        Then I should see the path:
             | Cell |
             | 65   |
             | 66   |
