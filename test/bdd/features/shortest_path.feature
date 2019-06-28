Feature: Trace path

    As a user
    I want to see the shortest path

    Scenario: Get shortest path
        Given a start point 5
          And an end point 6
          And an elevation raster data/dem-4x3.asc
          And a neighborhood of 8
        When I call ShortestPath
        Then I should see the path:
             | Cell |
             | 5    |
             | 6    |
