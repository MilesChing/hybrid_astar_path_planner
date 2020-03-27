# Hybrid A\* Path Planner

This project is a pure version (without ROS environment) of a [path\_planner](https://github.com/karlkurzer/path_planner) implemented by [Karl Kurzer](https://github.com/karlkurzer).

## Run this code

- install [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/)
- install [OpenCV](https://opencv.org/)
- cmake && make

## Notice

This repo is actually a temporally modified code of mine. Its planning results may be different with the original version by Karl Kurzer. 

In this repo, I:
- change the type of obstacle map to 2D uchar array
- remove the ROS dependence and create a console program which reads an image and convert it to the obstacle map
- remove the 2D A* which calculate the heuristic and add a shortest path algorithm to init a distance look-up once obstacle map is updated, this modification leads a faster search

Be aware of these differences if you want to use this code.
