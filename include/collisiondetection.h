#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"
#include <map>
#include <queue>
#include <cstring>

namespace HybridAStar {
namespace {
void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99;
}

void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}

typedef std::pair<u_char*,std::pair<int, int>> Map;

/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.
   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection();


  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
  template<typename T> bool isTraversable(const T* node) {
    /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
    float cost = 0;
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);

    // 2D collision test
    if (t == 99) {
      return !map.first[node->getIdx()];
    }

    if (true) {
      cost = configurationTest(x, y, t) ? 0 : 1;
    } else {
      cost = configurationCost(x, y, t);
    }

    return cost <= 0;
  }

  /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return the cost of the configuration q of W(q)
     \todo needs to be implemented correctly
  */
  float configurationCost(float x, float y, float t) {return 0;}

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t);

  /*!
     \brief updates the grid with the world map
  */
  void updateMap(Map map) {this->map = map;}

  float getDistance(int x1, int y1, int x2, int y2){
    if(xdis == x2 && ydis == y2){
      return distances[x1][y1];
    }
    else{
      refreshDistance(x2, y2);
      return distances[x1][y1];
    }
  }

 private:
  /// The occupancy grid
  Map map;
  /// The collision lookup table
  Constants::config collisionLookup[Constants::headings * Constants::positions];

  void refreshDistance(int x, int y)
  {
    xdis = x;
    ydis = y;
    memset(distances, 0x7f, sizeof(distances));
    memset(dis_visited, 0, sizeof(dis_visited));
    int dx[] = {-1, 1, 0, 0, -1, 1, -1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, 1, -1};
    float dd[] = {1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)};
    std::queue<std::pair<int, int>> que;
    que.push(std::make_pair(x, y));
    dis_visited[x][y] = true;
    distances[x][y] = 0;
    int w = map.second.first, h = map.second.second;
    while(!que.empty()){
      int nx = que.front().first;
      int ny = que.front().second;
      que.pop();
      for(int i = 0; i < 8; ++i){
        int tx = nx + dx[i];
        int ty = ny + dy[i];
        int idx = ty * w + tx;
        if((!dis_visited[tx][ty]) && tx >= 0 && tx < w && ty >= 0 && ty < h &&
          map.first[idx] == 0){
            dis_visited[tx][ty] = true;
            float d = distances[nx][ny] + dd[i];
            if(d < distances[tx][ty]){
              distances[tx][ty] = d;
              que.push(std::make_pair(tx, ty));
            }
        }
      }
    }
  }

  int xdis = -1, ydis = -1;
  float distances[151][401];
  bool dis_visited[151][401];
};
}
#endif // COLLISIONDETECTION_H