#ifndef SPD_ROBOTVISION_astar_H
#define SPD_ROBOTVISION_astar_H

#include "../include/map.h"
#include <queue>
#include <vector>
#include <unordered_set>
#include "iostream"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Node
{
public:
  Point position;
  double f, g, h;
  Node *parent;

  bool operator==(const Node &other) const
  {
    return position == other.position;
  }
  Node(Point position, double g, double h, Node *parent) : position(position), g(g), h(h), parent(parent) {}
};

namespace std
{
  template <>
  struct hash<Node>
  {
    std::size_t operator()(const Node &node) const
    {
      Point position = node.position;
      int row, col;
      std::tie(row, col) = position.getCoordinate();
      return std::hash<int>()(row) ^ std::hash<int>()(col);
    }
  };
}

std::vector<Node> getNeighbors(Node currentNode, GridMap &map)
{
  std::vector<Node> neighbors;
  int rows, cols;
  std::tie(rows, cols) = map.getDimensions();
  // Get the current node's position on the grid map
  Point position = currentNode.position;
  int row, col;
  std::tie(row, col) = position.getCoordinate();
  // Check the surrounding cells of the current node's position
  for (int i = row - 1; i <= row + 1; i++)
  {
    for (int j = col - 1; j <= col + 1; j++)
    {
      // Skip the current node itself
      if (i == row && j == col || i < 0 || j < 0 || j > cols - 1 || i > rows - 1)
      {
        continue;
      }

      // Check if the cell is traversable and add it as a neighbor if it is
      if (map.isFree(i, j))
      {
        Point neighborPosition(i, j);
        Node neighbor(neighborPosition, 0, 0, &currentNode);
        neighbors.push_back(neighbor);
      }
    }
  }

  return neighbors;
}
//====================================================================
void printList(std::vector<Node *> list)
{
  std::cout << "Nodes List : ";
  for (int i = 0; i < list.size(); i++)
  {
    int x, y;
    std::tie(x, y) = list[i]->position.getCoordinate();
    std::cout << "( x: " << x << ", y: " << y << ", h: " << list[i]->h << ", g: " << list[i]->g << ", f: " << list[i]->f << " ), ";
  }

  std::cout << "\n";
}
void markNodeClosed(Node &node, cv::Mat &mat)
{
  int x, y;
  std::tie(x, y) = node.position.getCoordinate();
  mat.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 255);
}
void markNodeOpen(Node &node, cv::Mat &mat)
{
  int x, y;
  std::tie(x, y) = node.position.getCoordinate();
  mat.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 255, 0);
}
void markNodeGoal(Node &node, cv::Mat &mat)
{
  int x, y;
  std::tie(x, y) = node.position.getCoordinate();
  cv::circle(mat, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);
}
struct NodeCompare
{
  bool operator()(const Node &a, const Node &b)
  {
    return a.f > b.f; // this will create a min heap
  }
};
//====================================================================
std::vector<Point> findShortestPath(Point &start, Point &goal, GridMap &map)
{
  cv::Mat monitor(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::cvtColor(map.getMapCopy(), monitor, cv::COLOR_GRAY2BGR);
  // Initialize start node and add it to the open list
  Node startNode(start, 0, start.length(goal), nullptr);
  Node goalNode(goal, 0, 0, nullptr);
  markNodeGoal(goalNode, monitor);
  std::priority_queue<Node, std::vector<Node>, NodeCompare> openList;
  std::unordered_set<Node> openListSearch;
  openList.push(startNode);
  openListSearch.insert(startNode);
  // Initialize empty closed list
  std::unordered_set<Node> closedList;

  // Loop until the goal is reached or the open list is empty

  while (!openList.empty())
  {
    Node currentNode = openList.top();
    openList.pop();
    openListSearch.erase(currentNode);
    closedList.insert(currentNode);
    markNodeClosed(currentNode, monitor);
    // Check if the goal has been reached
    if (currentNode.position == goal)
    {
      double x, y;
      std::tie(x, y) = currentNode.position.getCoordinate();
      std::vector<Point> path;
      Node *node = &currentNode;
      while (node != nullptr)
      {
        path.insert(path.begin(), node->position);
        node = node->parent;
      }
      return path;
    }

    // Expand the current node's neighbors
    std::vector<Node> neighbors = getNeighbors(currentNode, map);
    // printList(neighbors);
    // std::cout << "current : "<< &currentNode << " | parent : " << currentNode.parent << std::endl;
    for (Node neighbor : neighbors)
    {

      double g = currentNode.g + currentNode.position.length(neighbor.position);
      double h = currentNode.h + currentNode.position.length(goal);
      double f = h + g;
      bool isOpen = openListSearch.count(neighbor) > 0;
      bool isClose = closedList.count(neighbor) > 0;

      int x, y;
      std::tie(x, y) = neighbor.position.getCoordinate();
      // Check if the neighbor is on the closed list
      if (isClose)
        continue;
      // Update the neighbor's f value if the new path to it through the current node is shorter
      if (isOpen && f < neighbor.f)
      {
        neighbor.g = g;
        neighbor.f = g + neighbor.h;
        neighbor.parent = new Node(currentNode.position, currentNode.g, currentNode.h, currentNode.parent);

      }

      // If the neighbor is not on either list, add it to the open list and initialize its f, g, and h values
      if (!isOpen)
      {

        neighbor.g = g;
        neighbor.h = neighbor.position.length(goal);
        neighbor.f = neighbor.g + neighbor.h;
        neighbor.parent = new Node(currentNode.position, currentNode.g, currentNode.h, currentNode.parent);
        openList.push(neighbor);
        openListSearch.insert(neighbor);
        markNodeOpen(neighbor, monitor);

      }
    }
    cv::Mat image_scaled;

    // cv::resize(monitor, image_scaled, cv::Size(), 2, 2, cv::INTER_LINEAR);

    // cv::imshow("Monitor", image_scaled);
    // int key = cv::waitKey(1);
  }
}

#endif // SPD_ROBOTVISION_astar_H