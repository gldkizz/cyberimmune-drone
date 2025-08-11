#pragma once
#include <vector>

struct Point {
    double x{}, y{};
    Point() = default;
    Point(double _x, double _y) :x(_x), y(_y) {}
};

using Polygon = std::vector<Point>;
using Interval = std::pair<double, double>;

struct BuildResult {
    std::vector<Point> nodes;
    std::vector<std::pair<int, int>> edges;
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
    Polygon boundary;
    std::vector<Polygon> obstacles;
};

struct Edge {
    std::pair<int, int> edge;
	double length = 0;
};