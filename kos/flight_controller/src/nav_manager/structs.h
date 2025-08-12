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

struct PointOfInterest {
        int32_t latitude;
        int32_t longitude;
        double epsilon; // Радиус зоны точки интереса
        bool scanned;  // Флаг, была ли точка уже отсканирована
        bool scanning; // Флаг, что в данный момент идет сканирование
        uint8_t scan_attempts; // Добавляем счетчик попыток
        uint32_t last_scan_time; // Время последней попытки
};