#pragma once
#include <iostream>
#include <set>
#include <unordered_set>
#include <map>
#include <queue>
#include <math.h>
#include "structs.h"
#include "json.hpp"
#include "navConfig.h"



class NavManager {
public:
    NavManager() {}

    bool buildRoute(const Point& a, const Point& b, std::vector<Point>& route) {
        route.clear();
        if (intersects_any_polygon(a, b)) {
            int start_node_id = find_nearest_clear_node_id(a);
            int end_node_id = find_nearest_clear_node_id(b);
            if (start_node_id < 0 || end_node_id < 0)
                return false;
            route = calculate_path(start_node_id, end_node_id);
            if (route.size() < 1)
                return false;
        }
        route.insert(route.begin(), a);
        route.insert(route.end(), b);
        return true;
    }

    bool load_polygons(const nlohmann::json& data) {
        if (!data.contains("features") || !data["features"].is_array()) return false;
        for (auto& ft : data["features"]) {
            if (!ft.contains("geometry")) continue;
            auto& geom = ft["geometry"];
            if (!geom.contains("type") || geom["type"] != "Polygon") continue;
            if (!geom.contains("coordinates") || !geom["coordinates"].is_array()) continue;
            auto rings = geom["coordinates"];
            if (rings.empty()) continue;
            auto ring = rings[0];
            Polygon poly;
            for (auto& pt : ring) {
                if (pt.is_array() && pt.size() >= 2) {
                    double x = pt[0].get<double>();
                    double y = pt[1].get<double>();
                    poly.emplace_back(x, y);
                }
            }
            if (poly.size() > 1 &&
                std::fabs(poly.front().x - poly.back().x) < EPS &&
                std::fabs(poly.front().y - poly.back().y) < EPS) {
                poly.pop_back();
            }
            if (poly.size() >= 3) polys.push_back(poly);
        }
        return true;
    }

    bool build_graph(bool fallback = false)
    {
        std::string use_mode = BOUNDARY_MODE;

        //BuildResult res;
        Polygon boundary;
        std::vector<Polygon> obstacles;

        if (use_mode == "auto") {
            auto tmp1 = select_boundary(polys, "largest");
            Polygon b_largest = tmp1.first;
            std::vector<Polygon> obs_largest = tmp1.second;
            if (!outer_contains_all(b_largest, polys)) {
                if (VERBOSE) std::cout << "[auto] using hull (some points outside largest)\n";
                auto tmp2 = select_boundary(polys, "hull");
                boundary = tmp2.first; obstacles = tmp2.second;
            }
            else {
                boundary = b_largest; obstacles = obs_largest;
            }
        }
        else {
            auto tmp = select_boundary(polys, use_mode);
            boundary = tmp.first; obstacles = tmp.second;
        }

        auto bb = bbox({ boundary });
        double xmin = std::get<0>(bb);
        double xmax = std::get<1>(bb);
        double ymin = std::get<2>(bb);
        double ymax = std::get<3>(bb);
        //res.xmin = xmin; res.xmax = xmax; res.ymin = ymin; res.ymax = ymax;
        double rx = xmax - xmin, ry = ymax - ymin;
        double rmin = std::min(rx, ry);
        if (rmin <= 0) {
            //res.boundary = boundary;
            //res.obstacles = obstacles;
            return false;
        }

        double merge_factor = MERGE_FACTOR * (fallback ? 0.5 : 1.0);
        double min_cell_factor = MIN_CELL_FACTOR * (fallback ? 0.5 : 1.0);

        double merge_tol = merge_factor * rmin;
        double min_cell = min_cell_factor * rmin;
        double interior_margin = INTERIOR_MARGIN_FACTOR * rmin;
        double thin_threshold = FILTER_THIN_BANDS * rmin;

        std::set<double> xs_set{ xmin, xmax };
        std::set<double> ys_set{ ymin, ymax };
        for (auto& p : boundary) { xs_set.insert(p.x); ys_set.insert(p.y); }
        for (auto& obs : obstacles) {
            for (auto& p : obs) {
                xs_set.insert(p.x);
                ys_set.insert(p.y);
            }
        }
        std::vector<double> xs(xs_set.begin(), xs_set.end());
        std::vector<double> ys(ys_set.begin(), ys_set.end());

        double mt = merge_tol;
        for (int attempt = 0; attempt < MAX_MERGE_ITERS; ++attempt) {
            auto cx = cluster_vals(xs, mt);
            auto cy = cluster_vals(ys, mt);

            auto filter_lines = [&](std::vector<double> clines, const std::vector<double>& raw) {
                if (clines.size() < 2) return clines;
                std::vector<double> keep;
                keep.push_back(clines.front());
                for (size_t i = 1; i + 1 < clines.size(); ++i) {
                    double v = clines[i];
                    double prev = keep.back();
                    double nxt = clines[i + 1];
                    double gap1 = v - prev; double gap2 = nxt - v;
                    if (gap1 < min_cell && gap2 < min_cell) {
                        bool exists = false;
                        for (auto r : raw) {
                            if (std::fabs(v - r) <= EPS) { exists = true; break; }
                        }
                        if (exists) keep.push_back(v);
                    }
                    else {
                        keep.push_back(v);
                    }
                }
                keep.push_back(clines.back());
                return keep;
                };
            cx = filter_lines(cx, xs);
            cy = filter_lines(cy, ys);

            if (cx.size() >= 3 && cy.size() >= 3) {
                xs = cx; ys = cy;
                break;
            }
            mt *= 0.5;
            if (attempt == MAX_MERGE_ITERS - 1) {
                xs = cx; ys = cy;
            }
        }
        if (VERBOSE) {
            std::cout << "[info" << (fallback ? "(fallback)" : "") << "] mode=" << use_mode
                << " linesX=" << xs.size() << " linesY=" << ys.size()
                << " merge_tol=" << mt << "\n";
        }

        std::map<double, std::vector<Interval>> horiz;
        for (size_t i = 0; i + 1 < ys.size(); ++i) {
            double y0 = ys[i], y1 = ys[i + 1];
            if (y1 <= y0 + EPS) continue;
            if ((y1 - y0) < thin_threshold) continue;
            double ymid = 0.5 * (y0 + y1);
            auto ints = horizontal_inside(boundary, obstacles, ymid);
            if (CLEARANCE > 0) {
                std::vector<Interval> shr;
                for (auto& ab : ints) {
                    double a = ab.first, b = ab.second;
                    if (b - a <= 2 * CLEARANCE + 1e-12) continue;
                    shr.push_back({ a + CLEARANCE, b - CLEARANCE });
                }
                ints.swap(shr);
            }
            if (!ints.empty()) horiz[ymid] = ints;
        }

        std::map<double, std::vector<Interval>> vert;
        for (size_t j = 0; j + 1 < xs.size(); ++j) {
            double x0 = xs[j], x1 = xs[j + 1];
            if (x1 <= x0 + EPS) continue;
            if ((x1 - x0) < thin_threshold) continue;
            double xmid = 0.5 * (x0 + x1);
            auto ints = vertical_inside(boundary, obstacles, xmid);
            if (CLEARANCE > 0) {
                std::vector<Interval> shr;
                for (auto& ab : ints) {
                    double a = ab.first, b = ab.second;
                    if (b - a <= 2 * CLEARANCE + 1e-12) continue;
                    shr.push_back({ a + CLEARANCE, b - CLEARANCE });
                }
                ints.swap(shr);
            }
            if (!ints.empty()) vert[xmid] = ints;
        }

        std::map<std::pair<double, double>, int> node_id;
        std::vector<Point> nodes;
        for (auto& hy : horiz) {
            double y = hy.first;
            auto& x_ints = hy.second;
            for (auto& vx : vert) {
                double x = vx.first;
                auto& y_ints = vx.second;
                bool ok1 = false, ok2 = false;
                for (auto& ab : x_ints) {
                    if (ab.first + interior_margin < x && x < ab.second - interior_margin) {
                        ok1 = true; break;
                    }
                }
                if (!ok1) continue;
                for (auto& ab : y_ints) {
                    if (ab.first + interior_margin < y && y < ab.second - interior_margin) {
                        ok2 = true; break;
                    }
                }
                if (!ok2) continue;
                Point p(x, y);
                if (CLEARANCE > 0 && !point_clear(p, boundary, obstacles, CLEARANCE)) {
                    continue;
                }
                auto key = std::make_pair(x, y);
                if (!node_id.count(key)) {
                    node_id[key] = (int)nodes.size();
                    nodes.push_back(p);
                }
            }
        }

        if (VERBOSE) {
            std::cout << "[info] raw nodes=" << nodes.size() << "\n";
        }
        if (nodes.empty()) {
            resNodes = nodes;
            resEdges.clear();
            //res.boundary = boundary;
            //res.obstacles = obstacles;
            return true;
        }

        // group by Y (rounded)
        std::unordered_map<long double, std::vector<int>> by_y;
        std::unordered_map<long double, std::vector<int>> by_x;

        auto round_key = [](double v) {
            // Для C++14: просто округляем до 1e-9
            return static_cast<long double>(static_cast<long long>(v * 1e9 + (v >= 0 ? 0.5 : -0.5))) / 1e9;
        };

        for (int i = 0; i < (int)nodes.size(); ++i) {
            by_y[round_key(nodes[i].y)].push_back(i);
            by_x[round_key(nodes[i].x)].push_back(i);
        }

        struct PairHash {
            size_t operator()(const std::pair<int, int>& p) const noexcept {
                return (static_cast<size_t>(p.first) << 32) ^ static_cast<size_t>(p.second);
            }
        };
        std::unordered_set<std::pair<int, int>, PairHash> edges_set;

        auto add_edge = [&](int u, int v) {
            if (u == v) return;
            if (u > v) std::swap(u, v);
            edges_set.insert({ u,v });
            };

        // horizontal edges
        for (auto& kv : by_y) {
            auto lst = kv.second;
            std::sort(lst.begin(), lst.end(), [&](int a, int b) {
                return nodes[a].x < nodes[b].x;
                });
            for (size_t k = 0; k + 1 < lst.size(); ++k) {
                int u = lst[k], v = lst[k + 1];
                const Point& a = nodes[u], & b = nodes[v];
                double midx = (a.x + b.x) / 2.0;
                auto it = horiz.find(a.y);
                if (it == horiz.end()) continue;
                bool inside = false;
                for (auto& iv : it->second) {
                    if (iv.first <= midx && midx <= iv.second) { inside = true; break; }
                }
                if (!inside) continue;
                if (segment_clear(a, b, boundary, obstacles, CLEARANCE)) {
                    add_edge(u, v);
                }
            }
        }
        // vertical edges
        for (auto& kv : by_x) {
            auto lst = kv.second;
            std::sort(lst.begin(), lst.end(), [&](int a, int b) {
                return nodes[a].y < nodes[b].y;
                });
            for (size_t k = 0; k + 1 < lst.size(); ++k) {
                int u = lst[k], v = lst[k + 1];
                const Point& a = nodes[u], & b = nodes[v];
                double midy = (a.y + b.y) / 2.0;
                auto it = vert.find(a.x);
                if (it == vert.end()) continue;
                bool inside = false;
                for (auto& iv : it->second) {
                    if (iv.first <= midy && midy <= iv.second) { inside = true; break; }
                }
                if (!inside) continue;
                if (segment_clear(a, b, boundary, obstacles, CLEARANCE)) {
                    add_edge(u, v);
                }
            }
        }

        // adjacency
        std::unordered_map<int, std::unordered_set<int>> adj;
        for (auto& e : edges_set) {
            int u = e.first, v = e.second;
            adj[u].insert(v);
            adj[v].insert(u);
        }

        auto on_boundary = [&](const Point& p) {
            int n = (int)boundary.size();
            for (int i = 0; i < n; i++) {
                if (dist_point_seg(p, boundary[i], boundary[(i + 1) % n]) < std::max(CLEARANCE * 0.5, 5 * EPS)) {
                    return true;
                }
            }
            return false;
            };

        // CONTRACT ITTERS
        for (int iter = 0; iter < CONTRACT_ITERS; ++iter) {
            bool changed = false;
            std::vector<int> remove_nodes;
            for (int i = 0; i < (int)nodes.size(); ++i) {
                if (!adj.count(i)) continue;
                auto& nbrs = adj[i];
                if (nbrs.size() != 2) continue;
                if (KEEP_BOUNDARY_NODES && on_boundary(nodes[i])) continue;
                auto itn = nbrs.begin(); int a = *itn; ++itn; int b = *itn;
                const Point& pa = nodes[a], & pb = nodes[b], & p = nodes[i];
                bool collinear_axis = (
                    (std::fabs(pa.x - p.x) < EPS && std::fabs(p.x - pb.x) < EPS) ||
                    (std::fabs(pa.y - p.y) < EPS && std::fabs(p.y - pb.y) < EPS)
                    );
                if (!collinear_axis) continue;
                if (!segment_clear(pa, pb, boundary, obstacles, CLEARANCE)) continue;
                adj[a].erase(i); adj[b].erase(i);
                adj[a].insert(b); adj[b].insert(a);
                remove_nodes.push_back(i);
                changed = true;
            }
            if (!changed) break;
            for (auto i : remove_nodes) {
                adj.erase(i);
            }
        }

        // --- Visibility augmentation ---
        if (ADD_VISIBILITY_EDGES) {
            int total_new = 0;

            auto current_components = [&]() {
                std::vector<std::vector<int>> comps;
                std::unordered_set<int> seen;
                for (auto& kv : adj) {
                    int u = kv.first;
                    if (seen.count(u)) continue;
                    std::vector<int> stack{ u };
                    seen.insert(u);
                    std::vector<int> group;
                    while (!stack.empty()) {
                        int x = stack.back(); stack.pop_back();
                        group.push_back(x);
                        for (auto w : adj[x]) {
                            if (!seen.count(w)) {
                                seen.insert(w);
                                stack.push_back(w);
                            }
                        }
                    }
                    comps.push_back(group);
                }
                return comps;
                };

            auto try_add = [&](int u, int v) {
                if (u == v) return false;
                if (adj[u].count(v)) return false;
                if (total_new >= VIS_TOTAL_NEW_EDGES_LIMIT) return false;
                const Point& A = nodes[u], & B = nodes[v];
                if (!segment_clear(A, B, boundary, obstacles, CLEARANCE)) return false;
                adj[u].insert(v);
                adj[v].insert(u);
                total_new++;
                return true;
                };

            if (VISIBILITY_CONNECT_COMPONENTS) {
                auto comps = current_components();
                int attempts = 0;
                while (comps.size() > 1 && total_new < VIS_TOTAL_NEW_EDGES_LIMIT && attempts < VIS_MAX_COMPONENT_LINKS) {
                    std::sort(comps.begin(), comps.end(), [](const std::vector<int>& a, const std::vector<int>& b) { return a.size() < b.size(); });
                    auto base = comps[0];
                    std::vector<std::vector<int>> others(comps.begin() + 1, comps.end());

                    using T = std::tuple<double, int, int>;
                    std::priority_queue<T, std::vector<T>, std::greater<T>> cand;
                    bool limitReached = false;
                    for (auto u : base) {
                        const Point& pu = nodes[u];
                        for (auto& c : others) {
                            for (auto v : c) {
                                const Point& pv = nodes[v];
                                double dx = pu.x - pv.x;
                                double dy = pu.y - pv.y;
                                double d2 = dx * dx + dy * dy;
                                cand.push({ d2,u,v });
                                if ((int)cand.size() > VIS_COMP_PAIR_SEARCH_LIMIT) {
                                    limitReached = true;
                                    break;
                                }
                            }
                            if (limitReached) break;
                        }
                        if (limitReached) break;
                    }
                    bool linked = false;
                    while (!cand.empty() && !linked) {
                        auto top = cand.top(); cand.pop();
                        int u = std::get<1>(top), v = std::get<2>(top);
                        if (try_add(u, v)) {
                            linked = true;
                            attempts++;
                            if (VIS_RECHECK_AFTER_EACH_LINK) {
                                comps = current_components();
                            }
                        }
                    }
                    if (!linked) break;
                    if (!VIS_RECHECK_AFTER_EACH_LINK) {
                        comps = current_components();
                    }
                }
            }

            if (VISIBILITY_DENSIFY && total_new < VIS_TOTAL_NEW_EDGES_LIMIT) {
                double rx = xmax - xmin;
                double ry = ymax - ymin;
                double diag = std::hypot(rx, ry);
                double radius = VIS_LOCAL_RADIUS_FACTOR * diag;
                std::vector<int> node_list;
                node_list.reserve(adj.size());
                for (auto& kv : adj) node_list.push_back(kv.first);

                int new_local = 0;
                std::vector<int> degree_added(nodes.size(), 0);
                for (size_t i = 0; i < node_list.size(); ++i) {
                    if (total_new >= VIS_TOTAL_NEW_EDGES_LIMIT || new_local >= VIS_LOCAL_MAX_NEW_EDGES)
                        break;
                    int u = node_list[i];
                    const Point& pu = nodes[u];
                    std::vector<std::pair<double, int>> dists;
                    dists.reserve(node_list.size());
                    for (size_t j = 0; j < node_list.size(); ++j) {
                        int v = node_list[j];
                        if (u == v) continue;
                        const Point& pv = nodes[v];
                        double dx = pu.x - pv.x;
                        double dy = pu.y - pv.y;
                        double d = std::hypot(dx, dy);
                        if (d <= radius) {
                            dists.push_back({ d,v });
                        }
                    }
                    std::sort(dists.begin(), dists.end(), [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
                        return a.first < b.first;
                        });
                    int taken = 0;
                    for (auto& dv : dists) {
                        if (taken >= VIS_LOCAL_K) break;
                        int v = dv.second;
                        if (degree_added[u] >= VIS_LOCAL_MAX_DEG_INCREASE) break;
                        if (degree_added[v] >= VIS_LOCAL_MAX_DEG_INCREASE) {
                            // pass
                        }
                        if (adj[u].count(v)) continue;
                        if (try_add(u, v)) {
                            degree_added[u]++; degree_added[v]++;
                            new_local++; taken++;
                            if (new_local >= VIS_LOCAL_MAX_NEW_EDGES) break;
                            if (total_new >= VIS_TOTAL_NEW_EDGES_LIMIT) break;
                        }
                    }
                }
                if (VERBOSE) {
                    std::cout << "[visibility] new edges added total=" << total_new << "\n";
                }
            }
        }

        // ��������� ��������� �����
        std::set<std::pair<int, int>> final_edges;
        for (auto& kv : adj) {
            int u = kv.first;
            for (auto v : kv.second) {
                if (u < v) final_edges.insert({ u,v });
            }
        }

        // �����������
        std::vector<int> id_map(nodes.size(), -1);
        std::vector<Point> new_nodes;
        new_nodes.reserve(nodes.size());
        for (int i = 0; i < (int)nodes.size(); ++i) {
            if (adj.count(i)) {
                id_map[i] = (int)new_nodes.size();
                new_nodes.push_back(nodes[i]);
            }
        }
        std::vector<Edge> new_edges;
        new_edges.reserve(final_edges.size());
        for (auto& e : final_edges) {
            int u = e.first, v = e.second;
            if (id_map[u] >= 0 && id_map[v] >= 0 && id_map[u] < new_nodes.size() && id_map[v] < new_nodes.size()) {
                Point node1 = new_nodes[id_map[u]];
                Point node2 = new_nodes[id_map[v]];
                Edge edge;
                edge.edge = std::make_pair(id_map[u], id_map[v]);
                edge.length = sqrt(pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2));
                new_edges.push_back(edge);
            }
        }

        /*for (auto& e : final_edges) {
            int u = e.first, v = e.second;
            if (u >= new_nodes.size() || v >= new_nodes.size())
                continue;
            if (id_map[u] >= 0 && id_map[v] >= 0) {
                Edge edge{ { id_map[u], id_map[v] } };
                Point node1 = new_nodes[u];
                Point node2 = new_nodes[v];
                edge.length = sqrt(pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2));
                new_edges.push_back(edge);
            }
        }*/

        if (VERBOSE) {
            std::cout << "[info] final nodes=" << new_nodes.size()
                << " edges=" << new_edges.size()
                << " (added visibility edges=" << (ADD_VISIBILITY_EDGES ? "yes" : "no") << ")\n";
        }

        resNodes = std::move(new_nodes);
        resEdges = std::move(new_edges);
        //res.boundary = boundary;
        //res.obstacles = obstacles;
        return true;
    }

private:
    bool intersects_any_polygon(const Point& a, const Point& b) const {
        for (const auto& poly : polys) {
            // Проверка: хотя бы одна из точек лежит внутри или на границе полигона
            if (point_in_poly_non_strict(a, poly) || point_in_poly_non_strict(b, poly)) {
                return true;
            }
            // Проверка: пересечение рёбер
            int n = (int)poly.size();
            for (int i = 0; i < n; ++i) {
                const Point& c = poly[i];
                const Point& d = poly[(i + 1) % n];
                if (segments_strict_intersect(a, b, c, d)) {
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<Point> calculate_path(int start_node, int end_node) {
        std::vector<Point> result_path;
        if (start_node < 0 || end_node < 0 || start_node >= (int)resNodes.size() || end_node >= (int)resNodes.size())
            return result_path;

        // Построим граф смежности
        std::vector<std::vector<std::pair<int, double>>> graph(resNodes.size());
        for (const auto& edge : resEdges) {
            int u = edge.edge.first;
            int v = edge.edge.second;
            double len = edge.length;
            graph[u].emplace_back(v, len);
            graph[v].emplace_back(u, len);
        }

        // A*
        std::vector<double> g(resNodes.size(), std::numeric_limits<double>::infinity());
        std::vector<double> f(resNodes.size(), std::numeric_limits<double>::infinity());
        std::vector<int> prev(resNodes.size(), -1);
        using QElem = std::pair<double, int>; // (f, node)
        std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> open;

        auto heuristic = [&](int u) {
            const Point& a = resNodes[u];
            const Point& b = resNodes[end_node];
            return std::hypot(a.x - b.x, a.y - b.y);
        };

        g[start_node] = 0.0;
        f[start_node] = heuristic(start_node);
        open.emplace(f[start_node], start_node);

        std::vector<bool> closed(resNodes.size(), false);

        while (!open.empty()) {
            int u = open.top().second;
            open.pop();
            if (closed[u]) continue;
            closed[u] = true;
            if (u == end_node) break;
        for (size_t i = 0; i < graph[u].size(); ++i) {
            int v = graph[u][i].first;
            double cost = graph[u][i].second;
            if (closed[v]) continue;
            double tentative_g = g[u] + cost;
            if (tentative_g < g[v]) {
                g[v] = tentative_g;
                f[v] = g[v] + heuristic(v);
                prev[v] = u;
                open.push(std::make_pair(f[v], v));
            }
        }
        }

        // Восстановление пути
        //if (!closed[end_node]) return result_path;
        std::vector<int> path_ids;
        for (int v = end_node; v != -1; v = prev[v]) {
            path_ids.push_back(v);
        }
        std::reverse(path_ids.begin(), path_ids.end());
        for (int id : path_ids) {
            result_path.push_back(resNodes[id]);
        }
        return result_path;
    }

    int find_nearest_clear_node_id(const Point& p) const {
        int best_id = -1;
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < resNodes.size(); ++i) {
            const Point& node = resNodes[i];
            bool clear = true;
            for (const auto& poly : polys) {
                int n = (int)poly.size();
                for (int j = 0; j < n; ++j) {
                    const Point& a = poly[j];
                    const Point& b = poly[(j + 1) % n];
                    if (segments_strict_intersect(p, node, a, b)) {
                        clear = false;
                        break;
                    }
                }
                if (!clear) break;
            }
            if (clear) {
                double d = std::hypot(p.x - node.x, p.y - node.y);
                if (d < best_dist) {
                    best_dist = d;
                    best_id = static_cast<int>(i);
                }
            }
        }
        return best_id;
    }

    double orient(const Point& a, const Point& b, const Point& c) const {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    bool on_segment(const Point& a, const Point& b, const Point& p, double eps = EPS) const {
        if (std::fabs(orient(a, b, p)) > eps) return false;
        return (std::min(a.x, b.x) - eps <= p.x && p.x <= std::max(a.x, b.x) + eps &&
            std::min(a.y, b.y) - eps <= p.y && p.y <= std::max(a.y, b.y) + eps);
    }

    bool segments_strict_intersect(const Point& a, const Point& b, const Point& c, const Point& d, double eps = EPS) const {
        double o1 = orient(a, b, c), o2 = orient(a, b, d);
        double o3 = orient(c, d, a), o4 = orient(c, d, b);
        // Исправлено: eps сравнивается с абсолютным значением произведения, чтобы избежать ошибок при малых числах
        return (o1 * o2 < -std::max(eps * eps, 1e-20) && o3 * o4 < -std::max(eps * eps, 1e-20));
    }

    bool point_in_poly_non_strict(const Point& p, const Polygon& poly, double eps = EPS) const {
        bool inside = false;
        int n = (int)poly.size();
        for (int i = 0; i < n; i++) {
            const Point& a = poly[i];
            const Point& b = poly[(i + 1) % n];
            if (on_segment(a, b, p, eps)) return true;
            bool cond = ((a.y > p.y) != (b.y > p.y));
            if (cond) {
                double xin = (p.y - a.y) * (b.x - a.x) / ((b.y - a.y) + eps) + a.x;
                if (p.x < xin) inside = !inside;
            }
        }
        return inside;
    }

    bool point_in_poly_strict(const Point& p, const Polygon& poly, double eps = EPS) {
        if (!point_in_poly_non_strict(p, poly, eps)) return false;
        for (int i = 0; i < (int)poly.size(); ++i) {
            if (on_segment(poly[i], poly[(i + 1) % poly.size()], p, eps)) {
                return false;
            }
        }
        return true;
    }

    double poly_area(const Polygon& poly) {
        double a = 0.0;
        int n = (int)poly.size();
        for (int i = 0; i < n; i++) {
            const Point& p = poly[i];
            const Point& q = poly[(i + 1) % n];
            a += p.x * q.y - q.x * p.y;
        }
        return 0.5 * a;
    }

    std::tuple<double, double, double, double> bbox(const std::vector<Polygon>& polys) {
        double xmin = std::numeric_limits<double>::infinity();
        double xmax = -std::numeric_limits<double>::infinity();
        double ymin = std::numeric_limits<double>::infinity();
        double ymax = -std::numeric_limits<double>::infinity();
        for (auto& poly : polys) {
            for (auto& p : poly) {
                xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
                ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
            }
        }
        return { xmin,xmax,ymin,ymax };
    }

    double dist_point_seg(const Point& p, const Point& a, const Point& b) {
        double vx = b.x - a.x;
        double vy = b.y - a.y;
        double wx = p.x - a.x;
        double wy = p.y - a.y;
        double c1 = vx * wx + vy * wy;
        if (c1 <= 0) return std::hypot(p.x - a.x, p.y - a.y);
        double c2 = vx * vx + vy * vy;
        if (c2 <= EPS) return std::hypot(p.x - a.x, p.y - a.y);
        double t = c1 / c2;
        if (t >= 1) return std::hypot(p.x - b.x, p.y - b.y);
        double px = a.x + vx * t;
        double py = a.y + vy * t;
        return std::hypot(p.x - px, p.y - py);
    }

    Polygon convex_hull(std::vector<Point> pts) {
        std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
            if (a.x == b.x) return a.y < b.y;
            return a.x < b.x;
            });
        if (pts.size() <= 1) return pts;
        std::vector<Point> lower, upper;
        for (auto& p : pts) {
            while (lower.size() >= 2 && orient(lower[lower.size() - 2], lower.back(), p) <= 0) {
                lower.pop_back();
            }
            lower.push_back(p);
        }
        for (int i = (int)pts.size() - 1; i >= 0; --i) {
            auto& p = pts[i];
            while (upper.size() >= 2 && orient(upper[upper.size() - 2], upper.back(), p) <= 0) {
                upper.pop_back();
            }
            upper.push_back(p);
        }
        Polygon hull;
        for (size_t i = 0; i + 1 < lower.size(); ++i) hull.push_back(lower[i]);
        for (size_t i = 0; i + 1 < upper.size(); ++i) hull.push_back(upper[i]);
        if (poly_area(hull) < 0) std::reverse(hull.begin(), hull.end());
        return hull;
    }

    std::pair<Polygon, std::vector<Polygon>> select_boundary(const std::vector<Polygon>& polys, const std::string& mode) {
        if (polys.empty()) throw std::runtime_error("No polygons");
        if (mode == "hull") {
            std::vector<Point> pts;
            for (auto& p : polys) for (auto& q : p) pts.push_back(q);
            Polygon hull = convex_hull(pts);
            return { hull, polys };
        }
        // largest
        std::vector<double> areas;
        areas.reserve(polys.size());
        for (auto& p : polys) {
            areas.push_back(std::fabs(poly_area(p)));
        }
        int idx = (int)std::distance(areas.begin(), std::max_element(areas.begin(), areas.end()));
        Polygon boundary = polys[idx];
        if (poly_area(boundary) < 0) std::reverse(boundary.begin(), boundary.end());
        std::vector<Polygon> obstacles;
        for (int i = 0; i < (int)polys.size(); ++i) {
            if (i == idx) continue;
            bool inside = true;
            for (auto& pt : polys[i]) {
                if (!point_in_poly_non_strict(pt, boundary)) {
                    inside = false; break;
                }
            }
            if (inside) obstacles.push_back(polys[i]);
        }
        return { boundary, obstacles };
    }

    bool outer_contains_all(const Polygon& boundary, const std::vector<Polygon>& polys) {
        for (auto& poly : polys) {
            for (auto& pt : poly) {
                if (!point_in_poly_non_strict(pt, boundary)) return false;
            }
        }
        return true;
    }

    std::vector<double> scanline_intersections(const Polygon& poly, double y) {
        std::vector<double> xs;
        int n = (int)poly.size();
        for (int i = 0; i < n; i++) {
            const Point& a = poly[i];
            const Point& b = poly[(i + 1) % n];
            if ((a.y - y) * (b.y - y) <= 0) {
                if (std::fabs(a.y - b.y) < EPS) {
                    xs.push_back(a.x);
                    xs.push_back(b.x);
                }
                else {
                    double t = (y - a.y) / (b.y - a.y);
                    if (-EPS <= t && t <= 1 + EPS) {
                        xs.push_back(a.x + (b.x - a.x) * t);
                    }
                }
            }
        }
        std::sort(xs.begin(), xs.end());
        return xs;
    }

    std::vector<Interval> horizontal_inside(const Polygon& boundary,
        const std::vector<Polygon>& obstacles,
        double y) {
        auto xs = scanline_intersections(boundary, y);
        std::vector<std::pair<double, double>> inside;
        for (int i = 0; i + 1 < (int)xs.size(); i += 2) {
            double a = xs[i], b = xs[i + 1];
            if (b <= a + EPS) continue;
            inside.push_back({ a,b });
        }
        if (inside.empty()) return {};
        for (auto& obs : obstacles) {
            auto xo = scanline_intersections(obs, y);
            std::vector<std::pair<double, double>> obs_int;
            for (int i = 0; i + 1 < (int)xo.size(); i += 2) {
                double lo = xo[i], hi = xo[i + 1];
                if (hi <= lo + EPS) continue;
                obs_int.push_back({ lo,hi });
            }
            if (obs_int.empty()) continue;
            std::vector<std::pair<double, double>> New;
            for (auto& AB : inside) {
                double A = AB.first, B = AB.second;
                double cur = A;
                for (auto& oh : obs_int) {
                    double oA = oh.first, oB = oh.second;
                    if (oB <= cur + EPS || oA >= B - EPS) continue;
                    if (oA > cur + EPS) {
                        New.push_back({ cur, std::min(oA,B) });
                    }
                    cur = std::max(cur, oB);
                    if (cur >= B - EPS) break;
                }
                if (cur < B - EPS) New.push_back({ cur,B });
            }
            inside.swap(New);
            if (inside.empty()) break;
        }
        std::sort(inside.begin(), inside.end());
        std::vector<Interval> merged;
        for (auto& p : inside) {
            if (merged.empty() || p.first > merged.back().second + EPS) {
                merged.push_back(p);
            }
            else {
                merged.back().second = std::max(merged.back().second, p.second);
            }
        }
        return merged;
    }

    std::vector<double> scan_vertical(const Polygon& poly, double x0) {
        std::vector<double> ys;
        int n = (int)poly.size();
        for (int i = 0; i < n; i++) {
            const Point& a = poly[i];
            const Point& b = poly[(i + 1) % n];
            if ((a.x - x0) * (b.x - x0) <= 0) {
                if (std::fabs(a.x - b.x) < EPS) {
                    ys.push_back(a.y);
                    ys.push_back(b.y);
                }
                else {
                    double t = (x0 - a.x) / (b.x - a.x);
                    if (-EPS <= t && t <= 1 + EPS) {
                        ys.push_back(a.y + (b.y - a.y) * t);
                    }
                }
            }
        }
        std::sort(ys.begin(), ys.end());
        return ys;
    }

    std::vector<Interval> vertical_inside(const Polygon& boundary,
        const std::vector<Polygon>& obstacles,
        double x) {
        auto ys = scan_vertical(boundary, x);
        std::vector<std::pair<double, double>> inside;
        for (int i = 0; i + 1 < (int)ys.size(); i += 2) {
            double a = ys[i], b = ys[i + 1];
            if (b <= a + EPS) continue;
            inside.push_back({ a,b });
        }
        if (inside.empty()) return {};
        for (auto& obs : obstacles) {
            auto yo = scan_vertical(obs, x);
            std::vector<std::pair<double, double>> obs_int;
            for (int i = 0; i + 1 < (int)yo.size(); i += 2) {
                double lo = yo[i], hi = yo[i + 1];
                if (hi <= lo + EPS) continue;
                obs_int.push_back({ lo,hi });
            }
            if (obs_int.empty()) continue;
            std::vector<std::pair<double, double>> New;
            for (auto& AB : inside) {
                double A = AB.first, B = AB.second;
                double cur = A;
                for (auto& oh : obs_int) {
                    double oA = oh.first, oB = oh.second;
                    if (oB <= cur + EPS || oA >= B - EPS) continue;
                    if (oA > cur + EPS) {
                        New.push_back({ cur, std::min(oA,B) });
                    }
                    cur = std::max(cur, oB);
                    if (cur >= B - EPS) break;
                }
                if (cur < B - EPS) New.push_back({ cur,B });
            }
            inside.swap(New);
            if (inside.empty()) break;
        }
        std::sort(inside.begin(), inside.end());
        std::vector<Interval> merged;
        for (auto& p : inside) {
            if (merged.empty() || p.first > merged.back().second + EPS) {
                merged.push_back(p);
            }
            else {
                merged.back().second = std::max(merged.back().second, p.second);
            }
        }
        return merged;
    }

    std::vector<double> cluster_vals(const std::vector<double>& values, double tol) {
        std::vector<double> s = values;
        std::sort(s.begin(), s.end());
        if (s.empty()) return {};
        std::vector<double> out;
        std::vector<double> run;
        run.push_back(s[0]);
        for (size_t i = 1; i < s.size(); ++i) {
            double v = s[i];
            if (std::fabs(v - run.back()) <= tol) {
                run.push_back(v);
            }
            else {
                double sum = 0;
                for (auto x : run) sum += x;
                out.push_back(sum / run.size());
                run.clear();
                run.push_back(v);
            }
        }
        double sum = 0;
        for (auto x : run) sum += x;
        out.push_back(sum / run.size());
        return out;
    }

    bool point_clear(const Point& p,
        const Polygon& boundary,
        const std::vector<Polygon>& obstacles,
        double clearance) {
        if (clearance <= 0) {
            // ������ ���� Python (�������� ���������, �� ��������� ��� ����):
            return !point_in_poly_strict(p, boundary);
        }
        if (!point_in_poly_strict(p, boundary)) return false;
        auto check_edges = [&](const Polygon& poly) {
            int n = (int)poly.size();
            for (int i = 0; i < n; i++) {
                const Point& a = poly[i];
                const Point& b = poly[(i + 1) % n];
                if (dist_point_seg(p, a, b) < clearance - EPS) {
                    return false;
                }
            }
            return true;
            };
        for (size_t i = 0; i < 1 + obstacles.size(); ++i) {
            const Polygon* poly = (i == 0 ? &boundary : &obstacles[i - 1]);
            if (poly != &boundary && point_in_poly_non_strict(p, *poly)) {
                return false;
            }
            if (!check_edges(*poly)) return false;
        }
        return true;
    }

    bool segment_clear(const Point& a, const Point& b,
        const Polygon& boundary,
        const std::vector<Polygon>& obstacles,
        double clearance) {
        for (auto& obs : obstacles) {
            int n = (int)obs.size();
            for (int i = 0; i < n; i++) {
                const Point& c = obs[i];
                const Point& d = obs[(i + 1) % n];
                if (segments_strict_intersect(a, b, c, d)) return false;
            }
        }
        double L = std::hypot(a.x - b.x, a.y - b.y);
        if (clearance <= 0 && L < 1e-15) return true;
        if (L < EPS) return false;
        double base = (VIS_ENABLE_LENIENCY
            ? std::max(clearance, L / (VIS_SAMPLE_PER_SEG_PER_UNIT * 10.0))
            : std::max(clearance, 0.0));
        double denom = std::max(base, 1e-12);
        int samples = std::max(3, (int)(L / denom));
        samples = std::min(samples, 120);
        for (int i = 1; i < samples; i++) {
            double t = (double)i / samples;
            Point p(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
            if (!point_clear(p, boundary, obstacles, clearance)) {
                return false;
            }
        }
        return true;
    }

    std::vector<Polygon> polys;
    std::vector<Point> resNodes;
    std::vector<Edge> resEdges;
};