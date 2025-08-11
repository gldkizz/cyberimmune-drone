#pragma once
#include <string>

static const std::string BOUNDARY_MODE = "auto"; // "largest" | "hull" | "auto"

static const double CLEARANCE = 0.000002;
static const double MERGE_FACTOR = 0.01;
static const double MIN_CELL_FACTOR = 0.02;
static const int    MAX_MERGE_ITERS = 3;
static const int    CONTRACT_ITERS = 2;
static const bool   KEEP_BOUNDARY_NODES = false;
static const bool   WRITE_EDGE_LEN = true;
static const bool   VERBOSE = true;

static const double INTERIOR_MARGIN_FACTOR = 0;
static const double FILTER_THIN_BANDS = 0;
static const int    MAX_EMPTY_REBUILD = 1;

// Visibility augmentation
static const bool ADD_VISIBILITY_EDGES = true;
static const bool VISIBILITY_CONNECT_COMPONENTS = true;
static const bool VISIBILITY_DENSIFY = true;

static const int   VIS_MAX_COMPONENT_LINKS = 50;
static const int   VIS_SAMPLE_PER_SEG_PER_UNIT = 40;
static const int   VIS_COMP_PAIR_SEARCH_LIMIT = 2000;
static const bool  VIS_RECHECK_AFTER_EACH_LINK = true;

static const int   VIS_LOCAL_K = 6;
static const double VIS_LOCAL_RADIUS_FACTOR = 0.25;
static const int   VIS_LOCAL_MAX_NEW_EDGES = 150;
static const int   VIS_LOCAL_MAX_DEG_INCREASE = 0;

static const int   VIS_TOTAL_NEW_EDGES_LIMIT = 300;
static const bool  VIS_ENABLE_LENIENCY = true;

static const double EPS = 1e-12;