#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <stdbool.h>

// ----------------- Visualizer ----------------------------
#define MAX_SIZE 100
#define INF INT_MAX

typedef struct
{
    int x, y;
} Node;

void initGrid();
bool isValid(int x, int y);
void dijkstra_visual();
void bfs_visual();
void dfs_visual();
void printPath();
void print_path(int *predecessors, int start_node, int end_node);

// ------------------- Graph Structures -------------------
typedef struct mygraph graph;

typedef struct mygraph {
    int numnodes;
    bool **edges;
} graph;

// For Minimum Spanning Tree (Prim's Algorithm)
typedef struct {
    int from;
    int to;
    int weight;
} edge;



// ------------------- Graph Function Declarations -------------------
graph *create_graph(int numnodes);
void destroy_graph(graph *g);
void print_graph(graph *g);
bool add_edge(graph *g, unsigned int from_node, unsigned int to_node);
bool has_edge(graph *g, unsigned int from_node, unsigned int to_node);

void bfs(graph *g, int start_node);
void dfs(graph *g, int start_node);
bool is_cyclic(graph *g);
int *shortest_path_dijkstra(graph *g, int start_node, int end_node, int **predecessors);
graph *transpose_graph(graph *g);

//add in v1-version
bool remove_edge(graph *g, unsigned int from_node, unsigned int to_node);
int get_in_degree(graph *g, unsigned int node);
int get_out_degree(graph *g, unsigned int node);
// int *topological_sort(graph *g);

// void fill_order(graph *g, int node, bool *visited, int *stack, int *stack_index);

// int *get_strongly_connected_components(graph *g);
edge *get_minimum_spanning_tree(graph *g);

// edge* find_bridges(graph *g, int *num_bridges);

// int* color_graph(graph *g);

bool has_path(graph *g, int start, int end);

bool is_connected(graph *g);

// bool is_bipartite(graph *g);

graph* clone_graph(graph *g);

//########################## Path Findings start from here #################################

// path_list* create_path_list(int initial_capacity);
// path* create_path();
// void add_to_path(path* p, int vertex);
// path* clone_path(path* p);
// void add_path_to_list(path_list* list, path* p);
// void destroy_path(path* p);
// void destroy_path_list(path_list* list);
// path_list* find_all_paths(graph* g, int start, int end);
// bool* find_articulation_points(graph* g);
// bool has_euler_path(graph* g);
// bool has_euler_circuit(graph* g);
// int get_graph_diameter(graph* g);
// int* get_isolated_vertices(graph* g, int* count);
// bool is_tree(graph* g);
// bool is_complete(graph* g);

//########################## Menu Functions start from here #################################
void show_graph_menu();


