#include <limits.h>
#include <stdio.h>
#include <stdlib.h>


typedef struct {
    int dst;
    int weight;
} Edge;

typedef struct {
    Edge* edges;        /* dynamic array of outgoing edges */
    int   count;
    int   capacity;
} EdgeList;

typedef struct {
    int       numNodes;
    int       numEdges;
    int       querySrc;
    int       queryDst;
    EdgeList* adjList;  /* array of length numNodes */
} Graph;

static void graph_init(Graph* g) {
    g->numNodes = 0;
    g->numEdges = 0;
    g->querySrc = -1;
    g->queryDst = -1;
    g->adjList  = NULL;
}

static void graph_free(Graph* g) {
    if (!g || !g->adjList) {
        if (g) g->adjList = NULL;
        return;
    }
    for (int i = 0; i < g->numNodes; ++i) {
        free(g->adjList[i].edges);
    }
    free(g->adjList);
    g->adjList  = NULL;
    g->numNodes = 0;
    g->numEdges = 0;
}

/* Append one edge to a node's outgoing list (grow-by-doubling). */
static int edgelist_push(EdgeList* el, int dst, int weight) {
    if (el->count == el->capacity) {
        int new_cap = (el->capacity == 0) ? 4 : el->capacity * 2;
        Edge* tmp = (Edge*)realloc(el->edges, (size_t)new_cap * sizeof(Edge));
        if (!tmp) return 0;
        el->edges = tmp;
        el->capacity = new_cap;
    }
    el->edges[el->count].dst    = dst;
    el->edges[el->count].weight = weight;
    el->count++;
    return 1;
}


 // Binary min-heap of (distance, node), used by D
typedef struct {
    long long dist;
    int       node;
} HeapNode;

typedef struct {
    HeapNode* data;
    int       size;
    int       capacity;
} MinHeap;

static int heap_init(MinHeap* h, int cap) {
    if (cap < 16) cap = 16;
    h->data = (HeapNode*)malloc((size_t)cap * sizeof(HeapNode));
    if (!h->data) return 0;
    h->size = 0;
    h->capacity = cap;
    return 1;
}

static void heap_free(MinHeap* h) {
    free(h->data);
    h->data = NULL;
    h->size = 0;
    h->capacity = 0;
}

static void heap_swap(HeapNode* a, HeapNode* b) {
    HeapNode t = *a;
    *a = *b;
    *b = t;
}

static int heap_push(MinHeap* h, long long dist, int node) {
    if (h->size == h->capacity) {
        int nc = h->capacity * 2;
        HeapNode* tmp = (HeapNode*)realloc(h->data, (size_t)nc * sizeof(HeapNode));
        if (!tmp) return 0;
        h->data = tmp;
        h->capacity = nc;
    }
    int i = h->size++;
    h->data[i].dist = dist;
    h->data[i].node = node;
    /* sift up */
    while (i > 0) {
        int p = (i - 1) / 2;
        if (h->data[p].dist <= h->data[i].dist) break;
        heap_swap(&h->data[p], &h->data[i]);
        i = p;
    }
    return 1;
}

static HeapNode heap_pop(MinHeap* h) {
    HeapNode top = h->data[0];
    h->size--;
    if (h->size > 0) {
        h->data[0] = h->data[h->size];
        /* sift down */
        int i = 0;
        for (;;) {
            int l = 2 * i + 1;
            int r = 2 * i + 2;
            int smallest = i;
            if (l < h->size && h->data[l].dist < h->data[smallest].dist) smallest = l;
            if (r < h->size && h->data[r].dist < h->data[smallest].dist) smallest = r;
            if (smallest == i) break;
            heap_swap(&h->data[i], &h->data[smallest]);
            i = smallest;
        }
    }
    return top;
}
//  File loader

static int graph_load_from_file(Graph* g, const char* filename) {
    FILE* f = fopen(filename, "r");
    if (!f) {
        fprintf(stderr, "Error: cannot open file '%s'.\n", filename);
        return 0;
    }

    int N, M;
    if (fscanf(f, "%d %d", &N, &M) != 2) {
        fprintf(stderr, "Error: invalid file format (expected 'N M' on first line).\n");
        fclose(f);
        return 0;
    }
    if (N <= 0) {
        fprintf(stderr, "Error: number of nodes must be positive (got %d).\n", N);
        fclose(f);
        return 0;
    }
    if (M < 0) {
        fprintf(stderr, "Error: number of edges cannot be negative (got %d).\n", M);
        fclose(f);
        return 0;
    }

    g->numNodes = N;
    g->numEdges = M;
    g->adjList  = (EdgeList*)calloc((size_t)N, sizeof(EdgeList));
    if (!g->adjList) {
        fprintf(stderr, "Error: out of memory.\n");
        fclose(f);
        g->numNodes = 0;
        g->numEdges = 0;
        return 0;
    }

    for (int i = 0; i < M; ++i) {
        int s, d, w;
        if (fscanf(f, "%d %d %d", &s, &d, &w) != 3) {
            fprintf(stderr,
                    "Error: invalid edge format at edge index %d "
                    "(expected 'src dst weight').\n", i);
            fclose(f);
            graph_free(g);
            return 0;
        }
        if (s < 0 || d < 0 || w < 0) {
            fprintf(stderr,
                    "Error: negative numbers are not allowed "
                    "(edge %d: %d %d %d).\n", i, s, d, w);
            fclose(f);
            graph_free(g);
            return 0;
        }
        if (s >= N || d >= N) {
            fprintf(stderr,
                    "Error: node index out of range at edge %d "
                    "(src=%d, dst=%d, numNodes=%d).\n", i, s, d, N);
            fclose(f);
            graph_free(g);
            return 0;
        }
        if (!edgelist_push(&g->adjList[s], d, w)) {
            fprintf(stderr, "Error: out of memory.\n");
            fclose(f);
            graph_free(g);
            return 0;
        }
    }

    if (fscanf(f, "%d %d", &g->querySrc, &g->queryDst) != 2) {
        fprintf(stderr,
                "Error: missing or invalid query line "
                "(expected 'src dst' after the edges).\n");
        fclose(f);
        graph_free(g);
        return 0;
    }
    if (g->querySrc < 0 || g->queryDst < 0) {
        fprintf(stderr,
                "Error: negative numbers are not allowed (query: %d %d).\n",
                g->querySrc, g->queryDst);
        fclose(f);
        graph_free(g);
        return 0;
    }
    if (g->querySrc >= N || g->queryDst >= N) {
        fprintf(stderr,
                "Error: query node index out of range "
                "(src=%d, dst=%d, numNodes=%d).\n",
                g->querySrc, g->queryDst, N);
        fclose(f);
        graph_free(g);
        return 0;
    }

    fclose(f);
    return 1;
}


 //  Dijkstra
 //  Returns 1 on success and fills out_path / out_path_len / out_weight.
 // Returns 0 if no path exists. out_path must be at least numNodes long.

static int graph_dijkstra(const Graph* g,
                          int src, int dst,
                          int*       out_path,
                          int*       out_path_len,
                          long long* out_weight)
{
    int N = g->numNodes;

    long long* dist = (long long*)malloc((size_t)N * sizeof(long long));
    int*       prev = (int*)malloc((size_t)N * sizeof(int));
    if (!dist || !prev) {
        fprintf(stderr, "Error: out of memory.\n");
        free(dist);
        free(prev);
        return 0;
    }
    for (int i = 0; i < N; ++i) {
        dist[i] = LLONG_MAX;
        prev[i] = -1;
    }
    dist[src] = 0;

    MinHeap pq;
    if (!heap_init(&pq, g->numEdges + N + 1)) {
        fprintf(stderr, "Error: out of memory.\n");
        free(dist);
        free(prev);
        return 0;
    }
    heap_push(&pq, 0, src);

    while (pq.size > 0) {
        HeapNode top = heap_pop(&pq);
        long long d = top.dist;
        int       u = top.node;

        if (d > dist[u]) continue;   /* stale entry */
        if (u == dst)    break;      /* finalized destination */

        const EdgeList* el = &g->adjList[u];
        for (int k = 0; k < el->count; ++k) {
            int v = el->edges[k].dst;
            int w = el->edges[k].weight;
            long long nd = dist[u] + (long long)w;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                if (!heap_push(&pq, nd, v)) {
                    fprintf(stderr, "Error: out of memory.\n");
                    heap_free(&pq);
                    free(dist);
                    free(prev);
                    return 0;
                }
            }
        }
    }

    if (dist[dst] == LLONG_MAX) {
        heap_free(&pq);
        free(dist);
        free(prev);
        return 0;   /* no path */
    }

    /* count path length by walking prev[] from dst back to src */
    int len = 0;
    {
        int cur = dst;
        while (cur != -1) {
            len++;
            if (cur == src) break;
            cur = prev[cur];
        }
    }
    /* fill out_path[] in forward order */
    {
        int at = dst;
        for (int i = len - 1; i >= 0; --i) {
            out_path[i] = at;
            at = prev[at];
        }
    }
    *out_path_len = len;
    *out_weight   = dist[dst];

    heap_free(&pq);
    free(dist);
    free(prev);
    return 1;
}

int main(int argc, char* argv[]) {
    const char* filename = "input.txt";
    if (argc >= 2) {
        filename = argv[1];
    }

    Graph g;
    graph_init(&g);

    if (!graph_load_from_file(&g, filename)) {
        graph_free(&g);
        return 1;
    }

    int* path = (int*)malloc((size_t)g.numNodes * sizeof(int));
    if (!path) {
        fprintf(stderr, "Error: out of memory.\n");
        graph_free(&g);
        return 1;
    }

    int       path_len = 0;
    long long weight   = 0;

    int ok = graph_dijkstra(&g, g.querySrc, g.queryDst,path, &path_len, &weight);

    if (!ok) {
        printf("No path found\n");
    } else {
        for (int i = 0; i < path_len; ++i) {
            printf("%d", path[i]);
            if (i + 1 < path_len) {
                printf(" -> ");
            }
        }
        printf("\n");
        printf("%lld\n", weight);
    }

    free(path);
    graph_free(&g);
    return 0;
}