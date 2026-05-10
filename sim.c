#include <raylib.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

typedef struct { int dst; int weight; } Edge;
typedef struct { Edge *edges; int count; int capacity; } EdgeList;
typedef struct { int numNodes; int numEdges; int querySrc; int queryDst; EdgeList *adjList; } Graph;

#define WIN_W 1000
#define WIN_H 720
#define NODE_R 26
#define MAX_N 15
#define PIV 3.14159265f

/* ---------- Milestone 3: animation timing constants ---------- */
#define NODE_WAIT 1.0f   /* 1 second wait at intermediate nodes  */
#define JUMP_TIME 0.3f   /* 300 ms per jump on an edge           */

static float nx[MAX_N], ny[MAX_N];

static void ginit(Graph*g){g->numNodes=0;g->numEdges=0;g->querySrc=-1;g->queryDst=-1;g->adjList=NULL;}
static void gfree(Graph*g){
    if(!g||!g->adjList)return;
    for(int i=0;i<g->numNodes;i++)free(g->adjList[i].edges);
    free(g->adjList);g->adjList=NULL;
}
static void epush(EdgeList*el,int d,int w){
    if(el->count==el->capacity){
        int nc=(el->capacity==0)?4:el->capacity*2;
        el->edges=(Edge*)realloc(el->edges,(size_t)nc*sizeof(Edge));
        el->capacity=nc;
    }
    el->edges[el->count].dst=d;el->edges[el->count].weight=w;el->count++;
}
static int gload(Graph*g,const char*fn){
    FILE*f=fopen(fn,"r");if(!f){fprintf(stderr,"Cannot open %s\n",fn);return 0;}
    int N,M;
    if(fscanf(f,"%d %d",&N,&M)!=2){fprintf(stderr,"Bad header\n");fclose(f);return 0;}
    if(N<=0||N>MAX_N){fprintf(stderr,"Invalid nodes\n");fclose(f);return 0;}
    g->numNodes=N;g->numEdges=M;
    g->adjList=(EdgeList*)calloc((size_t)N,sizeof(EdgeList));
    for(int i=0;i<M;i++){
        int s,d,w;
        if(fscanf(f,"%d %d %d",&s,&d,&w)!=3){fprintf(stderr,"Bad edge %d\n",i);gfree(g);fclose(f);return 0;}
        if(s<0||d<0||w<0){fprintf(stderr,"Negative number not allowed\n");gfree(g);fclose(f);return 0;}
        if(s>=N||d>=N){fprintf(stderr,"Node index out of range\n");gfree(g);fclose(f);return 0;}
        epush(&g->adjList[s],d,w);
    }
    if(fscanf(f,"%d %d",&g->querySrc,&g->queryDst)!=2){fprintf(stderr,"Bad query\n");gfree(g);fclose(f);return 0;}
    fclose(f);return 1;
}
static int dijk(const Graph*g,int src,int dst,int*path,int*plen,long long*wt){
    int N=g->numNodes;
    long long*dist=(long long*)malloc((size_t)N*sizeof(long long));
    int*prev=(int*)malloc((size_t)N*sizeof(int));
    int*vis=(int*)calloc((size_t)N,sizeof(int));
    for(int i=0;i<N;i++){dist[i]=LLONG_MAX;prev[i]=-1;}
    dist[src]=0;
    for(int iter=0;iter<N;iter++){
        int u=-1;
        for(int i=0;i<N;i++)if(!vis[i]&&dist[i]!=LLONG_MAX&&(u==-1||dist[i]<dist[u]))u=i;
        if(u==-1)break;
        vis[u]=1;
        for(int k=0;k<g->adjList[u].count;k++){
            int v=g->adjList[u].edges[k].dst,w=g->adjList[u].edges[k].weight;
            if(dist[u]+w<dist[v]){dist[v]=dist[u]+w;prev[v]=u;}
        }
    }
    if(dist[dst]==LLONG_MAX){free(dist);free(prev);free(vis);return 0;}
    int tmp[MAX_N],len=0,cur=dst;
    while(cur!=-1){tmp[len++]=cur;if(cur==src)break;cur=prev[cur];}
    for(int i=0;i<len;i++)path[i]=tmp[len-1-i];
    *plen=len;*wt=dist[dst];free(dist);free(prev);free(vis);return 1;
}
static void cpos(int n){
    float cx=WIN_W/2.0f,cy=(WIN_H-80)/2.0f;
    if(n==1){nx[0]=cx;ny[0]=cy;return;}
    if(n<=8){float r=220;for(int i=0;i<n;i++){float a=-PIV/2+2*PIV*i/n;nx[i]=cx+r*cosf(a);ny[i]=cy+r*sinf(a);}}
    else{int in=n/2,ou=n-in;float r1=120,r2=255;
        for(int i=0;i<in;i++){float a=-PIV/2+2*PIV*i/in;nx[i]=cx+r1*cosf(a);ny[i]=cy+r1*sinf(a);}
        for(int i=0;i<ou;i++){float a=-PIV/2+2*PIV*i/ou;nx[in+i]=cx+r2*cosf(a);ny[in+i]=cy+r2*sinf(a);}
    }
}
static void darrow(int u,int v,Color c,float t){
    float dx=nx[v]-nx[u],dy=ny[v]-ny[u],l=sqrtf(dx*dx+dy*dy);if(l<1)return;
    float ux2=dx/l,uy2=dy/l,sx=nx[u]+ux2*(NODE_R+2),sy=ny[u]+uy2*(NODE_R+2),ex=nx[v]-ux2*(NODE_R+2),ey=ny[v]-uy2*(NODE_R+2);
    DrawLineEx((Vector2){sx,sy},(Vector2){ex,ey},t,c);
    float ar=13,aw=7,px2=-uy2,py2=ux2;
    DrawTriangle((Vector2){ex,ey},(Vector2){ex-ux2*ar+px2*aw,ey-uy2*ar+py2*aw},(Vector2){ex-ux2*ar-px2*aw,ey-uy2*ar-py2*aw},c);
}
static int oedge(int*p,int pl,int u,int v){for(int i=0;i<pl-1;i++)if(p[i]==u&&p[i+1]==v)return 1;return 0;}
static int onode(int*p,int pl,int n){for(int i=0;i<pl;i++)if(p[i]==n)return 1;return 0;}

/* =============================================================
   MILESTONE 3: animation state and helpers
   ============================================================= */

typedef enum { ANIM_AT_NODE, ANIM_ON_EDGE, ANIM_FINISHED } AnimState;

/* Animation runtime state (all start at "sitting at the source"). */
static AnimState animState   = ANIM_AT_NODE;
static int       animPathIdx = 0;     /* index into path[] of the node we are at / leaving from */
static int       animJump    = 0;     /* completed jumps on the current edge (0..W) */
static float     animTimer   = 0.0f;  /* seconds spent in the current sub-state    */
static int       animPlay    = 0;     /* 1 = playing, 0 = paused                    */
static float     entityX     = 0.0f;  /* on-screen position of the moving entity    */
static float     entityY     = 0.0f;

/* Look up the weight of edge u -> v in the adjacency list. */
static int edge_weight(const Graph* g, int u, int v) {
    for (int k = 0; k < g->adjList[u].count; k++) {
        if (g->adjList[u].edges[k].dst == v) return g->adjList[u].edges[k].weight;
    }
    return 1; /* fallback; should not happen for a valid Dijkstra path */
}

/* Reset animation back to the source so the user can replay. */
static void anim_reset(void) {
    animState   = ANIM_AT_NODE;
    animPathIdx = 0;
    animJump    = 0;
    animTimer   = 0.0f;
    animPlay    = 0;
}

/* Advance the animation by dt seconds. Called once per frame. */
static void anim_update(const Graph* g, const int* path, int plen, float dt) {
    if (!animPlay)              return;
    if (plen <= 1)              return; /* nothing to animate (no path / src==dst) */
    if (animState == ANIM_FINISHED) return;

    animTimer += dt;

    /* --- waiting at a node --- */
    if (animState == ANIM_AT_NODE) {
        if (animPathIdx >= plen - 1) {
            animState = ANIM_FINISHED;          /* reached destination */
            return;
        }
        if (animPathIdx == 0) {
            /* source: leave immediately, no waiting */
            animState = ANIM_ON_EDGE;
            animJump  = 0;
            animTimer = 0.0f;
        } else {
            /* intermediate node: wait exactly 1 second */
            if (animTimer >= NODE_WAIT) {
                animState = ANIM_ON_EDGE;
                animJump  = 0;
                animTimer = 0.0f;
            }
        }
    }

    /* --- moving along an edge --- */
    if (animState == ANIM_ON_EDGE) {
        int u = path[animPathIdx];
        int v = path[animPathIdx + 1];
        int W = edge_weight(g, u, v);
        if (W < 1) W = 1;

        /* consume any whole jumps that have elapsed (W jumps x 300 ms each = W*300 ms) */
        while (animTimer >= JUMP_TIME && animState == ANIM_ON_EDGE) {
            animTimer -= JUMP_TIME;
            animJump++;
            if (animJump >= W) {
                /* arrived at the next node */
                animPathIdx++;
                animState = ANIM_AT_NODE;
                animJump  = 0;
                animTimer = 0.0f;
            }
        }
    }
}

/* Compute the entity's current screen position based on the animation state. */
static void anim_compute_position(const Graph* g, const int* path, int plen) {
    if (plen == 0) return;

    if (animState == ANIM_AT_NODE || animState == ANIM_FINISHED) {
        int n = path[animPathIdx];
        entityX = nx[n];
        entityY = ny[n];
    } else { /* ANIM_ON_EDGE */
        int u = path[animPathIdx];
        int v = path[animPathIdx + 1];
        int W = edge_weight(g, u, v);
        if (W < 1) W = 1;

        /* sub-progress within the current jump (0..1), used for smooth motion */
        float subprogress = animTimer / JUMP_TIME;
        if (subprogress > 1.0f) subprogress = 1.0f;

        /* total progress along the edge: completed jumps + current sub-jump */
        float t = ((float)animJump + subprogress) / (float)W;
        if (t > 1.0f) t = 1.0f;

        entityX = nx[u] + (nx[v] - nx[u]) * t;
        entityY = ny[u] + (ny[v] - ny[u]) * t;
    }
}

int main(int argc,char*argv[]){
    if(argc<2){fprintf(stderr,"Usage: ./sim <file>\n");return 1;}
    Graph g;ginit(&g);
    if(!gload(&g,argv[1])){gfree(&g);return 1;}
    cpos(g.numNodes);

    int path[MAX_N], plen=0;
    long long weight=0;
    if(g.querySrc==g.queryDst){path[0]=g.querySrc;plen=1;}
    else dijk(&g,g.querySrc,g.queryDst,path,&plen,&weight);

    /* initialize the entity to sit at the source node */
    if (plen >= 1) { entityX = nx[path[0]]; entityY = ny[path[0]]; }

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(WIN_W,WIN_H,"OS Project - Graph Animation");
    SetTargetFPS(60);

    Color CSRC={50,200,110,255},CDST={220,70,70,255},CPT={255,190,50,255},CN={40,110,210,255};
    Color CE={60,75,115,255},CT={210,220,240,255},CW={130,155,200,255};
    Color CENT={255,240,70,255}; /* moving entity color (bright yellow) */

    /* Play/Stop button rectangle, in the bottom panel. */
    Rectangle btnRect = { WIN_W - 150.0f, WIN_H - 65.0f, 130.0f, 50.0f };

    while(!WindowShouldClose()){
        float dt = GetFrameTime();

        /* ---- handle the Play/Stop/Restart button ---- */
        Vector2 mp = GetMousePosition();
        int btnHover = CheckCollisionPointRec(mp, btnRect);
        if (btnHover && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            if (animState == ANIM_FINISHED) {
                anim_reset();
                animPlay = 1; /* restart and immediately play */
            } else {
                animPlay = !animPlay; /* toggle play/pause */
            }
        }
        /* spacebar also toggles, as a convenience */
        if (IsKeyPressed(KEY_SPACE)) {
            if (animState == ANIM_FINISHED) { anim_reset(); animPlay = 1; }
            else animPlay = !animPlay;
        }

        /* ---- update animation and entity position ---- */
        anim_update(&g, path, plen, dt);
        anim_compute_position(&g, path, plen);

        /* ---- draw everything ---- */
        BeginDrawing();
        ClearBackground((Color){12,15,26,255});

        /* background grid dots */
        for(int x=0;x<WIN_W;x+=38)
            for(int y=0;y<WIN_H-80;y+=38)
                DrawCircle(x,y,1,(Color){30,38,65,255});

        DrawText("OS Project - Graph Animation",18,14,20,(Color){90,115,185,255});

        /* draw all edges with their weights */
        for(int u=0;u<g.numNodes;u++)for(int k=0;k<g.adjList[u].count;k++){
            int v=g.adjList[u].edges[k].dst,w=g.adjList[u].edges[k].weight,h=oedge(path,plen,u,v);
            darrow(u,v,h?CPT:CE,h?3.0f:1.6f);
            char ws[8];snprintf(ws,8,"%d",w);
            DrawText(ws,(int)((nx[u]+nx[v])/2+10),(int)((ny[u]+ny[v])/2-10),15,CW);
        }

        /* draw all nodes (source green, destination red, on-path orange, others blue) */
        for(int i=0;i<g.numNodes;i++){
            Color nc=(i==g.querySrc)?CSRC:(i==g.queryDst)?CDST:onode(path,plen,i)?CPT:CN;
            Color gl=nc;gl.a=45;
            DrawCircle((int)nx[i],(int)ny[i],NODE_R+9,gl);
            DrawCircle((int)nx[i],(int)ny[i],NODE_R,nc);
            DrawCircleLines((int)nx[i],(int)ny[i],NODE_R,WHITE);
            char lb[8];snprintf(lb,8,"%d",i);
            DrawText(lb,(int)nx[i]-MeasureText(lb,19)/2,(int)ny[i]-9,19,CT);
        }

        /* draw the moving entity on top of nodes/edges */
        if (plen >= 1) {
            DrawCircle((int)entityX, (int)entityY, 13, CENT);
            DrawCircleLines((int)entityX, (int)entityY, 13, BLACK);
            DrawCircleLines((int)entityX, (int)entityY, 12, BLACK);
        }

        /* arrival message */
        if (animState == ANIM_FINISHED) {
            const char* msg = "Arrived at destination!";
            int msgW = MeasureText(msg, 30);
            int boxX = WIN_W/2 - msgW/2 - 25;
            int boxY = 28;
            int boxW = msgW + 50;
            int boxH = 50;
            DrawRectangle(boxX, boxY, boxW, boxH, (Color){10,50,25,230});
            DrawRectangleLines(boxX, boxY, boxW, boxH, (Color){80,220,100,255});
            DrawText(msg, WIN_W/2 - msgW/2, boxY + 10, 30, CENT);
        }

        /* bottom panel background */
        DrawRectangle(0,WIN_H-80,WIN_W,80,(Color){20,24,40,255});
        DrawLine(0,WIN_H-80,WIN_W,WIN_H-80,(Color){45,55,90,255});

        /* path text */
        char ps[512]="Path: ";
        if(plen==0)strcat(ps,"No path found");
        else for(int i=0;i<plen;i++){char t[12];if(i)strcat(ps," -> ");snprintf(t,12,"%d",path[i]);strcat(ps,t);}
        DrawText(ps,18,WIN_H-65,17,CT);
        if(plen>1){char ws[64];snprintf(ws,64,"Weight: %lld",weight);DrawText(ws,18,WIN_H-42,16,CW);}

        /* status indicator */
        const char* status =
            (animState == ANIM_FINISHED) ? "Status: Arrived" :
            (animPlay)                   ? "Status: Playing" :
                                           "Status: Paused";
        DrawText(status, 18, WIN_H - 22, 15, CW);

        /* draw the play/stop/restart button */
        Color btnCol;
        const char* btnText;
        if (animState == ANIM_FINISHED) { btnCol = (Color){80,130,220,255};  btnText = "RESTART"; }
        else if (animPlay)              { btnCol = (Color){200,80,80,255};   btnText = "STOP";    }
        else                            { btnCol = (Color){80,180,100,255};  btnText = "PLAY";    }
        if (btnHover) { /* lighten on hover */
            int r = btnCol.r + 50; if (r > 255) r = 255;
            int gC = btnCol.g + 50; if (gC > 255) gC = 255;
            int b = btnCol.b + 50; if (b > 255) b = 255;
            btnCol.r = (unsigned char)r; btnCol.g = (unsigned char)gC; btnCol.b = (unsigned char)b;
        }
        DrawRectangleRec(btnRect, btnCol);
        DrawRectangleLinesEx(btnRect, 2, WHITE);
        int twid = MeasureText(btnText, 22);
        DrawText(btnText, (int)(btnRect.x + (btnRect.width - twid)/2),
                          (int)(btnRect.y + 14), 22, WHITE);

        EndDrawing();
    }
    CloseWindow();gfree(&g);return 0;
}
