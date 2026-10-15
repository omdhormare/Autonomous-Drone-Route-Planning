#include "navigation.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <random>
#include <fstream>
#include <stdexcept>
#include <limits>

std::vector<Point> astar(int w,int h,const std::vector<Point>& obstacles,Point start,Point goal,double proximityPenalty){
 if(w<=0||h<=0||w>200||h>200) throw std::invalid_argument("grid must be 1..200");
 auto inside=[&](Point p){return p.x>=0&&p.y>=0&&p.x<w&&p.y<h;};
 if(!inside(start)||!inside(goal))return {};
 const int n=w*h;auto id=[&](Point p){return p.y*w+p.x;};
 std::vector<bool> blocked(n,false);
 for(auto p:obstacles)if(inside(p))blocked[id(p)]=true;
 if(blocked[id(start)]||blocked[id(goal)])return {};
 struct Node{double f,g;int index;};
 auto cmp=[](const Node&a,const Node&b){return a.f>b.f;};
 std::priority_queue<Node,std::vector<Node>,decltype(cmp)> open(cmp);
 std::vector<double> cost(n,std::numeric_limits<double>::infinity());std::vector<int> parent(n,-1);
 auto heuristic=[&](Point p){return std::abs(p.x-goal.x)+std::abs(p.y-goal.y);};
 int s=id(start),z=id(goal);cost[s]=0;open.push({double(heuristic(start)),0,s});
 const int dx[]={1,0,-1,0},dy[]={0,1,0,-1};
 while(!open.empty()){
  auto cur=open.top();open.pop();if(cur.g>cost[cur.index]+1e-9)continue;
  if(cur.index==z)break;
  Point p{cur.index%w,cur.index/w};
  for(int d=0;d<4;d++){
   Point q{p.x+dx[d],p.y+dy[d]};if(!inside(q)||blocked[id(q)])continue;
   int nearby=0;for(int k=0;k<4;k++){Point r{q.x+dx[k],q.y+dy[k]};if(inside(r)&&blocked[id(r)])nearby++;}
   double g=cur.g+1.0+std::max(0.0,proximityPenalty)*nearby;
   int v=id(q);if(g+1e-9<cost[v]){cost[v]=g;parent[v]=cur.index;open.push({g+heuristic(q),g,v});}
  }
 }
 if(!std::isfinite(cost[z]))return {};
 std::vector<Point> out;for(int i=z;i!=-1;i=parent[i])out.push_back({i%w,i/w});
 std::reverse(out.begin(),out.end());return out;
}

// Linear constant-velocity Kalman state: (x,y,vx,vy).
// This is the linear-special-case of an EKF; no nonlinear models are used.
RunResult simulate(const std::vector<Point>& path,unsigned seed,double gpsNoise,double imuNoise,int gpsPeriod,int gpsDelay){
 if(path.empty())throw std::invalid_argument("No route");
 if(gpsNoise<0||imuNoise<0||gpsPeriod<1||gpsDelay<0)throw std::invalid_argument("Invalid sensor settings");
 constexpr double dt=0.1;
 std::mt19937 rng(seed);std::normal_distribution<double> normal(0,1);
 auto jitter=[&](double sigma){return sigma>0?normal(rng)*sigma:0.0;};
 RunResult out;out.path=path;
 State truth{double(path[0].x),double(path[0].y),0,0};
 double x[4]={truth.x,truth.y,0,0};
 double P[4][4]={};for(int i=0;i<4;i++)P[i][i]=1;
 struct Fix{int release;double x,y;};std::queue<Fix> gps;
 size_t wp=path.size()>1?1:0;double total=0;
 for(int step=0;step<2500;step++){
  double t=step*dt;
  double ex=path[wp].x-truth.x,ey=path[wp].y-truth.y;
  if(std::hypot(ex,ey)<0.18&&wp+1<path.size())wp++;
  ex=path[wp].x-truth.x;ey=path[wp].y-truth.y;
  double speed=std::min(1.8,2.0*std::hypot(ex,ey));double desiredVx=0,desiredVy=0;
  if(std::hypot(ex,ey)>1e-8){desiredVx=speed*ex/std::hypot(ex,ey);desiredVy=speed*ey/std::hypot(ex,ey);}
  double ax=(desiredVx-truth.vx)*3,ay=(desiredVy-truth.vy)*3;
  truth.x+=truth.vx*dt+0.5*ax*dt*dt;truth.y+=truth.vy*dt+0.5*ay*dt*dt;
  truth.vx+=ax*dt;truth.vy+=ay*dt;
  double measuredAx=ax+jitter(imuNoise),measuredAy=ay+jitter(imuNoise);
  double F[4][4]={{1,0,dt,0},{0,1,0,dt},{0,0,1,0},{0,0,0,1}};
  x[0]+=x[2]*dt+0.5*measuredAx*dt*dt;x[1]+=x[3]*dt+0.5*measuredAy*dt*dt;
  x[2]+=measuredAx*dt;x[3]+=measuredAy*dt;
  double tmp[4][4]={},next[4][4]={};
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)tmp[i][j]+=F[i][k]*P[k][j];
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)next[i][j]+=tmp[i][k]*F[j][k];
  for(int i=0;i<4;i++)next[i][i]+=0.0005;
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)P[i][j]=next[i][j];
  double gpsX=std::numeric_limits<double>::quiet_NaN(),gpsY=gpsX;
  if(step%gpsPeriod==0)gps.push({step+gpsDelay,truth.x+jitter(gpsNoise),truth.y+jitter(gpsNoise)});
  while(!gps.empty()&&gps.front().release<=step){
   auto fix=gps.front();gps.pop();gpsX=fix.x;gpsY=fix.y;
   // Delayed GPS is intentionally processed as a stale measurement (no rewind).
   // This allows studying the accuracy impact of delay; it is not delay-compensated EKF.
   double R=std::max(1e-8,gpsNoise*gpsNoise);
   double a=P[0][0]+R,b=P[0][1],c=P[1][0],d=P[1][1]+R;
   double determinant=a*d-b*c;if(std::abs(determinant)<1e-12)continue;
   double inv[2][2]={{d/determinant,-b/determinant},{-c/determinant,a/determinant}};
   double K[4][2]={};for(int i=0;i<4;i++){K[i][0]=P[i][0]*inv[0][0]+P[i][1]*inv[1][0];K[i][1]=P[i][0]*inv[0][1]+P[i][1]*inv[1][1];}
   double innovation[2]={fix.x-x[0],fix.y-x[1]};
   for(int i=0;i<4;i++)x[i]+=K[i][0]*innovation[0]+K[i][1]*innovation[1];
   double old[4][4];for(int i=0;i<4;i++)for(int j=0;j<4;j++)old[i][j]=P[i][j];
   for(int i=0;i<4;i++)for(int j=0;j<4;j++)P[i][j]=old[i][j]-K[i][0]*old[0][j]-K[i][1]*old[1][j];
  }
  double error=std::hypot(x[0]-truth.x,x[1]-truth.y);total+=error;
  out.samples.push_back({t,truth.x,truth.y,gpsX,gpsY,x[0],x[1],error,int(wp)});
  if(wp==path.size()-1&&std::hypot(truth.x-path.back().x,truth.y-path.back().y)<0.1&&std::hypot(truth.vx,truth.vy)<0.15&&step>10)break;
 }
 out.finalError=out.samples.back().error;out.meanError=total/out.samples.size();return out;
}
void writeCsv(const RunResult& r,const std::string& file){
 std::ofstream f(file);if(!f)throw std::runtime_error("Cannot write telemetry CSV");
 f<<"time_s,true_x,true_y,gps_x,gps_y,estimated_x,estimated_y,estimate_error,waypoint\n";
 for(auto s:r.samples)f<<s.t<<','<<s.trueX<<','<<s.trueY<<','<<s.gpsX<<','<<s.gpsY<<','<<s.estX<<','<<s.estY<<','<<s.error<<','<<s.waypoint<<'\n';
}
