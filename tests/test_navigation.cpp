#include "navigation.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){auto r=astar(6,6,{{2,0},{2,1},{2,2},{2,3},{2,4}},{0,0},{5,5});assert(!r.empty());assert(r.front().x==0&&r.back().x==5);for(auto p:r)assert(!(p.x==2&&p.y<5));auto blocked=astar(3,3,{{1,0},{1,1},{1,2}},{0,0},{2,2});assert(blocked.empty());auto sim=simulate(r,123,0,0,1,0);assert(!sim.samples.empty());assert(std::isfinite(sim.finalError));std::cout<<"A* and sensor fusion smoke tests passed\n";}
