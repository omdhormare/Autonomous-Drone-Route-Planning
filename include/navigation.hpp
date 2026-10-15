#pragma once
#include <vector>
#include <string>
struct Point { int x=0,y=0; };
struct State {double x=0,y=0,vx=0,vy=0;};
struct Sample {double t,trueX,trueY,gpsX,gpsY,estX,estY,error;int waypoint;};
struct RunResult {std::vector<Point> path;std::vector<Sample> samples;double finalError=0,meanError=0;};
std::vector<Point> astar(int w,int h, const std::vector<Point>& obstacles,Point start,Point goal,double proximityPenalty=0);
RunResult simulate(const std::vector<Point>& path,unsigned seed=42,double gpsNoise=0.5,double imuNoise=0.15,int gpsPeriod=5,int gpsDelay=0);
void writeCsv(const RunResult& r,const std::string& path);
