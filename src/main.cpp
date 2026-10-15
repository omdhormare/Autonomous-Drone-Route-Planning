#include "navigation.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>
int main(int argc,char**argv){try{
 std::string mode=argc>1?argv[1]:"demo";
 if(mode!="demo"){std::cout<<"Usage: drone_route.exe [demo]\n";return 0;}
 std::vector<Point> obstacles;for(int y=0;y<7;y++)if(y!=6)obstacles.push_back({4,y});
 for(int x=6;x<10;x++)obstacles.push_back({x,4});
 auto path=astar(12,10,obstacles,{1,1},{10,8},0.25);
 if(path.empty()){std::cerr<<"No route found\n";return 1;}
 std::filesystem::create_directories("results");
 std::ofstream route("results/route.csv");route<<"x,y\n";for(auto p:path)route<<p.x<<','<<p.y<<'\n';
 auto result=simulate(path,42,0.45,0.12,5,2);writeCsv(result,"results/telemetry.csv");
 std::cout<<"AUTONOMOUS DRONE ROUTE PLANNING + SENSOR FUSION\n";
 std::cout<<"A* waypoints: "<<path.size()<<" | simulation samples: "<<result.samples.size()<<"\n";
 std::cout<<"Final estimated-position error: "<<result.finalError<<" grid units\n";
 std::cout<<"Mean estimated-position error: "<<result.meanError<<" grid units\n";
 std::cout<<"Saved results/route.csv and results/telemetry.csv\n";
 std::cout<<"Note: Windows-based 2D simulated sensors and linear Kalman filter, not a real drone or Linux/GDB test.\n";
 return 0;
 }catch(const std::exception&e){std::cerr<<"Error: "<<e.what()<<'\n';return 1;}}
