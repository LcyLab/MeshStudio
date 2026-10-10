#pragma once
// 仅供测试对照局部提交与整副本提交，生产入口不调用。
#include "mesh/planar/Delaunay2D.h"
namespace mesher { namespace testing {
bool insertPointReference(Mesh2D&,const Point2D&,const std::vector<std::pair<int,int>>&,
                         int,int*,int*,std::string*,const LinearMetric2D* =nullptr,MeshObservation ={});
bool enforceConstraintsReference(Mesh2D&,const std::vector<std::pair<int,int>>&,std::string*,MeshObservation ={});
} }
