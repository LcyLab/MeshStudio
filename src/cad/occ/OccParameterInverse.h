#pragma once
/** @file OccParameterInverse.h
 * @brief 通用反求的独立OCC几何源；数值迭代由mesher模块负责。
 * @date 2026-10-08
 */
#include "OccCadGeometry.h"
#include "mesh/surface/algorithm6/geometry/SurfaceParameterInverse.h"
/** face为零基；bounds按原OCCFace的ShapeAnalysis裁剪参数范围。
 * geometry须长于源；只提供支撑面Value/D1，不运行直接投影或改变CAD/网格。
 */
mesher::gmshReference::SurfaceParameterInverseSource makeOccSurfaceParameterInverseSource(const OccCadGeometry& geometry,int face);
