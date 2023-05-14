#ifndef _AABB_h
#define _AABB_h

#include "vector.h"

/// <summary>
/// ÷·∂‘∆Î≈ˆ◊≤œ‰
/// </summary>
typedef struct AABB
{
	Vector min;
	Vector max;
}AABB;

AABB GetAABB(double minX, double minY, double maxX, double maxY);

#endif
