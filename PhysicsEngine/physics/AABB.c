#include "AABB.h"

AABB GetAABB(double minX, double minY, double maxX, double maxY)
{
	AABB aabb = { GetVector(minX, minY), GetVector(maxX, maxY) };
	return aabb;
}
