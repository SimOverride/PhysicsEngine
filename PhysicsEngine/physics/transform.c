#include <math.h>

#include "vector.h"
#include "transform.h"

Transform GetTransform(double x, double y, double angle)
{
	Transform transform;
	transform.positionX = x;
	transform.positionY = y;
	transform.sin = sin(angle);
	transform.cos = cos(angle);

	return transform;
}

Transform GetTransformByVector(Vector position, double angle)
{	
	return GetTransform(position.x, position.y, angle);
}

Transform GetZeroTransform()
{
	return GetTransformByVector(GetZero(), 0.0);
}
