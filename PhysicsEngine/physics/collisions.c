#include <limits.h>
#include <float.h>

#include "boolean.h"
#include "collisions.h"

#define Min(a, b) (((a) < (b)) ? (a) : (b))

void PointSegmentDistance(
	Vector p, Vector a, Vector b, 
	double* distanceSquared, Vector* cp)
{
	/*proj = ap * ab
	ablen^2 = |ab|^2
	d = proj / ablen^2*/
	Vector ab = Substract(b, a);
	Vector ap = Substract(p, a);

	double proj = DotProduct(ap, ab);
	double ablenSq = GetLengthSquared(ab);
	double d = proj / ablenSq;

	if(d <= 0.0)
	{
		*cp = a;
	}
	else if(d >= 1.0)
	{
		*cp = b;
	}
	else
	{
		*cp = Add(a, ScalarProduct(ab, d));
	}

	*distanceSquared = GetDistanceSquared(p, *cp);
}

bool IntersectAABBs(AABB a, AABB b)
{
	if (a.max.x <= b.min.x || b.max.x <= a.min.x)
		return FALSE;

	if (a.max.y <= b.min.y || b.max.y <= a.min.y)
		return FALSE;

	return TRUE;
}

void FindContactPoints(Rigidbody* bodyA, Rigidbody* bodyB,
	Vector* contact1, Vector* contact2, int* contactCount)
{
	ShapeType shapeA = bodyA->shapeType;
	ShapeType shapeB = bodyB->shapeType;

	*contact1 = GetZero();
	*contact2 = GetZero();
	*contactCount = 0;

	if(shapeA == CIRCLE && shapeB == CIRCLE)
	{
		FindCirclesContactPoint(bodyA->position, bodyA->radius, bodyB->position, contact1);
		*contactCount = 1;
		return;
	}

	if(shapeA == CIRCLE)
	{
		FindCirclePolygonContactPoint(bodyA->position, bodyA->radius, 
			GetTransformedVertices(bodyB), bodyB->vertexNumber, contact1);
		*contactCount = 1;
		return;
	}

	if(shapeB == CIRCLE)
	{
		FindCirclePolygonContactPoint(bodyB->position, bodyB->radius,
			GetTransformedVertices(bodyA), bodyA->vertexNumber, contact1);
		*contactCount = 1;
		return;
	}

	FindPolygonsContactPoints(GetTransformedVertices(bodyA), bodyA->vertexNumber,
		GetTransformedVertices(bodyB), bodyB->vertexNumber,
		contact1, contact2, contactCount);
}

void FindPolygonsContactPoints(
	Vector* verticesA, int numA, 
	Vector* verticesB, int numB,
	Vector* contact1, Vector* contact2, int* contactCount)
{
	*contact1 = GetZero();
	*contact2 = GetZero();
	*contactCount = 0;

	double minDistSq = DBL_MAX;
	double distSq;
	Vector cp;

	int i, j;
	for(i = 0;i < numA;i++)
	{
		Vector p = verticesA[i];

		for(j = 0;j < numB;j++)
		{
			Vector va = verticesB[j];
			Vector vb = verticesB[(j + 1) % numB];

			PointSegmentDistance(p, va, vb, &distSq, &cp);

			if(IsNearlyEqual(distSq, minDistSq))
			{
				if(!IsVectorsNearlyEqual(cp, *contact1))
				{
					*contact2 = cp;
					*contactCount = 2;
				}
			}
			else if(distSq < minDistSq)
			{
				minDistSq = distSq;
				*contactCount = 1;
				*contact1 = cp;
			}
		}
	}

	for (i = 0; i < numB; i++)
	{
		Vector p = verticesB[i];

		for (j = 0; j < numA; j++)
		{
			Vector va = verticesA[j];
			Vector vb = verticesA[(j + 1) % numA];
			
			PointSegmentDistance(p, va, vb, &distSq, &cp);

			if (IsNearlyEqual(distSq, minDistSq))
			{
				if (!IsVectorsNearlyEqual(cp, *contact1))
				{
					*contact2 = cp;
					*contactCount = 2;
				}
			}
			else if (distSq < minDistSq)
			{
				minDistSq = distSq;
				*contactCount = 1;
				*contact1 = cp;
			}
		}
	}
}

void FindCirclePolygonContactPoint(
	Vector circleCenter, double radius, 
	Vector* vertices, int num, Vector* cp)
{
	*cp = GetZero();
	double minDistSq = DBL_MAX;

	int i;
	for(i = 0;i < num;i++)
	{
		Vector va = vertices[i];
		Vector vb = vertices[(i + 1) % num];

		double distSq;
		Vector contact;
		PointSegmentDistance(circleCenter, va, vb, &distSq, &contact);

		if(distSq < minDistSq)
		{
			minDistSq = distSq;
			*cp = contact;
		}
	}
}

void FindCirclesContactPoint(
	Vector centerA, double radiusA, 
	Vector centerB, Vector* cp)
{
	Vector ab = Substract(centerB, centerA);
	Vector dir = Normalize(ab);
	*cp = Add(centerA, ScalarProduct(dir, radiusA));
}

bool Collide(
	Rigidbody* bodyA, Rigidbody* bodyB, 
	Vector* normal, double* depth)
{
	*normal = GetZero();
	*depth = 0.0;

	ShapeType shapeA = bodyA->shapeType;
	ShapeType shapeB = bodyB->shapeType;

	if (shapeA == CIRCLE && shapeB == CIRCLE)
	{
		return IntersectCircles(
				bodyA->position, bodyA->radius,
				bodyB->position, bodyB->radius,
				normal, depth);
	}

	if (shapeA == CIRCLE)
	{
		return IntersectCirclePolygons(
			bodyA->position, bodyA->radius,
			bodyB->position, GetTransformedVertices(bodyB), bodyB->vertexNumber,
			normal, depth);
	}

	if(shapeB == CIRCLE)
	{
		bool result = IntersectCirclePolygons(
			bodyB->position, bodyB->radius,
			bodyA->position, GetTransformedVertices(bodyA), bodyA->vertexNumber,
			normal, depth);

		*normal = Negate(*normal);
		return result;
	}
	
	return IntersectPolygons(
			bodyA->position, GetTransformedVertices(bodyA), bodyA->vertexNumber,
			bodyB->position, GetTransformedVertices(bodyB), bodyB->vertexNumber,
			normal, depth);

	return FALSE;
}

bool IntersectCirclePolygons(
	Vector circleCenter, double circleRadius,
	Vector polygonCenter, Vector* vertices, int num,
	Vector* normal, double* depth)
{
	*normal = GetZero();
	*depth = DBL_MAX;

	Vector axis = GetZero();
	double axisDepth = 0.0;
	double minA, maxA;
	double minB, maxB;

	int i;
	for (i = 0; i < num; i++)
	{
		//获取多边形一边
		Vector edge = Substract(vertices[(i + 1) % num], vertices[i]);
		//以一边的法向量为轴
		axis = GetVector(-edge.y, edge.x);
		axis = Normalize(axis);

		ProjectVertices(vertices, num, axis, &minA, &maxA);
		ProjectCircles(circleCenter, circleRadius, axis, &minB, &maxB);

		//如果投影不相交，则多边形不相交
		if (minA >= maxB || minB >= maxA)
		{
			return FALSE;
		}

		axisDepth = Min(maxA - minB, maxB - minA);

		//获取最小深度值以及对应的法向量
		if (*depth > axisDepth)
		{
			*normal = axis;
			*depth = axisDepth;
		}
	}

	int index = FindClosestPointPolygon(circleCenter, vertices, num);
	Vector closestVertex = vertices[index];
	axis = Substract(closestVertex, circleCenter);
	axis = Normalize(axis);

	ProjectVertices(vertices, num, axis, &minA, &maxA);
	ProjectCircles(circleCenter, circleRadius, axis, &minB, &maxB);

	//如果投影不相交，则不相交
	if (minA >= maxB || minB >= maxA)
	{
		return FALSE;
	}

	axisDepth = Min(maxA - minB, maxB - minA);

	//获取最小深度值以及对应的法向量
	if (*depth > axisDepth)
	{
		*normal = axis;
		*depth = axisDepth;
	}

	//调整法向量方向
	Vector direction = Substract(polygonCenter, circleCenter);

	if (DotProduct(direction, *normal) < 0.0)
	{
		*normal = Negate(*normal);
	}

	return TRUE;
}

int FindClosestPointPolygon(Vector circleCenter, Vector* vertices, int num)
{
	int result = -1;
	double mindistance = DBL_MAX;
	
	int i;
	for(i = 0;i < num;i++)
	{
		double distance = Distance(vertices[i], circleCenter);

		if(distance < mindistance)
		{
			mindistance = distance;
			result = i;
		}
	}

	return result;
}

void ProjectCircles(
	Vector center, double radius, 
	Vector axis, double* min, double* max)
{
	//半径在轴方向的投影
	Vector radiusProjection = ScalarProduct(Normalize(axis), radius);

	Vector v1 = Add(center, radiusProjection);
	Vector v2 = Substract(center, radiusProjection);

	*min = DotProduct(v1, axis);
	*max = DotProduct(v2, axis);

	if(*min > *max)
	{
		double t = *max;
		*max = *min;
		*min = t;
	}
}

bool IntersectPolygons(
	Vector centerA, Vector* verticesA, int numA,
	Vector centerB, Vector* verticesB, int numB,
	Vector* normal, double* depth)
{
	*normal = GetZero();
	*depth = DBL_MAX;

	int i;
	for (i = 0; i < numA; i++)
	{
		//获取多边形一边
		Vector edge = Substract(verticesA[(i + 1) % numA], verticesA[i]);
		//以一边的法向量为轴
		Vector axis = GetVector(-edge.y, edge.x);
		axis = Normalize(axis);

		double minA, maxA;
		double minB, maxB;
		ProjectVertices(verticesA, numA, axis, &minA, &maxA);
		ProjectVertices(verticesB, numB, axis, &minB, &maxB);

		//如果投影不相交，则多边形不相交
		if (minA >= maxB || minB >= maxA)
		{
			return FALSE;
		}

		double axisDepth = Min(maxA - minB, maxB - minA);

		//获取最小深度值以及对应的法向量
		if (*depth > axisDepth)
		{
			*normal = axis;
			*depth = axisDepth;
		}
	}

	for (i = 0; i < numB; i++)
	{
		//获取多边形一边
		Vector edge = Substract(verticesB[(i + 1) % numB], verticesB[i]);
		//以一边的法向量为轴
		Vector axis = GetVector(-edge.y, edge.x);
		axis = Normalize(axis);

		double minA, maxA;
		double minB, maxB;
		ProjectVertices(verticesA, numA, axis, &minA, &maxA);
		ProjectVertices(verticesB, numB, axis, &minB, &maxB);

		//如果投影不相交，则多边形不相交
		if (minA >= maxB || minB >= maxA)
		{
			return FALSE;
		}

		double axisDepth = Min(maxA - minB, maxB - minA);

		//获取最小深度值以及对应的法向量
		if (*depth > axisDepth)
		{
			*normal = axis;
			*depth = axisDepth;
		}
	}

	//保证法向量方向为A指向B
	Vector direction = Substract(centerB, centerA);

	if (DotProduct(direction, *normal) < 0.0)
	{
		*normal = Negate(*normal);
	}

	return TRUE;
}

void ProjectVertices(
	Vector* vertices, int num, 
	Vector axis, double* min, double* max)
{
	int i;

	for(i = 0;i < num;i++)
	{
		Vector v = vertices[i];
		double proj = DotProduct(v, axis);
		if(i == 0)
		{
			*min = *max = proj;
			continue;
		}

		if (proj < *min)
			*min = proj;
		if (proj > *max)
			*max = proj;
	}
}

bool IntersectCircles(
	Vector centerA, double radiusA, 
	Vector centerB, double radiusB,
	Vector* normal, double* depth)
{
	*normal = GetZero();
	*depth = 0.0;
	
	double distance = Distance(centerA, centerB);
	double radii = radiusA + radiusB; 	// 半径之和
	
	if(distance >= radii)
	{
		return FALSE;
	}
	
	*normal = Normalize(Substract(centerB, centerA));  //法向向量 
	*depth = radii - distance;
	
	return TRUE;
}
