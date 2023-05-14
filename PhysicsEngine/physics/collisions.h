#ifndef _collision_h
#define _collision_h

#include "vector.h"
#include "rigidbody.h"

/// <summary>
/// 获得顶点对一边的距离
/// </summary>
void PointSegmentDistance(
	Vector p, Vector a, Vector b, 
	double* distanceSquared, Vector* contact);

/// <summary>
/// 检测轴对齐碰撞箱的碰撞
/// </summary>
bool IntersectAABBs(AABB a, AABB b);

void FindContactPoints(Rigidbody* bodyA, Rigidbody* bodyB, 
	Vector* contact1,Vector* contact2, int* contactCount);

void FindPolygonsContactPoints(
	Vector* verticesA, int numA,
	Vector* verticesB, int numB,
	Vector* contact1, Vector* contact2, int* contactCount);

void FindCirclePolygonContactPoint(
	Vector circleCenter, double radius,
	Vector* vertices, int num, Vector* cp);

/// <summary>
/// 获得两个圆形刚体的接触点
/// </summary>
void FindCirclesContactPoint(
	Vector centerA, double radiusA, 
	Vector centerB, Vector* cp);

/// <summary>
/// 碰撞检测
/// </summary>
bool Collide(
	Rigidbody* bodyA, Rigidbody* bodyB, 
	Vector* normal, double* depth);

bool IntersectCirclePolygons(
	Vector circleCenter, double circleRadius,
	Vector polygonCenter, Vector* vertices, int num,
	Vector* normal, double* depth);

/// <summary>
/// 找到离圆心最近的顶点
/// </summary>
int FindClosestPointPolygon(Vector circleCenter, Vector* vertices, int num);
/// <summary>
/// 获取圆在对应轴上的投影
/// </summary>
void ProjectCircles(
	Vector center, double radius, 
	Vector axis, double* min, double* max);

bool IntersectPolygons(
	Vector centerA, Vector* verticesA, int numA,
	Vector centerB, Vector* verticesB, int numB,
	Vector* normal, double* depth);

/// <summary>
/// 获取多边形顶点在对应轴上的最大、最小投影（对轴的点乘）
/// </summary>
void ProjectVertices(
	Vector* vertices, int num, 
	Vector axis, double* min, double* max);

bool IntersectCircles(
	Vector centerA, double radiusA,
	Vector centerB, double radiusB,
	Vector* normal, double* depth);

#endif
