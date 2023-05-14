#include <math.h>
#include "graphics.h"
#include "vector.h"
#include "transform.h"

#define Epsilon 0.00001

Vector GetVector(double x, double y)
{
	Vector v = {x, y};
	return v;
}

Vector GetZero()
{
	return GetVector(0.0, 0.0);
}

//向量加法
Vector Add(Vector a, Vector b)
{
	return GetVector(a.x + b.x, a.y + b.y);
}

//向量减法 
Vector Substract(Vector a, Vector b)
{
	return GetVector(a.x - b.x, a.y - b.y);
}

//取反 
Vector Negate(Vector v)
{
	return GetVector(-v.x, -v.y);
}

//向量数乘 
Vector ScalarProduct(Vector v, double c)
{
	return GetVector(v.x * c, v.y * c);
}

//向量点乘 
double DotProduct(Vector a, Vector b)
{
	return a.x * b.x + a.y * b.y;
}

//向量叉乘 
double CrossProduct(Vector a, Vector b)
{
	return a.x * b.y - a.y * b.x;
}

//两向量是否相等 
bool IsVectorsEqual(Vector a, Vector b)
{
	if(a.x == b.x && a.y == b.y)
		return TRUE;
	return FALSE;
}

bool IsNearlyEqual(double a, double b)
{
	return fabs(a - b) < Epsilon;
}

bool IsVectorsNearlyEqual(Vector a, Vector b)
{
	return GetDistanceSquared(a, b) < Epsilon * Epsilon;
}

//获取单位向量 
Vector Normalize(Vector v)
{
	double len = GetVectorLength(v);
	return GetVector(v.x / len, v.y / len);
}

//获取向量长度 
double GetVectorLength(Vector v)
{
	return sqrt(v.x * v.x + v.y * v.y); 
}

double GetLengthSquared(Vector v)
{
	return v.x * v.x + v.y * v.y;
}

//两点距离 
double Distance(Vector a, Vector b)
{
	return GetVectorLength(Substract(a, b));
}

double GetDistanceSquared(Vector a, Vector b)
{
	return GetLengthSquared(Substract(a, b));
}
//坐标变换 
Vector DoTransform(Vector v, Transform transform)
{
	//do rotation first, then do translation
	//x2 = cosθx1 - sinθy1; 
	//y2 = sinθx1 + cosθy1;
	Vector result;
	result.x = transform.cos * v.x - transform.sin * v.y + transform.positionX;
	result.y = transform.sin * v.x + transform.cos * v.y + transform.positionY;
	
	return result;
}

void MovePenByVector(Vector position)
{
	MovePen(position.x, position.y);
}

void DrawLineByVector(Vector direction)
{
	DrawLine(direction.x, direction.y);
}