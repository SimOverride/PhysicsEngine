#ifndef _vector_h
#define _vector_h

#include "boolean.h"

#define PI 3.1415926
typedef struct transform Transform;

typedef struct vector
{
	double x;
	double y;
}Vector;

/// <summary>
/// 创建向量
/// </summary>
Vector GetVector(double x, double y);
/// <summary>
/// 零向量
/// </summary>
Vector GetZero();

/// <summary>
/// 向量加法
/// </summary>
Vector Add(Vector a, Vector b);
/// <summary>
/// 向量减法
/// </summary>
Vector Substract(Vector a, Vector b);
/// <summary>
/// 取反
/// </summary>
Vector Negate(Vector v);

/// <summary>
/// 数乘
/// </summary>
Vector ScalarProduct(Vector v, double c);
/// <summary>
/// 点乘
/// </summary>
double DotProduct(Vector a, Vector b);
/// <summary>
/// 叉乘
/// </summary>
double CrossProduct(Vector a, Vector b);

/// <summary>
/// 判断两向量是否相等
/// </summary>
bool IsVectorsEqual(Vector a, Vector b);

bool IsNearlyEqual(double a, double b);

bool IsVectorsNearlyEqual(Vector a, Vector b);

/// <summary>
/// 取单位向量
/// </summary>
Vector Normalize(Vector v);
/// <summary>
/// 取模
/// </summary>
double GetVectorLength(Vector v);

double GetLengthSquared(Vector v);
/// <summary>
/// 两点距离
/// </summary>
double Distance(Vector a, Vector b);

double GetDistanceSquared(Vector a, Vector b);
/// <summary>
/// 坐标变换
/// </summary>
Vector DoTransform(Vector v, Transform transform);

/// <summary>
/// 通过向量移动画笔
/// </summary>
void MovePenByVector(Vector position);
/// <summary>
/// 通过向量画线
/// </summary>
void DrawLineByVector(Vector direction);

#endif
