#ifndef _rigidbody_h
#define _rigidbody_h

#include <string.h>
#include "AABB.h"

typedef char* string;

typedef enum shapeType
{
	CIRCLE = 0,
	BOX = 1,
	POLYGON = 2,
}ShapeType;

typedef struct rigidbody
{
	Vector position;
	Vector linearVelocity;
	double angle;
	double angularVelocity;
	Vector force;

	ShapeType shapeType;
	/// <summary>
	/// 面积
	/// </summary>
	double area; 
	/// <summary>
	/// 密度 
	/// </summary>
	double density;
	/// <summary>
	/// 质量
	/// </summary>
	double mass;
	/// <summary>
	/// 质量的倒数，方便计算
	/// </summary>
	double invMass;
	/// <summary>
	/// 转动惯量
	/// </summary>
	double inertia;
	/// <summary>
	/// 转动惯量的倒数
	/// </summary>
	double invInertia;
	/// <summary>
	/// 劲度系数
	/// </summary>
	double restitution;
	/// <summary>
	/// 是否是静态
	/// </summary>
	bool isStatic;
	/// <summary>
	/// 是否是运动学的（不受重力影响）
	/// </summary>
	bool isKinematic;
	double radius;
	double width;
	double height;
	double staticFriction;
	double dynamicFriction;
	
	/// <summary>
	/// 顶点数
	/// </summary>
	int vertexNumber;
	/// <summary>
	/// 顶点，通常不可用
	/// </summary>
	Vector* vertices;
	/// <summary>
	/// 储存变换后的顶点
	/// </summary>
	Vector* transformedVertices;
	AABB aabb;

	bool transformUpdateRequired;
	bool aabbUpdateRequired;
}Rigidbody;

/// <summary>
/// 初始化刚体
/// </summary>
Rigidbody* InitBody(
	ShapeType shapeType, double area, double density, double mass,
	double inertia, double restitution, bool isStatic, double radius,
	double width, double height, int vertexNumber, Vector* vertices);

/// <summary>
/// 初始化圆形刚体
/// </summary>
Rigidbody* CreateCircleBody(double radius, double density,
	bool isStatic, double restitution);

/// <summary>
/// 初始化矩形刚体
/// </summary>
Rigidbody* CreateBoxBody(double width, double height, double density,
	bool isStatic, double restitution);
/// <summary>
/// 创建矩形顶点
/// </summary>
Vector* CreateBoxVertices(double width, double height);

Rigidbody* CreatePolygonBody(Vector* vertices, int num, double density,
	bool isStatic, double restitution);
Vector CalculateCenterPoint(Vector* vertices, int num);
double CalculatePolygonArea(Vector* vertices, int num);
double CalculatePolygonInertia(Vector* vertices, int num, double mass);

/// <summary>
/// 获得变换后的顶点
/// </summary>
Vector* GetTransformedVertices(Rigidbody* body);
int* CreateBoxTriangles();

/*刚体运动*/
void MoveBody(Rigidbody *body, Vector amount);
void MoveBodyTo(Rigidbody* body, Vector position);
void RotateBody(Rigidbody* body, double amount);
void RotateBodyTo(Rigidbody* body, double angle);

void AddForce(Rigidbody* body, Vector amount);

AABB GetBodyAABB(Rigidbody* body);

/// <summary>
/// 刚体运动更新
/// </summary>
void RigidBodyStep(Rigidbody* body, double time, Vector gravity, int iterations);

/// <summary>
/// 使刚体趋于稳定，貌似没多大用
/// </summary>
void Stabilizer(Rigidbody* body);

#endif
