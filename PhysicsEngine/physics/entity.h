#ifndef _entity_h
#define _entity_h

#include "rigidbody.h"

/// <summary>
/// 实体，世界中的物体
/// </summary>
typedef struct entity
{
	Rigidbody* body;
	/// <summary>
	/// 填充颜色
	/// </summary>
	string fillerColor;
	/// <summary>
	/// 轮廓颜色
	/// </summary>
	string outlineColor;
}Entity;

/// <summary>
/// 初始化实体
/// </summary>
Entity* InitEntity(Rigidbody* body, Vector position);

Entity* CreateCircleEntity(Vector position, double radius, double density,
	bool isStatic, double restitution);

Entity* CreateBoxEntity(Vector position, double width, double height,
	double density, bool isStatic, double restitution);

Entity* CreatePolygonEntity(Vector* vertices, int num, double density,
	bool isStatic, double restitution);

void ChangeColor(Entity* entity, string fillerColor, string outlineColor);

/// <summary>
/// 弃用
/// </summary>
void DrawEntity(Entity entity);
#endif