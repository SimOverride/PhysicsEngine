#include <math.h>

#include "graphics.h"
#include "extgraph.h"
#include "entity.h"
#include "world.h"

Entity* InitEntity(Rigidbody* body, Vector position)
{
	Entity* entity = (Entity*)malloc(sizeof(Entity));
	entity->body = body;
	if(body->isStatic)
	{
		entity->fillerColor = "Red";
	}
	else
	{
		entity->fillerColor = "White";
	}
	entity->outlineColor = "Black";

	return entity;
}

Entity* CreateCircleEntity(Vector position, double radius, double density,
	bool isStatic, double restitution)
{
	Rigidbody* entityBody = CreateCircleBody(radius, density, isStatic, restitution);
	MoveBodyTo(entityBody, position);
	AddBody(entityBody);
	
	return InitEntity(entityBody, position);
}

Entity* CreateBoxEntity(Vector position, double width, double height, 
	double density, bool isStatic, double restitution)
{
	Rigidbody* entityBody = CreateBoxBody(width, height, density, isStatic, restitution);
	MoveBodyTo(entityBody, position);
	AddBody(entityBody);

	return InitEntity(entityBody, position);
}

Entity* CreatePolygonEntity(Vector* vertices, int num, double density,
	bool isStatic, double restitution)
{
	Vector position = CalculateCenterPoint(vertices, num);
	Rigidbody* entityBody = CreatePolygonBody(vertices, num, density, isStatic, restitution);
	MoveBodyTo(entityBody, position);
	AddBody(entityBody);

	return InitEntity(entityBody, position);
}

void ChangeColor(Entity* entity, string fillerColor, string outlineColor)
{
	entity->fillerColor = fillerColor;
	entity->outlineColor = outlineColor;
}

void DrawEntity(Entity entity)
{
	Rigidbody body = *entity.body;
	string outlineColor = "Black";

	if(body.shapeType == CIRCLE)
	{
		StartFilledRegion(1);
		SetPenColor(entity.fillerColor);
		MovePen(body.position.x + body.radius, body.position.y);
		DrawArc(body.radius, 0.0, 360.0);
		EndFilledRegion();
		//»­±ß¿ò
		SetPenColor(outlineColor);
		MovePen(body.position.x + body.radius, body.position.y);
		DrawArc(body.radius, 0.0, 360.0);
		SetPenColor("White");
		MovePen(body.position.x, body.position.y);
		DrawLine(body.radius * cos(body.angle), body.radius * sin(body.angle));
	}
	else
	{
		StartFilledRegion(1);
		SetPenColor(entity.fillerColor);
		int i;
		Vector* vertices = GetTransformedVertices(&body);
		MovePen(vertices[0].x, vertices[0].y);
		for (i = 0; i < 4; i++)
		{
			Vector edge = Substract(vertices[(i + 1) % 4], vertices[i]);
			DrawLine(edge.x, edge.y);
		}
		EndFilledRegion();
		//»­±ß¿ò
		SetPenColor(outlineColor);
		MovePen(vertices[0].x, vertices[0].y);
		for (i = 0; i < 4; i++)
		{
			Vector edge = Substract(vertices[(i + 1) % 4], vertices[i]);
			DrawLine(edge.x, edge.y);
		}
	}
}