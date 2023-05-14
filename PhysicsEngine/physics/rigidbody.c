#include<stdio.h>
#include <stdlib.h>
#include <math.h>


#include "boolean.h"
#include "rigidbody.h"
#include "world.h"
#include "vector.h"
#include "transform.h"

Rigidbody* InitBody(
	ShapeType shapeType, double area, double density, double mass, 
	double inertia, double restitution, bool isStatic, double radius, 
	double width, double height, int vertexNumber, Vector* vertices)
{
	Rigidbody* body = (Rigidbody*)malloc(sizeof(Rigidbody));

	if (density > GetMaxDensity() || density < GetMinDensity())
	{
		return NULL;
	}
	//限制偿还系数 
	if (restitution < 0.0)
	{
		restitution = 0.0;
	}
	else if (restitution > 1.0)
	{
		restitution = 1.0;
	}

	body->position = GetZero();
	body->linearVelocity = GetZero();
	body->angle = 0.0;
	body->angularVelocity = 0.0;
	body->force = GetZero();

	body->shapeType = shapeType;
	body->area = area;
	body->density = density;
	body->mass = mass;
	body->invMass = mass > 0.0 ? 1.0 / mass : 0.0;
	body->inertia = inertia;
	body->invInertia = inertia > 0.0 ? 1.0 / inertia : 0.0;
	body->isStatic = isStatic;
	body->isKinematic = FALSE;
	body->restitution = restitution;
	body->radius = radius;
	body->width = width;
	body->height = height;
	body->staticFriction = 0.6;
	body->dynamicFriction = 0.4;

	body->vertexNumber = vertexNumber;
	body->vertices = vertices;
	body->transformedVertices = vertices == NULL ? NULL : (Vector*)malloc(sizeof(Vector) * vertexNumber);

	body->transformUpdateRequired = TRUE;
	body->aabbUpdateRequired = TRUE;
}

Rigidbody* CreateCircleBody(double radius, double density, 
	bool isStatic, double restitution)
{
	double area = radius * radius * PI;
	if(area > GetMaxBodySize() || area < GetMinBodySize())
	{
		return NULL;
	}

	double mass = 0.0;
	double inertia = 0.0;
	if (!isStatic)
	{
		mass = area * density;
		inertia = 1.0 / 2 * mass * radius * radius;
	}

	return InitBody(CIRCLE, area, density, mass, inertia, 
		restitution, isStatic, radius, 0.0, 0.0, 0, NULL);
}

Rigidbody* CreateBoxBody(double width, double height, double density, 
	bool isStatic, double restitution)
{
	double area = height * width;
	if(area > GetMaxBodySize() || area < GetMinBodySize())
	{
		return NULL;
	}

	double mass = 0.0;
	double inertia = 0.0;
	if (!isStatic)
	{
		mass = area * density;
		inertia = (1.0 / 12) * mass * (width * width + height * height);
	}

	Vector* vertices = CreateBoxVertices(width, height);

	return InitBody(BOX, area, density, mass, inertia, 
		restitution, isStatic, 0.0, width, height, 4, vertices);
}

Vector* CreateBoxVertices(double width, double height)
{
	double left = -width / 2;
	double right = left + width;
	double bottom = -height / 2;
	double top = bottom + height;

	Vector* vertices = (Vector*)malloc(sizeof(Vector) * 4);
	vertices[0] = GetVector(left, top);
	vertices[1] = GetVector(right, top);
	vertices[2] = GetVector(right, bottom);
	vertices[3] = GetVector(left, bottom);
	return vertices;
}

int* CreateBoxTriangles()
{
	int triangles[6];
	triangles[0] = 0;
	triangles[1] = 1;
	triangles[2] = 2;
	triangles[3] = 0;
	triangles[4] = 2;
	triangles[5] = 3;

	return triangles;
}

Rigidbody* CreatePolygonBody(Vector* vertices, int num, double density, 
	bool isStatic, double restitution)
{
	double area = CalculatePolygonArea(vertices, num);
	if (area > GetMaxBodySize() || area < GetMinBodySize())
	{
		return NULL;
	}
	double mass = 0.0;
	double inertia = 0.0;
	if (!isStatic)
	{
		mass = area * density;
		inertia = CalculatePolygonInertia(vertices, num, mass);
	}

	return InitBody(POLYGON, area, density, mass, inertia, 
		restitution, isStatic, 0.0, 0.0, 0.0, num, vertices);
}

Vector CalculateCenterPoint(Vector* vertices, int num)
{
	double sumX = 0.0;
	double sumY = 0.0;

	int i;
	for(i = 0;i < num;i++)
	{
		sumX += vertices[i].x;
		sumY += vertices[i].y;
	}

	Vector center = { sumX / num, sumY / num };
	for(i = 0;i < num;i++)
	{
		vertices[i] = Substract(vertices[i], center);
	}

	return center;
}

double CalculatePolygonArea(Vector* vertices, int num)
{
	double area = 0.0;

	int i;
	for (i = 0; i < num; i++)
	{
		Vector a = vertices[i];
		Vector b = vertices[(i + 1) % num];

		area += a.x * b.y - b.x * a.y;
	}
	return fabs(area) * 0.5;
}

double CalculatePolygonInertia(Vector* vertices, int num, double mass)
{
	//I = m / 6 (Σ |Pn+1 × Pn|(Pn+1 ^2 + Pn+1 * Pn + Pn^2))/ (Σ|Pn+1 × Pn|)
	double numer = 0.0;
	double denom = 0.0;
	int i;
	for(i = 0;i < num;i++)
	{
		Vector a = vertices[i];
		Vector b = vertices[(i + 1) % num];

		double cross = CrossProduct(a, b);
		numer += cross * (GetLengthSquared(a) + DotProduct(a, b) + GetLengthSquared(b));
		denom += cross;
	}

	return mass / 6.0 * numer / denom;
}

Vector* GetTransformedVertices(Rigidbody* body)
{
	if(body->transformUpdateRequired)
	{
		Transform transform = GetTransformByVector(body->position, body->angle);

		int i;
		for(i = 0;i < body->vertexNumber;i++)
		{
			Vector v = body->vertices[i];
			body->transformedVertices[i] = DoTransform(v, transform);
		}
	}

	body->transformUpdateRequired = FALSE;
	return body->transformedVertices;
}

void MoveBody(Rigidbody* body, Vector amount)
{
	body->position = Add(body->position, amount);
	body->transformUpdateRequired = TRUE;
	body->aabbUpdateRequired = TRUE;
}

void MoveBodyTo(Rigidbody* body, Vector position)
{
	body->position = position;
	body->transformUpdateRequired = TRUE;
	body->aabbUpdateRequired = TRUE;
}

void RotateBody(Rigidbody* body, double amount)
{
	body->angle += amount;
	body->transformUpdateRequired = TRUE;
	body->aabbUpdateRequired = TRUE;
}

void RotateBodyTo(Rigidbody* body, double angle)
{
	body->angle = angle;
	body->transformUpdateRequired = TRUE;
	body->aabbUpdateRequired = TRUE;
}

void AddForce(Rigidbody* body, Vector amount)
{
	body->force = amount;
}

AABB GetBodyAABB(Rigidbody* body)
{
	if (!body->aabbUpdateRequired)
		return body->aabb;

	double minX, minY;
	double maxX, maxY;

	if(body->shapeType == CIRCLE)
	{
		minX = body->position.x - body->radius;
		maxX = body->position.x + body->radius;
		minY = body->position.y - body->radius;
		maxY = body->position.y + body->radius;
	}
	else
	{
		Vector* vertices = GetTransformedVertices(body);

		Vector v = vertices[0];
		maxX = minX = v.x;
		maxY = minY = v.y;

		int i;
		for (i = 1; i < body->vertexNumber; i++)
		{
			v = vertices[i];
			if (v.x < minX)
				minX = v.x;
			if (v.x > maxX)
				maxX = v.x;
			if (v.y < minY)
				minY = v.y;
			if (v.y > maxY)
				maxY = v.y;
		}
	}

	body->aabb = GetAABB(minX, minY, maxX, maxY);
	body->aabbUpdateRequired = FALSE;
	return body->aabb;
}

void RigidBodyStep(Rigidbody* body, double time, Vector gravity, int iterations)
{
	if (body->isStatic)
		return;

	time /= (double)iterations;

	Vector acceleration = Add(ScalarProduct(body->force, body->invMass), gravity);
	if(body->isKinematic)
	{
		acceleration = Substract(acceleration, gravity);
	}
	body->linearVelocity = Add(body->linearVelocity, ScalarProduct(acceleration, time));
	//body->linearVelocity = Add(body->linearVelocity, ScalarProduct(gravity, time));
	body->angle += body->angularVelocity * time;
	body->position = Add(body->position, ScalarProduct(body->linearVelocity, time));

	Stabilizer(body);
	body->force = GetZero();
	body->transformUpdateRequired = TRUE;
	body->aabbUpdateRequired = TRUE;
}

void Stabilizer(Rigidbody* body)
{
	if(IsVectorsNearlyEqual(body->linearVelocity, GetZero()))
	{
		body->linearVelocity = GetZero();
	}
	if(IsNearlyEqual(body->angularVelocity, 0.0))
	{
		body->angularVelocity = 0.0;
	}

	int i;
	for(i = 0;i < 4;i++)
	{
		if (IsNearlyEqual(body->angle, 360.0 / i))
		{
			body->angle = 360.0 / i;
		}
	}
	
}