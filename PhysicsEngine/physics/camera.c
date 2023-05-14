#include <math.h>
#include <stdio.h>
#include <windows.h>
#include <winuser.h>
#include "graphics.h"
#include "extgraph.h"
#include "camera.h"

/// <summary>
/// 相机，仅能存在一个
/// </summary>
static Camera camera;
static double scaleSpeed = 0.1;
static double moveSpeed = 0.05;
static double maxScale = 5.0;
static double minScale = 0.1;

/// <summary>
/// 需要渲染的实体编号
/// </summary>
static List* renderList;

void InitCamera()
{
	camera.position.x = GetWindowWidth() / 2;
	camera.position.y = GetWindowHeight() / 2;
	camera.scale = 1.0;
	camera.screenOrigin = GetScreenOrigin();

	renderList = CreateList(sizeof(Entity));
}

Vector GetScreenOrigin()
{
	camera.screenOrigin.x = camera.position.x - GetWindowWidth() / camera.scale / 2;
	camera.screenOrigin.y = camera.position.y - GetWindowHeight() / camera.scale / 2;

	return camera.screenOrigin;
}

Vector GetWorldPosition(Vector screenPosition)
{
	Vector v = ScalarProduct(screenPosition, 1.0 / camera.scale);
	return Add(v, GetScreenOrigin());
}

Vector GetScreenPosition(Vector worldPosition)
{
	Vector v = Substract(worldPosition, GetScreenOrigin());
	return ScalarProduct(v, camera.scale);
}

void CameraScale(int event)
{
	if(event == ROLL_UP)
	{
		camera.scale += scaleSpeed;
		if (camera.scale > maxScale)
		{
			camera.scale = maxScale;
		}
	}
	else if(event == ROLL_DOWN)
	{
		camera.scale -= scaleSpeed; 
		if (camera.scale < minScale)
		{
			camera.scale = minScale;
		}
	}
}

void CameraControl(int key, int event)
{
	if (event != KEY_DOWN)
		return;

	Vector v = GetZero();
	switch(key)
	{
	case VK_UP:
		camera.position.y += moveSpeed;
		break;
	case VK_DOWN:
		camera.position.y -= moveSpeed;
		break;
	case VK_LEFT:
		camera.position.x -= moveSpeed;
		break;
	case VK_RIGHT:
		camera.position.x += moveSpeed;
		break;
	}
}

void OcclusionCulling(List* entityList)
{
	ClearList(renderList);
	int i;
	for(i = 0;i < entityList->length;i++)
	{
		Rigidbody* body = ((Entity*)GetData(entityList, i))->body;
		AABB aabb = GetBodyAABB(body);

		if(
			aabb.max.x > GetScreenOrigin().x &&
			aabb.max.y > GetScreenOrigin().y &&
			aabb.min.x < GetScreenOrigin().x + GetWindowWidth() / camera.scale &&
			aabb.min.y < GetScreenOrigin().y + GetWindowHeight() / camera.scale)
		{
			InsertFromTail(renderList, GetData(entityList, i));
		}
	}
}

/// <summary>
/// 渲染实体
/// </summary>
void Render(List* entityList)
{
	OcclusionCulling(entityList);

	int i;
	for (i = 0; i < renderList->length; i++)
	{
		Entity entity = *(Entity*)GetData(renderList, i);
		Rigidbody bodyToDraw = *entity.body;
		ScaleBody(&bodyToDraw);
		Vector* vertices = NULL;
		int num = bodyToDraw.vertexNumber;

		//填充内部
		StartFilledRegion(1);
		SetPenColor(entity.fillerColor);
		if (bodyToDraw.shapeType == CIRCLE)
		{
			MovePen(bodyToDraw.position.x + bodyToDraw.radius, bodyToDraw.position.y);
			DrawArc(bodyToDraw.radius, 0.0, 360.0);
		}
		else
		{
			vertices = bodyToDraw.transformedVertices;
			MovePenByVector(vertices[0]);
			Vector edge;
			int j;
			for (j = 0; j < num; j++)
			{
				edge = Substract(vertices[(j + 1) % num], vertices[j]);
				DrawLineByVector(edge);
			}
		}
		EndFilledRegion();

		//绘制边框
		SetPenColor(entity.outlineColor);
		if (bodyToDraw.shapeType == CIRCLE)
		{
			MovePen(bodyToDraw.position.x + bodyToDraw.radius, bodyToDraw.position.y);
			DrawArc(bodyToDraw.radius, 0.0, 360.0);
			//绘制半径线
			SetPenColor("White");
			MovePenByVector(bodyToDraw.position);
			DrawLine(-bodyToDraw.radius * cos(bodyToDraw.angle), -bodyToDraw.radius * sin(bodyToDraw.angle));
		}
		else
		{
			MovePenByVector(vertices[0]);
			Vector edge;
			int j;
			for (j = 0; j < num; j++)
			{
				edge = Substract(vertices[(j + 1) % num], vertices[j]);
				DrawLineByVector(edge);
			}
		}
	}
}

void ScaleBody(Rigidbody* body)
{
	body->position = GetScreenPosition(body->position);
	body->radius *= camera.scale;
	if(body->shapeType != CIRCLE)
	{
		int i;
		Vector* temp = (Vector*)malloc(sizeof(Vector) * body->vertexNumber);
		for(i = 0;i < body->vertexNumber;i++)
		{
			temp[i] = GetScreenPosition(body->transformedVertices[i]);
		}
		body->transformedVertices = temp;
	}
}