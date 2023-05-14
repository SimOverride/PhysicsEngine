#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <windows.h>
#include <olectl.h>
#include <mmsystem.h>
#include <wingdi.h>
#include <ole2.h>
#include <ocidl.h>
#include <winuser.h>
#include <time.h>

#include "graphics.h"
#include "extgraph.h"
#include "genlib.h"
#include "simpio.h"
#include "random.h"
#include "strlib.h"
#include "conio.h"

#include "vector.h"
#include "rigidBody.h"
#include "world.h"
#include "collisions.h"
#include "transform.h"
#include "list.h"
#include "AABB.h"
#include "camera.h"
#include "entity.h"

#define PhysicsTimer 1
#define RendererTimer 2

#define colorCount 40

typedef enum mode
{
	/// <summary>
	/// 绘制实体模式
	/// </summary>
	DrawMode,
	/// <summary>
	/// 移动实体模式
	/// </summary>
	ControlMode,
	/// <summary>
	/// 随机生成模式（鼠标点击）
	/// </summary>
	RandomMode
}Mode;

void GenerateColors();
/// <summary>
/// 总更新
/// </summary>
/// <param name="TimeID"></param>
void Update(int TimeID);
void RenderOutline();
void ShowMessage();
void ShowDouble(string dataName, string color, double d);
void ShowVector(string dataName, string color, Vector v);

void KeyboardEvents(int key, int event);
void SwitchMode(int key, int event);

void MouseEvents(int x, int y, int button, int event);
void DrawShape(Vector position, int button, int event);
void DrawCircle(Vector position, int button, int event);
void DrawBox(Vector position, int button, int event);
void DrawPolygon(Vector position, int button, int event);
Vector* GetTempVertices(List* verticesList);

void MoveEntityByMouse(Vector position, int button, int event);

void GenerateByClick(Vector position, int button, int event);

static List* entityList;
static List* colorList;
double cx;
double cy;

static Mode mode = DrawMode;

static List* verticesList;
static bool drawStart = FALSE;
static ShapeType targetType = CIRCLE;
/*绘制模式变量*/
static Vector mousePositionToDraw;
static Vector startPosition;
static double density = 1.0;
static bool isStatic = FALSE;
static double restitution = 0.5;

/*控制模式变量*/
/// <summary>
/// 目标移动物体
/// </summary>
static Entity* targetEntity = NULL;
/// <summary>
/// 是否被鼠标抓取
/// </summary>
static bool isGrabed;
/// <summary>
/// 鼠标相对目标物体中心的偏移量
/// </summary>
static Vector positionOffset;
/// <summary>
/// 鼠标相对物体中心的角偏移量
/// </summary>
static double angleOffset;

void Main()
{
	InitGraphics();

	cx = GetWindowWidth();
    cy = GetWindowHeight();
	InitWorld();
	InitCamera();
	entityList = CreateList(sizeof(Entity));
	verticesList = CreateList(sizeof(Vector));;
	GenerateColors();
	srand(time(NULL));

	Vector start = { 5, 0.8 };
	Entity* entity = CreateBoxEntity(start, 6, 0.7, 1.0, TRUE, 0.5);
	InsertFromTail(entityList, entity);

	start.x = 3.5, start.y = 3;
	Entity* entity1 = CreateBoxEntity(start, 3, 0.3, 1.0, TRUE, 0.5);
	RotateBody(entity1->body, -PI / 9);
	InsertFromTail(entityList, entity1);

	start.x = 6.5, start.y = 5;
	Entity* entity2 = CreateBoxEntity(start, 3, 0.3, 1.0, TRUE, 0.5);
	RotateBody(entity2->body, PI / 9);
	InsertFromTail(entityList, entity2);

	registerTimerEvent(Update);
	registerMouseEvent(MouseEvents);
	registerKeyboardEvent(KeyboardEvents);

	startTimer(PhysicsTimer, 1);
	startTimer(RendererTimer, 10);
}

void GenerateColors()
{
	colorList = CreateList(sizeof(string));
	double r, g, b;
	int i;
	for(i = 0;i < colorCount;i++)
	{
		r = 1.0 * rand() / RAND_MAX;
		g = 1.0 * rand() / RAND_MAX;
		b = 1.0 * rand() / RAND_MAX;
		char n[3] = { i / 10 + '0', i % 10 + '0', 0 };
		string name = (string)malloc(sizeof(string));
		strcpy(name, n);
		DefineColor(name, r, g, b);
		InsertFromTail(colorList, name);
	}
}

void Update(int TimeID)
{
	int i;
	switch(TimeID)
	{
	case PhysicsTimer:
		Step(0.01, 20);
		//移除y轴过低的刚体
		for (i = 0; i < GetBodyCount(); i++)
		{
			AABB box = GetBodyAABB(GetBody(i));

			if (box.max.y < -2.0)
			{
				RemoveByIndex(entityList, i);
				RemoveBody(i);
			}
		}
		break;
	case RendererTimer:
		//清屏 
		DisplayClear();
		Render(entityList);
		RenderOutline();
		ShowMessage();
		break;
	}
}

void RenderOutline()
{
	if (!drawStart)
		return;

	if (isStatic)
		SetPenColor("Red");
	else
		SetPenColor("Green");

	if(targetType == CIRCLE)
	{
		Vector a = GetScreenPosition(startPosition);
		Vector b = GetScreenPosition(mousePositionToDraw);
		double radius = GetVectorLength(Substract(b, a));
		MovePen(a.x + radius, a.y);
		DrawArc(radius, 0.0, 360.0);
		return;
	}

	if(targetType == BOX)
	{
		Vector a = GetScreenPosition(startPosition);
		Vector b = GetScreenPosition(mousePositionToDraw);
		Vector diag = Substract(b, a);
		MovePenByVector(a);
		DrawLine(diag.x, 0);
		DrawLine(0, diag.y);
		DrawLine(-diag.x, 0);
		DrawLine(0, -diag.y);
		return;
	}

	int num = verticesList->length;
	if (num <= 1)
		return;
	
	Vector segment;
	Vector a, b;
	int i;
	a = *(Vector*)GetData(verticesList, 0);
	a = GetScreenPosition(a);
	MovePenByVector(a);
	for (i = 0; i < num - 1; i++)
	{
		a = *(Vector*)GetData(verticesList, i);
		b = *(Vector*)GetData(verticesList, i + 1);
		a = GetScreenPosition(a);
		b = GetScreenPosition(b);
		segment = Substract(b, a);
		DrawLineByVector(segment);
	}
}

static double dy = 0.6;

void ShowMessage()
{
	//显示刚体数量
	SetPenColor("Black");
	MovePen(0.0, cy - 0.1);
	DrawTextString("BodyCount : ");
	char count[4];
	sprintf(count, "%d", GetBodyCount());
	SetPenColor("Blue");
	DrawTextString(count);

	string text = NULL;
	//显示模式
	SetPenColor("Black");
	MovePen(0.0, cy - 0.3);
	DrawTextString("Mode : ");
	switch (mode)
	{
	case DrawMode:
		text = "DrawMode";
		break;
	case ControlMode:
		text = "ControlMode";
		break;
	case RandomMode:
		text = "RandomMode";
		break;
	}
	SetPenColor("Orange");
	DrawTextString(text);	

	if (mode == DrawMode)
	{
		//显示形状
		SetPenColor("Black");
		MovePen(0.0, cy - 0.5);
		DrawTextString("Shape : ");
		switch (targetType)
		{
		case CIRCLE:
			text = "Circle";
			break;
		case BOX:
			text = "Box";
			break;
		case POLYGON:
			text = "Polygon";
			break;
		}
		SetPenColor("Orange");
		DrawTextString(text);

		SetPenColor("Black");
		MovePen(0.0, cy - 0.7);
		DrawTextString("Static : ");
		if (isStatic)
		{
			SetPenColor("Red");
			text = "True";
		}
		else 
		{
			SetPenColor("Green");
			text = "False";
		}
		DrawTextString(text);
	}

	if (mode == ControlMode && targetEntity != NULL)
	{
		Rigidbody* body = targetEntity->body;
		string color = "Green";
		dy = 0.6;
		ShowVector("Position", color, body->position);
		if (body->shapeType == CIRCLE)
		{
			ShowDouble("Radius", color, body->radius);
		}
		else if (body->shapeType == BOX)
		{
			ShowDouble("Width", color, body->width);
			ShowDouble("Height", color, body->height);
		}
		if (body->isStatic)
			return;
		ShowVector("LinearVelocity", color, body->linearVelocity);
		ShowDouble("Angle", color, body->angle);
		ShowDouble("AngularVelocity", color, body->angularVelocity);
		ShowDouble("Area", color, body->area);
		ShowDouble("Mass", color, body->mass);
		ShowDouble("Inertia", color, body->inertia);
	}
}

void ShowDouble(string dataName, string color, double d)
{
	MovePen(0.0, cy - dy);
	dy += 0.2;
	SetPenColor("Black");
	DrawTextString(dataName);
	DrawTextString(" : ");
	SetPenColor(color);
	char data[30];
	sprintf(data, "%.3f", d);
	DrawTextString(data);
}

void ShowVector(string dataName, string color, Vector v)
{
	MovePen(0.0, cy - dy);
	dy += 0.2;
	SetPenColor("Black");
	DrawTextString(dataName);
	DrawTextString(" : ");
	SetPenColor(color);
	char data[30];
	sprintf(data, "%.3f %.3f", v.x, v.y);
	DrawTextString(data);
}

void KeyboardEvents(int key, int event)
{
	CameraControl(key, event);
	SwitchMode(key, event);
}

void SwitchMode(int key, int event)
{
	if (key == VK_SHIFT && event == KEY_DOWN)
	{
		switch (mode)
		{
		case DrawMode:
			mode = ControlMode;
			ClearList(verticesList);
			drawStart = FALSE;
			break;
		case ControlMode:
			mode = RandomMode;
			break;
		case RandomMode:
			mode = DrawMode;
			break;
		}
	}
	else if(mode == DrawMode)
	{
		if (key == VK_CONTROL && event == KEY_DOWN)
		{
			if (isStatic)
				isStatic = FALSE;
			else
				isStatic = TRUE;
		}
		//如果开始绘画，则不能切换形状
		if (drawStart)
			return;
		if(key == VK_SPACE && event == KEY_DOWN)
		{
			switch (targetType)
			{
			case CIRCLE:
				targetType = BOX;
				break;
			case BOX:
				targetType = POLYGON;
				break;
			case POLYGON:
				targetType = CIRCLE;
				break;
			}
		}
	}
}

void MouseEvents(int x, int y, int button, int event)
{
	CameraScale(event);
	Vector mouseWorldPosition = GetWorldPosition(GetVector(ScaleXInches(x), ScaleYInches(y)));
	switch (mode)
	{
	case DrawMode:
		DrawShape(mouseWorldPosition, button, event);
		break;
	case ControlMode:
		MoveEntityByMouse(mouseWorldPosition, button, event);
		break;
	case RandomMode:
		GenerateByClick(mouseWorldPosition, button, event);
		break;
	}
}

void DrawShape(Vector position, int button, int event)
{
	switch(targetType)
	{
	case CIRCLE:
		DrawCircle(position, button, event);
		break;
	case BOX:
		DrawBox(position, button, event);
		break;
	case POLYGON:
		DrawPolygon(position, button, event);
		break;
	}
	if(event == MOUSEMOVE)
	{
		mousePositionToDraw = position;
	}
}

void DrawCircle(Vector position, int button, int event)
{
	switch(event)
	{
	case BUTTON_DOWN:
		if(button == LEFT_BUTTON)
		{
			startPosition = position;
			drawStart = TRUE;
		}
		break;
	case BUTTON_UP:
		if(button == LEFT_BUTTON)
		{
			double radiusSq = GetLengthSquared(Substract(position, startPosition));
			if(radiusSq * PI > GetMinBodySize() && radiusSq * PI < GetMaxBodySize())
			{
				Entity* circle = CreateCircleEntity(startPosition, sqrt(radiusSq), 
					density, isStatic, restitution);
				if (!isStatic)
					circle->fillerColor = GetData(colorList, rand() % colorCount);
				InsertFromTail(entityList, circle);
			}
			drawStart = FALSE;
		}
		break;
	}
}

void DrawBox(Vector position, int button, int event)
{
	switch (event)
	{
	case BUTTON_DOWN:
		if (button == LEFT_BUTTON)
		{
			startPosition = position;
			drawStart = TRUE;
		}
		break;
	case BUTTON_UP:
		if (button == LEFT_BUTTON)
		{
			//对角向量
			Vector diag = Substract(position, startPosition);
			double width = fabs(diag.x);
			double height = fabs(diag.y);
			if (width * height > GetMinBodySize() && width * height < GetMaxBodySize())
			{
				Vector start = GetVector(startPosition.x + diag.x / 2, startPosition.y + diag.y / 2);
				Entity* box = CreateBoxEntity(start, width, height,
					density, isStatic, restitution);
				if (!isStatic)
					box->fillerColor = GetData(colorList, rand() % colorCount);
				InsertFromTail(entityList, box);
			}
			drawStart = FALSE;
		}
		break;
	}
}

void DrawPolygon(Vector position, int button, int event)
{
	if (event != BUTTON_DOWN)
		return;

	if (button == LEFT_BUTTON)
	{
		if (!drawStart)
		{
			drawStart = TRUE;
			
		}
		Vector* p = (Vector*)malloc(sizeof(Vector));
		*p = position;
		InsertFromTail(verticesList, p);
	}
	if (button == RIGHT_BUTTON)
	{
		if (verticesList->length < 3)
			return;

		Vector* vertices = GetTempVertices(verticesList);
		Entity* polygon = CreatePolygonEntity(vertices, verticesList->length, density, isStatic, restitution);
		if(!isStatic)
			polygon->fillerColor = GetData(colorList, rand() % colorCount);
		InsertFromTail(entityList, polygon);
		ClearList(verticesList);
		drawStart = FALSE;
	}
}

Vector* GetTempVertices(List* verticesList)
{
	Vector* vertices = (Vector*)malloc(sizeof(Vector) * verticesList->length);
	int i;
	for(i = 0;i < verticesList->length;i++)
	{
		vertices[i] = *(Vector*)GetData(verticesList, i);
	}
	return vertices;
}

void MoveEntityByMouse(Vector position, int button, int event)
{
	static int downButton; //0 = left = move, 1 = right = rotate
	
	if (!isGrabed)
	{
		int i;
		for (i = 0; i < entityList->length; i++)
		{
			Rigidbody* body = ((Entity*)GetData(entityList, i))->body;
			AABB aabb = GetBodyAABB(body);
			if (aabb.max.x > position.x && aabb.max.y > position.y &&
				aabb.min.x < position.x && aabb.min.y < position.y)
			{
				targetEntity = (Entity*)GetData(entityList, i);
			}
		}
	}
	switch (event)
	{
	case BUTTON_DOWN:
		if (button == LEFT_BUTTON)
		{
			downButton = 0;
		}
		else if (button == RIGHT_BUTTON)
		{
			downButton = 1;
		}
		else
		{
			break;
		}
		Rigidbody* body = targetEntity->body;
		positionOffset = Substract(position, body->position);
		angleOffset = body->angle;
		body->linearVelocity = GetZero();
		body->angularVelocity = 0.0;
		body->isKinematic = TRUE;
		isGrabed = TRUE;
		break;
	case BUTTON_UP:
		if (button == LEFT_BUTTON && downButton == 0 ||
			button == RIGHT_BUTTON && downButton == 1)
		{
			if (targetEntity != NULL)
				targetEntity->body->isKinematic = FALSE;
			isGrabed = FALSE;
		}
		break;
	case MOUSEMOVE:
		if (targetEntity == NULL || !isGrabed)
		{
			break;
		}
		if (downButton == 0)
		{
			MoveBodyTo(targetEntity->body, Substract(position, positionOffset));
			targetEntity->body->linearVelocity = GetZero();
			targetEntity->body->angularVelocity = 0.0;
		}
		else
		{
			Vector offsetNew = Substract(position, targetEntity->body->position);
			double cos = DotProduct(offsetNew, positionOffset) / GetVectorLength(offsetNew) / GetVectorLength(positionOffset);
			double angle = CrossProduct(offsetNew, positionOffset) < 0 ? acos(cos) : -acos(cos);
			RotateBodyTo(targetEntity->body, angle + angleOffset);
		}
		break;
	}
}

void GenerateByClick(Vector position, int button, int event)
{
	if (event != BUTTON_DOWN)
		return;
	
	Entity* entity = NULL;

	if(button == LEFT_BUTTON)
	{
		entity = CreateBoxEntity(position, 0.5 * rand() / RAND_MAX + 0.1, 
			0.5 * rand() / RAND_MAX + 0.1, 1.0, FALSE, 0.5);
	}
	else if(button == RIGHT_BUTTON)
	{
		entity = CreateCircleEntity(position, 0.3 * rand() / RAND_MAX + 0.1, 1.0, FALSE, 0.5);
	}

	entity->fillerColor = GetData(colorList, rand() % colorCount);
	InsertFromTail(entityList, entity);
}
