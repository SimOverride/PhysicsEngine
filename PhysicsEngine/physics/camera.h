#ifndef _camera_h
#define _camera_h

#include "vector.h"
#include "entity.h"
#include "list.h"

/// <summary>
/// 相机，用于视角控制及渲染
/// </summary>
typedef struct camera
{
	Vector position;
	double scale;
	/// <summary>
	/// 屏幕原点
	/// </summary>
	Vector screenOrigin;
}Camera;

/// <summary>
/// 初始化相机
/// </summary>
void InitCamera();

/// <summary>
/// 获取屏幕原点
/// </summary>
/// <returns></returns>
Vector GetScreenOrigin();
/// <summary>
/// 获取世界坐标
/// </summary>
Vector GetWorldPosition(Vector screenPosition);
/// <summary>
/// 获取屏幕坐标
/// </summary>
Vector GetScreenPosition(Vector worldPosition);

/// <summary>
/// 相机缩放
/// </summary>
void CameraScale(int event);
/// <summary>
/// 相机移动控制
/// </summary>
void CameraControl(int key, int event);

/// <summary>
/// 遮挡剔除，排除相机外的物体
/// </summary>
void OcclusionCulling(List* entityList);
/// <summary>
/// 渲染
/// </summary>
void Render(List* entityList);
/// <summary>
/// 获取用于绘制的刚体
/// </summary>
void ScaleBody(Rigidbody* body);

#endif