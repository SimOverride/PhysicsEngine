#ifndef _transform_h
#define _transform_h

typedef struct vector Vector;

typedef struct transform
{
	double positionX;
	double positionY;
	double sin;
	double cos;
}Transform;

/// <summary>
/// 创建变换
/// </summary>
Transform GetTransform(double x, double y, double angle);
/// <summary>
/// 通过向量创建变换
/// </summary>
Transform GetTransformByVector(Vector position, double angle);
/// <summary>
/// 获得零变换
/// </summary>
Transform GetZeroTransform();

#endif
