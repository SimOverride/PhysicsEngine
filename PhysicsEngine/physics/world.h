#ifndef _world_h
#define _world_h

#include "rigidbody.h"
#include "manifold.h"

double GetMinBodySize();
double GetMaxBodySize();
double GetMinDensity();
double GetMaxDensity();

void InitWorld();
Vector GetGravity();
void AddBody(Rigidbody* body);
bool RemoveBody(int index);
Rigidbody* GetBody(int index);
int GetBodyCount();

/// <summary>
/// 世界物理更新
/// </summary>
/// <param name="time">每帧时间</param>
/// <param name="iterations">迭代次数，越大越精确</param>
void Step(double time, unsigned int iterations);
void BroadPhase();
void NarrowPhase();
void StepBodies(double time, int iterations);
/// <summary>
/// 分离两个刚体
/// </summary>
/// <param name="mtv">: minimum translation vector</param>
void SeparateBodies(Rigidbody* bodyA, Rigidbody* bodyB, Vector mtv);
/// <summary>
/// 计算碰撞
/// </summary>
void ResolveCollisionWithFriction(Manifold contact);

#endif
