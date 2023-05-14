#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "world.h"
#include "list.h"
#include "collisions.h"

#define Min(a, b) (((a) < (b)) ? (a) : (b))

static double minBodySize = 0.01 * 0.01;
static double maxBodySize = 64.0 * 64.0;

static double minDensity = 0.5; // g/cm^3
static double maxDensity = 30.0;

double GetMinBodySize()
{
	return minBodySize;
}

double GetMaxBodySize()
{
	return maxBodySize;
}

double GetMinDensity()
{
	return minDensity;
}

double GetMaxDensity()
{
	return maxDensity;
}

static int minIteration = 1;
static int maxIteration = 64;

static Vector gravity;
/// <summary>
/// 世界刚体列表
/// </summary>
static List* bodyList;
/// <summary>
/// 保存待计算碰撞的刚体的序号
/// </summary>
static List* contactPairs;

void InitWorld()
{
	gravity = GetVector(0.0, -9.81);
	bodyList = CreateList(sizeof(Rigidbody));
	contactPairs = CreateList(sizeof(int) * 2);
}

Vector GetGravity()
{
	return gravity;
}

void AddBody(Rigidbody* body)
{
	InsertFromTail(bodyList, body);
}

bool RemoveBody(int index)
{
	return RemoveByIndex(bodyList, index);
}

Rigidbody* GetBody(int index)
{
	return (Rigidbody*)GetData(bodyList, index);
}

int GetBodyCount()
{
	return bodyList->length;
}

void Step(double time, unsigned int iterations)
{
	if (iterations < minIteration)
		iterations = minIteration;
	if (iterations > maxIteration)
		iterations = maxIteration;

	int currentIteration;
	for (currentIteration = 0; currentIteration < iterations; currentIteration++)
	{
		//清除上一次的碰撞信息
		ClearList(contactPairs);

		StepBodies(time, iterations);

		BroadPhase();
		NarrowPhase();
	}
}

void BroadPhase()
{
	int i, j;
	for (i = 0; i < bodyList->length - 1; i++)
	{
		Rigidbody* bodyA = GetBody(i);
		AABB aabb_A = GetBodyAABB(bodyA);

		for (j = i + 1; j < bodyList->length; j++)
		{
			Rigidbody* bodyB = GetBody(j);
			AABB aabb_B = GetBodyAABB(bodyB);

			if (bodyA->isStatic && bodyB->isStatic)
				continue;

			if (!IntersectAABBs(aabb_A, aabb_B))
				continue;

			int* cp = (int*)malloc(sizeof(int) * 2);
			cp[0] = i;
			cp[1] = j;
			InsertFromTail(contactPairs, cp);
		}
	}
}

void NarrowPhase()
{
	int i;
	for (i = 0; i < contactPairs->length; i++)
	{
		int* pair = (int*)GetData(contactPairs, i);
		Rigidbody* bodyA = GetBody(pair[0]);
		Rigidbody* bodyB = GetBody(pair[1]);

		Vector normal;
		double depth;
		if (Collide(bodyA, bodyB, &normal, &depth))
		{
			Vector mtv = ScalarProduct(normal, depth);
			SeparateBodies(bodyA, bodyB, mtv);

			Vector contact1, contact2;
			int contactCount;
			FindContactPoints(bodyA, bodyB, &contact1, &contact2, &contactCount);
			Manifold* contact = (Manifold*)malloc(sizeof(Manifold));
			*contact = GetManifold(bodyA, bodyB, normal, depth,
				contact1, contact2, contactCount);
			ResolveCollisionWithFriction(*contact);
		}
	}
}

void StepBodies(double time, int iterations)
{
	int i;
	for(i = 0;i < GetBodyCount();i++)
	{
		RigidBodyStep(GetBody(i), time, gravity, iterations);
	}
}

/// <summary>
/// 分离两个刚体
/// </summary>
/// <param name="mtv">: minimum translation vector</param>
void SeparateBodies(Rigidbody* bodyA, Rigidbody* bodyB, Vector mtv)
{
	if (bodyA->isStatic)
	{
		MoveBody(bodyB, mtv);
	}
	else if (bodyB->isStatic)
	{
		MoveBody(bodyA, Negate(mtv));
	}
	else
	{
		MoveBody(bodyB, ScalarProduct(mtv, 0.5));
		MoveBody(bodyA, ScalarProduct(mtv, -0.5));
	}
}

static Vector contactList[2];
static Vector impulseList[2];
static Vector frictionImpulseList[2];
static Vector raList[2];
static Vector rbList[2];
static double jList[2];

void ResolveCollisionWithFriction(Manifold contact)
{
	Rigidbody* bodyA = contact.bodyA;
	Rigidbody* bodyB = contact.bodyB;
	Vector normal = contact.normal;
	contactList[0] = contact.contact1;
	contactList[1] = contact.contact2;
	int contactCount = contact.contactCount;

	double e = Min(bodyA->restitution, bodyB->restitution);

	double sf = (bodyA->staticFriction + bodyB->staticFriction) * 0.5;
	double df = (bodyA->dynamicFriction + bodyB->dynamicFriction) * 0.5;

	int i;

	for (i = 0; i < contactCount; i++)
	{
		impulseList[i] = GetZero();
		frictionImpulseList[i] = GetZero();
		raList[i] = GetZero();
		rbList[i] = GetZero();
		jList[i] = 0.0;
	}

	//冲击力计算
	for (i = 0; i < contactCount; i++)
	{
		Vector ra = Substract(contactList[i], bodyA->position);
		Vector rb = Substract(contactList[i], bodyB->position);
		raList[i] = ra;
		rbList[i] = rb;

		Vector raPerp = GetVector(-ra.y, ra.x);
		Vector rbPerp = GetVector(-rb.y, rb.x);

		Vector angularLinearVelocityA = ScalarProduct(raPerp, bodyA->angularVelocity);
		Vector angularLinearVelocityB = ScalarProduct(rbPerp, bodyB->angularVelocity);

		Vector relativeVelocity = Substract(
			Add(bodyB->linearVelocity, angularLinearVelocityB),
			Add(bodyA->linearVelocity, angularLinearVelocityA));

		double contactVelocityMug = DotProduct(relativeVelocity, normal);

		if (contactVelocityMug > 0.0)
		{
			continue;
		}
		//VA2 = VA1 - j / massA * n  j is the magnitude of the impluse
		//VB2 = VB1 + j / massB * n
		//j = [-(1 + e)vAB1 * n] / [n * n(1/massA + 1/massB) + (rAP⊥ * n)^2 / IA + (rBP⊥ * n) / IB]  e is the restitution
		double raPerpDotN = DotProduct(raPerp, normal);
		double rbPerpDotN = DotProduct(rbPerp, normal);

		double denom = bodyA->invMass + bodyB->invMass +
			(raPerpDotN * raPerpDotN) * bodyA->invInertia +
			(rbPerpDotN * rbPerpDotN) * bodyB->invInertia;

		double j = -(1.0 + e) * contactVelocityMug;
		j /= denom * contactCount;
		jList[i] = j;
		impulseList[i] = ScalarProduct(normal, j);
	}

	for (i = 0; i < contactCount; i++)
	{
		Vector impulse = impulseList[i];

		Vector ra = raList[i];
		Vector rb = rbList[i];

		bodyA->linearVelocity = Add(bodyA->linearVelocity, 
			ScalarProduct(impulse, -bodyA->invMass));
		bodyA->angularVelocity += -CrossProduct(ra, impulse) * bodyA->invInertia;
		bodyB->linearVelocity = Add(bodyB->linearVelocity, 
			ScalarProduct(impulse, bodyB->invMass));
		bodyB->angularVelocity += CrossProduct(rb, impulse) * bodyB->invInertia;
	}

	//摩擦力计算
	for (i = 0; i < contactCount; i++)
	{
		Vector ra = Substract(contactList[i], bodyA->position);
		Vector rb = Substract(contactList[i], bodyB->position);
		raList[i] = ra;
		rbList[i] = rb;

		Vector raPerp = GetVector(-ra.y, ra.x);
		Vector rbPerp = GetVector(-rb.y, rb.x);

		Vector angularLinearVelocityA = ScalarProduct(raPerp, bodyA->angularVelocity);
		Vector angularLinearVelocityB = ScalarProduct(rbPerp, bodyB->angularVelocity);

		Vector relativeVelocity = Substract(
			Add(bodyB->linearVelocity, angularLinearVelocityB),
			Add(bodyA->linearVelocity, angularLinearVelocityA));

		//切线向量
		Vector tangent = Substract(relativeVelocity, 
			ScalarProduct(normal, DotProduct(relativeVelocity, normal)));

		//如果切线向量为零，说明没有碰撞
		if(IsVectorsNearlyEqual(tangent, GetZero()))
		{
			continue;
		}else
		{
			tangent = Normalize(tangent);
		}

		double raPerpDotT = DotProduct(raPerp, tangent);
		double rbPerpDotT = DotProduct(rbPerp, tangent);

		double denom = bodyA->invMass + bodyB->invMass +
			(raPerpDotT * raPerpDotT) * bodyA->invInertia +
			(rbPerpDotT * rbPerpDotT) * bodyB->invInertia;

		double jt = -DotProduct(relativeVelocity, tangent);
		jt /= denom * contactCount;
		
		//Ff <= μ * Fn  ==> jt <= j * sf  sf : staticFriction
		Vector frictionImpulse;
		double j = jList[i];
		if(fabs(jt) <= j * sf)
		{
			frictionImpulse = ScalarProduct(tangent, jt);
		}
		else
		{
			frictionImpulse = ScalarProduct(tangent, -j * df);
		}

		frictionImpulseList[i] = frictionImpulse;
	}

	for (i = 0; i < contactCount; i++)
	{
		Vector frictionImpulse = frictionImpulseList[i];

		Vector ra = raList[i];
		Vector rb = rbList[i];

		bodyA->linearVelocity = Add(bodyA->linearVelocity, 
			ScalarProduct(frictionImpulse, -bodyA->invMass));
		bodyA->angularVelocity += -CrossProduct(ra, frictionImpulse) * bodyA->invInertia;
		bodyB->linearVelocity = Add(bodyB->linearVelocity, 
			ScalarProduct(frictionImpulse, bodyB->invMass));
		bodyB->angularVelocity += CrossProduct(rb, frictionImpulse) * bodyB->invInertia;
	}
}

void RenderBodies()
{
	Node* t = bodyList->head;
	while (t != NULL)
	{
		Rigidbody* body = t->data;

		if(body->shapeType == CIRCLE)
		{

		}
		t = t->next;
	}
}
