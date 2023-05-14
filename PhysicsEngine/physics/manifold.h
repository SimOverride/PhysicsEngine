#ifndef _manifold_h
#define _manifold_h

#include "rigidbody.h"

/// <summary>
/// ´¢´æÅö×²¼¯
/// </summary>
typedef struct manifold
{
	Rigidbody* bodyA;
	Rigidbody* bodyB;
	Vector normal;
	double depth;

	Vector contact1;
	Vector contact2;
	int contactCount;
}Manifold;

Manifold GetManifold(
	Rigidbody* bodyA, Rigidbody* bodyB,
	Vector normal, double depth,
	Vector contact1, Vector contact2, int contactCount);

#endif
