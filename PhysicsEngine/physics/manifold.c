#include "manifold.h"

Manifold GetManifold(
	Rigidbody* bodyA, Rigidbody* bodyB,
	Vector normal, double depth,
	Vector contact1, Vector contact2, int contactCount)
{
	Manifold manifold;

	manifold.bodyA = bodyA;
	manifold.bodyB = bodyB;
	manifold.normal = normal;
	manifold.depth = depth;
	
	manifold.contact1 = contact1;
	manifold.contact2 = contact2;
	manifold.contactCount = contactCount;

	return manifold;
}