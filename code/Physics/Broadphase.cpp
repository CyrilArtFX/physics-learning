#include "Broadphase.h"
#include "../Math/Bounds.h"
#include "Shape.h"
#include <vector>
#include <algorithm>


int CompareSAP(const void* a, const void* b) 
{
	const PseudoBody* ea = (const PseudoBody*)a;
	const PseudoBody* eb = (const PseudoBody*)b;

	if (ea->value > eb->value) return -1;
	return 1;
}

bool SortPseudoBodies(const PseudoBody& a, const PseudoBody& b)
{
	return a.value < b.value;
}


void SortBodiesBounds(const std::vector<std::shared_ptr<Body>> bodies, const size_t num, std::vector<PseudoBody>& sortedArray, const float dt_sec)
{
	Vec3 axis = Vec3(1, 1, 1);
	axis.Normalize();

	for (int i = 0; i < num; i++)
	{
		const std::shared_ptr<Body> body = bodies[i];
		Bounds bounds = body->shape->GetBounds(body->position, body->orientation);

		// Expand the bounds by the linear velocity
		bounds.Expand(bounds.mins + body->linearVelocity * dt_sec);
		bounds.Expand(bounds.maxs + body->linearVelocity * dt_sec);

		const float epsilon = 0.01f;
		bounds.Expand(bounds.mins + Vec3(-1, -1, -1) * epsilon);
		bounds.Expand(bounds.maxs + Vec3(1, 1, 1) * epsilon);

		sortedArray.push_back(PseudoBody{ i, axis.Dot(bounds.mins), true });
		sortedArray.push_back(PseudoBody{ i, axis.Dot(bounds.maxs), false });

		/*sortedArray[i * 2 + 0].id = i;
		sortedArray[i * 2 + 0].value = axis.Dot(bounds.mins);
		sortedArray[i * 2 + 0].ismin = true;

		sortedArray[i * 2 + 1].id = i;
		sortedArray[i * 2 + 1].value = axis.Dot(bounds.maxs);
		sortedArray[i * 2 + 1].ismin = false;*/
	}

	std::sort(sortedArray.begin(), sortedArray.end(), SortPseudoBodies);
	//qsort(sortedArray, num * 2, sizeof(PseudoBody), CompareSAP);
}


void BuildPairs(std::vector<CollisionPair>& collisionPairs, const std::vector<PseudoBody>& sortedBodies, const int num)
{
	collisionPairs.clear();

	// Now that the bodies are sorted, build the collision pairs
	for (int i = 0; i < num * 2; i++) 
	{
		const PseudoBody& a = sortedBodies[i];
		if (!a.ismin) continue;

		CollisionPair pair;
		pair.a = a.id;

		for (int j = i + 1; j < num * 2; j++)
		{
			const PseudoBody& b = sortedBodies[j];

			if (b.id == a.id) break;

			if (!b.ismin) continue;

			pair.b = b.id;
			collisionPairs.push_back(pair);
		}
	}
}


void SweepAndPrune1D(const std::vector<std::shared_ptr<Body>> bodies, const size_t num, std::vector<CollisionPair>& finalPairs, const float dt_sec)
{
	//PseudoBody* sortedBodies = (PseudoBody*)_malloca(sizeof(PseudoBody) * num * 2);
	std::vector<PseudoBody> sortedBodies;
	sortedBodies.reserve(num * 2);

	SortBodiesBounds(bodies, num, sortedBodies, dt_sec);
	BuildPairs(finalPairs, sortedBodies, num);
}

void BroadPhase(const std::vector<std::shared_ptr<Body>> bodies, const int num, std::vector<CollisionPair>& finalPairs, const float dt_sec)
{
	finalPairs.clear(); 

	SweepAndPrune1D(bodies, num, finalPairs, dt_sec); 
}
