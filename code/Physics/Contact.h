#pragma once
#include <memory>
#include "../Math/Vector.h"
#include "Body.h"

class Contact
{
public:
	Vec3 ptOnAWorldSpace;
	Vec3 ptOnALocalSpace;
	Vec3 ptOnBWorldSpace;
	Vec3 ptOnBLocalSpace;
	Vec3 normal;
	float separationDistance;
	float timeOfImpact;

	std::shared_ptr<Body> a;
	std::shared_ptr<Body> b;

	static void ResolveContact(Contact& contact);

	static int CompareContact(const void* p1, const void* p2);
	static bool SortContacts(const Contact& a, const Contact& b);
};
