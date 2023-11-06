#pragma once
#include "Body.h"
#include "Shape.h"
#include "Contact.h"

class Intersections
{
public:
	static bool Intersect(Body& a, Body& b, Contact& contact);
};

