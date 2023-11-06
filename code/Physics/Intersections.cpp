#include "Intersections.h"

bool Intersections::Intersect(Body& a, Body& b, Contact& contact)
{
	contact.a = &a; 
	contact.b = &b; 

	const Vec3 ab = b.position - a.position;
	contact.normal = ab;
	contact.normal.Normalize();

	if (a.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE && b.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE) 
	{
		ShapeSphere* sphere_a = static_cast<ShapeSphere*>(a.shape); 
		ShapeSphere* sphere_b = static_cast<ShapeSphere*>(b.shape);

		contact.ptOnAWorldSpace = a.position + contact.normal * sphere_a->radius;
		contact.ptOnBWorldSpace = b.position - contact.normal * sphere_b->radius; 

		const float radiusAB = sphere_a->radius + sphere_b->radius; 

		// We compare squares
		if (ab.GetLengthSqr() < radiusAB * radiusAB) 
		{
			return true;
		}
	}

	return false;
}
