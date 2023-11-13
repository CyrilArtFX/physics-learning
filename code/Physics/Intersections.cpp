#include "Intersections.h"

bool Intersections::Intersect(std::shared_ptr<Body> a, std::shared_ptr<Body> b, const float dt, Contact& contact)
{
	contact.a = a;
	contact.b = b; 
	const Vec3 ab = b->position - a->position;
	contact.normal = ab; 
	contact.normal.Normalize();

	if (a->shape->GetType() == Shape::ShapeType::SHAPE_SPHERE && b->shape->GetType() == Shape::ShapeType::SHAPE_SPHERE)
	{
		ShapeSphere* sphere_a = static_cast<ShapeSphere*>(a->shape);
		ShapeSphere* sphere_b = static_cast<ShapeSphere*>(b->shape);

		Vec3 pos_a = a->position;
		Vec3 pos_b = b->position;
		Vec3 vel_a = a->linearVelocity;
		Vec3 vel_b = b->linearVelocity;

		if (Intersections::SphereSphereDynamic(*sphere_a, *sphere_b, pos_a, pos_b, vel_a, vel_b, dt,
			contact.ptOnAWorldSpace, contact.ptOnBWorldSpace, contact.timeOfImpact))
		{
			// Step bodies forward to get local space collision points
			a->Update(contact.timeOfImpact);
			b->Update(contact.timeOfImpact);

			// Convert world space contacts to local space
			contact.ptOnALocalSpace = a->WorldSpaceToBodySpace(contact.ptOnAWorldSpace);
			contact.ptOnBLocalSpace = b->WorldSpaceToBodySpace(contact.ptOnBWorldSpace);

			Vec3 ab = a->position - b->position;
			contact.normal = ab; 
			contact.normal.Normalize(); 

			// Unwind time step
			a->Update(-contact.timeOfImpact);
			b->Update(-contact.timeOfImpact);

			// Calculate separation distance
			float r = ab.GetMagnitude() - (sphere_a->radius + sphere_b->radius);
			contact.separationDistance = r;

			return true;
		}
	}

	return false;
}

bool Intersections::RaySphere(const Vec3& rayStart, const Vec3& rayDir, const Vec3& sphereCenter, const float sphereRadius, float& t0, float& t1)
{
	const Vec3& s = sphereCenter - rayStart;
	const float a = rayDir.Dot(rayDir);
	const float b = s.Dot(rayDir);
	const float c = s.Dot(s) - sphereRadius * sphereRadius;
	const float delta = b * b - a * c;
	const float inverse_a = 1.0f / a;

    // No solution
	if (delta < 0) return false;

	const float delta_root = sqrtf(delta);
	t0 = (b - delta_root) * inverse_a;
	t1 = (b + delta_root) * inverse_a;

	return true;
}

bool Intersections::SphereSphereDynamic(const ShapeSphere& shapeA, const ShapeSphere& shapeB, const Vec3& posA, const Vec3& posB, 
	const Vec3& velA, const Vec3& velB, const float dt, Vec3& ptOnA, Vec3& ptOnB, float& timeOfImpact)
{
	const Vec3 relative_velocity = velA - velB;

	const Vec3 start_pt_a = posA;
	const Vec3 end_pt_a = start_pt_a + relative_velocity * dt;

	const Vec3 ray_dir = end_pt_a - start_pt_a;

	float t0 = 0;
	float t1 = 0;

	if (ray_dir.GetLengthSqr() < 0.001f * 0.001f)
	{
		// Ray is too short, just check if already intersecting
		Vec3 ab = posB - posA;
		float radius = shapeA.radius + shapeB.radius + 0.001f;
		if (ab.GetLengthSqr() > radius * radius) return false;
	}
	else if (!RaySphere(start_pt_a, ray_dir, posB, shapeA.radius + shapeB.radius, t0, t1)) return false;
		

	// Change from [0, 1] to [0, dt];
	t0 *= dt;
	t1 *= dt;

	// If the collision in only in the past, there will be no future collision for this frame
	if (t1 < 0) return false;

	// Get earliest positive time of impact
	timeOfImpact = t0 < 0.0f ? 0.0f : t0;

	// If the earliest collision is too far in the future, then there's no collision this frame
	if (timeOfImpact > dt) return false;

	// Get the points on the respective points of collision and return true
	Vec3 newPosA = posA + velA * timeOfImpact;
	Vec3 newPosB = posB + velB * timeOfImpact;
	Vec3 ab = newPosB - newPosA;
	ab.Normalize();

	ptOnA = newPosA + ab * shapeA.radius;
	ptOnB = newPosB - ab * shapeB.radius;
	return true;
}
