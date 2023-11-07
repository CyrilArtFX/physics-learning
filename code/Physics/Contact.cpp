#include "Contact.h"

void Contact::ResolveContact(Contact& contact)
{
	Body* a = contact.a;
	Body* b = contact.b;

	const float inv_mass_a = a->inverseMass;
	const float inv_mass_b = b->inverseMass;

	const float elasticity_a = a->elasticity;
	const float elasticity_b = b->elasticity;
	const float elasticity = elasticity_a * elasticity_b; 
	
	const Vec3 pt_on_a = contact.ptOnAWorldSpace;
	const Vec3 pt_on_b = contact.ptOnBWorldSpace;

	const Mat3 inverse_world_inertia_a = a->GetInverseInertiaTensorWorldSpace();
	const Mat3 inverse_world_inertia_b = b->GetInverseInertiaTensorWorldSpace();
	const Vec3 n = contact.normal;
	const Vec3 ra = pt_on_a - a->GetCenterOfMassWorldSpace();
	const Vec3 rb = pt_on_b - b->GetCenterOfMassWorldSpace();  

	const Vec3 angular_ja = (inverse_world_inertia_a * ra.Cross(n)).Cross(ra); 
	const Vec3 angular_jb = (inverse_world_inertia_b * rb.Cross(n)).Cross(rb); 
	const float angular_factor = (angular_ja + angular_jb).Dot(n);  

	// Get world space velocity of the motion and rotation
	const Vec3 vel_a = a->linearVelocity + a->angularVelocity.Cross(ra); 
	const Vec3 vel_b = b->linearVelocity + b->angularVelocity.Cross(rb); 

	const Vec3& vel_ab = vel_a - vel_b;
	const float impulse_value_j = (1.0f + elasticity) * vel_ab.Dot(n) / (inv_mass_a + inv_mass_b + angular_factor);
	const Vec3& impulse = n * impulse_value_j;

	a->ApplyImpulse(pt_on_a, impulse * -1.0f);
	b->ApplyImpulse(pt_on_b, impulse);


	// Friction-caused impulse
	const float friction_a = a->friction;
	const float friction_b = b->friction;
	const float friction = friction_a * friction_b;

	// -- Find the normal direction of the velocity
	// -- with respect to the normal of the collision
	const Vec3 vel_normal = n * n.Dot(vel_ab);
	// -- Find the tengent direction of the velocity
	// -- with respect to the normal of the collision
	const Vec3 vel_tengent = vel_ab - vel_normal;
	// -- Get the tengential velocities relative to the other body
	Vec3 relativ_vel_tengent = vel_tengent;

	relativ_vel_tengent.Normalize();
	const Vec3 inertia_a = (inverse_world_inertia_a * ra.Cross(relativ_vel_tengent)).Cross(ra);
	const Vec3 inertia_b = (inverse_world_inertia_b * rb.Cross(relativ_vel_tengent)).Cross(rb);
	const float inverse_inertia = (inertia_a + inertia_b).Dot(relativ_vel_tengent);

	// -- Tengential impulse for friction
	const float reduced_mass = 1.0f / (a->inverseMass + b->inverseMass + inverse_inertia); 
	const Vec3 impulse_friction = vel_tengent * reduced_mass * friction; 
	// -- Apply kinetic friction
	a->ApplyImpulse(pt_on_a, impulse_friction * -1.0f);  
	b->ApplyImpulse(pt_on_b, impulse_friction);


	// If object are interpenetrating, use this to set them on contact
	if (contact.timeOfImpact == 0.0f)
	{
		const float ta = inv_mass_a / (inv_mass_a + inv_mass_b);
		const float tb = inv_mass_b / (inv_mass_a + inv_mass_b);
		const Vec3 d = contact.ptOnBWorldSpace - contact.ptOnAWorldSpace;
		a->position += d * ta;
		b->position -= d * tb;
	}
}

int Contact::CompareContact(const void* p1, const void* p2)
{
	const Contact& a = *(Contact*)p1;
	const Contact& b = *(Contact*)p1;

	if (a.timeOfImpact < b.timeOfImpact) 
	{
		return -1;
	}
	else if (a.timeOfImpact == b.timeOfImpact) 
	{
		return -0;
	}
	return 1;
}
