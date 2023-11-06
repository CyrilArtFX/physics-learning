#include "Body.h"
#include "Shape.h"

Vec3 Body::GetCenterOfMassWorldSpace() const
{
	const Vec3 center_of_mass = shape->GetCenterOfMass();
	const Vec3 pos = position + orientation.RotatePoint(center_of_mass);
	return pos;
}

Vec3 Body::GetCenterOfMassBodySpace() const
{
	return shape->GetCenterOfMass(); 
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPoint)
{
	const Vec3 temp = worldPoint - GetCenterOfMassWorldSpace();
	const Quat invert_orient = orientation.Inverse();
	Vec3 body_space = invert_orient.RotatePoint(temp);
	return body_space; 
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& bodyPoint)
{
	Vec3 world_space = GetCenterOfMassWorldSpace() + orientation.RotatePoint(bodyPoint);
	return world_space; 
}

Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
	Mat3 inertia_tensor = shape->InertiaTensor(); 
	Mat3 inverse_inertia_tensor = inertia_tensor.Inverse() * inverseMass;
	return inverse_inertia_tensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
	Mat3 inertia_tensor = shape->InertiaTensor(); 
	Mat3 inverse_inertia_tensor = inertia_tensor.Inverse() * inverseMass; 
	Mat3 orient = orientation.ToMat3(); 
	inverse_inertia_tensor = orient * inverse_inertia_tensor * orient.Transpose(); 
	return inverse_inertia_tensor; 
}



void Body::Update(const float dt_sec)
{
	position += linearVelocity * dt_sec;

	Vec3 position_cm = GetCenterOfMassWorldSpace();  
	Vec3 cm_to_position = position - position_cm; 
	 
	Mat3 orientation_mat = orientation.ToMat3(); 
	Mat3 inertia_tensor = orientation_mat * shape->InertiaTensor() * orientation_mat.Transpose(); 
	Vec3 alpha = inertia_tensor.Inverse() * (angularVelocity.Cross(inertia_tensor * angularVelocity));

	angularVelocity += alpha * dt_sec;

	// Update orientation
	Vec3 dangle = angularVelocity * dt_sec;  
	Quat dq = Quat(dangle, dangle.GetMagnitude()); 
	orientation = dq * orientation; 
	orientation.Normalize();

	// Get the new model position
	position = position_cm + dq.RotatePoint(cm_to_position); 
}



void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	if (inverseMass == 0.0f) return;

	linearVelocity += impulse * inverseMass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse)
{
	if (inverseMass == 0.0f) return;

	angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	const float max_angular_speed = 30.0f;
	if (angularVelocity.GetLengthSqr() > max_angular_speed * max_angular_speed)
	{
		angularVelocity.Normalize();
		angularVelocity *= max_angular_speed;
	}
}

void Body::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
	if (inverseMass == 0.0f) return;

	ApplyImpulseLinear(impulse);

	Vec3 position = GetCenterOfMassWorldSpace();
	Vec3 r = impulsePoint - position;
	Vec3 dl = r.Cross(impulse);
	ApplyImpulseAngular(dl);
}
