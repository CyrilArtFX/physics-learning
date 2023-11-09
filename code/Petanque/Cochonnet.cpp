#include "Cochonnet.h"
#include "../Physics/Shape.h"

Cochonnet::Cochonnet()
{
	position = Vec3(0, 0, 0);
	orientation = Quat(0, 0, 0, 1);
	shape = new ShapeSphere(0.5);
	inverseMass = 1.0f;
	elasticity = 0.4f;
	friction = 0.5f;
}

void Cochonnet::Update(float dt_sec)
{
	PhysicUpdate(dt_sec);

	if (linearVelocity.GetLengthSqr() < 0.1f)
	{
		linearVelocity.Zero();
		angularVelocity.Zero();
	}
}
