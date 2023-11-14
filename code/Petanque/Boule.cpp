#include "Boule.h"
#include "../Physics/Shape.h"

Boule::Boule()
{
	position = Vec3(0, 0, 10);
	orientation = Quat(0, 0, 0, 1);
	shape = new ShapeSphere(1.5f);
	inverseMass = 0.05f;
	elasticity = 0.0f;
	friction = 0.5f;
}

void Boule::Update(float dt_sec)
{
	PhysicUpdate(dt_sec);

	if (linearVelocity.GetLengthSqr() < 2.0f)
	{
		linearVelocity.Zero();
		angularVelocity.Zero();
	}
}
