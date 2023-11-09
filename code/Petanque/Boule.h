#pragma once
#include "../Physics/Body.h"

class Boule : public Body
{
public:
	Boule();

	void Update(float dt_sec) override;
};

