#pragma once
#include "../Physics/Body.h"

class Cochonnet : public Body
{
public:
	Cochonnet();

	void Update(float dt_sec) override;
};

