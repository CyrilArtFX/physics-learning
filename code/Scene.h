//
//  Scene.h
//
#pragma once
#include <vector>

#include "Physics/Body.h"
#include "Petanque/Boule.h"
#include "Petanque/Cochonnet.h"

/*
====================================================
Scene
====================================================
*/
class Scene {
public:
	Scene() { bodies.reserve( 128 ); }
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );

	void LaunchCochonnet();
	void LaunchBoule();

	std::vector<Body&> bodies;

	bool cochonnetLaunched{ false };
	Cochonnet* cochonnet;
	std::vector<Boule*> boules;
};

