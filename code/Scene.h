//
//  Scene.h
//
#pragma once
#include <vector>
#include <memory>


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

	std::vector<std::shared_ptr<Body>> bodies;

	bool cochonnetLaunched{ false };
	std::shared_ptr<Cochonnet> cochonnet;
	std::vector<std::shared_ptr<Boule>> boules;

	bool bodiesUpdated{ false };

	Vec3 camPos{ Vec3{0,0,0} };
	Vec3 camDir{ Vec3{1,0,0} };
};

