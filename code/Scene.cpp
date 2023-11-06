//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Shape.h"
#include "Physics/Intersections.h"


/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;
	body.position = Vec3( 1, 0, 10 );
	body.orientation = Quat( 0, 0, 0, 1 );
	body.shape = new ShapeSphere( 1.0f );
	body.inverseMass = 1.0f;
	body.elasticity = 0.5f;
	body.friction = 0.5f;
	bodies.push_back( body );

	Body earth; 
	earth.position = Vec3(0, 0, -1000); 
	earth.orientation = Quat(0, 0, 0, 1); 
	earth.shape = new ShapeSphere(1000.0f); 
	earth.inverseMass = 0.0f;
	earth.elasticity = 0.99f;
	earth.friction = 0.5f;
	bodies.push_back(earth); 
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) 
{
	for (auto& body : bodies)
	{
		if (body.inverseMass == 0.0f) continue;
		float mass = 1.0f / body.inverseMass;
		Vec3 impulse_gravity = Vec3(0.0f, 0.0f, -50.0f) * mass * dt_sec;
		body.ApplyImpulseLinear(impulse_gravity);
	}

	//  collision checks
	for (int i = 0; i < bodies.size(); i++)
	{
		for (int j = i + 1; j < bodies.size(); j++)
		{
			Body& a = bodies[i];
			Body& b = bodies[j];
			if (a.inverseMass == 0.0f && b.inverseMass == 0.0f) continue;

			Contact contact;
			if (Intersections::Intersect(a, b, contact))
			{
				Contact::ResolveContact(contact);
			}
		}
	}

	//  position updates
	for (auto& body : bodies)
	{
		body.Update(dt_sec);
	}
}