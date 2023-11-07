//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Shape.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"


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
void Scene::Initialize()
{
	for (int i = 0; i < 6; ++i)
	{
		for (int j = 0; j < 6; ++j)
		{
			Body body;
			float radius = 0.5f;
			float x = (i - 3) * radius * 1.5f; 
			float y = (j - 3) * radius * 1.5f; 
			body.position = Vec3(x, y, 10); 
			body.orientation = Quat(0, 0, 0, 1); 
			body.shape = new ShapeSphere(radius); 
			body.inverseMass = 1.0f; 
			body.elasticity = 0.5f;  
			body.friction = 0.5f; 
			body.linearVelocity.Zero();
			bodies.push_back(body); 
		}
	}

	float incrementalAngle = 0;
	float radiusArena = 5;
	float gap = 5;
	float n_balls = 22;

	for (int i = 0; i < n_balls; i++)
	{
		Body barrier;
		barrier.position = Vec3(cos(incrementalAngle) * radiusArena * gap, sin(incrementalAngle) * radiusArena * gap, 0);
		barrier.orientation = Quat(0, 0, 0, 1);
		barrier.shape = new ShapeSphere(radiusArena);
		barrier.inverseMass = 0.00f;
		barrier.elasticity = 0.5f;
		barrier.friction = 0.05f;
		barrier.linearVelocity = Vec3(0, 0, 0);
		incrementalAngle += 2 * 3.14159265 / n_balls;
		bodies.push_back(barrier);
	}


	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			Body earth;
			float radius = 50.0f; 
			float x = (i - 2) * radius * 0.25f; 
			float y = (j - 2) * radius * 0.25f; 
			earth.position = Vec3(x, y, -radius); 
			earth.orientation = Quat(0, 0, 0, 1);
			earth.shape = new ShapeSphere(radius);
			earth.inverseMass = 0.0f;
			earth.elasticity = 0.99f;
			earth.friction = 0.5f;
			bodies.push_back(earth);
		}
	}
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) 
{
	//  gravity
	for (auto& body : bodies)
	{
		if (body.inverseMass == 0.0f) continue;
		float mass = 1.0f / body.inverseMass;
		
		Vec3 impulse_gravity = Vec3{ 0.0f, 0.0f, -1.0f } * 50.0f * mass * dt_sec;
		body.ApplyImpulseLinear(impulse_gravity);
	}

	//  broadphase
	/*std::vector<CollisionPair> collisionPairs;
	BroadPhase(bodies.data(), bodies.size(), collisionPairs, dt_sec);*/

	//  collision checks (narrow phase)
	int num_contacts = 0; 
	const int max_contacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)alloca(sizeof(Contact) * max_contacts);

	for (int i = 0; i < bodies.size(); i++)
	{
		/*const CollisionPair& pair = collisionPairs[i];
		Body& bodyA = bodies[pair.a]; 
		Body& bodyB = bodies[pair.b]; 

		if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f) continue;

		Contact contact;
		if (Intersections::Intersect(bodyA, bodyB, dt_sec, contact)) 
		{
			contacts[num_contacts] = contact; 
			num_contacts++; 
		}*/

		for (int j = i + 1; j < bodies.size(); j++)
		{ 
			Body& bodyA = bodies[i]; 
			Body& bodyB = bodies[j]; 
			if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f) continue;
				 
			Contact contact; 
			if (Intersections::Intersect(bodyA, bodyB, dt_sec, contact)) 
			{
				contacts[num_contacts] = contact; 
				num_contacts++;
			}
		}
	}

	//  sort time of impact
	if (num_contacts > 1)
	{
		qsort(contacts, num_contacts, sizeof(Contact), Contact::CompareContact);
	}

	//  resolve contacts in order
	float accumulated_time = 0.0f;

	for (int i = 0; i < num_contacts; ++i)
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulated_time;
		Body* body_a = contact.a;
		Body* body_b = contact.b;

		// Skip body par with infinite mass
		if (body_a->inverseMass == 0.0f && body_b->inverseMass == 0.0f) continue;

		// Position update
		for (int j = 0; j < bodies.size(); ++j) 
		{
			bodies[j].Update(dt);
		}

		Contact::ResolveContact(contact);
		accumulated_time += dt;
	}


	// Other physics behavirous, outside collisions.
	
	// Update the positions for the rest of this frame's time.
	const float timeRemaining = dt_sec - accumulated_time; 
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < bodies.size(); ++i) 
		{
			bodies[i].Update(timeRemaining);
		}
	}
}