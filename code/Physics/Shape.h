#pragma once
#include "../Math/Matrix.h"

class Shape {
public:
	enum class ShapeType
	{
		SHAPE_SPHERE,
	};

	virtual ShapeType GetType() const = 0;
	Vec3 GetCenterOfMass() const { return centerOfMass; }
	virtual Mat3 InertiaTensor() const = 0;

protected:
	Vec3 centerOfMass;
};

class ShapeSphere : public Shape {
public:
	ShapeSphere(float radiusP) : radius(radiusP)
	{
		centerOfMass.Zero();
	}

	ShapeType GetType() const override { return ShapeType::SHAPE_SPHERE; }
	Mat3 InertiaTensor() const override;

	float radius;
};

