#pragma once
#include "../Math/Matrix.h"
#include "../Math/Vector.h"
#include "../Math/Quat.h"
#include "../Math/Bounds.h"


class Shape 
{
public:
	enum class ShapeType
	{
		SHAPE_SPHERE,
		SHAPE_BOX,
		SHAPE_CONVEX
	};

	virtual ShapeType GetType() const = 0;
	Vec3 GetCenterOfMass() const { return centerOfMass; }
	virtual Mat3 InertiaTensor() const = 0;

	virtual Bounds GetBounds(const Vec3& pos, const Quat& orient) const = 0;
	virtual Bounds GetBounds() const = 0;

	virtual void Build(const std::vector<Vec3> pts, const int num) {}
	virtual Vec3 Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) = 0;
	virtual float FastestLinearSpeed(const Vec3& angularVelocity, const Vec3& dir) const { return 0.0f; }

protected:
	Vec3 centerOfMass;
};


class ShapeSphere : public Shape 
{
public:
	ShapeSphere(float radiusP) : radius(radiusP)
	{
		centerOfMass.Zero();
	}

	ShapeType GetType() const override { return ShapeType::SHAPE_SPHERE; }
	Mat3 InertiaTensor() const override;

	Bounds GetBounds(const Vec3& pos, const Quat& orient) const override;
	Bounds GetBounds() const override;

	Vec3 Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) override;

	float radius;
};


class ShapeBox : public Shape
{
public:
	ShapeBox(const std::vector<Vec3> points_, const int num)
	{
		Build(points_, num);
	}

	ShapeType GetType() const override { return ShapeType::SHAPE_BOX; }
	Mat3 InertiaTensor() const override;

	Bounds GetBounds(const Vec3& pos, const Quat& orient) const override;
	Bounds GetBounds() const override;

	void Build(const std::vector<Vec3> pts, const int num) override;
	Vec3 Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) override;
	float FastestLinearSpeed(const Vec3& angularVelocity, const Vec3& dir) const override;

	std::vector<Vec3> points;
	Bounds bounds;
};