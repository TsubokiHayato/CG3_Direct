#pragma once
#include"cmath"
#include"Vector3.h"
#include<algorithm>
#undef min//minマクロを無効化
#undef max//maxマクロを無効化



struct TransformSRT {
	Vector3 scale;
	Vector3 rotate;
	Vector3 translate;
};

struct AABB {

	Vector3 min;
	Vector3 max;
};

struct  Sphere
{
	Vector3 center;
	float radius;
};

struct  Plane
{
	Vector3 normal;
	float distance;
};

//直線
struct  Line
{
	Vector3 origin;//始点
	Vector3 diff;//終点への差分ベクトル
};

//半直線
struct  Ray
{
	Vector3 origin;//始点
	Vector3 diff;//終点への差分ベクトル
};

//線分
struct  Segment
{
	Vector3 origin;//始点
	Vector3 diff;//終点への差分ベクトル
};

struct OBB {
	Vector3 center;
	Vector3 orientations[3];
	Vector3 size;
};

//直線と平面の当たり判定
bool IsCollision(const Line& line, const Plane& plane) {
	float dot = Dot(plane.normal, line.diff);

	if (dot == 0.0f) {
		return false;
	}

	float t = (plane.distance - Dot(line.origin, plane.normal)) / dot;

	if (t >= 0 && t <= 1) {
		return true;
	}

	if (t == -1.0f) {
		return true;
	}

	if (t == 2.0f) {
		return true;
	}

	return false;


}


//半直線と平面の当たり判定
bool IsCollision(const Ray& ray, const Plane& plane) {
	float dot = Dot(plane.normal, ray.diff);

	if (dot == 0.0f) {
		return false;
	}

	float t = (plane.distance - Dot(ray.origin, plane.normal)) / dot;

	if (t >= 0 && t <= 1) {
		return true;
	}

	if (t == -1.0f) {
		return false;
	}

	if (t == 2.0f) {
		return true;
	}

	return false;

}


//線分と平面の当たり判定
bool IsCollision(const Segment& segment, const Plane& plane) {
	float dot = Dot(plane.normal, segment.diff);

	if (dot == 0.0f) {
		return false;
	}

	float t = (plane.distance - Dot(segment.origin, plane.normal)) / dot;

	if (t >= 0 && t <= 1) {
		return true;
	}

	if (t == -1.0f) {
		return false;
	}

	if (t == 2.0f) {
		return false;
	}

	return false;
}


Vector3 Project(const Vector3& v1, const Vector3& v2) {

	// v1 を v2 に正射影する
	float dotProduct = v1.dot(v2);
	float v2LengthSquared = v2.dot(v2);
	return v2.scale(dotProduct / v2LengthSquared);

}

Vector3 ClosestPoint(const Vector3& point, const Segment& segment) {
	Vector3 segmentVec = segment.diff;
	Vector3 pointToOrigin = Subtract(point, segment.origin);
	float t = Dot(pointToOrigin, segmentVec) / Dot(segmentVec, segmentVec);

	Vector3 a = Add(segment.origin, Multiply(t, segmentVec));

	return a;
}
struct Triangle {
	Vector3 vertices[3];
};

Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 result;
	result.x = v1.y * v2.z - v1.z * v2.y;
	result.y = v1.z * v2.x - v1.x * v2.z;
	result.z = v1.x * v2.y - v1.y * v2.x;
	return result;
}

bool IsCollision(const Triangle& triangle, const Segment& segment) {

	Vector3 edge[2];
	edge[0] = Subtract(triangle.vertices[1], triangle.vertices[0]);
	edge[1] = Subtract(triangle.vertices[2], triangle.vertices[0]);


	Vector3 normal = Cross(edge[0], edge[1]);
	normal = Normalize(normal);

	Vector3 dir = segment.diff;
	dir = Normalize(dir);

	Vector3 diff = Subtract(triangle.vertices[0], segment.origin);

	float dotND = Dot(normal, dir);

	if (fabs(dotND) < 1e-6f) {
		// Segment is parallel to the triangle plane
		return false;
	}

	float t = Dot(normal, diff) / dotND;
	if (t < 0.0f || t > 1.0f) {
		// Segment does not intersect the triangle plane within the segment range
		return false;
	}


	Vector3 intersection = {
	   segment.origin.x + t * dir.x,
	   segment.origin.y + t * dir.y,
	   segment.origin.z + t * dir.z
	};


	Vector3 cross01 = Cross(Subtract(triangle.vertices[1], triangle.vertices[0]), Subtract(intersection, triangle.vertices[0]));
	Vector3 cross12 = Cross(Subtract(triangle.vertices[2], triangle.vertices[1]), Subtract(intersection, triangle.vertices[1]));
	Vector3 cross20 = Cross(Subtract(triangle.vertices[0], triangle.vertices[2]), Subtract(intersection, triangle.vertices[2]));


	if (Dot(cross01, normal) >= 0.0f &&
		Dot(cross12, normal) >= 0.0f &&
		Dot(cross20, normal) >= 0.0f)
	{
		return true;

	}
	return false;
}
bool IsCollision(const AABB& a, const AABB& b) {

	if ((a.min.x <= b.max.x && a.max.x >= b.min.x) &&
		(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
		(a.min.z <= b.max.z && a.max.z >= b.min.z)) {
		return true;
	}
	return false;
}



bool IsCollision(const AABB& aabb, const Sphere& sphere) {

	Vector3 closestPoint
	{
		std::clamp(sphere.center.x,aabb.min.x,aabb.max.x),
		std::clamp(sphere.center.y,aabb.min.y,aabb.max.y),
		std::clamp(sphere.center.z,aabb.min.z,aabb.max.z)
	};

	float distance = Length(closestPoint - sphere.center);
	if (distance <= sphere.radius) {
		return true;
	}
	return false;
}


bool IsCollision(const AABB& aabb, const Segment& segment) {
	// x軸方向の交差点でのt値
	float tNearX = (aabb.min.x - segment.origin.x) / segment.diff.x;
	float tFarX = (aabb.max.x - segment.origin.x) / segment.diff.x;

	if (std::isnan(tNearX) || std::isnan(tFarX)) return false; // NaNが検出された場合は衝突なしとする
	if (tNearX > tFarX) std::swap(tNearX, tFarX);

	// y軸方向の交差点でのt値
	float tNearY = (aabb.min.y - segment.origin.y) / segment.diff.y;
	float tFarY = (aabb.max.y - segment.origin.y) / segment.diff.y;

	if (std::isnan(tNearY) || std::isnan(tFarY)) return false; // NaNが検出された場合は衝突なしとする
	if (tNearY > tFarY) std::swap(tNearY, tFarY);

	// z軸方向の交差点でのt値
	float tNearZ = (aabb.min.z - segment.origin.z) / segment.diff.z;
	float tFarZ = (aabb.max.z - segment.origin.z) / segment.diff.z;

	if (std::isnan(tNearZ) || std::isnan(tFarZ)) return false; // NaNが検出された場合は衝突なしとする
	if (tNearZ > tFarZ) std::swap(tNearZ, tFarZ);

	// tNearの最大値とtFarの最小値を求める
	float tmin = std::max(std::max(tNearX, tNearY), tNearZ);
	float tmax = std::min(std::min(tFarX, tFarY), tFarZ);

	// tminとtmaxがNaNでないことを確認
	if (std::isnan(tmin) || std::isnan(tmax)) return false;

	// 線分がAABBと交差するかどうかを判定
	if (tmin <= tmax && tmax >= 0.0f && tmin <= 1.0f) {
		return true;
	}
	return false;
}


// Linear interpolation between two vectors v1 and v2
Vector3 Lerp(const Vector3& v1, const Vector3& v2, float t) {
	return v1 + (v2 - v1) * t;
}


Vector3 Bezier(const Vector3& p0, const Vector3& p1, const Vector3& p2, float t) {
	Vector3 p0p1 =
		Lerp(p0, p1, t);
	Vector3 p1p2 =
		Lerp(p1, p2, t);


	return Lerp(p0p1, p1p2, t);
}


bool IsCollision(const AABB& aabb, const Vector3& point) {
	if (point.x >= aabb.min.x && point.x <= aabb.max.x &&
		point.y >= aabb.min.y && point.y <= aabb.max.y &&
		point.z >= aabb.min.z && point.z <= aabb.max.z) {
		return true;
	}
	return false;
}