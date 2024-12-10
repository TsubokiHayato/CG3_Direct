#pragma once
class Vector3
{

public:
    float x, y, z;

    // += 演算子のオーバーロード
    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }


    // スカラーとの掛け算の演算子オーバーロード
    Vector3 operator*(float scalar) const {
        return Vector3{ x * scalar, y * scalar, z * scalar };
    }

};

