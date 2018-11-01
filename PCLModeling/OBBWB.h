/* Start Header =========================================================================
File Name:		OBB.h
Purpose:		Implements an oriented bounding box.
Language:		C++, Visual Studio 2013 compiler
Author:			Hew Jun-Wei
== End Header =========================================================================*/

// Header Guard:
//---------------------------------------------------------------------------------------
#ifndef OBB_H
#define OBB_H
#include "MathGeoLib/Math/float3.h"

class Vector3 :public float3
{
public:
    Vector3(vec f) :float3(f){};
//    Vector3(float3 f) :float3(f){};
    Vector3() :float3(){};
    Vector3(float x, float y, float z) :float3(x,y,z){};
    float		operator*(const Vector3& rhs)	const	// dot product
    {
        return (x * rhs.x) + (y * rhs.y) + (z * rhs.z);
    }
    Vector3		operator%(const Vector3& rhs)	const// cross product
    {
        return Vector3((y * rhs.z) - (z * rhs.y),
            (z * rhs.x) - (x * rhs.z),
            (x * rhs.y) - (y * rhs.x));
    }
    Vector3& operator+=(const Vector3& rhs)// +=
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
};

const float INV_SQRT_TWO = 0.707106781f;

template <typename T>
inline T Max1(const T x, const T y)
{
    return (x < y) ? y : x;
}

template <typename T>
inline T Min1(const T x, const T y)
{
    return (x > y) ? y : x;
}
// 
// struct Vector3
// {
//     union
//     {
//         struct
//         {
//             float x, y, z;
//         };
//         float data[3];
//     };
//     Vector3();
//     ~Vector3();
//     Vector3(const Vector3& v);
//     Vector3(float x, float y, float z);
// 
//     Vector3		Normalize()						const;
//     Vector3&	NormalizeThis();
// 
//     float		Length()						const;
//     float		SquaredLength()					const;
// 
//     Vector3		operator-()						const;
//     Vector3&	operator=(const Vector3& rhs);
// 
//     Vector3		operator+(const Vector3& rhs)	const;
//     Vector3		operator-(const Vector3& rhs)	const;
//     Vector3&	operator+=(const Vector3& rhs);
//     Vector3&	operator-=(const Vector3& rhs);
// 
//     float		operator*(const Vector3& rhs)	const;	// dot product
//     Vector3     operator*(const float rhs);
//     Vector3		operator%(const Vector3& rhs)	const;	// cross product
// 
//     bool		operator==(const Vector3& rhs)	const;
// 
//     Vector3		operator/(const float rhs)		const;
//     Vector3&	operator*=(const float rhs);
//     Vector3&	operator/=(const float rhs);
// 
//     float &At(int index)
//     {
//         return data[index];
//     }
// 
//     float &operator [](int index) { return At(index); }
// };
// 
// Vector3 operator*(const float lhs, const Vector3& rhs);

struct Matrix33
{
    // Constructors/Destructors:
    //-------------------------------------------------
    Matrix33();
    ~Matrix33();

    Matrix33(float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22);
    Matrix33(Vector3 a1, Vector3 a2, Vector3 a3);
    Matrix33(const Matrix33& rhs);

    float& ColRow(unsigned column, unsigned row);
    float  ColRow(unsigned column, unsigned row) const;
    float& operator[](unsigned index);
    float  operator[](unsigned index) const;
    float& operator()(unsigned column, unsigned row);
    float  operator()(unsigned column, unsigned row) const;

    Matrix33 Transpose() const;
    Matrix33& TransposeThis();

    Matrix33 Inverse() const;
    Matrix33& InverseThis();

    float Determinant() const;

    Matrix33 operator+(const Matrix33& rhs) const;
    Matrix33 operator-(const Matrix33& rhs) const;
    Matrix33 operator*(const Matrix33& rhs) const;

    Matrix33& operator+=(const Matrix33& rhs);
    Matrix33& operator-=(const Matrix33& rhs);

    Vector3 operator*(const Vector3& rhs) const;

    Matrix33 operator*(const float rhs) const;
    Matrix33 operator/(const float rhs) const;
    Matrix33& operator*=(const float rhs);
    Matrix33& operator/=(const float rhs);

    static Matrix33 Identity();
    static Matrix33 Zero();
    static Matrix33 Scale(float x, float y, float z = 1.0f);
    static Matrix33 Rotate(float x, float y, float z, float angle);
    static Matrix33 Rotate(Vector3 axis, float angle);

    union
    {
        struct
        {
            // matrix data is stored COLUMN-MAJOR
            // naming convention is m[col][row]
            float	m00, m01, m02,
            m10, m11, m12,
            m20, m21, m22;
        };
        struct
        {
            Vector3 a1, a2, a3;
        };
        float data[9];
    };
};

Matrix33 operator*(float lhs, const Matrix33& rhs);

class OBBWB
{
public:
    OBBWB::OBBWB() : position(0.0f, 0.0f, 0.0f), halfExtents(1.0f, 1.0f, 1.0f)
    {
        orientation = Matrix33::Identity();
    }

    ~OBBWB() {}
    bool containPoint(const Vector3& point) const;

    Vector3		position;
    Matrix33	orientation;
    Vector3		halfExtents;	// half-extents along local x, y, z axes
};

OBBWB				GenerateOBB(const Vector3* pVertices, const int numVertices);

#endif