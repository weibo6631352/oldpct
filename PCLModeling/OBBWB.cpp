/* Start Header =========================================================================
File Name:		main.cpp
Purpose:		Generates an oriented bounding box (OBB) from a provided set of vertices
via principal-component analysis (PCA).
Language:		C++, Visual Studio 2013 compiler
Author:			Hew Jun-Wei
== End Header =========================================================================*/

// Includes:
//---------------------------------------------------------------------------------------
#include "OBBWB.h"
#include <vector>
#include <iostream>
#include <sstream>
/*#include <cmath>	// sqrt()*/
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3::Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3::~Vector3() {}
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3::Vector3(float x, float y, float z)
// {
//     this->x = x;
//     this->y = y;
//     this->z = z;
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3::Vector3(const Vector3& v)
// {
//     x = v.x;
//     y = v.y;
//     z = v.z;
// }
// 
// 
// /****************************************************************************************
// Overloading the * operator to represent the dot product.
// ****************************************************************************************/
// float Vector3::operator*(const Vector3& rhs) const
// {
//     return (x * rhs.x) + (y * rhs.y) + (z * rhs.z);
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3 Vector3::operator+(const Vector3& rhs) const
// {
//     return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3 Vector3::operator-(const Vector3& rhs) const
// {
//     return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3 Vector3::operator/(const float rhs) const
// {
//     return Vector3(x / rhs, y / rhs, z / rhs);
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3& Vector3::operator+=(const Vector3& rhs)
// {
//     x += rhs.x;
//     y += rhs.y;
//     z += rhs.z;
//     return *this;
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3& Vector3::operator-=(const Vector3& rhs)
// {
//     x -= rhs.x;
//     y -= rhs.y;
//     z -= rhs.z;
//     return *this;
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3& Vector3::operator*=(const float rhs)
// {
//     x *= rhs;
//     y *= rhs;
//     z *= rhs;
//     return *this;
// }
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3 Vector3::operator*(const float rhs)
// {
//     Vector3 pt = *this;
//     pt.x *= rhs;
//     pt.y *= rhs;
//     pt.z *= rhs;
//     return pt;
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3& Vector3::operator/=(const float rhs)
// {
//     x /= rhs;
//     y /= rhs;
//     z /= rhs;
//     return *this;
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3& Vector3::operator=(const Vector3& rhs)
// {
//     x = rhs.x;
//     y = rhs.y;
//     z = rhs.z;
//     return *this;
// }
// 
// 
// /****************************************************************************************
// Overloading the % operator to represent the cross product.
// ****************************************************************************************/
// Vector3 Vector3::operator%(const Vector3& rhs) const
// {
//     return Vector3((y * rhs.z) - (z * rhs.y),
//         (z * rhs.x) - (x * rhs.z),
//         (x * rhs.y) - (y * rhs.x));
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3 Vector3::operator-() const
// {
//     return Vector3(-x, -y, -z);
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// bool Vector3::operator==(const Vector3& rhs) const
// {
//     if (fabs(x - rhs.x) > static_cast<float>(1.e-5))
//         return false;
// 
//     if (fabs(y - rhs.y) > static_cast<float>(1.e-5))
//         return false;
// 
//     if (fabs(z - rhs.z) > static_cast<float>(1.e-5))
//         return false;
// 
//     return true;
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// float Vector3::Length() const
// {
//     return sqrt((*this) * (*this));
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// float Vector3::SquaredLength() const
// {
//     return (*this) * (*this);
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3 Vector3::Normalize() const
// {
//     return (*this) / Length();
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3& Vector3::NormalizeThis()
// {
//     (*this) /= Length();
//     return *this;
// }
// 
// 
// /***************************************************************************************/
// /***************************************************************************************/
// Vector3 operator*(const float lhs, const Vector3& rhs)
// {
//     return Vector3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
// }



// Constructors/Destructors:
//---------------------------------------------------------------------------------------
Matrix33::Matrix33() : a1(1.0f, 0.0f, 0.0f), a2(0.0f, 1.0f, 0.0f), a3(0.0f, 0.0f, 1.0f) {};
Matrix33::~Matrix33() {}
Matrix33::Matrix33(float m00, float m01, float m02,
    float m10, float m11, float m12,
    float m20, float m21, float m22) :
    a1(m00, m01, m02), a2(m10, m11, m12), a3(m20, m21, m22) {}


/***************************************************************************************/
/***************************************************************************************/
Matrix33::Matrix33(Vector3 a1, Vector3 a2, Vector3 a3) : a1(a1), a2(a2), a3(a3) {}


/***************************************************************************************/
/***************************************************************************************/
Matrix33::Matrix33(const Matrix33& rhs)
{
    (*this) = rhs;
}

// Member accessors:
//---------------------------------------------------------------------------------------

/***************************************************************************************/
/***************************************************************************************/
float& Matrix33::operator[](unsigned index)
{
    return data[index];
}


/***************************************************************************************/
/***************************************************************************************/
float Matrix33::operator[](unsigned index) const
{
    return data[index];
}


/***************************************************************************************/
/***************************************************************************************/
float& Matrix33::ColRow(unsigned column, unsigned row)
{
    return data[(3 * column) + row];
}


/***************************************************************************************/
/***************************************************************************************/
float Matrix33::ColRow(unsigned column, unsigned row) const
{
    return data[(3 * column) + row];
}


/***************************************************************************************/
/***************************************************************************************/
float& Matrix33::operator()(unsigned column, unsigned row)
{
    return data[(3 * column) + row];
}


/***************************************************************************************/
/***************************************************************************************/
float Matrix33::operator()(unsigned column, unsigned row) const
{
    return data[(3 * column) + row];
}


// Member operations:
//---------------------------------------------------------------------------------------

/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::Transpose() const
{
    return Matrix33(data[0], data[3], data[6],
        data[1], data[4], data[7],
        data[2], data[5], data[8]);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33& Matrix33::TransposeThis()
{
    (*this) = this->Transpose();

    return (*this);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::Inverse() const
{
    // cofactor matrix
    Matrix33 result((m11 * m22) - (m12 * m21), (m10 * m22) - (m20 * m12), (m10 * m21) - (m11 * m20),
        (m01 * m22) - (m21 * m02), (m00 * m22) - (m02 * m20), (m00 * m21) - (m20 * m01),
        (m01 * m12) - (m11 * m02), (m00 * m12) - (m10 * m02), (m00 * m11) - (m10 * m01));

    for (int i = 1; i < 8; i += 2)
        result.data[i] *= -1;

    result.TransposeThis();

    return result / Determinant();
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33& Matrix33::InverseThis()
{
    (*this) = this->Inverse();

    return (*this);
}


/***************************************************************************************/
/***************************************************************************************/
float Matrix33::Determinant() const
{
    return (m00 * ((m11 * m22) - (m12 * m21))) -
        (m10 * ((m01 * m22) - (m02 * m21))) +
        (m20 * ((m01 * m12) - (m02 * m11)));
}

// Matrix operations:
//---------------------------------------------------------------------------------------

/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::operator+(const Matrix33& rhs) const
{
    return Matrix33(a1 + rhs.a1, a2 + rhs.a2, a3 + rhs.a3);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::operator-(const Matrix33& rhs) const
{
    return Matrix33(a1 - rhs.a1, a2 - rhs.a2, a3 - rhs.a3);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33& Matrix33::operator+=(const Matrix33& rhs)
{
    for (int i = 0; i < 9; ++i)
        data[i] += rhs.data[i];

    return (*this);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33& Matrix33::operator-=(const Matrix33& rhs)
{
    for (int i = 0; i < 9; ++i)
        data[i] -= rhs.data[i];

    return (*this);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::operator*(const Matrix33& rhs) const
{
    return Matrix33((m00 * rhs.m00) + (m10 * rhs.m01) + (m20 * rhs.m02),
        (m01 * rhs.m00) + (m11 * rhs.m01) + (m21 * rhs.m02),
        (m02 * rhs.m00) + (m12 * rhs.m01) + (m22 * rhs.m02),

        (m00 * rhs.m10) + (m10 * rhs.m11) + (m20 * rhs.m12),
        (m01 * rhs.m10) + (m11 * rhs.m11) + (m21 * rhs.m12),
        (m02 * rhs.m10) + (m12 * rhs.m11) + (m22 * rhs.m12),

        (m00 * rhs.m20) + (m10 * rhs.m21) + (m20 * rhs.m22),
        (m01 * rhs.m20) + (m11 * rhs.m21) + (m21 * rhs.m22),
        (m02 * rhs.m20) + (m12 * rhs.m21) + (m22 * rhs.m22));
}


// Vector operations:
//---------------------------------------------------------------------------------------

/***************************************************************************************/
/***************************************************************************************/
Vector3 Matrix33::operator*(const Vector3& rhs) const
{
    return Vector3((data[0] * rhs.x) + (data[3] * rhs.y) + (data[6] * rhs.z),
        (data[1] * rhs.x) + (data[4] * rhs.y) + (data[7] * rhs.z),
        (data[2] * rhs.x) + (data[5] * rhs.y) + (data[8] * rhs.z));
}


// Scalar operations:
//---------------------------------------------------------------------------------------

/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::operator*(const float rhs) const
{
    Matrix33 result = (*this);

    for (int i = 0; i < 9; ++i)
        result.data[i] *= rhs;

    return result;
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33 operator*(float lhs, const Matrix33& rhs)
{
    return rhs * lhs;
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::operator/(const float rhs) const
{
    Matrix33 result = (*this);

    for (int i = 0; i < 9; ++i)
        result.data[i] /= rhs;

    return result;
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33& Matrix33::operator*=(const float rhs)
{
    for (int i = 0; i < 9; ++i)
        data[i] *= rhs;

    return (*this);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33& Matrix33::operator/=(const float rhs)
{
    for (int i = 0; i < 9; ++i)
        data[i] /= rhs;

    return (*this);
}


// The following functions construct a matrix:
//---------------------------------------------------------------------------------------

/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::Zero()
{
    return Matrix33(0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::Identity()
{
    return Matrix33(1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::Scale(float x, float y, float z)
{
    return Matrix33(x, 0.0f, 0.0f,
        0.0f, y, 0.0f,
        0.0f, 0.0f, z);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::Rotate(float x, float y, float z, float angle)
{
    return Rotate(Vector3(x, y, z), angle);
}


/***************************************************************************************/
/***************************************************************************************/
Matrix33 Matrix33::Rotate(Vector3 axis, float angle)
{
    axis.Normalize();
    float& x = axis.x;
    float& y = axis.y;
    float& z = axis.z;

    float cos = cosf(angle);
    float sin = sinf(angle);

    Matrix33 uCross(0.0f, z, -y,
        -z, 0.0f, x,
        y, -x, 0.0f);
    Matrix33 uTensor(x*x, x*y, x*z,
        y*x, y*y, y*z,
        z*x, z*y, z*z);

    return (cos * Identity()) + (sin * uCross) + ((1 - cos) * uTensor);
}






// Function prototypes:
//---------------------------------------------------------------------------------------
Matrix33		ComputeCovarianceMatrix(const Vector3* pVertices, const int numVertices);
void			GrahmSchmidt(Vector3& v0, Vector3& v1, Vector3& v2);
void			JacobiSolver(Matrix33 m, float eValues[3], Vector3 eVectors[3]);
std::ostream&	operator<<(std::ostream& os, const Vector3& v);

/****************************************************************************************
Function: GenerateOBB

Computes a bounding oriented bounding box (OBB) given the supplied vertices.

pVertex		: A pointer to the first Vector3 in the series.
numVertices : The number of vertices to compute the covariance matrix over.
****************************************************************************************/
OBBWB GenerateOBB(const Vector3* pVertices, const int numVertices)
{
    OBBWB			result;
    float		eValue[3];
    Vector3		eVector[3];
    Matrix33	covariance;
    Vector3		axis;

    for (int i = 0; i < numVertices; ++i)
    {
        result.position += pVertices[i];
    }

    result.position /= static_cast<float>(numVertices);

    covariance = ComputeCovarianceMatrix(pVertices, numVertices);

    JacobiSolver(covariance, eValue, eVector);

    float temp;
    Vector3 tempVec;

    // sort to obtain eValue[0] <= eValue[1] <= eValue[2]:
    if (eValue[0] <= eValue[1])
    {
        if (eValue[1] > eValue[2])
        {
            if (eValue[0] < eValue[2])
            {
                temp = eValue[0];
                tempVec = eVector[0];

                eValue[0] = eValue[2];
                eValue[2] = eValue[1];
                eValue[1] = temp;

                eVector[0] = eVector[2];
                eVector[2] = eVector[1];
                eVector[1] = tempVec;
            }
            else
            {
                temp = eValue[1];
                tempVec = eVector[1];

                eValue[1] = eValue[2];
                eValue[2] = temp;

                eVector[1] = eVector[2];
                eVector[2] = tempVec;
            }
        }
        // else do nothing
    }
    else
    {
        if (eValue[0] > eValue[2])
        {
            if (eValue[1] < eValue[2])
            {
                temp = eValue[0];
                tempVec = eVector[0];

                eValue[0] = eValue[1];
                eValue[1] = eValue[2];
                eValue[2] = temp;

                eVector[0] = eVector[1];
                eVector[1] = eVector[2];
                eVector[2] = tempVec;
            }
            else
            {
                temp = eValue[0];
                tempVec = eVector[0];

                eValue[0] = eValue[2];
                eValue[2] = temp;

                eVector[0] = eVector[2];
                eVector[2] = tempVec;
            }
        }
        else
        {
            temp = eValue[0];
            tempVec = eVector[0];

            eValue[0] = eValue[1];
            eValue[1] = temp;

            eVector[0] = eVector[1];
            eVector[1] = tempVec;
        }
    }

    result.orientation.a1 = eVector[2];
    result.orientation.a2 = eVector[1];
    result.orientation.a3 = eVector[0];

    // perform Grahm-Schmidt orthogonalization using the eigenvector corresponding to the
    // largest eigenvalue as the base vector
    GrahmSchmidt(result.orientation.a1, result.orientation.a2, result.orientation.a3);

    // eigenbasis set- now center the OBB in the middle
    float infinity = std::numeric_limits<float>::infinity();

    Vector3 minExtents(infinity, infinity, infinity);
    Vector3 maxExtents(-infinity, -infinity, -infinity);

    for (int index = 0; index < numVertices; ++index)
    {
        Vector3 vec = pVertices[index];
        Vector3 displacement = vec - result.position;

        minExtents.x = Min1(minExtents.x, displacement * result.orientation.a1);
        minExtents.y = Min1(minExtents.y, displacement * result.orientation.a2);
        minExtents.z = Min1(minExtents.z, displacement * result.orientation.a3);

        maxExtents.x = Max1(maxExtents.x, displacement * result.orientation.a1);
        maxExtents.y = Max1(maxExtents.y, displacement * result.orientation.a2);
        maxExtents.z = Max1(maxExtents.z, displacement * result.orientation.a3);
    }

    Vector3 offset = (maxExtents - minExtents) / 2.0f + minExtents;

    result.position += (offset.x * result.orientation.a1) +
        (offset.y * result.orientation.a2) +
        (offset.z * result.orientation.a3);

    for (int i = 0; i < 3; ++i)
    {
        result.halfExtents[i] = (maxExtents[i] - minExtents[i]) / 2.0f;
    }

    return result;
}

//判断一点是否在OBB包围盒内  
bool OBBWB::containPoint(const Vector3& point) const
{
    //相当于将点坐标从世界坐标系中转换到了OBB包围盒的物体坐标系中  
    Vector3 vd = point - position;
    /*
    dot方法为求点积
    由于_xAxis为单位矢量
    vd与_xAxis的点击即为在_xAxis方向的投影
    */
    float d = vd*orientation.a1 ;//计算x方向投影d  
    //判断投影是否大于x正方向的半长或小于x负方向半长  
    if (d >halfExtents.x || d < -halfExtents.x)
        return false;//满足条件说明不在包围盒内  

    d = vd*orientation.a2;//计算y方向投影  
    if (d >halfExtents.y || d < -halfExtents.y)
        return false;

    d = vd*orientation.a3;//计算z方向投影  
    if (d >halfExtents.z || d < -halfExtents.z)
        return false;

    return true;
}

/****************************************************************************************
Function: ComputeCovarianceMatrix

Computes a covariance matrix given a series of Vector3s.

pVec		: A pointer to the first Vector3 in the series.
numVertices : The number of vertices to compute the covariance matrix over.
****************************************************************************************/
Matrix33 ComputeCovarianceMatrix(const Vector3* pVertices, const int numVertices)
{
    Matrix33 covariance;

    // duplicate the vector array
    Vector3* pVectors = new Vector3[numVertices];

    // compute the average x, y, z values
    Vector3 avg(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < numVertices; ++i)
    {
        pVectors[i] = pVertices[i];
        avg += pVertices[i];
    }
    avg /= static_cast<float>(numVertices);

    for (int i = 0; i < numVertices; ++i)
    {
        pVectors[i] -= avg;
    }

    // compute the covariance (we are computing the lower-triangular entries then using
    // the symmetric property):
    for (int c = 0; c < 3; ++c)
    {
        for (int r = c; r < 3; ++r)
        {
            float& acc = covariance.ColRow(c, r);
            acc = 0.0f;

            // cov(X, Y) = E[(X - x)(Y - y)]
            for (int i = 0; i < numVertices; ++i)
            {
                acc += pVectors[i][c] * pVectors[i][r];
            }

            acc /= static_cast<float>(numVertices);

            covariance.ColRow(r, c) = acc;	// covariance matrix is symmetric
        }
    }

    delete[] pVectors;
    return covariance;
}


/****************************************************************************************
Function: GrahmSchmidt

Orthonormalizes three given vectors in the order provided.

v0 :	The major axis vector. This vector's orientation will be unchanged.
v1 :	The secondary axis vector.
v2 :	The tertiary axis vector.
****************************************************************************************/
void GrahmSchmidt(Vector3& v0, Vector3& v1, Vector3& v2)
{
    v0.Normalize();

    v1 -= (v1 * v0) * v0;
    v1.Normalize();

    v2 = v0 % v1;	// no need to normalize
}


/****************************************************************************************
Function: JacobiSolver

Computes the eigenvalues and corresponding eigenvectors of a given 3x3 matrix using
Jacobi's method.

Re: Numerical Methods with Computer Programs in C++ by Pallab Ghosh, Section 5.7.1

m			: The 3x3 matrix to compute the eigensystem for.
eValues		: An array of floats to hold the resulting eigenvalues.
eVectors	: An array of Vector3s to hold the resulting eigenvectors.
****************************************************************************************/
void JacobiSolver(Matrix33 m, float eValues[3], Vector3 eVectors[3])
{
    const float eps1 = static_cast<float>(1.e-5);	// Error tolerances
    const float eps2 = static_cast<float>(1.e-5);
    const float eps3 = static_cast<float>(1.e-5);

    float p, q, spq;
    float cosa, sina;					// holds cos(alpha) and sin(alpha)
    float temp;							// used for temporary storage
    float s1 = 0.0f;					// sums of squares of diagonal
    float s2;							// elements

    bool flag = true;					// determines whether to iterate again.
    int iteration = 0;					// iteration counter

    Vector3 mik;						// used for temporary storage of m[i][k]

    Matrix33 t = Matrix33::Identity();	// stores the product of the rotation matrices.
    // Its columns ultimately hold the eigenvectors

    do
    {
        iteration++;

        for (int i = 0; i < 2; ++i)
        {
            for (int j = i + 1; j < 3; ++j)
            {
                if ((fabs(m.ColRow(j, i)) < eps1))
                {
                    m.ColRow(j, i) = 0.0f;
                }
                else
                {
                    q = fabs(m.ColRow(i, i) - m.ColRow(j, j));

                    if (q > eps2)
                    {
                        p = 2.0f * m.ColRow(j, i) * q / (m.ColRow(i, i) - m.ColRow(j, j));
                        spq = sqrt(p * p + q * q);
                        cosa = sqrt((1.0f + q / spq) / 2.0f);
                        sina = p / (2.0f * cosa * spq);
                    }
                    else
                    {
                        sina = cosa = INV_SQRT_TWO;
                    }

                    for (int k = 0; k < 3; ++k)
                    {
                        temp = t.ColRow(i, k);
                        t.ColRow(i, k) = temp * cosa + t.ColRow(j, k) * sina;
                        t.ColRow(j, k) = temp * sina - t.ColRow(j, k) * cosa;
                    }

                    for (int k = i; k < 3; ++k)
                    {
                        if (k > j)
                        {
                            temp = m.ColRow(k, i);
                            m.ColRow(k, i) = cosa * temp + sina * m.ColRow(k, j);
                            m.ColRow(k, j) = sina * temp - cosa * m.ColRow(k, j);
                        }
                        else
                        {
                            mik[k] = m.ColRow(k, i);
                            m.ColRow(k, i) = cosa * mik[k] + sina * m.ColRow(j, k);

                            if (k == j)
                            {
                                m.ColRow(k, j) = sina * mik[k] - cosa * m.ColRow(k, j);
                            }
                        }
                    }

                    mik[j] = sina * mik[i] - cosa * mik[j];

                    for (int k = 0; k <= j; ++k)
                    {
                        if (k <= i)
                        {
                            temp = m.ColRow(i, k);
                            m.ColRow(i, k) = cosa * temp + sina * m.ColRow(j, k);
                            m.ColRow(j, k) = sina * temp - cosa * m.ColRow(j, k);
                        }
                        else
                        {
                            m.ColRow(j, k) = sina * mik[k] - cosa * m.ColRow(j, k);
                        }
                    }
                }
            }
        }

        s2 = 0.0f;

        for (int i = 0; i < 3; ++i)
        {
            eValues[i] = m.ColRow(i, i);
            s2 += eValues[i] * eValues[i];
        }

        if (fabs(s2) < static_cast<float>(1.e-5) || fabs(1 - s1 / s2) < eps3)
        {
            flag = false;
        }
        else
        {
            s1 = s2;
        }
    } while (flag);

    eVectors[0] = t.a1;
    eVectors[1] = t.a2;
    eVectors[2] = t.a3;

    // preserve righthanded-ness:
    if ((eVectors[0] % eVectors[1]) * eVectors[2] < 0.0f)
    {
        eVectors[2] = -eVectors[2];
    }

    std::cout << "\nEigenvectors converged in " << iteration << " iteration(s)" << std::endl;
}


/****************************************************************************************
Function: operator<<

Overloading the << operator to insert a Vector3 into a std::ostream for convenience.

os			: The std::ostream object to insert the Vector3 to.
v			: The Vector3 object to insert.
****************************************************************************************/
std::ostream& operator<<(std::ostream& os, const Vector3& v)
{
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}