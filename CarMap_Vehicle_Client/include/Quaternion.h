#ifndef _QUATERNION_H_
#define _QUATERNION_H_

/*
Copyright (c) 2007 Jernej Barbic

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

Except as contained in this notice, the name(s) of the above 
copyright holders shall not be used in advertising or otherwise 
to promote the sale, use or other dealings in this Software 
without prior written authorization.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

=====================================================

Quaternion C++ class.
This class implements quaternions and the commonly used 
algebraic operations on quaternions. The class is templated:
you can use either float or double precision.
Supports using quaternions to represent/manipulate rotations.
Version 1.0

Jernej Barbic
Carnegie Mellon University, Computer Graphics Lab, 2007

My contact information (email):
my_first_name.my_last_name@gmail.com
where you need to replace my_first_name with jernej
and my_last_name with barbic
(these are anti-spam measures)
Don't forget the . in between my first name and last name.

q = s + x * i + y * j + z * k

If using quaternions to represent rotations, q must be a unit quaternion. 
  Then the following relates q to the corresponding rotation:
    s = cos(angle/2)
    (x,y,z) = sin(angle/2) * axis_of_rotation
    (axis_of_rotation is unit length)

See also example.cpp .
*/


#include <stdio.h>
#include <math.h>

// forward declarations for the friend template
template <typename real> class Quaternion;
template <typename real> Quaternion<real> operator* (real alpha, Quaternion<real> q2);

template <typename real>
class Quaternion
{
public:

  inline Quaternion(); // q = 0
  inline Quaternion(real s, real x, real y, real z); // q = s + x * i + y * j + z * k
  inline Quaternion(real s); // q = s + 0 * i + 0 * j + 0 * k;

  // Makes the unit quaternion corresponding to a rotation around axis 'unitAxis' of angle 'angle'.
  // Angle is in radians; unitAxis must be a unit 3D vector.
  inline Quaternion(real angle, real unitAxis[3]); 

  inline void Set(real s, real x, real y, real z); // sets quaternion to the new value

  inline real Gets();
  inline real Getx();
  inline real Gety();
  inline real Getz();

  inline Quaternion operator+ (Quaternion q2) const; // q3 = q1+q2
  inline Quaternion operator- (Quaternion q2) const; // q3 = q1-q2
  inline Quaternion operator* (Quaternion q2) const; // q3 = q1 * q2
  inline Quaternion operator/ (Quaternion q2) const; // q3 = q1 / q2

  // Multiply quaternion with a scalar; e.g. q1 = alpha * q2;
  friend Quaternion<real> operator* (real alpha, Quaternion<real> q2)
  {
    return Quaternion<real>(alpha * q2.s, alpha * q2.x, alpha * q2.y, alpha * q2.z);
  }

  inline Quaternion conj(); // q2 = q1.conj()

  inline Quaternion & operator= (Quaternion rhs); // q2 = q1;
  inline Quaternion & operator= (real s_); // sets quaternion equal to the scalar quaternion s_
  inline int operator== (Quaternion rhs); // q2 == q1
  inline int operator!= (Quaternion rhs); // q2 != q1

  void Normalize(); // q.Normalize() scales q such that it is unit size

  inline void MoveToRightHalfSphere(); //  if scalar part (that is, 's') is negative, this will multiply the quaternion by -1
  
  inline real Norm2(); // returns the squared norm of the quaternion, i.e. s*s + x*x + y*y + z*z
  inline real Norm() { return sqrt(Norm2()); }

  // Transforms the quaternion to the corresponding rotation matrix.
  // Quaternion is assumed to be a unit quaternion.
  // R is a 3x3 orthogonal matrix and will be returned in row-major order.
  inline void Quaternion2Matrix(real * R); 

  // Transforms the given matrix (assumed orthogonal) into one of the two corresponding quaternions.
  // Matrix is assumed to be in row-major order.
  // There are two quaternions corresponding to a rotation (and they have opposite signs). You can't directly control which one you get, but you can force the real part to be non-negative by a subsequent call to MoveToRightHalfSphere() .
  // The implementation follows Baraff & Witkin SIGGRAPH 2003 course notes.
  static Quaternion Matrix2Quaternion(real * R);
  
  // Returns the angle of rotation (in radians), and the unit rotation axis corresponding to the quaternion.
  // Assumes a unit quaternion (use Normalize() to remove any noise due to floating point errors).
  // If s >= 0, the angle will be on the interval [0,pi] .
  // If s < 0, the angle will be on the interval (pi,2pi]. To get a representation where the angle is on [0, pi], you can manually flip the sign of the returned unitAxis, and use the angle of 2pi-angle. Alternatively, you can use MoveToRightHalfSphere before calling GetRotation to ensure that you are always in the s >= 0 case.
  inline void GetRotation(real * angle, real unitAxis[3]);

  // Returns (x,y,z) = sin(theta/2) * axis, where
  //   theta is the angle of rotation, theta is on [-pi,pi), and axis is the unit axis of rotation.
  // Assumes a unit quaternion.
  // Note: this routine is a bit exotic; I expect it to be not so widely used.
  inline void GetSinExponential(real * x, real * y, real * z);

  // Prints the quaternion to stdout.
  inline void Print();

protected:
  real s,x,y,z; 

};

template <typename real>
inline Quaternion<real>::Quaternion()
{
  s = x = y = z = 0;
}

template <typename real>
inline Quaternion<real>::Quaternion(real s_)
{
  s = s_;
  x = y = z = 0;
}

template <typename real>
inline Quaternion<real>::Quaternion(real angle, real unitAxis[3])
{
  s = cos(angle/2.0);
  real sin2 = sin(angle/2.0);
  x = sin2 * unitAxis[0];
  y = sin2 * unitAxis[1];
  z = sin2 * unitAxis[2];
}

template <typename real>
inline Quaternion<real>::Quaternion(real s_, real x_, real y_, real z_)
{
  s = s_;
  x = x_;
  y = y_;
  z = z_;
}

template <typename real>
inline void Quaternion<real>::Set(real s_g, real x_g, real y_g, real z_g) // sets quaternion to the new value
{
  s = s_g;
  x = x_g;
  y = y_g;
  z = z_g;
}

template <typename real>
inline real Quaternion<real>::Gets() { return s; }

template <typename real>
inline real Quaternion<real>::Getx() { return x; } 

template <typename real>
inline real Quaternion<real>::Gety() { return y; } 

template <typename real>
inline real Quaternion<real>::Getz() { return z; }

template <typename real>
inline Quaternion<real> & Quaternion<real>::operator= (Quaternion<real> rhs)
{
  s = rhs.s;
  x = rhs.x;
  y = rhs.y;
  z = rhs.z;

  return *this;
}

template <typename real>
inline Quaternion<real> & Quaternion<real>::operator= (real s_g)
{
  s = s_g;
  x = 0;
  y = 0;
  z = 0;

  return *this;
}

template <typename real>
inline int Quaternion<real>::operator== (Quaternion<real> rhs)
{
  return ((s == rhs.s) && (x == rhs.x) &&
          (y == rhs.y) && (z == rhs.z));
}

template <typename real>
inline int Quaternion<real>::operator!= (Quaternion<real> rhs)
{
  return ((s != rhs.s) || (x != rhs.x) ||
          (y != rhs.y) || (z != rhs.z));
}

template <typename real>
inline void Quaternion<real>::Normalize()
{
  real invNorm;
  invNorm = (real)1.0 / (real)sqrt(Norm2());

  s *= invNorm;
  x *= invNorm;
  y *= invNorm;
  z *= invNorm;

}

template <typename real>
inline real Quaternion<real>::Norm2() 
{ 
  return (s*s + x*x + y*y + z*z); 
}

template <typename real>
inline Quaternion<real> Quaternion<real>::operator+ (Quaternion<real> q2) const
{
  Quaternion<real> w(s + q2.s, x + q2.x, y + q2.y, z + q2.z);

  return w;  
}

template <typename real>
inline Quaternion<real> Quaternion<real>::operator- (Quaternion<real> q2) const
{
  Quaternion<real> w(s - q2.s, x - q2.x, y - q2.y, z - q2.z);
  return w;  
}

template <typename real>
inline Quaternion<real> Quaternion<real>::operator* (Quaternion<real> q2) const 
{
  Quaternion<real> w(
        s * q2.s - x * q2.x - y    * q2.y - z * q2.z,
        s * q2.x + q2.s * x + y    * q2.z - q2.y * z,
        s * q2.y + q2.s * y + q2.x * z    - x    * q2.z,
        s * q2.z + q2.s * z + x    * q2.y - q2.x * y);

  return w;  
}

template <typename real>
inline Quaternion<real> Quaternion<real>::operator/ (Quaternion<real> q2) const
{
  // compute invQ2 = q2^{-1}
  Quaternion<real> invQ2; 
  real invNorm2 = 1.0 / q2.Norm2();
  invQ2.s = q2.s * invNorm2;
  invQ2.x = -q2.x * invNorm2;
  invQ2.y = -q2.y * invNorm2;
  invQ2.z = -q2.z * invNorm2;

  // result = *this * invQ2
  return (*this * invQ2); 

}

template <typename real>
inline Quaternion<real> Quaternion<real>::conj()
{
  Quaternion<real> w(s,-x,-y,-z);
  return w;
}

// Transforms the quaternion to the corresponding rotation matrix.
// Quaternion is assumed to be a unit quaternion.
// R is a 3x3 orthogonal matrix and will be returned in row-major order.
template <typename real>
inline void Quaternion<real>::Quaternion2Matrix(real * R)
{
  R[0] = 1 - 2*y*y - 2*z*z; R[1] = 2*x*y - 2*s*z;     R[2] = 2*x*z + 2*s*y;
  R[3] = 2*x*y + 2*s*z;     R[4] = 1 - 2*x*x - 2*z*z; R[5] = 2*y*z - 2*s*x;
  R[6] = 2*x*z - 2*s*y;     R[7] = 2*y*z + 2*s*x;     R[8] = 1 - 2*x*x - 2*y*y;
}


// Returns (x,y,z) = sin(theta/2) * axis, where
//   theta is the angle of rotation, theta\in\{-pi,pi\}, and
//   axis is the unit axis of rotation.
template <typename real>
inline void Quaternion<real>::GetSinExponential(real * sex, real * sey, real * sez)
{
  if (s<0)
  {
    *sex = -x;
    *sey = -y;
    *sez = -z;
  }
  else
  {
    *sex = x;
    *sey = y;
    *sez = z;
  }
}

template <typename real>
inline void Quaternion<real>::GetRotation(real * angle, real unitAxis[3])
{
  if ((s >= ((real)1)) || (s <= (real)(-1)))
  {
    // identity; this check is necessary to avoid problems with acos if s is 1 + eps
    *angle = 0;
    unitAxis[0] = 1;
    unitAxis[0] = 0;
    unitAxis[0] = 0;
    return;
  }

  *angle = 2.0 * acos(s);
  real sin2 = x*x + y*y + z*z; //sin^2(*angle / 2.0)

  if (sin2 == 0)
  {
    // identity rotation; angle is zero, any axis is equally good
    unitAxis[0] = 1;
    unitAxis[0] = 0;
    unitAxis[0] = 0;
  }
  else
  {
    real inv = 1.0 / sqrt(sin2); // note: *angle / 2.0 is on [0,pi], so sin(*angle / 2.0) >= 0, and therefore the sign of sqrt can be safely taken positive
    unitAxis[0] = x * inv;
    unitAxis[1] = y * inv;
    unitAxis[2] = z * inv;
  }
}

template <typename real>
inline void Quaternion<real>::MoveToRightHalfSphere()
{
  if (s<0)
  {
    s *= -1;
    x *= -1;
    y *= -1;
    z *= -1;
  }
}

template <typename real>
inline void Quaternion<real>::Print()
{
  printf("%f + %fi + %fj + %fk\n",s,x,y,z);
}

#endif

