/*************************************************************************
 *                                                                       *
 * Dual quaternions                                                      *
 *                                                                       *
 * Copyright (C) 2019 University of Southern California                  *
 * All rights reserved.                                                  *
 *                                                                       *
 * If you would like to use this code for any purpose                    *
 * (commercial, academic, non-profit, etc.),                             *
 * please contact jernej.barbic@gmail.com, or write to:                  *
 *                                                                       *
 * USC Stevens Center for Innovation                                     *
 * 1150 South Olive Street, Suite 2300                                   *
 * Los Angeles, California 90015                                         *
 * United States                                                         *
 *                                                                       *
 *************************************************************************/

#include "dualQuaternion.h"

DualDoubleQuaternion::DualDoubleQuaternion(const Quaternion<double> & rotation, const Vec3d & translation) : _real(rotation)
{
  computeDual(&translation[0]);
}


DualDoubleQuaternion::DualDoubleQuaternion(const double rotation[9], const double translation[3]) : _real(Quaternion<double>::Matrix2Quaternion(rotation))
{
  computeDual(translation);
}

DualDoubleQuaternion::DualDoubleQuaternion(const double transform[16])
{
  // 0 1 2 3
  double rotation[9] = { transform[0], transform[1], transform[2], transform[4], transform[5], transform[6],transform[8], transform[9], transform[10] };
  double translation[3] = { transform[3], transform[7], transform[11] };
  _real = Quaternion<double>::Matrix2Quaternion(rotation);
  computeDual(translation);
}

void DualDoubleQuaternion::decompose(Quaternion<double> & rotation, Vec3d & translation) const
{
  rotation = _real;
  computeTranslation(&translation[0]);
}


void DualDoubleQuaternion::decompose(double rotation[9], double translation[3]) const
{
  _real.Quaternion2Matrix(rotation);
  computeTranslation(translation);
}


inline void DualDoubleQuaternion::computeDual(const double translation[3])
{
  _real.MoveToRightHalfSphere();
  _dual.Set(-0.5*(translation[0] * _real.Getx() + translation[1] * _real.Gety() + translation[2] * _real.Getz()),
    0.5*(translation[0] * _real.Gets() + translation[1] * _real.Getz() - translation[2] * _real.Gety()),
    0.5*(-translation[0] * _real.Getz() + translation[1] * _real.Gets() + translation[2] * _real.Getx()),
    0.5*(translation[0] * _real.Gety() - translation[1] * _real.Getx() + translation[2] * _real.Gets()));
}

inline void DualDoubleQuaternion::computeTranslation(double translation[3]) const
{
  translation[0] = 2.0*(-_dual.Gets() * _real.Getx() + _dual.Getx() * _real.Gets() - _dual.Gety() * _real.Getz() + _dual.Getz() * _real.Gety());
  translation[1] = 2.0*(-_dual.Gets() * _real.Gety() + _dual.Getx() * _real.Getz() + _dual.Gety() * _real.Gets() - _dual.Getz() * _real.Getx());
  translation[2] = 2.0*(-_dual.Gets() * _real.Getz() - _dual.Getx() * _real.Gety() + _dual.Gety() * _real.Getx() + _dual.Getz() * _real.Gets());
}
