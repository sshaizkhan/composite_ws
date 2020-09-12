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

#ifndef _DUAL_QUATERNION_H_
#define _DUAL_QUATERNION_H_

#include "vec3d.h"
#include "quaternion.h"

class DualDoubleQuaternion
{
public:
  DualDoubleQuaternion() {}
  DualDoubleQuaternion(const Quaternion<double> & real_, const Quaternion<double> & dual_) : _real(real_), _dual(dual_) {}
  DualDoubleQuaternion(const Quaternion<double> & rotation, const Vec3d & translation);
  DualDoubleQuaternion(const double rotation[9], const double translation[3]);
  DualDoubleQuaternion(const double transform[16]);
  DualDoubleQuaternion operator * (const double s) const { return DualDoubleQuaternion(_real * s, _dual * s); }
  DualDoubleQuaternion & operator += (const DualDoubleQuaternion & p) {
    _real = _real + p.real();
    _dual = _dual + p.dual();
    return *this;
  }
  const Quaternion<double> & real() const { return _real; }
  const Quaternion<double> & dual() const { return _dual; }

  void normalize() {
    double norm = _real.Norm();
    _real = _real / norm;
    _dual = _dual / norm;
  }

  // Warning: when calling these following two decompose functions, 
  // you must make sure the dual quaterinon is a unit dual quaternion,
  // otherwises, the results are not correct.
  void decompose(Quaternion<double> & rotation, Vec3d & translation) const;
  void decompose(double rotation[9], double translation[3]) const;
protected:
  Quaternion<double> _real, _dual;
  inline void computeDual(const double translation[3]);
  inline void computeTranslation(double translation[3]) const;
};

#endif
