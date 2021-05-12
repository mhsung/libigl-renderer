// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "Symmetry.h"

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utils/google_tools.h>
#include <utils/glut_geometry/glut_geometry.h>
#include <utils/utils.h>


namespace {
  const double kZeroTol = 1.0e-6;
  const double kAxisLength = 1000;
  const double kAxisRadius = 0.01;

  void glVertex3dv(const Vector3d& _vec) {
    glVertex3d(_vec[0], _vec[1], _vec[2]);
  }

  void glTranslate(const Vector3d& _vec) {
    glTranslated(_vec[0], _vec[1], _vec[2]);
  }

  void glRotate(const Vector3d& _axis) {
    const double dot_prod = std::min(std::max(
          _axis.normalized().dot(Vector3d::UnitZ()), -1.0), +1.0);
    Vector3d rot_axis = Vector3d::UnitZ().cross(_axis);
    if (rot_axis.norm() < kZeroTol) rot_axis = Vector3d::UnitX();
    const double rot_angle = std::acos(dot_prod) / M_PI * 180.0;
    glRotated(rot_angle, rot_axis[0], rot_axis[1], rot_axis[2]);
  }
}

Symmetry* Symmetry::from_json(const json& _j) {
  if (_j.at("type").get<std::string>() == "rotational") {
    return static_cast<Symmetry*>(RotationalSymmetry::from_json(_j));
  }
  else {
    LOG(WARNING) << "The symmetry type is not supported: '"
      << _j.at("type").get<std::string>() << "'.";
  }
  return nullptr;
}

RotationalSymmetry::RotationalSymmetry()
  : c_(Vector3d::Zero()),
    d_(Vector3d::UnitZ()) {
}

RotationalSymmetry::RotationalSymmetry(const Vector3d& _c, const Vector3d& _d)
  : c_(_c),
    d_(_d) {
    CHECK_GT(d_.norm(), kZeroTol);
    d_.normalize();
}

RotationalSymmetry::RotationalSymmetry(const RotationalSymmetry& _other)
  : c_(_other.c_),
    d_(_other.d_) {
}

void RotationalSymmetry::render() const {
  glPushMatrix();

  // Move to the base center.
  const Vector3d base_center = c_ - 0.5 * kAxisLength * d_;
  glTranslate(base_center);

  glRotate(d_);

  glutSolidCylinder(kAxisRadius, kAxisLength, 32, 32);
  glPopMatrix();
}

void RotationalSymmetry::to_json(json& _j) const {
  _j["type"] = "rotational_symmetry";

  _j["center_x"] = c_[0];
  _j["center_y"] = c_[1];
  _j["center_z"] = c_[2];

  _j["direction_x"] = d_[0];
  _j["direction_y"] = d_[1];
  _j["direction_z"] = d_[2];
}

RotationalSymmetry* RotationalSymmetry::from_json(const json& _j) {
  RotationalSymmetry* p = new RotationalSymmetry();

  p->c_[0] = _j.at("center_x").get<double>();
  p->c_[1] = _j.at("center_y").get<double>();
  p->c_[2] = _j.at("center_z").get<double>();

  p->d_[0] = _j.at("direction_x").get<double>();
  p->d_[1] = _j.at("direction_y").get<double>();
  p->d_[2] = _j.at("direction_z").get<double>();

  return p;
}

