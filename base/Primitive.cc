// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "Primitive.h"

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utils/google_tools.h>
#include <utils/glut_geometry/glut_geometry.h>
#include <utils/utils.h>


namespace {
  const double kZeroTol = 1.0e-6;

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

Primitive* Primitive::from_json(const json& _j) {
  Primitive* p = nullptr;
  if (_j.at("type").get<std::string>() == "plane") {
    p = static_cast<Primitive*>(Plane::from_json(_j));
  }
  else if (_j.at("type").get<std::string>() == "cylinder") {
    p = static_cast<Primitive*>(Cylinder::from_json(_j));
  }
  else if (_j.at("type").get<std::string>() == "sphere") {
    p = static_cast<Primitive*>(Sphere::from_json(_j));
  }
  else if (_j.at("type").get<std::string>() == "cone") {
    p = static_cast<Primitive*>(Cone::from_json(_j));
  }
  else if (_j.at("type").get<std::string>() == "torus") {
    p = static_cast<Primitive*>(Torus::from_json(_j));
  }
  else if (_j.at("type").get<std::string>() == "prism") {
    p = static_cast<Primitive*>(Prism::from_json(_j));
  }
  else if (_j.at("type").get<std::string>() == "box") {
    p = static_cast<Primitive*>(Box::from_json(_j));
  }
  else {
    LOG(WARNING) << "The primitive type is not supported: '"
      << _j.at("type").get<std::string>() << "'.";
  }

  if (p != nullptr) {
    p->set_label(_j.at("label").get<int>());
  }

  return p;
}

Primitive::Primitive()
  : label_(-1) {
}

Primitive::Primitive(const Primitive& _other)
  : label_(_other.label_) {
}

int Primitive::get_label() const {
  return label_;
}

void Primitive::set_label(const int _label) {
  CHECK_GE(_label, 0);
  label_ = _label;
}

Plane::Plane()
  : PrimitiveBase(),
    n_(Vector3d::UnitZ()),
    c_(Vector3d::Zero()) {
    set_default_clipping_params();
}

Plane::Plane(const Vector3d& _n, const Vector3d& _c)
  : PrimitiveBase(),
    n_(_n),
    c_(_c) {
    n_.normalize();
    set_default_clipping_params();
}

Plane::Plane(const Vector3d& _n, const double _d)
  : PrimitiveBase(),
    n_(_n),
    c_(_d * _n) {
    n_.normalize();
    set_default_clipping_params();
}

Plane::Plane(const Plane& _other)
  : PrimitiveBase(_other),
    n_(_other.n_),
    c_(_other.c_),
    x_(_other.x_),
    y_(_other.y_),
    sx_(_other.sx_),
    sy_(_other.sy_) {
}

double Plane::area() const {
  return sx_ * sy_;
}

double Plane::signed_distance(const Vector3d& _p) const {
  const Vector3d v = _p - c_;
  const double d = (v).dot(n_);
  return d;
}

Vector3d Plane::project(const Vector3d& _p) const {
  const Vector3d v = _p - c_;
  const double d = (v).dot(n_);

  const Vector3d proj_p = _p - d * n_;
  return proj_p;
}

void Plane::set_default_clipping_params() {
  int idx;
  n_.array().abs().minCoeff(&idx);
  x_ = n_.cross(Vector3d::Unit(idx)).normalized();
  y_ = n_.cross(x_);
  sx_ = 1.0;
  sy_ = 1.0;
}

void Plane::compute_clipping_params(const MatrixXd& _P) {
  CHECK_GT(_P.rows(), 0);
  CHECK_EQ(_P.cols(), 3);

  // Offset. n^T * x + d = 0.
  const double d = -(n_.dot(c_));

  const int kNumAngles = 36;
  const double unit_angle = 2.0 * M_PI / (double)kNumAngles;
  const Vector3d orig_x = x_;
  const Vector3d orig_y = y_;
  double min_area = std::numeric_limits<double>::max();

  for (int i = 0; i < kNumAngles; ++i) {
    const double angle = i * unit_angle;
    const Matrix3d rotation = AngleAxisd(angle, n_).toRotationMatrix();
    const Vector3d x_i = rotation * orig_x;
    const Vector3d y_i = rotation * orig_y;

    const VectorXd x_proj_P = _P * x_i;
    const VectorXd y_proj_P = _P * y_i;
    const double sx_i = x_proj_P.maxCoeff() - x_proj_P.minCoeff();
    const double sy_i = y_proj_P.maxCoeff() - y_proj_P.minCoeff();
    const double area_i = sx_i * sy_i;

    if (area_i < min_area) {
      min_area = area_i;
      x_ = x_i;
      y_ = y_i;
      sx_ = sx_i;
      sy_ = sy_i;

      const double cx_i = 0.5 * (x_proj_P.maxCoeff() + x_proj_P.minCoeff());
      const double cy_i = 0.5 * (y_proj_P.maxCoeff() + y_proj_P.minCoeff());
      c_ = (-d) * n_ + cx_i * x_i + cy_i * y_i;
    }
  }
}

MatrixXd Plane::randomly_sample_points(const int n_points) const {
  // [-0.5sx, +0.5sx].
  const VectorXd x_coeffs = 0.5 * sx_ * VectorXd::Random(n_points);
  // [-0.5sy, +0.5sy].
  const VectorXd y_coeffs = 0.5 * sy_ * VectorXd::Random(n_points);
  const MatrixXd centered_points =
    x_coeffs * x_.transpose() + y_coeffs * y_.transpose();
  const MatrixXd points = centered_points.rowwise() + c_.transpose();
  return points;
}

void Plane::render() const {
  const Vector3d dx = 0.5 * sx_ * x_;
  const Vector3d dy = 0.5 * sy_ * y_;
  std::vector<Vector3d> vertices;
  vertices.push_back(c_ - dx - dy);
  vertices.push_back(c_ + dx - dy);
  vertices.push_back(c_ - dx + dy);
  vertices.push_back(c_ + dx + dy);

  glPushMatrix();
  glBegin(GL_TRIANGLES);

  // (0, 1, 2)
  glVertex3dv(vertices[0]);
  glVertex3dv(vertices[1]);
  glVertex3dv(vertices[2]);

  // (1, 3, 2)
  glVertex3dv(vertices[1]);
  glVertex3dv(vertices[3]);
  glVertex3dv(vertices[2]);

  glEnd();
  glPopMatrix();
}

void Plane::render_axes() const {
  LOG(WARNING) << "Not implemented yet.";
}

void Plane::to_json(json& _j) const {
  _j["type"] = "plane";
  _j["label"] = label_;

  _j["normal_x"] = n_[0];
  _j["normal_y"] = n_[1];
  _j["normal_z"] = n_[2];

  _j["center_x"] = c_[0];
  _j["center_y"] = c_[1];
  _j["center_z"] = c_[2];

  _j["x_axis_x"] = x_[0];
  _j["x_axis_y"] = x_[1];
  _j["x_axis_z"] = x_[2];

  _j["y_axis_x"] = y_[0];
  _j["y_axis_y"] = y_[1];
  _j["y_axis_z"] = y_[2];

  _j["x_size"] = sx_;
  _j["y_size"] = sy_;
}

Plane* Plane::from_json(const json& _j) {
  Plane* p = new Plane();

  try {
    p->n_[0] = _j.at("normal_x").get<double>();
    p->n_[1] = _j.at("normal_y").get<double>();
    p->n_[2] = _j.at("normal_z").get<double>();
  } catch (json::out_of_range& e) {
    try {
      p->n_[0] = _j.at("axis_x").get<double>();
      p->n_[1] = _j.at("axis_y").get<double>();
      p->n_[2] = _j.at("axis_z").get<double>();
    } catch (json::out_of_range& e) {
      LOG(FATAL) << "[Plane] normal (or axis) is not given.";
    }
  }

  try {
    p->c_[0] = _j.at("center_x").get<double>();
    p->c_[1] = _j.at("center_y").get<double>();
    p->c_[2] = _j.at("center_z").get<double>();
  } catch (json::out_of_range& e) {
    try {
      p->c_[0] = _j.at("location_x").get<double>();
      p->c_[1] = _j.at("location_y").get<double>();
      p->c_[2] = _j.at("location_z").get<double>();
    } catch (json::out_of_range& e) {
      LOG(FATAL) << "[Plane] center (or location) is not given.";
    }
  }

  try {
    p->x_[0] = _j.at("x_axis_x").get<double>();
    p->x_[1] = _j.at("x_axis_y").get<double>();
    p->x_[2] = _j.at("x_axis_z").get<double>();

    p->y_[0] = _j.at("y_axis_x").get<double>();
    p->y_[1] = _j.at("y_axis_y").get<double>();
    p->y_[2] = _j.at("y_axis_z").get<double>();
  } catch (json::out_of_range& e) {
    p->set_default_clipping_params();
  }

  try {
    p->sx_ = _j.at("x_size").get<double>();
    p->sy_ = _j.at("y_size").get<double>();
  } catch (json::out_of_range& e) {
    p->sx_ = 1.0;
    p->sy_ = 1.0;
  }

  return p;
}

Cylinder::Cylinder()
  : PrimitiveBase(),
    c_(Vector3d::Zero()),
    n_(Vector3d::UnitZ()),
    r_(1.0) {
    set_default_clipping_params();
  }

Cylinder::Cylinder(const Vector3d& _c, const Vector3d& _n, const double _r,
    const double _h)
  : PrimitiveBase(),
    c_(_c),
    n_(_n),
    r_(_r),
    h_(_h) {
    CHECK_GT(r_, 0.0);
    CHECK_GT(n_.norm(), kZeroTol);
    n_.normalize();
    CHECK_GT(h_, 0.0);
    //set_default_clipping_params();
  }

Cylinder::Cylinder(const Cylinder& _other)
  : PrimitiveBase(_other),
    c_(_other.c_),
    n_(_other.n_),
    r_(_other.r_),
    h_(_other.h_) {
  }

double Cylinder::area() const {
  return 2.0 * M_PI * r_ * h_;
}

double Cylinder::signed_distance(const Vector3d& _p) const {
  const Vector3d v = _p - c_;
  const Vector3d w = v.dot(n_) * n_;
  const Vector3d x = v - w;
  const double d = x.norm() - r_;
  return d;
}

Vector3d Cylinder::project(const Vector3d& _p) const {
  const Vector3d v = _p - c_;
  const Vector3d w = v.dot(n_) * n_;
  const Vector3d x = v - w;
  const double d = x.norm() - r_;

  if (x.norm() < kZeroTol) {
    LOG(WARNING) << "Point is on the cylinder axis.";
    return _p;
  }

  const Vector3d proj_p = _p - d * x.normalized();
  return proj_p;
}

void Cylinder::set_default_clipping_params() {
  h_ = 1.0;
}

void Cylinder::compute_clipping_params(const MatrixXd& _P) {
  CHECK_GT(_P.rows(), 0);
  CHECK_EQ(_P.cols(), 3);

  const MatrixXd centered_P = _P.rowwise() - c_.transpose();
  const VectorXd axis_dists = centered_P * n_;
  const double min_axis_dists = axis_dists.minCoeff();
  const double max_axis_dists = axis_dists.maxCoeff();
  c_ += 0.5 * (min_axis_dists + max_axis_dists) * n_;
  h_ = max_axis_dists - min_axis_dists;
}

MatrixXd Cylinder::randomly_sample_points(const int n_points) const {
  int idx;
  n_.array().abs().minCoeff(&idx);
  const Vector3d x = n_.cross(Vector3d::Unit(idx)).normalized();
  const Vector3d y = n_.cross(x);

  // [-pi, +pi]
  const VectorXd a_coeffs = M_PI * VectorXd::Random(n_points);
  const VectorXd x_coeffs = r_ * a_coeffs.array().cos();
  const VectorXd y_coeffs = r_ * a_coeffs.array().sin();
  const MatrixXd centered_circular_points =
    x_coeffs * x.transpose() + y_coeffs * y.transpose();
  const MatrixXd circular_points =
    centered_circular_points.rowwise() + c_.transpose();

  // [-0.5h, +0.5h]
  const VectorXd h_coeffs = 0.5 * h_ * VectorXd::Random(n_points);
  const MatrixXd points = h_coeffs * n_.transpose() + circular_points;
  return points;
}

void Cylinder::render() const {
  glPushMatrix();

  // Move to the base center.
  const Vector3d base_center = c_ - 0.5 * h_ * n_;
  glTranslate(base_center);

  glRotate(n_);

  glutSolidCylinder(r_, h_, 32, 32);
  glPopMatrix();
}

void Cylinder::render_axes() const {
  LOG(WARNING) << "Not implemented yet.";
}

void Cylinder::to_json(json& _j) const {
  _j["type"] = "cylinder";
  _j["label"] = label_;

  _j["center_x"] = c_[0];
  _j["center_y"] = c_[1];
  _j["center_z"] = c_[2];

  _j["axis_x"] = n_[0];
  _j["axis_y"] = n_[1];
  _j["axis_z"] = n_[2];

  _j["radius"] = r_;

  _j["height"] = h_;
}

Cylinder* Cylinder::from_json(const json& _j) {
  Cylinder* p = new Cylinder();

  try {
    p->c_[0] = _j.at("center_x").get<double>();
    p->c_[1] = _j.at("center_y").get<double>();
    p->c_[2] = _j.at("center_z").get<double>();
  } catch (json::out_of_range& e) {
    try {
      p->c_[0] = _j.at("location_x").get<double>();
      p->c_[1] = _j.at("location_y").get<double>();
      p->c_[2] = _j.at("location_z").get<double>();
    } catch (json::out_of_range& e) {
      LOG(FATAL) << "[Cylinder] center (or location) is not given.";
    }
  }

  try {
    p->n_[0] = _j.at("axis_x").get<double>();
    p->n_[1] = _j.at("axis_y").get<double>();
    p->n_[2] = _j.at("axis_z").get<double>();
  } catch (json::out_of_range& e) {
    LOG(FATAL) << "[Cylinder] axis is not given.";
  }

  p->r_ = _j.at("radius").get<double>();

  try {
    p->h_ = _j.at("height").get<double>();
  }
  catch (json::out_of_range& e) {
    p->set_default_clipping_params();
  }

  return p;
}

Sphere::Sphere()
  : PrimitiveBase(),
    c_(Vector3d::Zero()),
    r_(1.0) {
    set_default_clipping_params();
  }

Sphere::Sphere(const Vector3d& _c, const double _r)
  : PrimitiveBase(),
    c_(_c),
    r_(_r) {
    CHECK_GT(r_, 0.0);
    set_default_clipping_params();
  }

Sphere::Sphere(const Sphere& _other)
  : PrimitiveBase(_other),
    c_(_other.c_),
    r_(_other.r_) {
  }

double Sphere::area() const {
  return 4.0 * M_PI * r_ * r_;
}

double Sphere::signed_distance(const Vector3d& _p) const {
  const Vector3d v = _p - c_;
  const double d = (v).norm() - r_;
  return d;
}

Vector3d Sphere::project(const Vector3d& _p) const {
  const Vector3d v = _p - c_;
  const double d = (v).norm() - r_;

  if (v.norm() < kZeroTol) {
    LOG(WARNING) << "Point is on the sphere center.";
    return _p;
  }

  const Vector3d proj_p = _p - d * v.normalized();
  return proj_p;
}

void Sphere::set_default_clipping_params() {
}

void Sphere::compute_clipping_params(const MatrixXd& _P) {
  CHECK_GT(_P.rows(), 0);
  CHECK_EQ(_P.cols(), 3);
}

MatrixXd Sphere::randomly_sample_points(const int n_points) const {
  // http://mathworld.wolfram.com/SpherePointPicking.html
  // Muller 1959, Marsaglia 1972

  // Uniform random sampling in [-1.0, +1.0] range.
  const MatrixXd coeffs = MatrixXd::Random(n_points, 3);
  const MatrixXd centered_points = r_ * coeffs.rowwise().normalized();
  const MatrixXd points = centered_points.rowwise() + c_.transpose();
  return points;
}

void Sphere::render() const {
  glPushMatrix();
  glTranslate(c_);
  glutSolidSphere(r_, 32, 32);
  glPopMatrix();
}

void Sphere::render_axes() const {
  // Nothing to render.
}

void Sphere::to_json(json& _j) const {
  _j["type"] = "sphere";
  _j["label"] = label_;

  _j["center_x"] = c_[0];
  _j["center_y"] = c_[1];
  _j["center_z"] = c_[2];

  _j["radius"] = r_;
}

Sphere* Sphere::from_json(const json& _j) {
  Sphere* p = new Sphere();

  try {
    p->c_[0] = _j.at("center_x").get<double>();
    p->c_[1] = _j.at("center_y").get<double>();
    p->c_[2] = _j.at("center_z").get<double>();
  } catch (json::out_of_range& e) {
    try {
      p->c_[0] = _j.at("location_x").get<double>();
      p->c_[1] = _j.at("location_y").get<double>();
      p->c_[2] = _j.at("location_z").get<double>();
    } catch (json::out_of_range& e) {
      LOG(FATAL) << "[Sphere] center (or location) is not given.";
    }
  }

  try {
    p->r_ = _j.at("radius").get<double>();
  } catch (json::out_of_range& e) {
    LOG(FATAL) << "[Sphere] radius is not given.";
  }

  return p;
}

Cone::Cone()
  : PrimitiveBase(),
    c_(Vector3d::Zero()),
    n_(Vector3d::UnitZ()),
    a_(M_PI_4) {
    set_default_clipping_params();
  }

Cone::Cone(const Vector3d& _c, const Vector3d& _n, const double _a,
    const double _z_min, const double _z_max)
  : PrimitiveBase(),
    c_(_c),
    n_(_n),
    a_(_a),
    z_min_(_z_min),
    z_max_(_z_max) {
    CHECK_GT(a_, 0.0);
    CHECK_GT(n_.norm(), kZeroTol);
    n_.normalize();
    //set_default_clipping_params();
  }

Cone::Cone(const Cone& _other)
  : PrimitiveBase(_other),
    c_(_other.c_),
    n_(_other.n_),
    a_(_other.a_),
    z_min_(_other.z_min_),
    z_max_(_other.z_max_) {
  }

double Cone::radius_top() const {
  return std::tan(a_/2.0) * z_min_;
}

double Cone::radius_bottom() const {
  return std::tan(a_/2.0) * z_max_;
}

double Cone::area() const {
  const double l_min = z_min_ / std::cos(a_/2.0);
  const double r_min = radius_top();
  const double l_max = z_max_ / std::cos(a_/2.0);
  const double r_max = radius_bottom();
  return M_PI * ((r_max * l_max) - (r_min * l_min));
}

double Cone::signed_distance(const Vector3d& _p) const {
  const Vector3d v = _p - c_;

  const double cos_angle_to_axis = std::min(std::max(
        v.normalized().dot(n_), -1.0), +1.0);
  const double angle_to_axis = std::acos(cos_angle_to_axis);
  const double angle_to_surface = angle_to_axis - (a_/2.0);
  if (angle_to_surface >= M_PI_4) return v.norm();

  const double d = v.norm() * std::sin(angle_to_surface);
  return d;
}

Vector3d Cone::project(const Vector3d& _p) const {
  const Vector3d v = _p - c_;
  if (v.norm() < kZeroTol) return c_;

  const double cos_half_angle = std::cos(a_/2.0);
  if (std::abs(cos_half_angle) < kZeroTol) {
    LOG(WARNING) << "Cone angle is Pi.";
    return _p - v.dot(n_) * n_;
  }

  const double cos_angle_to_axis = std::min(std::max(
        v.normalized().dot(n_), -1.0), +1.0);
  const double angle_to_axis = std::acos(cos_angle_to_axis);
  const double angle_to_surface = angle_to_axis - (a_/2.0);
  if (angle_to_surface >= M_PI_4) return _p;

  const double d = v.norm() * std::sin(angle_to_surface);

  const double t = v.norm() * std::cos(angle_to_surface) / cos_half_angle;
  const Vector3d w = t * n_;
  const Vector3d x = v - w;

  if (x.norm() < kZeroTol) {
    LOG(WARNING) << "Point is on the cone axis.";
    return _p;
  }

  const Vector3d proj_p = _p - d * x.normalized();

  return proj_p;
}

void Cone::set_default_clipping_params() {
  z_min_ = 0.0;
  z_max_ = 1.0;
}

void Cone::compute_clipping_params(const MatrixXd& _P) {
  CHECK_GT(_P.rows(), 0);
  CHECK_EQ(_P.cols(), 3);

  const MatrixXd centered_P = _P.rowwise() - c_.transpose();
  const VectorXd axis_dists = centered_P * n_;
  const double min_axis_dist = axis_dists.minCoeff();
  const double max_axis_dist = axis_dists.maxCoeff();
  z_min_ = std::max(min_axis_dist, 0.0);
  z_max_ = std::max(max_axis_dist, 0.0);
}

MatrixXd Cone::randomly_sample_points(const int n_points) const {
  const double l_min = z_min_ / std::cos(a_/2.0);
  const double r_min = radius_top();
  const double l_max = z_max_ / std::cos(a_/2.0);
  const double r_max = radius_bottom();

  // [0, 2*pi)
  const VectorXd angles = 2.0 * M_PI *
    (0.5 * VectorXd::Random(n_points).array() + 0.5);
  // [l_min*l_min, l_max*l_max)
  const VectorXd l_squared = (l_max*l_max - l_min*l_min) *
    (0.5 * VectorXd::Random(n_points).array() + 0.5) + l_min*l_min;

  int min_idx;
  n_.minCoeff(&min_idx);
  const Vector3d b = n_.cross(Eigen::Vector3d::Unit(min_idx));

  MatrixXd points = MatrixXd::Zero(n_points, 3);

  for (int i = 0; i < n_points; ++i) {
    const double r = std::sqrt(l_squared[i]) * std::sin(a_/2.0);
    const double h = std::sqrt(l_squared[i]) * std::cos(a_/2.0);
    const AngleAxisd R(angles[i], n_);
    const Vector3d b_i = R.matrix() * b;
    points.row(i) = (c_ + h * n_ + r * b_i).transpose();
  }

  return points;
}

void Cone::render() const {
  glPushMatrix();

  // FIXME:
  // Truncate top.

  // Move to the bottom center.
  const Vector3d bottom_center = c_ + z_max_ * n_;
  glTranslate(bottom_center);

  // NOTE:
  // 'n_' is the axis from the apex to the bottom,
  // while GLUT axis is from the bottom to the apex.
  glRotate(-n_);

  const double r = radius_bottom();
  glutSolidCone(r, z_min_, z_max_, 32, 32);
  glPopMatrix();
}

void Cone::render_axes() const {
  LOG(WARNING) << "Not implemented yet.";
}

void Cone::to_json(json& _j) const {
  _j["type"] = "cone";
  _j["label"] = label_;

  _j["apex_x"] = c_[0];
  _j["apex_y"] = c_[1];
  _j["apex_z"] = c_[2];

  _j["axis_x"] = n_[0];
  _j["axis_y"] = n_[1];
  _j["axis_z"] = n_[2];

  _j["angle"] = a_;

  _j["z_min"] = z_min_;
  _j["z_max"] = z_max_;
}

Cone* Cone::from_json(const json& _j) {
  Cone* p = new Cone();

  try {
    p->c_[0] = _j.at("apex_x").get<double>();
    p->c_[1] = _j.at("apex_y").get<double>();
    p->c_[2] = _j.at("apex_z").get<double>();
  } catch (json::out_of_range& e) {
    LOG(FATAL) << "[Cone] apex is not given.";
  }

  try {
    p->n_[0] = _j.at("axis_x").get<double>();
    p->n_[1] = _j.at("axis_y").get<double>();
    p->n_[2] = _j.at("axis_z").get<double>();
  } catch (json::out_of_range& e) {
    LOG(FATAL) << "[Cone] axis is not given.";
  }

  try {
    p->a_ = _j.at("angle").get<double>();
  } catch (json::out_of_range& e) {
    try {
      p->a_ = _j.at("semi_angle").get<double>() * 2;
    } catch (json::out_of_range& e) {
      LOG(FATAL) << "[Cone] angle (or semi_angle) is not given.";
    }
  }

  try {
    p->z_min_ = _j.at("z_min").get<double>();
    p->z_max_ = _j.at("z_max").get<double>();
  }
  catch (json::out_of_range& e) {
    p->set_default_clipping_params();
  }

  return p;
}

Torus::Torus()
  : PrimitiveBase(),
    c_(Vector3d::Zero()),
    n_(Vector3d::UnitZ()),
    r1_(0.5),
    r2_(1.0) {
    set_default_clipping_params();
  }

Torus::Torus(const Vector3d& _c, const Vector3d& _n,
    const double _r1, const double _r2)
  : PrimitiveBase(),
    c_(_c),
    n_(_n),
    r1_(_r1),
    r2_(_r2) {
    CHECK_GT(r1_, 0.0);
    CHECK_GT(r2_, 0.0);
    set_default_clipping_params();
  }

Torus::Torus(const Torus& _other)
  : PrimitiveBase(_other),
    c_(_other.c_),
    n_(_other.n_),
    r1_(_other.r1_),
    r2_(_other.r2_) {
  }

double Torus::area() const {
  return 4.0 * M_PI * r1_ * r2_;
}

double Torus::signed_distance(const Vector3d& _p) const {
  LOG(FATAL) << "Not implemented yet.";
  return 0.0;
}

Vector3d Torus::project(const Vector3d& _p) const {
  return _p;
}

void Torus::set_default_clipping_params() {
}

void Torus::compute_clipping_params(const MatrixXd& _P) {
  CHECK_GT(_P.rows(), 0);
  CHECK_EQ(_P.cols(), 3);
}

MatrixXd Torus::randomly_sample_points(const int n_points) const {
  LOG(FATAL) << "Not implemented yet.";
  const MatrixXd points;
  return points;
}

void Torus::render() const {
  //CHECK_LT(r1_, r2_);
  glPushMatrix();
  glTranslate(c_);
  glRotate(n_);
  glutSolidTorus(r1_, r2_, 32, 32);
  glPopMatrix();
}

void Torus::render_axes() const {
  // Nothing to render.
}

void Torus::to_json(json& _j) const {
  _j["type"] = "torus";
  _j["label"] = label_;

  _j["center_x"] = c_[0];
  _j["center_y"] = c_[1];
  _j["center_z"] = c_[2];

  _j["axis_x"] = n_[0];
  _j["axis_y"] = n_[1];
  _j["axis_z"] = n_[2];

  _j["minor_radius"] = r1_;
  _j["major_radius"] = r2_;
}

Torus* Torus::from_json(const json& _j) {
  Torus* p = new Torus();

  p->c_[0] = _j.at("center_x").get<double>();
  p->c_[1] = _j.at("center_y").get<double>();
  p->c_[2] = _j.at("center_z").get<double>();

  p->n_[0] = _j.at("axis_x").get<double>();
  p->n_[1] = _j.at("axis_y").get<double>();
  p->n_[2] = _j.at("axis_z").get<double>();

  p->r1_ = _j.at("minor_radius").get<double>();
  p->r2_ = _j.at("major_radius").get<double>();

  return p;
}

Prism::Prism()
  : PrimitiveBase(),
    c_(Vector3d::Zero()),
    n_(Vector3d::UnitZ()),
    r_(1.0) {
    set_default_clipping_params();
  }

Prism::Prism(const Vector3d& _c, const Vector3d& _n,
    const Vector3d& _b, const double _r, const int _N)
  : PrimitiveBase(),
    c_(_c),
    n_(_n),
    b_(_b),
    r_(_r),
    N_(_N) {
    CHECK_GT(r_, 0.0);
    CHECK_GT(n_.norm(), kZeroTol);
    n_.normalize();
    set_default_clipping_params();
  }

Prism::Prism(const Prism& _other)
  : PrimitiveBase(_other),
    c_(_other.c_),
    n_(_other.n_),
    b_(_other.b_),
    r_(_other.r_),
    N_(_other.N_),
    h_(_other.h_) {
  }

double Prism::area() const {
  const double angle = 2.0 * M_PI / N_;
  const double side_length = 2.0 * r_ * std::sin(angle / 2.0);

  const double area_top = 0.5 * r_ * r_ * std::sin(angle);
  const double area_bottom = 0.5 * r_ * r_ * std::sin(angle);
  const double area_side = N_ * side_length * h_;

  return N_ * (area_top + area_bottom + area_side);
}

double Prism::signed_distance(const Vector3d& _p) const {
  LOG(FATAL) << "Not implemented yet.";
  return 0.0;
}

Vector3d Prism::project(const Vector3d& _p) const {
  LOG(FATAL) << "Not implemented yet.";
  return _p;
}

void Prism::set_default_clipping_params() {
  h_ = 1.0;
}

void Prism::compute_clipping_params(const MatrixXd& _P) {
  CHECK_GT(_P.rows(), 0);
  CHECK_EQ(_P.cols(), 3);

  const MatrixXd centered_P = _P.rowwise() - c_.transpose();
  const VectorXd axis_dists = centered_P * n_;
  const double min_axis_dists = axis_dists.minCoeff();
  const double max_axis_dists = axis_dists.maxCoeff();
  c_ += 0.5 * (min_axis_dists + max_axis_dists) * n_;
  h_ = max_axis_dists - min_axis_dists;
}

MatrixXd Prism::randomly_sample_points(const int n_points) const {
  const double angle = 2.0 * M_PI / N_;
  const double side_length = 2.0 * r_ * std::sin(angle / 2.0);

  const double area_top = 0.5 * r_ * r_ * std::sin(angle);
  const double area_bottom = 0.5 * r_ * r_ * std::sin(angle);
  const double area_side = N_ * side_length * h_;

  // [0, N)
  const VectorXd side_idxs = N_ *
    (0.5 * VectorXd::Random(n_points).array() + 0.5);
  // [0, total_area)
  const VectorXd tbs_idxs = (area_top + area_bottom + area_side) *
    (0.5 * VectorXd::Random(n_points).array() + 0.5);
  // [0.0, 1.0)
  const VectorXd x_coeffs = 0.5 * VectorXd::Random(n_points).array() + 0.5;
  // [0.0, 1.0)
  const VectorXd y_coeffs = 0.5 * VectorXd::Random(n_points).array() + 0.5;

  MatrixXd points = MatrixXd::Zero(n_points, 3);

  for (int i = 0; i < n_points; ++i) {
    const int k = std::floor(side_idxs[i]);
    const AngleAxisd R_k(k * angle, n_);
    const Vector3d b_k = R_k.matrix() * b_;
    const AngleAxisd R_kn((k + 1) * angle, n_);
    const Vector3d b_kn = R_kn.matrix() * b_;

    double x, y;
    if (tbs_idxs[i] < area_top) {
      if (x_coeffs[i] + y_coeffs[i] <= 1.0) {
        x = r_ * x_coeffs[i];
        y = r_ * y_coeffs[i];
      } else {
        x = r_ * (1.0 - x_coeffs[i]);
        y = r_ * (1.0 - y_coeffs[i]);
      }
      const Vector3d c = c_ + 0.5 * h_ * n_;
      const Vector3d x_axis = b_k.normalized();
      const Vector3d y_axis = b_kn.normalized();
      points.row(i) = (c + x * x_axis + y * y_axis).transpose();

    } else if (tbs_idxs[i] < area_top + area_bottom) {
      if (x_coeffs[i] + y_coeffs[i] <= 1.0) {
        x = r_ * x_coeffs[i];
        y = r_ * y_coeffs[i];
      } else {
        x = r_ * (1.0 - x_coeffs[i]);
        y = r_ * (1.0 - y_coeffs[i]);
      }
      const Vector3d c = c_ - 0.5 * h_ * n_;
      const Vector3d x_axis = b_k.normalized();
      const Vector3d y_axis = b_kn.normalized();
      points.row(i) = (c + x * x_axis + y * y_axis).transpose();

    } else {
      x = r_ * (b_kn - b_k).norm() * x_coeffs[i];
      y = h_ * y_coeffs[i];
      const Vector3d c = c_ - 0.5 * h_ * n_ + r_ * b_k;
      const Vector3d x_axis = (b_kn - b_k).normalized();
      const Vector3d y_axis = n_.normalized();
      points.row(i) = (c + x * x_axis + y * y_axis).transpose();
    }
  }

  return points;
}

void Prism::render() const {
  const double angle = 2.0 * M_PI / N_;

  const Vector3d top_center = c_ + 0.5 * h_ * n_;
  glBegin(GL_POLYGON);
  for (int k = 0; k < N_; ++k) {
    const AngleAxisd R_k(k * angle, n_);
    const Vector3d b_k = R_k.matrix() * b_;
    glVertex3dv(top_center + r_ * b_k);
  }
  glEnd();

  const Vector3d bottom_center = c_ - 0.5 * h_ * n_;
  glBegin(GL_POLYGON);
  for (int k = N_ - 1; k >= 0; --k) {
    const AngleAxisd R_k(k * angle, n_);
    const Vector3d b_k = R_k.matrix() * b_;
    glVertex3dv(bottom_center + r_ * b_k);
  }
  glEnd();

  glBegin(GL_QUADS);
  for (int k = 0; k < N_; ++k) {
    const AngleAxisd R_k(k * angle, n_);
    const Vector3d b_k = R_k.matrix() * b_;
    const AngleAxisd R_kn((k + 1) * angle, n_);
    const Vector3d b_kn = R_kn.matrix() * b_;
    glVertex3dv(bottom_center + r_ * b_k);
    glVertex3dv(bottom_center + r_ * b_kn);
    glVertex3dv(top_center + r_ * b_kn);
    glVertex3dv(top_center + r_ * b_k);
  }
  glEnd();
}

void Prism::render_axes() const {
  LOG(WARNING) << "Not implemented yet.";
}

void Prism::to_json(json& _j) const {
  _j["type"] = "cylinder";
  _j["label"] = label_;

  _j["center_x"] = c_[0];
  _j["center_y"] = c_[1];
  _j["center_z"] = c_[2];

  _j["axis_x"] = n_[0];
  _j["axis_y"] = n_[1];
  _j["axis_z"] = n_[2];

  _j["base_x"] = b_[0];
  _j["base_y"] = b_[1];
  _j["base_z"] = b_[2];

  _j["N"] = N_;

  _j["radius"] = r_;

  _j["height"] = h_;
}

Prism* Prism::from_json(const json& _j) {
  Prism* p = new Prism();

  p->c_[0] = _j.at("center_x").get<double>();
  p->c_[1] = _j.at("center_y").get<double>();
  p->c_[2] = _j.at("center_z").get<double>();

  p->n_[0] = _j.at("axis_x").get<double>();
  p->n_[1] = _j.at("axis_y").get<double>();
  p->n_[2] = _j.at("axis_z").get<double>();

  p->b_[0] = _j.at("base_x").get<double>();
  p->b_[1] = _j.at("base_y").get<double>();
  p->b_[2] = _j.at("base_z").get<double>();

  p->N_ = _j.at("N").get<double>();

  p->r_ = _j.at("radius").get<double>();

  try {
    p->h_ = _j.at("height").get<double>();
  }
  catch (json::out_of_range& e) {
    p->set_default_clipping_params();
  }

  return p;
}

Box::Box()
  : PrimitiveBase(),
    c_(Vector3d::Zero()),
    x_(Vector3d::UnitX()),
    y_(Vector3d::UnitY()),
    z_(Vector3d::UnitZ()),
    sx_(1.0),
    sy_(1.0),
    sz_(1.0) {
    for (int i = 0; i < 3; ++i) {
      draw_axis_pos_[i] = false;
      draw_axis_neg_[i] = false;
      axis_pos_color_[i] = Vector4f::Unit(i);
      axis_neg_color_[i] = Vector4f::Unit(i);
    }
  }

Box::Box(const Vector3d& _c, const Vector3d& _x, const Vector3d& _y,
        const double _sx, const double _sy, const double _sz)
  : PrimitiveBase(),
    c_(_c),
    x_(_x),
    y_(_y),
    sx_(_sx),
    sy_(_sy),
    sz_(_sz) {
    CHECK_GT(sx_, 0.0);
    CHECK_GT(sy_, 0.0);
    CHECK_GT(sz_, 0.0);
    CHECK_GT(x_.norm(), kZeroTol);
    CHECK_GT(y_.norm(), kZeroTol);
    CHECK_LT(std::abs(x_.dot(y_)), kZeroTol);
    x_.normalize();
    y_.normalize();
    z_ = x_.cross(y_);
    for (int i = 0; i < 3; ++i) {
      draw_axis_pos_[i] = false;
      draw_axis_neg_[i] = false;
      axis_pos_color_[i] = Vector4f::Unit(i);
      axis_neg_color_[i] = Vector4f::Unit(i);
    }
  }

Box::Box(const Box& _other)
  : PrimitiveBase(_other),
    c_(_other.c_),
    x_(_other.x_),
    y_(_other.y_),
    z_(_other.z_),
    sx_(_other.sx_),
    sy_(_other.sy_),
    sz_(_other.sz_) {
    for (int i = 0; i < 3; ++i) {
      draw_axis_pos_[i] = _other.draw_axis_pos_[i];
      draw_axis_neg_[i] = _other.draw_axis_neg_[i];
      axis_pos_color_[i] = _other.axis_pos_color_[i];
      axis_neg_color_[i] = _other.axis_neg_color_[i];
    }
  }

double Box::area() const {
  return 2 * (sx_ * sy_ + sy_ * sz_ + sz_ * sx_);
}

double Box::signed_distance(const Vector3d& _p) const {
  LOG(FATAL) << "Not implemented yet.";
  return 0.0;
}

Vector3d Box::project(const Vector3d& _p) const {
  LOG(FATAL) << "Not implemented yet.";
  return _p;
}

void Box::set_default_clipping_params() {
}

void Box::compute_clipping_params(const MatrixXd& _P) {
  CHECK_GT(_P.rows(), 0);
  CHECK_EQ(_P.cols(), 3);
  LOG(FATAL) << "Not implemented yet.";
}

MatrixXd Box::randomly_sample_points(const int n_points) const {
  MatrixXd points = MatrixXd::Zero(n_points, 3);

  const VectorXd u = 0.5 * VectorXd::Random(n_points);
  const VectorXd v = 0.5 * VectorXd::Random(n_points);

  const int n_faces = 6;

  for (int i = 0; i < n_points; ++i) {
    const int face_idx = i % n_faces;
    switch(face_idx) {
      case 0:
        points.row(i) = c_ + (u[i] * sx_ * x_) + (v[i] * sy_ * y_) +
          (0.5 * sz_ * z_);
        break;
      case 1:
        points.row(i) = c_ + (u[i] * sy_ * y_) + (v[i] * sz_ * z_) +
          (0.5 * sx_ * x_);
        break;
      case 2:
        points.row(i) = c_ + (u[i] * sz_ * z_) + (v[i] * sx_ * x_) +
          (0.5 * sy_ * y_);
        break;
      case 3:
        points.row(i) = c_ + (u[i] * sx_ * x_) + (v[i] * sy_ * y_) +
          (-0.5 * sz_ * z_);
        break;
      case 4:
        points.row(i) = c_ + (u[i] * sy_ * y_) + (v[i] * sz_ * z_) +
          (-0.5 * sx_ * x_);
        break;
      case 5:
        points.row(i) = c_ + (u[i] * sz_ * z_) + (v[i] * sx_ * x_) +
          (-0.5 * sy_ * y_);
        break;
    }
  }

  return points;
}

void Box::render() const {
  glPushMatrix();

  VectorXd m = VectorXd::Zero(16);
  for (int i = 0; i < 3; ++i) {
    m[4*0+i] = x_[i];
    m[4*1+i] = y_[i];
    m[4*2+i] = z_[i];
    m[4*3+i] = c_[i];
  }
  m[15] = 1.0;

  glMultMatrixd(&m[0]);
  glScaled(sx_, sy_, sz_);
  glutSolidCube(1.0);
  glPopMatrix();
}

void draw_box_axis(const Vector3d _c, const Vector3d _a, const double _s,
    const Vector4f _color) {
  // FIXME.
  const float arrow_length = 0.15;
  const float arrow_radius = 0.1 * arrow_length;
  const float cone_angle = M_PI / 3.0;
  const float cone_length = 0.5 * arrow_length;

  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, _color.data());
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, _color.data());
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, _color.data());

  const Vector3d cylinder_center = _c + (0.5 * _s * _a) +
    (0.5 * arrow_length * _a);
  Cylinder cylinder(cylinder_center, _a, arrow_radius, arrow_length);
  cylinder.render();

  const Vector3d cone_center = _c + (0.5 * _s * _a) +
    (arrow_length * _a) + (cone_length * _a);
  Cone cone(cone_center, -_a, cone_angle, 0.0, cone_length);
  cone.render();
}

void Box::render_axes() const {
  for (int i = 0; i < 3; ++i) {
    if (draw_axis_pos_[i]) {
      if (i == 0) draw_box_axis(c_, x_, sx_, axis_pos_color_[i]);
      if (i == 1) draw_box_axis(c_, y_, sy_, axis_pos_color_[i]);
      if (i == 2) draw_box_axis(c_, z_, sz_, axis_pos_color_[i]);
    }

    if (draw_axis_neg_[i]) {
      if (i == 0) draw_box_axis(c_, -x_, sx_, axis_neg_color_[i]);
      if (i == 1) draw_box_axis(c_, -y_, sy_, axis_neg_color_[i]);
      if (i == 2) draw_box_axis(c_, -z_, sz_, axis_neg_color_[i]);
    }
  }
}

void Box::to_json(json& _j) const {
  _j["type"] = "box";
  _j["label"] = label_;

  _j["center_x"] = c_[0];
  _j["center_y"] = c_[1];
  _j["center_z"] = c_[2];

  _j["x_axis_x"] = x_[0];
  _j["x_axis_y"] = x_[1];
  _j["x_axis_z"] = x_[2];

  _j["y_axis_x"] = y_[0];
  _j["y_axis_y"] = y_[1];
  _j["y_axis_z"] = y_[2];

  _j["z_axis_x"] = z_[0];
  _j["z_axis_y"] = z_[1];
  _j["z_axis_z"] = z_[2];

  _j["x_size"] = sx_;
  _j["y_size"] = sy_;
  _j["z_size"] = sz_;
}

Box* Box::from_json(const json& _j) {
  Box* p = new Box();

  p->c_[0] = _j.at("center_x").get<double>();
  p->c_[1] = _j.at("center_y").get<double>();
  p->c_[2] = _j.at("center_z").get<double>();

  p->x_[0] = _j.at("x_axis_x").get<double>();
  p->x_[1] = _j.at("x_axis_y").get<double>();
  p->x_[2] = _j.at("x_axis_z").get<double>();
  p->x_.normalize();

  p->y_[0] = _j.at("y_axis_x").get<double>();
  p->y_[1] = _j.at("y_axis_y").get<double>();
  p->y_[2] = _j.at("y_axis_z").get<double>();
  p->y_.normalize();

  try {
    p->z_[0] = _j.at("z_axis_x").get<double>();
    p->z_[1] = _j.at("z_axis_y").get<double>();
    p->z_[2] = _j.at("z_axis_z").get<double>();
  } catch (json::out_of_range& e) {
    p->z_ = p->x_.cross(p->y_);
  }
  p->z_.normalize();

  p->sx_ = _j.at("x_size").get<double>();
  p->sy_ = _j.at("y_size").get<double>();
  p->sz_ = _j.at("z_size").get<double>();

  return p;
}

