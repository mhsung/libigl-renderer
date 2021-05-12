// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "LibiglMeshRendererT.h"

#include <iostream>
#include <Eigen/Geometry>
#include <igl/frustum.h>
#include <utils/google_tools.h>
#include <utils/utils.h>


DEFINE_double(fovy_deg, 45.0, "fovy angle degree.");
DEFINE_double(camera_distance, 3.0, "camera distance.");
DEFINE_double(point_radius, 0.01, "point radius.");
DEFINE_bool(auto_adjust_camera, true, "auto-adjust camera.");


LibiglMeshRendererT::LibiglMeshRendererT(const int _width, const int _height)
  : width_(_width),
    height_(_height),
    center_(Vector3f::Zero()),
    radius_(1.0f),
    point_radius_(FLAGS_point_radius) {
}

bool LibiglMeshRendererT::read_projection(const std::string& _filename) {
  Matrix4f projection;
  if (!Utils::read_eigen_matrix_from_file(_filename, &projection)) {
    return false;
  }
  set_projection(projection);
  return true;
}

bool LibiglMeshRendererT::read_modelview(const std::string& _filename) {
  Matrix4f modelview;
  if (!Utils::read_eigen_matrix_from_file(_filename, &modelview)) {
    return false;
  }
  set_modelview(modelview);
  return true;
}

bool LibiglMeshRendererT::write_projection(const std::string& _filename) const {
  if (!Utils::write_eigen_matrix_to_file(_filename, get_projection())) {
    return false;
  }
  return true;
}

bool LibiglMeshRendererT::write_modelview(const std::string& _filename) const {
  if (!Utils::write_eigen_matrix_to_file(_filename, get_modelview())) {
    return false;
  }
  return true;
}

void LibiglMeshRendererT::reset_projection() {
  const float aspect_ratio = (float)width_ / (float)height_;
  const float znear = 0.01f * radius_;
  const float zfar = 100.0f * radius_;
  const float ymax = znear * tanf(FLAGS_fovy_deg * M_PI / 360.0);
  const float xmax = ymax * aspect_ratio;
  Matrix4f projection;
  igl::frustum(-xmax, +xmax, -ymax, +ymax, znear, zfar, projection);
  set_projection(projection);
}

void LibiglMeshRendererT::reset_modelview() {
  Matrix4f modelview = Matrix4f::Identity();
  modelview(0, 3) = -center_[0];
  modelview(1, 3) = -center_[1];
  //modelview(2, 3) = -center_[2] - FLAGS_camera_distance * radius_;
  modelview(2, 3) = -center_[2] - FLAGS_camera_distance;
  set_modelview(modelview);
}

void LibiglMeshRendererT::set_window_size(
    const int _width, const int _height) {
	width_ = _width;
	height_ = _height;
	reset_projection();
}

void LibiglMeshRendererT::set_scene_pos(
    const Vector3f& _center, const float _radius) {
	center_ = _center;
	radius_ = _radius;
	reset_projection();
	reset_modelview();
}

Vector3f LibiglMeshRendererT::get_camera_params() const {
  const ::Matrix4f modelview = get_modelview();

  // Default rotation R_d.
  // R = R_z R_x R_y R_d.
  const AngleAxisf default_rotation(
      -0.5 * M_PI, Vector3f::UnitY());

  // R R_d^T = R_z R_x R_y.
  const Matrix3f rotation =
      modelview.block(0, 0, 3, 3) *
      default_rotation.toRotationMatrix().transpose();

  // ZXY Euler angles.
  Vector3f angles = rotation.eulerAngles(2, 0, 1);

  Vector3f camera_params;
  // Y-axis (Azimuth, [R_y | 0])
  // Note: Change the sign.
  camera_params[0] = (-angles[2] / M_PI * 180.0) + 90;

  // X-axis (Elevation, [R_x | 0])
  camera_params[1] = +angles[1] / M_PI * 180.0;

  // Z-axis (Theta, [R_z | 0])
  // Note: Change the sign.
  camera_params[2] = -angles[0] / M_PI * 180.0;


  // Elevation must be in range [-90, 90].
  while(camera_params[1] < -90.0) camera_params[1] += 360.0;

  if (camera_params[1] > 270.0) {
    camera_params[1] = 360.0 - camera_params[1];
  } else if (camera_params[1] > 90.0) {
    camera_params[1] = 180.0 - camera_params[1];
    camera_params[0] += 180.0;
    camera_params[2] += 180.0;
  }

  // Azimuth and theta must be in range [0, 360).
  while(camera_params[0] >= 360.0) camera_params[0] -= 360.0;
  while(camera_params[0] < 0.0) camera_params[0] += 360.0;

  while(camera_params[2] >= 360.0) camera_params[2] -= 360.0;
  while(camera_params[2] < 0.0) camera_params[2] += 360.0;

  return camera_params;
}

void LibiglMeshRendererT::set_camera_params(const Vector3f& _camera_params,
    const Vector3f& _center, const float _radius) {
  const float azimuth_deg = _camera_params[0] - 90;
  const float elevation_deg = _camera_params[1];
  const float theta_deg = _camera_params[2];

  Affine3f modelview(Affine3f::Identity());

  if (FLAGS_auto_adjust_camera) {
    modelview.pretranslate(-_center);
  }

  // Default transformation.
  modelview.prerotate(AngleAxisf(-0.5 * M_PI, Vector3f::UnitY()));

  // Y-axis (Azimuth)
  // Note: Change the sign.
  const double azimuth = -azimuth_deg / 180.0 * M_PI;
  modelview.prerotate(AngleAxisf(azimuth, Vector3f::UnitY()));

  // X-axis (Elevation)
  const double elevation = elevation_deg / 180.0 * M_PI;
  modelview.prerotate(AngleAxisf(elevation, Vector3f::UnitX()));

  // Translation
  if (FLAGS_auto_adjust_camera) {
    modelview.pretranslate(Vector3f(0, 0, -FLAGS_camera_distance * _radius));
  } else {
    modelview.pretranslate(Vector3f(0, 0, -FLAGS_camera_distance));
  }

  // Z-axis (Theta)
  // Note: Change the sign.
  const double theta = -theta_deg / 180.0 * M_PI;
  modelview.prerotate(AngleAxisf(theta, Vector3f::UnitZ()));

  set_modelview(modelview.matrix());
}


//=============================================================================

