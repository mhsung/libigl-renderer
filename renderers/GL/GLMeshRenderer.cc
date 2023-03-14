// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "GLMeshRenderer.h"

#include <iostream>
#include <igl/material_colors.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <utils/google_tools.h>
#include <utils/utils.h>
#include <utils/glut_geometry/glut_geometry.h>


GLMeshRenderer::GLMeshRenderer(const int _width, const int _height)
  : LibiglMeshRendererT(_width, _height),
    projection_(Matrix4f::Identity()),
    modelview_(Matrix4f::Identity()),
    draw_mode_names_({"Wireframe", "Solid Smooth"}),
    draw_mode_idx_(draw_mode_names_.size()) {
}

GLMeshRenderer::~GLMeshRenderer() {
}

const Matrix4f& GLMeshRenderer::get_projection() const {
  return projection_;
}

const Matrix4f& GLMeshRenderer::get_modelview() const {
  return modelview_;
}

void GLMeshRenderer::set_projection(const Matrix4f& _projection) {
  projection_ = _projection;
}

void GLMeshRenderer::set_modelview(const Matrix4f& _modelview) {
  modelview_ = _modelview;
}

void GLMeshRenderer::set_mesh(
    const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F) {
  CHECK_EQ(_V.cols(), 3);
  V_.resize(_V.rows(), 3);
  for (int i = 0 ; i < _V.rows(); ++i) V_.row(i) = _V.row(i);
  F_.resize(_F.rows(), 3);
  for (int i = 0 ; i < _F.rows(); ++i) F_.row(i) = _F.row(i);

  if (F_.rows() > 0) igl::per_face_normals(V_, F_, FN_);
  if (F_.rows() > 0 && V_.rows() > 0) igl::per_vertex_normals(V_, F_, FN_, VN_);
}

void GLMeshRenderer::set_vertex_colors(const Eigen::MatrixXf& _VC) {
  CHECK_EQ(_VC.rows(), V_.rows());
  CHECK_EQ(_VC.cols(), 3);
  VC_.resize(_VC.rows(), 3);
  for (int i = 0 ; i < _VC.rows(); ++i) VC_.row(i) = _VC.row(i);
}

void GLMeshRenderer::set_face_colors(const Eigen::MatrixXf& _FC) {
  CHECK_EQ(_FC.rows(), F_.rows());
  CHECK_EQ(_FC.cols(), 3);
  FC_.resize(_FC.rows(), 3);
  for (int i = 0 ; i < _FC.rows(); ++i) FC_.row(i) = _FC.row(i);
}

void GLMeshRenderer::set_vertex_normals(const Eigen::MatrixXd& _VN) {
  // TO BE IMPLEMENTED.
}

void GLMeshRenderer::clear_mesh() {
  V_.resize(0, 3);
  F_.resize(0, 3);
  VN_.resize(0, 3);
  FN_.resize(0, 3);
  FC_.resize(0, 3);
}

void GLMeshRenderer::set_point_cloud(const Eigen::MatrixXd& _P) {
  CHECK_EQ(_P.cols(), 3);
  P_.resize(_P.rows(), 3);
  for (int i = 0 ; i < _P.rows(); ++i) P_.row(i) = _P.row(i);
}

void GLMeshRenderer::set_point_colors(const Eigen::MatrixXf& _PC) {
  CHECK_EQ(_PC.rows(), P_.rows());
  CHECK_EQ(_PC.cols(), 3);
  PC_.resize(_PC.rows(), 3);
  for (int i = 0 ; i < _PC.rows(); ++i) PC_.row(i) = _PC.row(i);
}

void GLMeshRenderer::set_point_normals(const Eigen::MatrixXd& _PN) {
  LOG(ERROR) << "Not implemented yet.";
}

void GLMeshRenderer::clear_point_cloud() {
  P_.resize(0, 3);
  PC_.resize(0, 3);
}

void GLMeshRenderer::initialize_opengl() {
  // OpenGL state
  glClearColor(1.0, 1.0, 1.0, 0.0);
  glDisable(GL_DITHER);
  glEnable(GL_DEPTH_TEST);

  // Lighting
  set_default_light();

  // Fog
  GLfloat fogColor[4] = { 0.3, 0.3, 0.4, 1.0 };
  glFogi(GL_FOG_MODE, GL_LINEAR);
  glFogfv(GL_FOG_COLOR, fogColor);
  glFogf(GL_FOG_DENSITY, 0.35);
  glHint(GL_FOG_HINT, GL_DONT_CARE);
  glFogf(GL_FOG_START, 5.0f);
  glFogf(GL_FOG_END, 25.0f);

  // Scene pos and size
  set_scene_pos(Vector3f::Zero(), 1.0f);
}

void GLMeshRenderer::set_default_material() {
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, igl::SILVER_AMBIENT);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, igl::SILVER_DIFFUSE);
  //glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, igl::SILVER_SPECULAR);
}

void GLMeshRenderer::set_default_light() {
  glLoadIdentity();

  // Front light
  GLfloat pos1[] = { 1, 1, -0.2, 0.0 };
  GLfloat pos2[] = { -1, 1, -0.2, 0.0 };
  GLfloat pos3[] = { 0, 0, 1, 0.0 };
  /*
  GLfloat col1[] = { 0.7, 0.7, 0.8, 1.0 };
  GLfloat col2[] = { 0.8, 0.7, 0.7, 1.0 };
  GLfloat col3[] = { 1.0, 1.0, 1.0, 1.0 };
  */
  /*
  GLfloat col1[] = { 0.21, 0.21, 0.24, 1.0 };
  GLfloat col2[] = { 0.24, 0.21, 0.21, 1.0 };
  GLfloat col3[] = { 0.3, 0.3, 0.3, 1.0 };
  */
  GLfloat col1[] = { 0.49, 0.49, 0.56, 1.0 };
  GLfloat col2[] = { 0.56, 0.49, 0.49, 1.0 };
  GLfloat col3[] = { 0.70, 0.70, 0.70, 1.0 };

  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_POSITION, pos1);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, col1);
  glLightfv(GL_LIGHT0, GL_SPECULAR, col1);

  glEnable(GL_LIGHT1);
  glLightfv(GL_LIGHT1, GL_POSITION, pos2);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, col2);
  glLightfv(GL_LIGHT1, GL_SPECULAR, col2);

  glEnable(GL_LIGHT2);
  glLightfv(GL_LIGHT2, GL_POSITION, pos3);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, col3);
  glLightfv(GL_LIGHT2, GL_SPECULAR, col3);
}

void GLMeshRenderer::render() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(projection_.data());
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(modelview_.data());

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glDisable(GL_LIGHTING);
  glLineWidth(8);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glShadeModel(GL_SMOOTH);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  render_point_cloud();
  render_mesh();

  glDisable(GL_BLEND);
  set_default_material();

  glFlush();
}

void GLMeshRenderer::render_mesh() {
  const bool has_vertex_color = (VC_.rows() == V_.rows());
  const bool has_face_color = (!has_vertex_color) && (FC_.rows() == F_.rows());

  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_DOUBLE, 0, V_.data());

  glEnableClientState(GL_NORMAL_ARRAY);
  glNormalPointer(GL_DOUBLE, 0, VN_.data());

  if (has_vertex_color) {
      glEnable(GL_COLOR_MATERIAL);
      glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
      glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
      //glColorMaterial(GL_FRONT_AND_BACK, GL_SPECULAR);
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(3, GL_FLOAT, 0, VC_.data());
  } 

  glBegin(GL_TRIANGLES);
  for (int fid = 0; fid < F_.rows(); ++fid) {
    if (has_vertex_color) {
      // Do nothing.
    } else if (has_face_color) {
      const Vector4f color(FC_(fid, 0), FC_(fid, 1), FC_(fid, 2), 1.0f);
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color.data());
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color.data());
      //glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, color.data());
    } else {
      set_default_material();
    }

    for (int i = 0; i < 3; ++i) {
      const int vid = F_(fid, i);
      glArrayElement(vid);
    }
  }
  glEnd();

  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_NORMAL_ARRAY);
  if (has_vertex_color) {
      glDisableClientState(GL_COLOR_ARRAY);
  }
}

/*
void GLMeshRenderer::render_point_cloud() {
  const bool has_point_color = (PC_.rows() == P_.rows());

  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_DOUBLE, 0, P_.data());

  glPointSize(4.0f);
  glBegin(GL_POINTS);
  for (int pid = 0; pid < P_.rows(); ++pid) {
    if (has_point_color) {
      const Vector3f color = PC_.row(pid);
      glColor3fv(color.data());
    } else {
      glColor4fv(igl::MAYA_GREY.data());
    }
    glArrayElement(pid);
  }
  glEnd();

  glDisableClientState(GL_VERTEX_ARRAY);
}
*/

void GLMeshRenderer::render_point_cloud() {
  const bool has_point_color = (PC_.rows() == P_.rows());

  for (int pid = 0; pid < P_.rows(); ++pid) {
    if (has_point_color) {
      const Vector4f color(PC_(pid, 0), PC_(pid, 1), PC_(pid, 2), 1.0f);
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color.data());
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color.data());
      //glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, color.data());
    } else {
      set_default_material();
    }

    glPushMatrix();
    glTranslated(P_(pid, 0), P_(pid, 1), P_(pid, 2));
    glutSolidSphere(point_radius_ * radius_, 16, 16);
    glPopMatrix();

    glArrayElement(pid);
  }
}

void GLMeshRenderer::run_loop() {
}

