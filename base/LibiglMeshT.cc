// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "LibiglMeshT.h"

#include <sstream>
#include <Eigen/Geometry>
#include <igl/embree/reorient_facets_raycast.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/read_triangle_mesh.h>
#include <igl/readPLY.h>
#include <igl/write_triangle_mesh.h>
#include <igl/writePLY.h>
#include <modules/merge_meshes.h>
#include <utils/filesystem/path.h>
#include <utils/google_tools.h>
#include <utils/utils.h>
#include <utils/SparseICP/ICP.h>


// Define input variables.
DEFINE_string(mesh, "", "mesh file.");
DEFINE_string(face_labels, "", "face label file.");
DEFINE_string(point_cloud, "", "point cloud file.");
DEFINE_string(point_labels, "", "point label file.");
DEFINE_string(point_values, "", "point value file.");
DEFINE_string(meshes, "", "a list of mesh files (separated by comma).");
DEFINE_string(mesh_labels, "", "a list of mesh labels (separated by comma).");
DEFINE_double(azimuth_deg, 0.0, "azimuth (degree). "
    "ignored if 'modelview_matrix' is set");
DEFINE_double(elevation_deg, 0.0, "elevation (degree). "
    "ignored if 'modelview_matrix' is set");
DEFINE_double(theta_deg, 0.0, "theta (degree). "
    "ignored if 'modelview_matrix' is set");
DEFINE_string(projection_matrix, "", "projection matrix file.");
DEFINE_string(modelview_matrix, "", "modelview matrix file.");
DEFINE_string(bbox, "", "bounding box file.");
DEFINE_string(snapshot, "", "snapshot file.");
DEFINE_string(out_mesh, "", "output mesh file.");
DEFINE_string(out_face_labels, "", "output face label file.");
DEFINE_string(out_point_cloud, "", "output point cloud file.");
DEFINE_string(out_point_labels, "", "output point label file.");
DEFINE_string(out_projection_matrix, "", "output projection matrix file.");
DEFINE_string(out_modelview_matrix, "", "output modelview matrix file.");
DEFINE_bool(reorient_faces, false, "reorient mesh faces.");


LibiglMeshT::LibiglMeshT()
  : renderer_(nullptr),
    mesh_name_(""),
    V_(MatrixXd(0, 3)),
    F_(MatrixXi(0, 3)),
    VC_(MatrixXf(0, 3)),
    FC_(MatrixXf(0, 3)),
    VL_(VectorXi(0, 3)),
    FL_(VectorXi(0, 3)),
    VN_(MatrixXd(0, 3)),
    FN_(MatrixXd(0, 3)),
    P_(MatrixXd(0, 3)),
    PC_(MatrixXf(0, 3)),
    PL_(VectorXi(0, 3)),
    PN_(MatrixXd(0, 3)),
    bb_min_(Vector3d::Zero()),
    bb_max_(Vector3d::Zero()),
    center_(Vector3d::Zero()),
    radius_(1.0) {
}

LibiglMeshT::LibiglMeshT(LibiglMeshRendererT* _renderer)
  : renderer_(_renderer),
    mesh_name_(""),
    V_(MatrixXd(0, 3)),
    F_(MatrixXi(0, 3)),
    VC_(MatrixXf(0, 3)),
    FC_(MatrixXf(0, 3)),
    VL_(VectorXi(0, 3)),
    FL_(VectorXi(0, 3)),
    VN_(MatrixXd(0, 3)),
    FN_(MatrixXd(0, 3)),
    P_(MatrixXd(0, 3)),
    PC_(MatrixXf(0, 3)),
    PL_(VectorXi(0, 3)),
    PN_(MatrixXd(0, 3)),
    bb_min_(Vector3d::Zero()),
    bb_max_(Vector3d::Zero()),
    center_(Vector3d::Zero()),
    radius_(1.0) {
}

bool LibiglMeshT::read_mesh(const std::string& _filename) {
  if (!igl::read_triangle_mesh(_filename, V_, F_)) {
    LOG(WARNING) << "Can't read the file: '" << _filename << "'.";
    return false;
  }
  mesh_name_ = filesystem::path(_filename).filename();

  update_bounding_box();

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
    renderer_->set_mesh(V_, F_);
    renderer_->set_scene_pos(center_.cast<float>(), (float) radius_);
  }

  LOG(INFO) << "Read " << V_.rows() << " vertices and " << F_.rows()
            << " faces.";

  if (FLAGS_reorient_faces) {
    LOG(INFO) << "Reorient mesh faces...";
    Eigen::MatrixXi new_F;
    Eigen::VectorXi I;
    igl::embree::reorient_facets_raycast(V_, F_, new_F, I);
    F_.swap(new_F);
    LOG(INFO) << "Done.";
  }

  return true;
}


/*
bool LibiglMeshT::read_meshes(const std::string& _filenames) {
  const std::vector<std::string> filenames = Utils::split_string(_filenames);

  MatrixXd all_V = MatrixXd(0, 3);
  MatrixXi all_F = MatrixXi(0, 3);
  double offset = 0.0;

  for (const auto filename : filenames) {
    if (!read_mesh(filename)) return false;

    // Centerize.
    V_ = V_.rowwise() - center_.transpose();

    offset -= (1.5 * radius_);

    // Translate.
    V_ = V_.rowwise() + (offset * RowVector3d::UnitX());

    // Add the mesh.
    igl::add_mesh(all_V, all_F, V_, F_);

    // Add offset.
    offset -= (1.5 * radius_);
  }

  V_ = all_V;
  F_ = all_F;
  update_bounding_box();

  // Centerize.
  V_ = V_.rowwise() - center_.transpose();

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
    renderer_->set_mesh(V_, F_);
    renderer_->set_scene_pos(center_.cast<float>(), (float) radius_);
  }

  return true;
}
*/

bool LibiglMeshT::read_meshes(const std::string& _filenames) {
  const std::vector<std::string> filenames = Utils::split_string(_filenames);
  const int n_meshes = filenames.size();

  MatrixXd all_V = MatrixXd(0, 3);
  MatrixXi all_F = MatrixXi(0, 3);

  std::vector<int> n_faces;
  int n_total_faces = 0;

  for (const auto filename : filenames) {
    if (!read_mesh(filename)) return false;

    n_faces.push_back(F_.rows());
    n_total_faces += F_.rows();

    // Add the mesh.
    igl::add_mesh(all_V, all_F, V_, F_);
  }

  V_ = all_V;
  F_ = all_F;
  update_bounding_box();

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
    renderer_->set_mesh(V_, F_);
    renderer_->set_scene_pos(center_.cast<float>(), (float) radius_);
  }

  // Parse labels if given.
  if (FLAGS_mesh_labels != "") {
    const auto label_strs = Utils::split_string(FLAGS_mesh_labels);
    if (label_strs.size() != n_meshes) {
      LOG(ERROR) << "Numbers of meshes and labels do not match.";
    }
    std::vector<int> labels(n_meshes);
    for (int i = 0; i < n_meshes; ++i) labels[i] = std::stoi(label_strs[i]);

    FL_ = VectorXi(n_total_faces);
    int fid = 0;
    for (int i = 0; i < n_meshes; ++i) {
      for (int j = 0; j < n_faces[i]; ++j) {
        FL_[fid++] = labels[i];
      }
    }

    // Set face colors.
    set_face_label_colors();
  }

  return true;
}


bool LibiglMeshT::read_face_labels(const std::string& _filename) {
  FL_ = VectorXi(n_faces());
  if (!Utils::read_eigen_matrix_from_file(_filename, &FL_)) {
    return false;
  }

  // Set face colors.
  set_face_label_colors();
  return true;
}

bool LibiglMeshT::read_point_cloud(const std::string& _filename) {
  filesystem::path path(_filename);
  const std::string ext = path.extension();
  if (ext == "obj" || ext == "off" || ext == "ply") {
    MatrixXi temp_F;
    if (!igl::read_triangle_mesh(_filename, P_, temp_F)) {
      LOG(WARNING) << "Can't read the file: '" << _filename << "'.";
      return false;
    }
  } else {
    MatrixXd X;

    if (!Utils::read_eigen_matrix_from_file(_filename, &X, ' ')) {
      LOG(WARNING) << "Can't read the file: '" << _filename << "'";
      return false;
    }

    if (X.cols() == 7) {
      // px, py, pz, nx, ny, nz, label
      P_ = MatrixXd(X.rows(), 3);
      P_ = X.leftCols(3);

      PN_ = MatrixXd(X.rows(), 3);
      PN_ = X.middleCols(3, 3);

      PL_ = VectorXi(X.rows());
      PL_ = X.rightCols(1).cast<int>();
    } else if (X.cols() == 6) {
      // px, py, pz, nx, ny, nz
      P_ = MatrixXd(X.rows(), 3);
      P_ = X.leftCols(3);

      PN_ = MatrixXd(X.rows(), 3);
      PN_ = X.rightCols(3);
    } else if (X.cols() == 3) {
      // px, py, pz
      P_ = MatrixXd(X.rows(), 3);
      P_ = X;
    } else {
      LOG(WARNING) << "Wrong file format: '" << _filename << "'.";
      return false;
    }
  }

  update_bounding_box();

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
    renderer_->set_point_cloud(P_);
    renderer_->set_scene_pos(center_.cast<float>(), (float) radius_);
  }

  if (P_.rows() == PN_.rows() && P_.rows() == PL_.rows()) {
    LOG(INFO) << "Read " << P_.rows() << " points with normals and labels.";
  } else if (P_.rows() == PN_.rows()) {
    LOG(INFO) << "Read " << P_.rows() << " points with normals.";
  } else {
    LOG(INFO) << "Read " << P_.rows() << " points.";
  }

  return true;
}

bool LibiglMeshT::write_point_cloud(const std::string& _filename) {
  // FIXME:
  // Consider other formats.
  MatrixXi temp_F;
  MatrixXd temp_UV;

	if (P_.rows() == PN_.rows()) {
		if (!igl::writePLY(_filename, P_, temp_F, PN_, temp_UV)) {
			LOG(WARNING) << "Can't write the file: '" << _filename << "'.";
			return false;
		}
		LOG(INFO) << "Wrote " << P_.rows() << " points with normals.";
	}
	else{
		if (!igl::writePLY(_filename, P_, temp_F)) {
			LOG(WARNING) << "Can't write the file: '" << _filename << "'.";
			return false;
		}
		LOG(INFO) << "Wrote " << P_.rows() << " points.";
	}

  return true;
}

bool LibiglMeshT::write_face_labels(const std::string& _filename) {
  if (!Utils::write_eigen_matrix_to_file(_filename, FL_)) {
    return false;
  }
  return true;
}

void LibiglMeshT::set_face_label_colors() {
  if (FL_.rows() != F_.rows()) {
    LOG(WARNING) << "Number of face labels does not match number of faces.";
    return;
  }

  FC_ = MatrixXf(n_faces(), 3);
  for (int fid = 0; fid < n_faces(); ++fid) {
    Vector3f color;
    Utils::random_label_rgb_color(FL_(fid), &color);
    FC_.row(fid) = color.transpose();
  }

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
    renderer_->set_face_colors(FC_);
  }
}

bool LibiglMeshT::read_point_labels(const std::string& _filename) {
  if (!Utils::read_eigen_matrix_from_file(_filename, &PL_)) {
    return false;
  }

  // Set point colors.
  set_point_label_colors();
  return true;
}

bool LibiglMeshT::write_point_labels(const std::string& _filename) {
  if (!Utils::write_eigen_matrix_to_file(_filename, PL_)) {
    return false;
  }
  return true;
}

bool LibiglMeshT::read_point_values(const std::string& _filename) {
  VectorXf PV;
  if (!Utils::read_eigen_matrix_from_file(_filename, &PV)) {
    return false;
  }

  if (PV.size() != P_.rows()) {
    LOG(WARNING) << "Number of point values does not match number of points.";
    return false;
  }

  PC_ = compute_color_map(PV);

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set";
  } else {
    renderer_->set_point_colors(PC_);
  }

  return true;
}

void LibiglMeshT::set_point_label_colors() {
  if (PL_.rows() != P_.rows()) {
    LOG(WARNING) << "Number of point labels does not match number of points.";
    return;
  }

  PC_ = MatrixXf(n_points(), 3);
  for (int pid = 0; pid < n_points(); ++pid) {
    Vector3f color;
    Utils::random_label_rgb_color(PL_(pid), &color);
    PC_.row(pid) = color.transpose();
  }

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
    renderer_->set_point_colors(PC_);
  }
}

MatrixXf LibiglMeshT::compute_color_map(const VectorXf& _values) {
  const int n_values = _values.size();
  MatrixXf colors = MatrixXf(n_values, 3);
  colors.setZero();

  const float vmin = _values.minCoeff();
  const float vmax = _values.maxCoeff();
  const float dv = vmax - vmin;

  if (dv > 1.0e-8) {
    for (int pid = 0; pid < n_values; ++pid) {
      Vector3f color = Vector3f::Ones();
      const float v = _values[pid];

      // https://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
      if (v < (vmin + 0.25f * dv)) {
        color[0] = 0.0f;
        color[1] = 4.0f * (v - vmin) / dv;
      } else if (v < (vmin + 0.5f * dv)) {
        color[0] = 0.0f;
        color[2] = 1.0f + 4.0f * (vmin + 0.25f * dv - v) / dv;
      } else if (v < (vmin + 0.75f * dv)) {
        color[0] = 4.0f * (v - vmin - 0.5f * dv) / dv;
        color[2] = 0.0f;
      } else {
        color[1] = 1.0f + 4.0f * (vmin + 0.75f * dv - v) / dv;
        color[2] = 0.0f;
      }

      colors.row(pid) = color.transpose();
    }
  }

  return colors;
}

void LibiglMeshT::update_bounding_box(const MatrixXd& _P) {
  bb_min_ = _P.colwise().minCoeff();
  bb_max_ = _P.colwise().maxCoeff();
  center_ = 0.5 * (bb_min_ + bb_max_);
  radius_ = 0.5 * (bb_max_ - bb_min_).norm();
}

void LibiglMeshT::update_bounding_box() {
  if (V_.rows() == 0 && P_.rows() == 0) {
    return;
  }

  for (int i = 0; i < 3; ++i) {
    bb_min_[i] = +std::numeric_limits<double>::max();
    bb_max_[i] = -std::numeric_limits<double>::max();
  }

  if (V_.rows() > 0) {
    const auto V_bb_min = V_.colwise().minCoeff();
    const auto V_bb_max = V_.colwise().maxCoeff();
    for (int i = 0; i < 3; ++i) {
      bb_min_[i] = std::min(bb_min_[i], V_bb_min[i]);
      bb_max_[i] = std::max(bb_max_[i], V_bb_max[i]);
    }
  }
  else if (P_.rows() > 0) {
    const auto P_bb_min = P_.colwise().minCoeff();
    const auto P_bb_max = P_.colwise().maxCoeff();
    for (int i = 0; i < 3; ++i) {
      bb_min_[i] = std::min(bb_min_[i], P_bb_min[i]);
      bb_max_[i] = std::max(bb_max_[i], P_bb_max[i]);
    }
  }

  center_ = 0.5 * (bb_min_ + bb_max_);
  radius_ = 0.5 * (bb_max_ - bb_min_).norm();
}

bool LibiglMeshT::write_bounding_box(const std::string& _filename) {
  const double bbox_diagonal = (bb_max_ - bb_min_).norm();
  Eigen::VectorXd bb_info(4);
  bb_info << center_, bbox_diagonal;
  if (!Utils::write_eigen_matrix_to_file(_filename, bb_info.transpose())) {
    return false;
  }
  return true;
}

void LibiglMeshT::pre_processing() {
  if (FLAGS_mesh != "") {
    if (!read_mesh(FLAGS_mesh)) {
      return;
    }
  }

  if (FLAGS_meshes != "") {
    if (!read_meshes(FLAGS_meshes)) {
      return;
    }
  }

  if (FLAGS_face_labels != "") {
    if (!read_face_labels(FLAGS_face_labels)) {
      return;
    }
  }

  if (FLAGS_point_cloud != "") {
    if (!read_point_cloud(FLAGS_point_cloud)) {
      return;
    }
  }

  if (FLAGS_point_labels != "") {
    if (!read_point_labels(FLAGS_point_labels)) {
      return;
    }
  }

  if (FLAGS_point_values != "") {
    if (!read_point_values(FLAGS_point_values)) {
      return;
    }
  }
}

void LibiglMeshT::post_processing() {
  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
    renderer_->set_camera_params(
        Vector3f(FLAGS_azimuth_deg, FLAGS_elevation_deg, FLAGS_theta_deg),
        center_.cast<float>(), (float) radius_);
  }

  if (FLAGS_projection_matrix != "") {
    if (renderer_ == nullptr) {
      LOG(WARNING) << "Renderer is not set.";
    } else if (!renderer_->read_projection(FLAGS_projection_matrix)) {
      return;
    }
  }

  if (FLAGS_modelview_matrix != "") {
    if (renderer_ == nullptr) {
      LOG(WARNING) << "Renderer is not set.";
    } else if (!renderer_->read_modelview(FLAGS_modelview_matrix)) {
      return;
    }
  }

  if (FLAGS_bbox != "") {
    if (!write_bounding_box(FLAGS_bbox)) {
      return;
    }
  }

  if (FLAGS_out_mesh != "") {
    igl::write_triangle_mesh(FLAGS_out_mesh, V_, F_);
  }

  if (FLAGS_out_face_labels != "") {
    if (!write_face_labels(FLAGS_out_face_labels)) {
      return;
    }
  }

  if (FLAGS_out_point_cloud != "") {
    if (!write_point_cloud(FLAGS_out_point_cloud)) {
      return;
    }
  }

  if (FLAGS_out_point_labels != "") {
    if (!write_point_labels(FLAGS_out_point_labels)) {
      return;
    }
  }

  if (FLAGS_out_projection_matrix != "") {
    if (renderer_ == nullptr) {
      LOG(WARNING) << "Renderer is not set";
    } else if (!renderer_->write_projection(FLAGS_out_projection_matrix)) {
      return;
    }
  }

  if (FLAGS_out_modelview_matrix != "") {
    if (renderer_ == nullptr) {
      LOG(WARNING) << "Renderer is not set";
    } else if (!renderer_->write_modelview(FLAGS_out_modelview_matrix)) {
      return;
    }
  }

  // Compute normals.
  if (F_.rows() > 0) igl::per_face_normals(V_, F_, FN_);
  if (F_.rows() > 0 && V_.rows() > 0) igl::per_vertex_normals(V_, F_, FN_, VN_);

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
#ifdef OFFSCREEN
    if (FLAGS_snapshot != "") {
      renderer_->snapshot(FLAGS_snapshot);
    }
#else
    renderer_->run_loop();
#endif
  }
}

void LibiglMeshT::parse_arguments_and_run() {
  pre_processing();
  processing();
  post_processing();
}
