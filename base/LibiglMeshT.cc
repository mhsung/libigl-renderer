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
#include <igl/median.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/read_triangle_mesh.h>
#include <igl/readPLY.h>
#include <igl/write_triangle_mesh.h>
#include <igl/writePLY.h>
#include <modules/merge_meshes.h>
#include <modules/principal_curvature_cgal.h>
#include <utils/filesystem/path.h>
#include <utils/google_tools.h>
#include <utils/utils.h>
#include <utils/SparseICP/ICP.h>


const int kNumPointCloudNeighbors = 8;
const double kZeroTol = 1.0e-6;


// Define input variables.
DEFINE_string(mesh, "", "mesh file.");
DEFINE_string(meshes, "", "a list of mesh files.");
DEFINE_string(face_labels, "", "face label file.");
DEFINE_string(point_cloud, "", "point cloud file.");
DEFINE_string(point_labels, "", "point label file.");
DEFINE_string(point_values, "", "point value file.");
DEFINE_string(point_displacements, "", "point displacement file.");
DEFINE_string(primitives, "", "primitives file.");
DEFINE_string(symmetries, "", "symmetrys file.");
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
DEFINE_string(out_primitives, "", "output primitives file.");
DEFINE_string(out_symmetries, "", "output symmetries file.");
DEFINE_string(out_projection_matrix, "", "output projection matrix file.");
DEFINE_string(out_modelview_matrix, "", "output modelview matrix file.");
DEFINE_string(snapshot_prefix_per_primitive, "",
		"pre-primitive snapshot file prefix.");
DEFINE_bool(reorient_faces, false, "reorient mesh faces.");
DEFINE_bool(draw_mesh_curvatures, false, "draw mesh curvatures.");
DEFINE_bool(draw_point_cloud_curvatures, false,
		"draw point cloud curvatures.");


LibiglMeshT::LibiglMeshT()
  : renderer_(nullptr),
	bb_min_(Vector3d(-1.0)),
	bb_max_(Vector3d(+1.0)),
	center_(Vector3d::Zero()),
	radius_(1.0),
	median_P_nn_dists_(0.0) {
}

LibiglMeshT::LibiglMeshT(LibiglMeshRendererT* _renderer)
  : renderer_(_renderer),
	bb_min_(Vector3d(-1.0)),
	bb_max_(Vector3d(+1.0)),
	center_(Vector3d::Zero()),
	radius_(1.0),
	median_P_nn_dists_(0.0) {
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

    if (ext == "h5") {
      if (!Utils::read_eigen_matrix_from_hdf5(_filename, "point_cloud", &X)) {
        LOG(WARNING) << "Can't read the file: '" << _filename << "'.";
        return false;
      }
    } else {
      if (!Utils::read_eigen_matrix_from_file(_filename, &X, ' ')) {
        LOG(WARNING) << "Can't read the file: '" << _filename << "'";
        return false;
      }
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

bool LibiglMeshT::read_point_displacements(const std::string& _filename) {
  if (!Utils::read_eigen_matrix_from_file(_filename, &PD_, ' ')) {
    return false;
  }

  if (PD_.rows() != P_.rows()) {
    LOG(WARNING) << "Number of point displacements does not match number of "
      "points.";
    return false;
  }

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set";
  } else {
    renderer_->set_point_displacements(PD_);
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
  if (V_.rows() == 0 && P_.rows() == 0 && primitives_.empty()) {
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
  else if (!primitives_.empty()) {
    const int kNumSamplesPerPrimitive = 1024;
    const MatrixXd S = randomly_sample_points_on_primitives(
        primitives_.size() * kNumSamplesPerPrimitive);

    const auto S_bb_min = S.colwise().minCoeff();
    const auto S_bb_max = S.colwise().maxCoeff();
    for (int i = 0; i < 3; ++i) {
      bb_min_[i] = std::min(bb_min_[i], S_bb_min[i]);
      bb_max_[i] = std::max(bb_max_[i], S_bb_max[i]);
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

bool LibiglMeshT::read_primitives_json(const std::string& _filename) {
  std::ifstream file(_filename);
  if (!file.good()) return false;

  json all_j;
  file >> all_j;
  file.close();

  CHECK(all_j.is_array());

  primitives_.clear();
  for (const auto& j : all_j) {
		Primitive* new_primitive = Primitive::from_json(j);
		if (new_primitive != nullptr) {

      // NOTE: Compute cropping parameters if point cloud exists.
      if (P_.rows() > 0 && P_.rows() == PL_.rows()) {
        const VectorXi pids = Utils::find(PL_, new_primitive->get_label());
        MatrixXd primitive_P = Utils::slice_rows(P_, pids);
        if (primitive_P.rows() > 0) {
          new_primitive->compute_clipping_params(primitive_P);
        }
      }

      primitives_.emplace_back(new_primitive);
    }
	}
  LOG(INFO) << "Read " << primitives_.size() << " primitives.";

  update_bounding_box();

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
    renderer_->set_primitives(primitives_);
    renderer_->set_scene_pos(center_.cast<float>(), (float) radius_);
  }

  return true;
}

bool LibiglMeshT::write_primitives_json(const std::string& _filename) {
  if (primitives_.empty()) {
    LOG(WARNING) << "No primitive exists.";
    return false;
  }

  json all_j;
  for (const auto& primitive : primitives_) {
    json j;
    primitive->to_json(j);
    all_j.push_back(j);
  }

  std::ofstream file(_filename);
  if (!file.good()) return false;
  file << all_j.dump(2) << std::endl;
  file.close();
  LOG(INFO) << "Wrote " << primitives_.size() << " primitives.";

  return true;
}

bool LibiglMeshT::read_symmetries_json(const std::string& _filename) {
  std::ifstream file(_filename);
  if (!file.good()) return false;

  json all_j;
  file >> all_j;
  file.close();

  CHECK(all_j.is_array());

  symmetries_.clear();
  for (const auto& j : all_j) {
		Symmetry* new_primitive = Symmetry::from_json(j);
		CHECK(new_primitive != nullptr);
    symmetries_.emplace_back(new_primitive);
	}
  LOG(INFO) << "Read " << symmetries_.size() << " symmetries.";

  update_bounding_box();

  if (renderer_ == nullptr) {
    LOG(WARNING) << "Renderer is not set.";
  } else {
    renderer_->set_symmetries(symmetries_);
    renderer_->set_scene_pos(center_.cast<float>(), (float) radius_);
  }

  return true;
}

bool LibiglMeshT::write_symmetries_json(const std::string& _filename) {
  if (symmetries_.empty()) {
    LOG(WARNING) << "No primitive exists.";
    return false;
  }

  json all_j;
  for (const auto& primitive : symmetries_) {
    json j;
    primitive->to_json(j);
    all_j.push_back(j);
  }

  std::ofstream file(_filename);
  if (!file.good()) return false;
  file << all_j.dump(2) << std::endl;
  file.close();
  LOG(INFO) << "Wrote " << symmetries_.size() << " symmetries.";

  return true;
}

void LibiglMeshT::compute_mesh_principal_curvatures(bool _render) {
	// NOTE:
	// Vertex normals are updated.
	LOG(INFO) << "Computing mesh principal curvatures...";
	MatrixXd old_VN;
	igl::per_vertex_normals(V_, F_, old_VN);
	igl::principal_curvatures_mesh(V_, F_, old_VN,
			VPD1_, VPD2_, VN_, VPV1_, VPV2_);

	if (_render) {
		if (renderer_ == nullptr) {
			LOG(WARNING) << "Renderer is not set.";
		} else {
			renderer_->set_vertex_curvatures(VPD1_, VPD2_, VPV1_, VPV2_);
			renderer_->set_vertex_normals(VN_);
		}
	}

	LOG(INFO) << "Done.";
}

void LibiglMeshT::compute_point_cloud_principal_curvatures(bool _render) {
	// NOTE:
	// Point cloud normals are updated.
	LOG(INFO) << "Computing point cloud principal curvatures...";
	MatrixXd old_PN = PN_;
	igl::principal_curvatures_point_cloud(P_, old_PN,
			PPD1_, PPD2_, PN_, PPV1_, PPV2_);

	if (_render) {
		if (renderer_ == nullptr) {
			LOG(WARNING) << "Renderer is not set.";
		} else {
			renderer_->set_point_curvatures(PPD1_, PPD2_, PPV1_, PPV2_);
			renderer_->set_point_normals(PN_);
		}
	}

	LOG(INFO) << "Done.";
}

void LibiglMeshT::update_point_cloud_neighbors() {
	const int n_points = P_.rows();
  CHECK_GT(n_points, kNumPointCloudNeighbors);

	const Matrix3Xd P_transpose = P_.transpose();
	nanoflann::KDTreeAdaptor<MatrixBase<Matrix<double, 3, -1, 0, 3, -1>>, 3,
		nanoflann::metric_L2_Simple> kdtree(P_transpose);

	MatrixXi P_nids_(n_points, kNumPointCloudNeighbors);
	VectorXd P_nn_dists(n_points);
	P_nn_dists.setConstant(std::numeric_limits<double>::max());

  for (int pid = 0; pid < n_points; ++pid) {
		// Find N neighbors and the query point itself.
    VectorXi idx(kNumPointCloudNeighbors + 1);
    VectorXd sq_dist(kNumPointCloudNeighbors + 1);
    kdtree.query(P_transpose.col(pid).data(), kNumPointCloudNeighbors + 1,
				idx.data(), sq_dist.data());

		// Add neighbors except the query point.
		int count = 0;
		for (int i = 0; i < kNumPointCloudNeighbors + 1; ++i) {
			if (idx[i] == pid) continue;

			CHECK_LT(count, kNumPointCloudNeighbors);
			P_nids_(pid, count) = idx[i];
			P_nn_dists(pid) = std::min(P_nn_dists(pid), std::sqrt(sq_dist[i]));

			++count;
		}
  }

	igl::median(P_nn_dists, median_P_nn_dists_);
	LOG(INFO) << "Median of point cloud nearest neighbor distances: "
		<< median_P_nn_dists_;
}

MatrixXd LibiglMeshT::randomly_sample_points_on_primitives(const int n_points) {
  CHECK (!primitives_.empty());
  const int n_primitives = primitives_.size();

  std::vector<double> areas(n_primitives, 0.0);
  double sum_area = 0.0;
  for (int i = 0; i < n_primitives; ++i) {
    areas[i] = primitives_[i]->area();
    sum_area += areas[i];
  }
  CHECK_GT(sum_area, 0.0);

  MatrixXd points = MatrixXd::Zero(n_points, 3);
  int starting_index = 0;

  for (int i = 0; i < n_primitives; ++i) {
    const int n_remaining_points = n_points - starting_index;
    const int n_primitive_points = std::min(n_remaining_points,
				static_cast<int>(std::ceil(areas[i] / sum_area * n_points)));
    const MatrixXd primitive_points =
        primitives_[i]->randomly_sample_points(n_primitive_points);
    points.block(starting_index, 0, n_primitive_points, 3) = primitive_points;

    starting_index += n_primitive_points;
  }
  CHECK_EQ(starting_index, n_points);

  return points;
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

  if (FLAGS_point_displacements != "") {
    if (!read_point_displacements(FLAGS_point_displacements)) {
      return;
    }
  }

  if (FLAGS_primitives != "") {
    if (!read_primitives_json(FLAGS_primitives)) {
      return;
    }
  }
}

void LibiglMeshT::post_processing() {
	if (FLAGS_draw_mesh_curvatures) {
		compute_mesh_principal_curvatures();
	}

	if (FLAGS_draw_point_cloud_curvatures) {
		compute_point_cloud_principal_curvatures();
	}

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

  if (FLAGS_out_primitives != "") {
    if (!write_primitives_json(FLAGS_out_primitives)) {
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
#ifdef USE_OSMESA
    if (FLAGS_snapshot != "") {
      renderer_->snapshot(FLAGS_snapshot);
    }
#else
		renderer_->run_loop();
#endif

		if (FLAGS_snapshot_prefix_per_primitive != "") {
			for (int i = 0; i < primitives_.size(); ++i) {
				std::vector<PrimitivePtr> each_primitive;
				each_primitive.emplace_back(primitives_[i]->clone());
				renderer_->set_primitives(each_primitive);

				// NOTE:
				// The primitive IDs start from 1.
				Vector3f color;
				Utils::random_label_rgb_color(i + 1, &color);
				MatrixXf each_primitive_color(1, 3);
				each_primitive_color.row(0) = color.transpose();
				renderer_->set_primitive_colors(each_primitive_color);

				std::stringstream sstr;
				sstr << FLAGS_snapshot_prefix_per_primitive << "_"
					<< std::setfill('0') << std::setw(4) << i;
				renderer_->snapshot(sstr.str());
			}

			renderer_->set_primitives(primitives_);
		}
	}
}

void LibiglMeshT::parse_arguments_and_run() {
  pre_processing();
  processing();
  post_processing();
}
