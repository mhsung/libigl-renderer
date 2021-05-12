// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "GlutMeshViewer.h"

#include <ctime>
#include <iostream>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GL/freeglut.h>
#include <GL/freeglut_ext.h>
#include <igl/png/writePNG.h>
#include <utils/google_tools.h>


using namespace Eigen;


GlutMeshViewer* GlutMeshViewer::g_instance_ = nullptr;
bool GlutMeshViewer::g_is_left_down_ = false;
const double kTrackballRadius = 0.6;


GlutMeshViewer::GlutMeshViewer(const int _width, const int _height)
	: GLMeshRenderer(_width, _height) {
	int argc = 1;
	char *argv[1] = {(char*)"Nothing"};
	glutInit(&argc, argv);
	glutInitWindowSize(_width, _height);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("GlutViewer");

	// Initialize OpenGL graphics state.
	initialize_opengl();

	// Register callbacks.
	CHECK(g_instance_ == nullptr);
	g_instance_ = this;

	glutDisplayFunc(GlutMeshViewer::display);
	glutReshapeFunc(GlutMeshViewer::reshape);
	glutMouseFunc(GlutMeshViewer::mouse);
	glutMotionFunc(GlutMeshViewer::motion);
	glutKeyboardFunc(GlutMeshViewer::Keyboard);

	// Create popup rendering menu.
	glutCreateMenu(menu);
	for (int i = 0; i < g_instance_->draw_mode_names_.size(); ++i)
		glutAddMenuEntry(g_instance_->draw_mode_names_[i].c_str(), i);

	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

GlutMeshViewer::~GlutMeshViewer() {
}

bool GlutMeshViewer::snapshot(const std::string& _filename) {
	CHECK_NOTNULL(g_instance_);

	GLenum buffer(GL_BACK);
	glReadBuffer(buffer);

	display();
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	std::vector<GLubyte> frame_buffer(4 * width_ * height_);
	glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE,
			&frame_buffer[0]);

	// Allocate temporary buffers.
	Matrix<unsigned char, Dynamic, Dynamic> R(width_, height_);
	Matrix<unsigned char, Dynamic, Dynamic> G(width_, height_);
	Matrix<unsigned char, Dynamic, Dynamic> B(width_, height_);
	Matrix<unsigned char, Dynamic, Dynamic> A(width_, height_);

	for (int y = height_ - 1; y >= 0; --y) {
		for (int x = 0; x < width_; ++x) {
			const int i = (y * width_ + x) * 4;
			R(x, y) = frame_buffer[i];
			G(x, y) = frame_buffer[i + 1];
			B(x, y) = frame_buffer[i + 2];
			A(x, y) = frame_buffer[i + 3];
		}
	}

	// Save it to a PNG.
	return igl::png::writePNG(R, G, B, A, _filename + std::string(".png"));
}

void GlutMeshViewer::run_loop() {
	glutMainLoop();
}

void GlutMeshViewer::display() {
	CHECK_NOTNULL(g_instance_);
	g_instance_->render();
	glutSwapBuffers();
}

void GlutMeshViewer::reshape(GLint _width, GLint _height) {
	CHECK_NOTNULL(g_instance_);
	g_instance_->set_window_size(_width, _height);
	glViewport(0, 0, _width, _height);
	glutSwapBuffers();
}

void GlutMeshViewer::mouse(int _button, int _state, int _x, int _y) {
	CHECK_NOTNULL(g_instance_);

	Vector2f new_point_2d(_x, _y);

	bool is_left_button = (_button == GLUT_LEFT_BUTTON);
	bool is_mid_button = (_button == GLUT_MIDDLE_BUTTON);
	bool is_right_button = (_button == GLUT_RIGHT_BUTTON);

	bool is_ctrl_pressed = (glutGetModifiers() == GLUT_ACTIVE_CTRL);
	bool is_alt_pressed = (glutGetModifiers() == GLUT_ACTIVE_ALT);
	bool is_shift_pressed = (glutGetModifiers() == GLUT_ACTIVE_SHIFT);

	if (_state == GLUT_DOWN) {
		g_is_left_down_ = true;
		g_instance_->mouse_press_event(new_point_2d,
				is_left_button, is_mid_button, is_right_button,
				is_ctrl_pressed, is_alt_pressed, is_shift_pressed);
	}
	else if (_state == GLUT_UP) {
		g_is_left_down_ = false;
		g_instance_->mouse_release_event(new_point_2d,
				is_left_button, is_mid_button, is_right_button,
				is_ctrl_pressed, is_alt_pressed, is_shift_pressed);
	}
}

void GlutMeshViewer::motion(int _x, int _y) {
	CHECK_NOTNULL(g_instance_);

	if (g_is_left_down_) {
		Vector2f new_point_2d(_x, _y);

		bool is_left_button = true;
		bool is_mid_button = false;
		bool is_right_button = false;

		bool is_ctrl_pressed = (glutGetModifiers() == GLUT_ACTIVE_CTRL);
		bool is_alt_pressed = (glutGetModifiers() == GLUT_ACTIVE_ALT);
		bool is_shift_pressed = (glutGetModifiers() == GLUT_ACTIVE_SHIFT);

		g_instance_->mouse_move_event(new_point_2d,
				is_left_button, is_mid_button, is_right_button,
				is_ctrl_pressed, is_alt_pressed, is_shift_pressed);

		glutPostRedisplay();
	}
}

void GlutMeshViewer::Keyboard(unsigned char _key, int _x, int _y) {
	CHECK_NOTNULL(g_instance_);

	bool is_ctrl_pressed = (glutGetModifiers() == GLUT_ACTIVE_CTRL);
	bool is_alt_pressed = (glutGetModifiers() == GLUT_ACTIVE_ALT);
	bool is_shift_pressed = (glutGetModifiers() == GLUT_ACTIVE_SHIFT);

	g_instance_->key_press_event(_key,
			is_ctrl_pressed, is_alt_pressed, is_shift_pressed);
}

void GlutMeshViewer::menu(int value) {
	CHECK_NOTNULL(g_instance_);

	g_instance_->draw_mode_idx_ = value;
	CHECK_LT(g_instance_->draw_mode_idx_,
			g_instance_->draw_mode_names_.size());

	glutPostRedisplay();
}

void GlutMeshViewer::mouse_press_event(
		const Vector2f& _new_point_2d,
		bool _is_left_button, bool _is_mid_button, bool _is_right_button,
		bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed) {
	last_point_2d_ = _new_point_2d;
	last_point_ok_ = map_to_sphere(_new_point_2d, last_point_3d_);
}

void GlutMeshViewer::mouse_move_event(
		const Vector2f& _new_point_2d,
		bool _is_left_button, bool _is_mid_button, bool _is_right_button,
		bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed) {
	// Left button: rotate around center
	// Middle button: translate object
	// Left & middle button: zoom in/out

	Vector3f new_point_3d;
	bool new_point_hit_sphere = map_to_sphere(_new_point_2d, new_point_3d);

	float dx = _new_point_2d[0] - last_point_2d_[0];
	float dy = _new_point_2d[1] - last_point_2d_[1];

	float w = width_;
	float h = height_;

	// move in z direction
	if ((_is_left_button && _is_mid_button) ||
		(_is_left_button && _is_ctrl_pressed))
	{
		float value_y = radius_ * dy * 3.0 / h;
		translate(Vector3f(0.0, 0.0, value_y));
	}
	// move in x,y direction
	else if ((_is_mid_button) ||
		(_is_left_button && _is_alt_pressed))
	{
		Vector3f t = (modelview_ * center_.colwise().homogeneous()).
			colwise().hnormalized();
		float z = -t[2];
		float aspect = w / h;
		float near_plane = 0.01 * radius_;
		float top = std::tan(kFovyDeg / 2.0f*M_PI / 180.0f) * near_plane;
		float right = aspect*top;

		translate(Vector3f(
					2.0*dx / w*right / near_plane*z,
					-2.0*dy / h*top / near_plane*z,
					0.0f));
	}
	// rotate
	else if (_is_left_button) {
		if (last_point_ok_) {
			if ((new_point_hit_sphere = map_to_sphere(_new_point_2d, new_point_3d))) {
				Vector3f axis = last_point_3d_.cross(new_point_3d);
				if (axis.squaredNorm() < 1e-7) {
					axis = Vector3f(1, 0, 0);
				} else {
					axis.normalize();
				}
				// find the amount of rotation
				Vector3f d = last_point_3d_ - new_point_3d;
				float t = 0.5 * d.norm() / kTrackballRadius;
				if (t < -1.0) t = -1.0;
				else if (t > 1.0) t = 1.0;
				float phi = 2.0 * std::asin(t);
				float angle = phi * 180.0 / M_PI;
				rotate(axis, angle);
			}
		}
	}

	// remember this point
	last_point_2d_ = _new_point_2d;
	last_point_3d_ = new_point_3d;
	last_point_ok_ = new_point_hit_sphere;

	// trigger redraw
	glutPostRedisplay();
}

void GlutMeshViewer::mouse_release_event(
		const Vector2f& _new_point_2d,
		bool _is_left_button, bool _is_mid_button, bool _is_right_button,
		bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed) {
	last_point_ok_ = false;
}

void GlutMeshViewer::mouse_wheel_event(const float _new_delta,
		bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed) {
	// Use the mouse wheel to zoom in/out
	float d = -_new_delta / 120.0 * 0.2 * radius_;
	translate(Vector3f(0.0, 0.0, d));
	glutPostRedisplay();
}

void GlutMeshViewer::key_press_event(const char _new_key,
		bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed) {
	std::time_t t = std::time(nullptr);
  char buf[80];
  std::strftime(buf, sizeof(buf), "%Y_%m_%d_%H_%M_%S", std::localtime(&t));
  const std::string time(buf);

	switch (_new_key)
	{
	case 'h':
		std::cout << "Keys:\n";
		std::cout << "  c\tEnable/disable back face culling\n";
		std::cout << "  f\tEnable/disable fog\n";
		std::cout << "  h\tHelp\n";
		std::cout << "  i\tDisplay information\n";
		std::cout << "  m\tSave modelview matrix\n";
		std::cout << "  p\tSave projection matrix\n";
		std::cout << "  s\tSave snapshot\n";
		std::cout << "  ESC\tQuit\n";
		break;

	case 'c':
		if (glIsEnabled(GL_CULL_FACE)) {
			glDisable(GL_CULL_FACE);
			std::cout << "Back face culling: disabled\n";
		} else {
			glEnable(GL_CULL_FACE);
			std::cout << "Back face culling: enabled\n";
		}
		glutPostRedisplay();
		break;

	case 'f':
		if (glIsEnabled(GL_FOG)) {
			glDisable(GL_FOG);
			std::cout << "Fog: disabled\n";
		} else {
			glEnable(GL_FOG);
			std::cout << "Fog: enabled\n";
		}
		glutPostRedisplay();
		break;

	case 'i':
		std::cout << "Scene radius: " << radius_ << std::endl;
		std::cout << "Scene center: " << center_.transpose() << std::endl;
		break;

	case 'm':
		write_modelview("Modelview_" + time + ".csv");
		std::cout << "Saved 'Modelview_" << time << ".csv'." << std::endl;
		break;

	case 'p':
		write_projection("Projection_" + time + ".csv");
		std::cout << "Saved 'Projection_" << time << ".csv'." << std::endl;
		break;

	case 's':
		snapshot("Snapshot_" + time);
		std::cout << "Saved 'Snapshot_" << time << ".png'." << std::endl;
		break;

	case 27:	// ESCAPE key
		exit(0);
	}
}

void GlutMeshViewer::translate(const Vector3f& _trans) {
	glLoadIdentity();
	glTranslatef(_trans[0], _trans[1], _trans[2]);
	glMultMatrixf(modelview_.data());
	glGetFloatv(GL_MODELVIEW_MATRIX, modelview_.data());
}

void GlutMeshViewer::rotate(const Vector3f& _axis, float _angle) {
	Vector3f t = (modelview_ * center_.colwise().homogeneous()).
		colwise().hnormalized();

	glLoadIdentity();
	glTranslatef(t[0], t[1], t[2]);
	glRotatef(_angle, _axis[0], _axis[1], _axis[2]);
	glTranslatef(-t[0], -t[1], -t[2]);
	glMultMatrixf(modelview_.data());
	glGetFloatv(GL_MODELVIEW_MATRIX, modelview_.data());
}

bool GlutMeshViewer::map_to_sphere(const Vector2f& _v2d, Vector3f& _v3d) {
	// This is actually doing the Sphere/Hyperbolic sheet hybrid thing,
	// based on Ken Shoemake's ArcBall in Graphics Gems IV, 1993.
	double x = (2.0*_v2d[0] - width_) / width_;
	double y = -(2.0*_v2d[1] - height_) / height_;
	double xval = x;
	double yval = y;
	double x2y2 = xval*xval + yval*yval;

	const double rsqr = kTrackballRadius*kTrackballRadius;
	_v3d[0] = xval;
	_v3d[1] = yval;
	if (x2y2 < 0.5*rsqr) {
		_v3d[2] = sqrt(rsqr - x2y2);
	}
	else {
		_v3d[2] = 0.5*rsqr / sqrt(x2y2);
	}

	return true;
}

