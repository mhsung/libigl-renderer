// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef GLUT_MESH_VIEWER_T_H
#define GLUT_MESH_VIEWER_T_H

#include "GLMeshRenderer.h"


class GlutMeshViewer : public GLMeshRenderer {
  public:
    GlutMeshViewer(const int _width, const int _height);
    virtual ~GlutMeshViewer();

	// Static variables.
	private:
		static GlutMeshViewer* g_instance_;
		static bool g_is_left_down_;

	public:
    // Virtual functions.
    virtual void run_loop();
    virtual bool snapshot(const std::string& _filename);

	// GLUT callback functions.
  private:
		static void display();
		static void reshape(GLint _width, GLint _height);
		static void mouse(int _button, int _state, int _x, int _y);
		static void mouse_wheel(int _button, int _dir, int _x, int _y);
		static void motion(int _x, int _y);
		static void Keyboard(unsigned char _key, int _x, int _y);
		static void menu(int value);

	// User interaction functions.
	private:
		void mouse_press_event(
				const Vector2f& _new_point_2d,
				bool _is_left_button, bool _is_mid_button, bool _is_right_button,
				bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed);
		void mouse_move_event(
				const Vector2f& _new_point_2d,
				bool _is_left_button, bool _is_mid_button, bool _is_right_button,
				bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed);
		void mouse_release_event(
				const Vector2f& _new_point_2d,
				bool _is_left_button, bool _is_mid_button, bool _is_right_button,
				bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed);
		void mouse_wheel_event(const float _new_delta,
				bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed);
		void key_press_event(const char _new_key,
				bool _is_ctrl_pressed, bool _is_alt_pressed, bool _is_shift_pressed);

		void translate(const Vector3f& _trans);
		void rotate(const Vector3f& _axis, float _angle);
		bool map_to_sphere(const Vector2f& _v2d, Vector3f& _v3d);

	private:
		Vector2f last_point_2d_;
		Vector3f last_point_3d_;
		bool last_point_ok_;
};

#endif	// GLUT_MESH_VIEWER_T_H
