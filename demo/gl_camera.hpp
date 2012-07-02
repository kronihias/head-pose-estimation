#ifndef _GLCAMERA_H
#define _GLCAMERA_H

#ifdef _WIN32
#include <windows.h>
//#include <gl/gl.h>
//#include <glext.h>
//#include <wglext.h>
#endif

#include "vector.hpp"
#include "matrix.hpp"
#include "rigid_motion.hpp"
#include "freeglut.h"


namespace Mouse {
	enum button { NONE, ROTATE, MOVEXY, MOVEZ, WHEELUP, WHEELDOWN };
};

class gl_camera {
private:
	int lastmousex, lastmousey;
	Mouse::button lastb;

	math_vector_3f lightdir;    // light direction

	int viewx, viewy, vieww, viewh;

	rigid_motion<float> m_camera_rm;
	math_vector_3f m_scene_center;
	float m_scene_size;

	math_vector_3f spincenter;	// center for spinning

	float field_of_view;

	math_vector_3f mouse2tb(float x, float y); // convert mouse to trackball position

	void rotate(int mousex, int mousey);
	void movexy(int mousex, int mousey);
	void movez(int mousey);
	void wheel(Mouse::button updown);

public:

	gl_camera() :lastb(Mouse::NONE), lightdir(math_vector_3f(0.0f,0.0f,1.0f)),
			 field_of_view(45.0f)
	{
		lightdir[0] = lightdir[1] = 0; lightdir[2] = 1;
	}

	enum Mouse::button get_last_mouse_button() const { return lastb; }

	const rigid_motion<float> & get_camera_motion() { return m_camera_rm; }

	void set_camera_motion( const rigid_motion<float> & rm ) { m_camera_rm = rm ; }

	// need to be called before all other functions
	void set_viewport(int _viewx, int _viewy, int _vieww, int _viewh) {
		viewx = _viewx;
		viewy = _viewy;
		vieww = _vieww;
		viewh = _viewh;
		glViewport(viewx,viewy,vieww,viewh);
	}
	void get_viewport(int & _viewx, int & _viewy, int & _vieww, int & _viewh) {
		_viewx = viewx;
		_viewy = viewy;
		_vieww = vieww;
		_viewh = viewh;
	}

	void clear(float r = 0, float g = 0, float b = 0, float a = 0);

	// look at scene center
	void resetview(const math_vector_3f &scene_center, float scene_size);
	void resetview(const math_vector_3f &scene_center);
	// rotate around scene center 180 degrees
	void rotate_180();
	void rotate( const math_matrix_3x3f & _rot );
	void pre_rotate( const math_matrix_3x3f & _rot );

	void translate( const math_vector_3f & _t );

	void setup() const {
		setup(m_scene_center, m_scene_size);
	}
	void setup(	const math_vector_3f &scene_center, float scene_size) const;

	void setup( const math_matrix_3x3f & K, int w, int h, float zNear, float zFar);

	// mouse click event
	void mouse(bool left, bool right, int mousex, int mousey);
	void mouse(int mousex, int mousey, Mouse::button b) { mouse(mousex,mousey,b,m_scene_center); }
	void mouse(int mousex, int mousey, Mouse::button b, const math_vector_3f &scene_center);

	void mouse_rotate(int mousex, int mousey) { mouse(mousex,mousey,Mouse::ROTATE,m_scene_center); }
	void mouse_rotate(int mousex, int mousey, const math_vector_3f &scene_center) { mouse(mousex,mousey,Mouse::ROTATE,scene_center); }

	void mouse_translate(int mousex, int mousey) { mouse(mousex,mousey,Mouse::MOVEXY,m_scene_center); }
	void mouse_translate(int mousex, int mousey, const math_vector_3f &scene_center)  { mouse(mousex,mousey,Mouse::MOVEXY,scene_center); }

	// mouse move event
	void mouse_move(int mousex, int mousey) { mouse_move(mousex,mousey,m_scene_center); }
	void mouse_move(int mousex, int mousey, const math_vector_3f & scene_center);

	// mouse wheel event
	void mouse_wheel(int delta);
	// mouse wheel event simulated based on motion in y direction
	void mouse_wheel_y(int delta);

	math_vector_3f light() const { return lightdir; }
	void set_light(const math_vector_3f & _lightdir) { lightdir = _lightdir; }

	float fov() const { return field_of_view; }
	void set_fov(float _fov) { field_of_view = _fov; }

	void use_light( bool _use_light);
};

#endif
