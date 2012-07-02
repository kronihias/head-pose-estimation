/*
modified by Thibaut Weise

Szymon Rusinkiewicz
Princeton University

gl_camera.cc
Manages OpenGL camera and trackball/arcball interaction
*/

#include "common.hpp"
#include "gl_camera.hpp"
#include "trackball.hpp"
using namespace std;

#define DOF 10.0f
#define MAXDOF 10000.0f
#define TRACKBALL_R 0.8f
#define WHEEL_MOVE 0.2f

// Mouse rotation helper - compute trackball position from mouse pos
math_vector_3f gl_camera::mouse2tb(float x, float y)
{
	// normalize points to -0.5 to 0.5
	x = (x-(float)viewx)/(float)vieww - 0.5f;
	y = (y-(float)viewy)/(float)viewh - 0.5f;

	float r2 = x*x + y*y;
	float t = 0.5 * (TRACKBALL_R*TRACKBALL_R);

	float z;
	if (r2 < t)
		z = sqrt(2*t - r2);
	else
		z = t / sqrt(r2);

	math_vector_3f pos(x, y, z);

	return pos;
}

// Mouse helper - rotate
void gl_camera::rotate(int mousex, int mousey)
{

	float x1 = 2 * (float)lastmousex / (float) vieww - 1;
	float y1 = 2 * (float)lastmousey / (float) viewh - 1;
	float x2 = 2 * (float)mousex / (float) vieww - 1;
	float y2 = 2 * (float)mousey / (float) viewh - 1;
	x1 = MAX( -1, MIN( x1, 1 ) );
	y1 = MAX( -1, MIN( y1, 1 ) );
	x2 = MAX( -1, MIN( x2, 1 ) );
	y2 = MAX( -1, MIN( y2, 1 ) );
	math_matrix_3x3f rot = trackball( x1,y1,x2,y2);

	// apply rotation
	m_camera_rm = rigid_motion<float>(spincenter) * rigid_motion<float>(rot) *
	     rigid_motion<float>(-spincenter) * m_camera_rm;
}

// Mouse helper - translate
void gl_camera::movexy(int mousex, int mousey)
{
	float dx = (mousex - lastmousex) * (m_scene_size / 500.0f);
	float dy = (mousey - lastmousey) * (m_scene_size / 500.0f);
	m_camera_rm = rigid_motion<float>(math_vector_3f(dx, dy, 0.0f))  * m_camera_rm;

}

// Mouse helper - translate in z
void gl_camera::movez(int mousey)
{
	float dy = (float)(mousey - lastmousey);
	m_camera_rm = rigid_motion<float>(math_vector_3f(0.0f,0.0f, dy))  * m_camera_rm;
}

// Mouse helper - wheel motion
void gl_camera::wheel(Mouse::button updown)
{
	float dz = 50.0f * WHEEL_MOVE;
	if (updown == Mouse::WHEELUP)
		dz = -dz;
	m_camera_rm = rigid_motion<float>(math_vector_3f(0.0f, 0.0f, dz)) * m_camera_rm;

}

void gl_camera::rotate_180() {

	math_vector_3f spinaxis = math_vector_3f(1.f,0.f,0.f);

	float spinamount = (float)PI;

	math_vector_3f center = m_camera_rm * m_scene_center;

	// apply rotation
	m_camera_rm = rigid_motion<float>(center) * rigid_motion<float>(axis_to_rotation_matrix(spinamount, spinaxis)) * rigid_motion<float>(-center) * m_camera_rm;
}

// mouse wheel event simulated based on motion in y direction
void gl_camera::mouse_wheel_y(int y) {
	mouse_wheel(y-lastmousey);
	lastmousey = y;
}

// Handle a mouse event
void gl_camera::mouse_wheel(int delta) {
	m_camera_rm = rigid_motion<float>(math_vector_3f(0.0f, 0.0f, delta * m_scene_size / 500.0f)) * m_camera_rm;
}

// Handle a mouse event
void gl_camera::mouse(
	bool left,
	bool right,
	int mousex,
	int mousey
)
{
	Mouse::button b = Mouse::NONE;
	if(left && !right) b = Mouse::ROTATE;
	else if(right && !left) b = Mouse::MOVEXY;
	mouse(mousex,mousey,b);
}

// Handle a mouse event
void gl_camera::mouse(
	int mousex,
	int mousey,
	Mouse::button b,
	const math_vector_3f &scene_center
)
{
	spincenter = m_camera_rm * scene_center;
	lastmousex = mousex;
	lastmousey = mousey;
	lastb = b;
}

// Handle a mouse event
void gl_camera::mouse_move(
	int mousex,
	int mousey,
	const math_vector_3f & /*scene_center*/ )
{
	Mouse::button b = lastb;

	if (b == Mouse::NONE && lastb == Mouse::NONE) {
		return;
	}
	// Handle rotation
	if ((b == Mouse::ROTATE) && (lastb == Mouse::ROTATE)) {
		rotate(mousex, mousey);
	}
	if ((b == Mouse::MOVEZ) && (lastb == Mouse::MOVEZ))
		movez(mousey);
	// Handle translation
	if ((b == Mouse::MOVEXY) && (lastb == Mouse::MOVEXY))
		movexy(mousex, mousey);
	if (b == Mouse::WHEELUP || b == Mouse::WHEELDOWN)
		wheel(b);

	lastmousex = mousex;
	lastmousey = mousey;
	lastb = b;
}

void gl_camera::use_light( bool _use_light) {
	if(_use_light) {
		if(true)
		{
			// set light parameters
			GLfloat mat_specular[4] = { 0.18f, 0.18f, 0.18f, 1.f };
			GLfloat mat_shininess[] = { 64.f };
			GLfloat global_ambient[] = { 0.05f, 0.05f, 0.05f, 1.f };
			GLfloat light0_ambient[] = { 0.0f, 0.0f, 0.0f, 1.f };
			GLfloat light0_diffuse[] = { 0.9f, 0.9f, 0.9f, 1.f };
			GLfloat light0_specular[] = { 0.85f, 0.85f, 0.85f, 1.f };

			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
			glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
			glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
			glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
			glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 0.0);

			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);

			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);// todo include this into spotlight node

			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);
			glEnable(GL_COLOR_MATERIAL);
			glEnable(GL_NORMALIZE);
		}
		else
		{

			float defaultPosition[4] = {0.0,0.0,0.0,1.0};
			float defaultSpotDirection[3] = {0.0,0.0,1.0};

			float defaultAmbientColor[4] = {0.005f,0.005f,0.005f,1.f};
			float defaultDiffuseColor[4] = {0.9f,0.9f,0.9f,1.f};
			float defaultSpecularColor[4] = {0.5f,0.5f,0.5f,1.f};

			glShadeModel(GL_SMOOTH);
			glEnable(GL_LIGHTING);

			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,defaultAmbientColor);
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,defaultDiffuseColor);
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,defaultSpecularColor);
			glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,200.f);

			glLightfv(GL_LIGHT0, GL_POSITION, defaultPosition);
			glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, defaultSpotDirection);

			glLightfv(GL_LIGHT0, GL_AMBIENT, defaultAmbientColor);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, defaultDiffuseColor);
			glLightfv(GL_LIGHT0, GL_SPECULAR, defaultSpecularColor);

			glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 0.f);
			glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 120.f);

			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);

			glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);// todo include this into spotlight node

			glEnable(GL_COLOR_MATERIAL);
		}
	}
	else {
		glDisable(GL_LIGHTING);
	}
}

// Set up the OpenGL camera for rendering
void gl_camera::setup(const math_vector_3f &scene_center, float scene_size) const
{
	glViewport(viewx, viewy, vieww, viewh); // set viewport for rendering

	math_vector_3f center = m_camera_rm * scene_center;

	float fardist  = -(center[2] - 8*scene_size);//max( -(center[2] - scene_size), scene_size / DOF);
	float neardist = max( -(center[2] + scene_size), scene_size / MAXDOF);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(field_of_view, (float)vieww/(float)viewh, neardist, fardist );

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// setup light
	GLfloat light0_position[] = { lightdir[0], lightdir[1], lightdir[2], 0 };
	GLfloat light1_position[] = { -lightdir[0], -lightdir[1], -lightdir[2], 0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);

	// global camera position
	float gl_rm[16];
	to_opengl( m_camera_rm, gl_rm );
	glMultMatrixf( gl_rm );

}

void gl_camera::setup( const math_matrix_3x3f & K, int w, int h, float zNear, float zFar) {

	glViewport(viewx, viewy, vieww, viewh); // set viewport for rendering

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	math_matrix_4x4f mat;
	mat.set_identity();
	// X
	mat[0][0] = 2.f/(float)w*K[0][0]; // use camera instrinsics and convert to GL [0,h] => [-1,1]
	mat[0][2] = (2.f/(float)w*(K[0][2]+0.5f))-1.f; // 0.5 offset as GL pixel middle point is at 0.5,0.5
	// Y
	mat[1][1] = 2.f/(float)h*K[1][1]; // use camera instrinsics and convert to GL [0,h] => [-1,1]
	mat[1][2] = (2.f/(float)h*(K[1][2]+0.5f))-1.f;
	// Z
	mat[2][2] = (zFar+zNear)/(zFar-zNear);
	mat[2][3] = -2.f*zFar*zNear/(zFar-zNear);
	// W
	mat[3][2] = 1; // not as in GL where it would be -1
	mat[3][3] = 0;

	mat = mat.transpose();
	glMultMatrixf( (float*) (&mat) );

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// setup light
	GLfloat light0_position[] = { lightdir[0], lightdir[1], lightdir[2], 0 };
	GLfloat light1_position[] = { -lightdir[0], -lightdir[1], -lightdir[2], 0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);

	// global camera position
	float gl_rm[16];
	to_opengl( m_camera_rm, gl_rm );
	glMultMatrixf( gl_rm );
}

// look at scene center
void gl_camera::resetview(const math_vector_3f &scene_center, float scene_size) {

	m_scene_center = scene_center;
	m_scene_size = scene_size;
	m_camera_rm =	rigid_motion<float>(math_vector_3f(0, 0, -2.0f * scene_size)) * rigid_motion<float>(-scene_center);
	spincenter = m_camera_rm * m_scene_center;

}

// look at scene center
void gl_camera::resetview(const math_vector_3f &scene_center) {

	m_scene_center = scene_center;
	m_camera_rm =	rigid_motion<float>(math_vector_3f(0, 0, -2.0f * m_scene_size)) * rigid_motion<float>(-scene_center);
	spincenter = m_camera_rm * m_scene_center;

}

void gl_camera::rotate( const math_matrix_3x3f & _rot ) {

	math_vector_3f center = m_camera_rm * m_scene_center;

	// apply rotation
	m_camera_rm = rigid_motion<float>(center) * rigid_motion<float>(_rot) *
	     rigid_motion<float>(-center) * m_camera_rm;

}

void gl_camera::pre_rotate( const math_matrix_3x3f & _rot ) {

	math_vector_3f center = m_scene_center;

	// apply rotation
	m_camera_rm = m_camera_rm * rigid_motion<float>(center) * rigid_motion<float>(_rot) *
	     rigid_motion<float>(-center);

}

void gl_camera::translate( const math_vector_3f & _t ) {
	m_camera_rm = rigid_motion<float>(_t) * m_camera_rm;
}

void gl_camera::clear(float r, float g, float b, float a)
{

	//glDisable( GL_TEXTURE_RECTANGLE_NV );
	glDisable( GL_TEXTURE_1D );
	glDisable( GL_TEXTURE_2D );
	//glDisable( GL_TEXTURE_3D );
	glDisable( GL_FOG );
	glDisable( GL_ALPHA_TEST );
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_BLEND );
	glDisable(GL_DITHER);
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_NORMALIZE);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);
	glDisable(GL_COLOR_MATERIAL);
	glDisable(GL_TEXTURE_2D);

	glEnable(GL_SCISSOR_TEST);
	glScissor(viewx, viewy, vieww, viewh); // set viewport for rendering
	glClearColor(r,g,b,a);
	glClearDepth(1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);
}
