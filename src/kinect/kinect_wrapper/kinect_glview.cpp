#include <pthread.h>
#include <vector>
#include <cmath>

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "libfreenect.h"
#include "kinect_glview.hpp"

using namespace std;

class Mutex {
	public:
		Mutex() {
			pthread_mutex_init( &m_mutex, NULL );
		}
		void lock() {
			pthread_mutex_lock( &m_mutex );
		}
		void unlock() {
			pthread_mutex_unlock( &m_mutex );
		}
	private:
		pthread_mutex_t m_mutex;
};

vector<uint8_t> m_buffer_depth(640*480*4);
vector<uint8_t> m_buffer_video(640*480*4);
vector<uint16_t> m_gamma(2048);

Mutex m_rgb_mutex;
Mutex m_depth_mutex;

bool m_new_rgb_frame;
bool m_new_depth_frame;

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

int got_frames(0);
int window(0);

ON_KEY_PRESSED on_key_pressed = NULL;
ON_DRAW_SCENE on_draw_scene = NULL;

void init(){
	m_new_rgb_frame = false;
	m_new_depth_frame = false;

	for( unsigned int i = 0 ; i < 2048 ; i++) {
		float v = i/2048.0;
		v = pow(v, 3)* 6;
		m_gamma[i] = v*6*256;
	}
}

void convertDepth11bToRGB(uint16_t* depth){
	for( unsigned int i = 0 ; i < 640*480 ; i++) {
		int pval = m_gamma[depth[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
		case 0:
			m_buffer_depth[3*i+0] = 255;
			m_buffer_depth[3*i+1] = 255-lb;
			m_buffer_depth[3*i+2] = 255-lb;
			break;
		case 1:
			m_buffer_depth[3*i+0] = 255;
			m_buffer_depth[3*i+1] = lb;
			m_buffer_depth[3*i+2] = 0;
			break;
		case 2:
			m_buffer_depth[3*i+0] = 255-lb;
			m_buffer_depth[3*i+1] = 255;
			m_buffer_depth[3*i+2] = 0;
			break;
		case 3:
			m_buffer_depth[3*i+0] = 0;
			m_buffer_depth[3*i+1] = 255;
			m_buffer_depth[3*i+2] = lb;
			break;
		case 4:
			m_buffer_depth[3*i+0] = 0;
			m_buffer_depth[3*i+1] = 255-lb;
			m_buffer_depth[3*i+2] = 255;
			break;
		case 5:
			m_buffer_depth[3*i+0] = 0;
			m_buffer_depth[3*i+1] = 0;
			m_buffer_depth[3*i+2] = 255-lb;
			break;
		default:
			m_buffer_depth[3*i+0] = 0;
			m_buffer_depth[3*i+1] = 0;
			m_buffer_depth[3*i+2] = 0;
			break;
		}
	}
}

void setKeyPressedHandler(ON_KEY_PRESSED handler) {
	on_key_pressed = handler;
}

void setDrawSceneHandler(ON_DRAW_SCENE handler) {
	on_draw_scene = handler;
}

void VideoCallback(int kinect_id, uint8_t* video, uint32_t timestamp, int size) {
	m_rgb_mutex.lock();
	copy(video, video+size, m_buffer_video.begin());
	m_new_rgb_frame = true;
	m_rgb_mutex.unlock();
}

void DepthCallback(int kinect_id, uint16_t* depth, uint32_t timestamp, int size) {
	m_depth_mutex.lock();
	convertDepth11bToRGB(depth);
	m_new_depth_frame = true;
	m_depth_mutex.unlock();
}

bool getVideo(vector<uint8_t> &buffer) {
	m_rgb_mutex.lock();
	if(m_new_rgb_frame) {
		buffer.swap(m_buffer_video);
		m_new_rgb_frame = false;
		m_rgb_mutex.unlock();
		return true;
	} else {
		m_rgb_mutex.unlock();
		return false;
	}
}
bool getDepth(vector<uint8_t> &buffer) {
	m_depth_mutex.lock();
	if(m_new_depth_frame) {
		buffer.swap(m_buffer_depth);
		m_new_depth_frame = false;
		m_depth_mutex.unlock();
		return true;
	} else {
		m_depth_mutex.unlock();
		return false;
	}
}

void keyPressed(unsigned char key, int x, int y)
{
	if (on_key_pressed)
		(*on_key_pressed)(key, x, y);

	if (key == 27) {
		glutDestroyWindow(window);
	}
}

void DrawGLScene()
{
	static std::vector<uint8_t> depth(640*480*4);
	static std::vector<uint8_t> video(640*480*4);

	if (on_draw_scene)
		(*on_draw_scene)();

	getDepth(depth);
	getVideo(video);

	got_frames = 0;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &depth[0]);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
	glTexCoord2f(0, 0); glVertex3f(0,0,0);
	glTexCoord2f(1, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 1); glVertex3f(640,480,0);
	glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &video[0]);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
	glTexCoord2f(0, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 0); glVertex3f(1280,0,0);
	glTexCoord2f(1, 1); glVertex3f(1280,480,0);
	glTexCoord2f(0, 1); glVertex3f(640,480,0);
	glEnd();

	glutSwapBuffers();
}

void ReSizeGLScene(int Width, int Height)
{
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 1280, 480, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

void InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);

	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glGenTextures(1, &gl_rgb_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	ReSizeGLScene(Width, Height);
}

void display(int *argc, char **argv){
	init();

	glutInit(argc, argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(1280, 480);
	glutInitWindowPosition(0, 0);

	window = glutCreateWindow("c++ wrapper example");

	glutDisplayFunc(&DrawGLScene);
	glutIdleFunc(&DrawGLScene);
	glutKeyboardFunc(&keyPressed);

	InitGL(1280, 480);

	glutMainLoop();
}

