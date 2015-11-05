#include "voslam_opengl_interface.h"

using std::stringstream;
using std::cout;
using std::endl;
using std::ends;

// GLUT CALLBACK functions
void displayCB();
void reshapeCB(int w, int h);
void timerCB(int millisec);
void idleCB();
void keyboardCB(unsigned char key, int x, int y);
void mouseCB(int button, int stat, int x, int y);
void mouseMotionCB(int x, int y);
void exitCB();

#ifndef NO_CUDA
// cuda functions
extern "C" void create_point_cloud(unsigned short* depth, unsigned char* color, float* posV,
	float* posC, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, float rgb_center_x, float rgb_center_y, double* Rmat, double* tvec);

extern "C" void copy_point_cloud(pcl::PointXYZRGB *pointcloud, float* posV, float *posC, int width, int height, int size);

#endif

CCamera Camera;


// global variables
void *font = GLUT_BITMAP_8_BY_13;
bool mouseLeftDown;
bool mouseRightDown;
bool mouseMiddleDown;
int mouseX, mouseY;
float cameraAngleX;
float cameraAngleY;
float cameraDistance;
float cameraTranslateX;
float cameraTranslateY;
bool vboSupported, vboUsed;
int drawMode = 0;
int pointCloudCount = 0;
int pointCloudNumber = 0;
int pointCloudSize = 0;
int pointCloudDim = 0;
int pointCloudDisplayCount = 0;
int cuda_enable = 0;

void (*external_callback_g)(int) = NULL;

// ID of VBO for vertex arrays
GLuint vboId = 0;
// vertex coords array
GLfloat *vertices = NULL;
// color array
GLfloat *colors = NULL;
// cuda vbo pointer
float *cuda_vbo_ptr = NULL;

int p = 0;

std::vector<carmen_voslam_pointcloud_t> global_keyframe_chain;

DLL_EXPORT void initialize_point_cloud_viewer(int argc, char **argv, int point_cloud_size, int point_cloud_dim, int number_of_point_clouds, void (*handler_t)(int), int is_cuda_enable, int device)
{
	/* aloca memoria RAM para todos os pontos das nuvens do visualizador*/
	if(vertices == NULL)
		vertices = (GLfloat*) malloc (point_cloud_size * point_cloud_dim * sizeof(GLfloat));

	/* aloca memoria RAM para as cores de cada pixel da nuvem de pontos (RGB)*/
	if(colors == NULL)
		colors = (GLfloat*) malloc (point_cloud_size * 3 * sizeof(GLfloat));

	/* inicializa variaveis globais da interface grafica */
	init_global_variables(number_of_point_clouds, point_cloud_size, point_cloud_dim);

	/* inicializa a glut e a janela de exibicao do opengl. registro das funcoes de callback de desenho e controle da interface */

	init_glut(argc, argv, handler_t);
	init_gl();


#ifndef NO_CUDA
	if(is_cuda_enable)
	{
		cudaGLSetGLDevice(device);
	}

	cuda_enable = is_cuda_enable;

#endif

	/* registra callback de saida do program. limpa os vertex object buffers */
	atexit(exitCB);

	vboSupported = true;
	vboUsed = true;
}

DLL_EXPORT void allocate_vbo_arrays()
{
	if(vboSupported)
    {
		long int vertices_size = pointCloudSize * pointCloudDim * pointCloudNumber * sizeof(GLfloat);
		long int colors_size = pointCloudSize * 3 * pointCloudNumber * sizeof(GLfloat);

		/* cria um novo VBO array na placa grafica */
        glGenBuffersARB(1, &vboId);

		/* associa o array VBO criado a sua ID antes de usa-lo */
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, vboId);

		/* aloca na placa grafica memoria para a nuvem de ponto mais a cor de cada ponto*/
		glBufferDataARB(GL_ARRAY_BUFFER_ARB, vertices_size + colors_size, 0, GL_DYNAMIC_DRAW_ARB);

		glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

#ifndef NO_CUDA
		if(cuda_enable)
		{
			cudaGLRegisterBufferObject(vboId);
		}
#endif

    }

	return;
}

DLL_EXPORT void upload_data_to_vbo_array()
{

    int vertice_size = pointCloudSize * pointCloudDim * sizeof(GLfloat);
	int color_size = pointCloudSize * 3 * sizeof(GLfloat);

	if(pointCloudCount == pointCloudNumber)
	{
		pointCloudCount = 0;
	}
	else if(pointCloudDisplayCount < (pointCloudSize * pointCloudNumber))
		pointCloudDisplayCount+=pointCloudSize;

	glBindBufferARB(GL_ARRAY_BUFFER_ARB, vboId);

	/* copia os dados dos pontos da nuvem para o inicio do VBO */
	glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, pointCloudCount * vertice_size, vertice_size, vertices);

	/* copia dos dados de cor da nuvem apos o vetor de pontos*/
	glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, vertice_size * pointCloudNumber + (pointCloudCount * color_size), color_size, colors);

	glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

	pointCloudCount++;
}

#ifndef NO_CUDA

DLL_EXPORT void upload_chain_depth_data_to_vbo_cuda_cloud(std::vector<carmen_voslam_pointcloud_t> keyframe_chain, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, float rgb_center_x, float rgb_center_y)
{
	int vertice_size = pointCloudSize * pointCloudDim;
	int color_size = pointCloudSize * 3;
	int pos_vertice = 0;
	int pos_color = 0;
	int start;

	double Rmat[9];
	double tvec[3];

	pointCloudCount = 0;
	pointCloudDisplayCount = 0;

	global_keyframe_chain = keyframe_chain;

	start = keyframe_chain.size() - pointCloudNumber;
	if(start < 0)
		start = 0;

	for(unsigned int i = start; i < keyframe_chain.size(); i++)
	{
		if(pointCloudCount == pointCloudNumber)
		{
			pointCloudCount = 0;
		}
		else if(pointCloudDisplayCount < (pointCloudSize * pointCloudNumber))
			pointCloudDisplayCount+=pointCloudSize;

		// bind vbo to cuda memory
		cudaGLMapBufferObject((void**)&cuda_vbo_ptr, vboId);

		// compute vertice and color position on vbo
		pos_vertice = pointCloudCount * vertice_size;
		pos_color = vertice_size * pointCloudNumber + (pointCloudCount * color_size);

		tvec[0] = keyframe_chain[i].pose.position[0];
		tvec[1] = keyframe_chain[i].pose.position[1];
		tvec[2] = keyframe_chain[i].pose.position[2];

		Rmat[0] = keyframe_chain[i].pose.rotation[0][0];
		Rmat[1] = keyframe_chain[i].pose.rotation[0][1];
		Rmat[2] = keyframe_chain[i].pose.rotation[0][2];
		Rmat[3] = keyframe_chain[i].pose.rotation[1][0];
		Rmat[4] = keyframe_chain[i].pose.rotation[1][1];
		Rmat[5] = keyframe_chain[i].pose.rotation[1][2];
		Rmat[6] = keyframe_chain[i].pose.rotation[2][0];
		Rmat[7] = keyframe_chain[i].pose.rotation[2][1];
		Rmat[8] = keyframe_chain[i].pose.rotation[2][2];

		// create point cloud with cuda
		create_point_cloud(keyframe_chain[i].depth, keyframe_chain[i].image, cuda_vbo_ptr + pos_vertice,
				cuda_vbo_ptr + pos_color, depth_width, depth_height, depth_focal_length_x, depth_focal_length_y, rgb_focal_length_x, rgb_focal_length_y, rgb_center_x, rgb_center_y, Rmat, tvec);

		// unbind cuda
		cudaGLUnmapBufferObject(vboId);

		pointCloudCount++;
	}
}

#endif

DLL_EXPORT void copy_data_to_vertices(float* v)
{
	memcpy(vertices, v, pointCloudSize * pointCloudDim * sizeof(GLfloat));
}

DLL_EXPORT void copy_data_to_colors(float* c)
{
	memcpy(colors, c, pointCloudSize * 3 * sizeof(GLfloat));
}

DLL_EXPORT void run_glut()
{
    glutMainLoop();
    return;
}

///////////////////////////////////////////////////////////////////////////////
// destroy a VBO
// If VBO id is not valid or zero, then OpenGL ignores it silently.
///////////////////////////////////////////////////////////////////////////////
void delete_vbo(GLuint vboId)
{
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, vboId);
	glDeleteBuffersARB(1, &vboId);

#ifndef NO_CUDA
	if(cuda_enable)
		cudaGLUnregisterBufferObject(vboId);
#endif
}

DLL_EXPORT void clear_global_variables()
{
    if(vboSupported)
    {
        delete_vbo(vboId);
    }
}

///////////////////////////////////////////////////////////////////////////////
// initialize GLUT for windowing
///////////////////////////////////////////////////////////////////////////////
int init_glut(int argc, char **argv, void (*external_callback)(int))
{
    // GLUT stuff for windowing
    // initialization openGL window.
    // it is called before any other GLUT routine
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);   // display mode

    glutInitWindowSize(640, 480);               // window size

    glutInitWindowPosition(100, 100);           // window location

    // finally, create a window with openGL context
    // Window will not displayed until glutMainLoop() is called
    // it returns a unique ID
    int handle = glutCreateWindow(argv[0]);     // param is the title of window

    // register GLUT callback functions
    glutDisplayFunc(displayCB);
    external_callback_g = external_callback;
    glutTimerFunc(30, timerCB, 30);             // redraw only every given millisec
    glutReshapeFunc(reshapeCB);
    glutKeyboardFunc(keyboardCB);
    glutMouseFunc(mouseCB);
    glutMotionFunc(mouseMotionCB);

    return handle;
}

///////////////////////////////////////////////////////////////////////////////
// initialize OpenGL
// disable unused features
///////////////////////////////////////////////////////////////////////////////
void init_gl()
{
    glShadeModel(GL_SMOOTH);                    // shading mathod: GL_SMOOTH or GL_FLAT
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);      // 4-byte pixel alignment

    // enable /disable features
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    //glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_CULL_FACE);

     // track material ambient and diffuse from surface color, call it before glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glClearColor(1.0, 1.0, 1.0, 0);                   // background color
    glClearStencil(0);                          // clear stencil buffer
    glClearDepth(1.0f);                         // 0 is near, 1 is far
    glDepthFunc(GL_LEQUAL);

    initLights();
//
	Camera.Move(F3dVector(-5.0, 3.0, 0.0 ));
    Camera.RotateY(-90);
}

void drawAxes()
{
	glPushMatrix();

	glLineWidth(2.0);

	glColor3d(1.0, 0.0, 0.0);

	glBegin(GL_LINES);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(1.0, 0.0, 0.0);
	glEnd();

	glColor3d(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(0.0, 1.0, 0.0);
	glEnd();

	glColor3d(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(0.0, 0.0, 1.0);
	glEnd();

	glLineWidth(1.0);

	glPopMatrix();
}

double x, y, z;
double roll, pitch, yaw;

void drawRobot()
{
	int k;
	glPushMatrix();
	glLineWidth(2.0);
	glPointSize(2.0);

	for(int i = 1; i < global_keyframe_chain.size(); i++)
	{

		glColor3d(1.0, 0.0, i / 100.0);
		glBegin(GL_LINES);
		glVertex3f(global_keyframe_chain[i].pose.position[0], global_keyframe_chain[i].pose.position[1], global_keyframe_chain[i].pose.position[2]);
		glVertex3f(global_keyframe_chain[i-1].pose.position[0], global_keyframe_chain[i-1].pose.position[1], global_keyframe_chain[i-1].pose.position[2]);
		glEnd();

		for(int j = 0; j < global_keyframe_chain[i].loop_partners_indexes.size(); j++)
		{
			k = global_keyframe_chain[i].loop_partners_indexes[j];

			glColor3d(0.0, 1.0, 0.0);
			glBegin(GL_LINES);
			glVertex3f(global_keyframe_chain[i].pose.position[0], global_keyframe_chain[i].pose.position[1], global_keyframe_chain[i].pose.position[2]);
			glVertex3f(global_keyframe_chain[k].pose.position[0], global_keyframe_chain[k].pose.position[1], global_keyframe_chain[k].pose.position[2]);
			glEnd();
		}
	}

	glPopMatrix();
	glLineWidth(1.0);
	glPointSize(1.0);
}

///////////////////////////////////////////////////////////////////////////////
// initialize global variables
///////////////////////////////////////////////////////////////////////////////
bool init_global_variables(int number_of_point_clouds, int point_cloud_size, int point_cloud_dim)
{
    mouseLeftDown = mouseRightDown = false;

	pointCloudNumber = number_of_point_clouds;
	pointCloudSize = point_cloud_size;
	pointCloudDim = point_cloud_dim;
    return true;
}

///////////////////////////////////////////////////////////////////////////////
// initialize lights
///////////////////////////////////////////////////////////////////////////////
void initLights()
{
    // set up light colors (ambient, diffuse, specular)
    GLfloat lightKa[] = {.8f, .8f, .8f, 1.0f};  // ambient light
    GLfloat lightKd[] = {.7f, .7f, .7f, 1.0f};  // diffuse light
    GLfloat lightKs[] = {1, 1, 1, 1};           // specular light

	glLightfv(GL_LIGHT0, GL_AMBIENT, lightKa);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightKd);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightKs);

    // position the light
    float lightPos[4] = {0, 0, 20, 1}; // positional light
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    glEnable(GL_LIGHT0);                        // MUST enable each light source after configuration
}

///////////////////////////////////////////////////////////////////////////////
// set camera position and lookat direction
///////////////////////////////////////////////////////////////////////////////
void setCamera(float posX, float posY, float posZ, float targetX, float targetY, float targetZ)
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(posX, posY, posZ, targetX, targetY, targetZ, 0, 1, 0); // eye(x,y,z), focal(x,y,z), up(x,y,z)
}

//=============================================================================
// CALLBACKS
//=============================================================================

void displayCB()
{
    // clear buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glLoadIdentity();

    Camera.Render();

    if(vboUsed)
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, vboId);

        glEnableClientState(GL_COLOR_ARRAY);
        glEnableClientState(GL_VERTEX_ARRAY);

		glColorPointer(3, GL_FLOAT, 0, (void*)((char*)NULL + (pointCloudSize * pointCloudNumber * pointCloudDim * sizeof(GLfloat))));
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glDrawArrays(GL_POINTS, 0, pointCloudDisplayCount);

        glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
        glDisableClientState(GL_COLOR_ARRAY);

        glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
    }
    else
    {
        glEnableClientState(GL_COLOR_ARRAY);
        glEnableClientState(GL_VERTEX_ARRAY);

        glColorPointer(3, GL_FLOAT, 0, colors);
        glVertexPointer(3, GL_FLOAT, 0, vertices);

        glDrawArrays(GL_POINTS, 0, pointCloudDisplayCount);

        glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
        glDisableClientState(GL_COLOR_ARRAY);
    }

    //draw axes
    drawAxes();

    drawRobot();

    glFlush();
    glutSwapBuffers();
}

void reshapeCB(int w, int h)
{
	if (h == 0 || w == 0) return;  //Nothing is visible then, so return

	//Set a new projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//Angle of view:40 degrees
	//Near clipping plane distance: 0.5
	//Far clipping plane distance: 20.0
	gluPerspective(60.0,(GLdouble)w/(GLdouble)h, 1.0f, 10000.0f);

	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, w, h);  //Use the whole window for rendering
}

void timerCB(int millisec)
{
    glutTimerFunc(millisec, timerCB, millisec);

    if(external_callback_g != NULL)
    	glutTimerFunc(1, external_callback_g, 0);

    glutPostRedisplay();
}

void idleCB()
{
    glutPostRedisplay();
}

void keyboardCB(unsigned char key, int x, int y)
{
	switch (key)
		{
		case 27:		//ESC
			break;
		case 'a':
			Camera.RotateY(5.0);
			break;
		case 'd':
			Camera.RotateY(-5.0);
			break;
		case 'w':
			Camera.MoveForward( -0.3 ) ;
			break;
		case 's':
			Camera.MoveForward( 0.3 ) ;
			break;
		case 'x':
			Camera.RotateX(5.0);
			break;
		case 'y':
			Camera.RotateX(-5.0);
			break;
		case 'c':
			Camera.StrafeRight(-0.3);
			break;
		case 'v':
			Camera.StrafeRight(0.3);
			break;
		case 'f':
			Camera.MoveUpward(-0.3);
			break;
		case 'r':
			Camera.MoveUpward(0.3);
			break;

		case 'm':
			Camera.RotateZ(-5.0);
			break;
		case 'n':
			Camera.RotateZ(5.0);
			break;
		}

	glutPostRedisplay();
}

void mouseCB(int button, int state, int x, int y)
{
    mouseX = x;
    mouseY = y;

    if(button == GLUT_LEFT_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseLeftDown = true;
            mouseX = mouseY = -1;
        }
        else if(state == GLUT_UP)
            mouseLeftDown = false;
    }

    else if(button == GLUT_RIGHT_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseRightDown = true;
            mouseX = mouseY = -1;
        }
        else if(state == GLUT_UP)
            mouseRightDown = false;
    }

	else if(button == GLUT_MIDDLE_BUTTON)
	{
		if(state == GLUT_DOWN)
        {
            mouseMiddleDown = true;
            mouseX = mouseY = -1;
        }
        else if(state == GLUT_UP)
            mouseMiddleDown = false;
	}
}

void mouseMotionCB(int x, int y)
{
    if(mouseLeftDown)
    {
    	if(mouseX != -1 && mouseY != -1)
		{
			cameraTranslateX = -(x - mouseX) * 0.15f;
			cameraTranslateY = -(y - mouseY) * 0.15f;

			Camera.MoveForward(cameraTranslateY);
			Camera.StrafeRight(cameraTranslateX);
		}

        mouseX = x;
        mouseY = y;
    }

    if(mouseRightDown)
    {
        cameraDistance += (y - mouseY) * 0.1f;
        mouseY = y;
    }

	if(mouseMiddleDown)
	{
		if(mouseX != -1 && mouseY != -1)
		{
			cameraAngleX = (x - mouseX) * 0.5;
			cameraAngleY = (y - mouseY) * 0.5;

			//Camera.RotateX(cameraAngleY);
			Camera.RotateY(cameraAngleX);

		}

		mouseX = x;
		mouseY = y;
	}

    glutPostRedisplay();
}

void exitCB()
{
    clear_global_variables();
}
