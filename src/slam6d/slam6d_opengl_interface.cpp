#include "slam6d_opengl_interface.h"

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

// cuda functions
extern "C" void create_point_cloud(unsigned short* depth, unsigned char* color, float* posV,
	float* posC, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, float rgb_center_x, float rgb_center_y, double* Rmat, double* tvec);

// global variables
void *font = GLUT_BITMAP_8_BY_13;
bool mouseLeftDown;
bool mouseRightDown;
bool mouseMiddleDown;
float mouseX, mouseY;
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

	if(is_cuda_enable)
	{
		cudaGLSetGLDevice(device);
	}

	/* registra callback de saida do program. limpa os vertex object buffers */
	atexit(exitCB);

	cuda_enable = is_cuda_enable;

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

		if(cuda_enable)
		{
			cudaGLRegisterBufferObject(vboId);
		}
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

DLL_EXPORT void upload_depth_data_to_vbo_cuda_cloud(unsigned short* depth, unsigned char* color, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, float rgb_center_x, float rgb_center_y, double* Rmat, double* tvec)
{
	int vertice_size = pointCloudSize * pointCloudDim;
	int color_size = pointCloudSize * 3;
	int pos_vertice = 0;
	int pos_color = 0;

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

	// create point cloud with cuda
	create_point_cloud(depth, color, cuda_vbo_ptr + pos_vertice, cuda_vbo_ptr + pos_color, depth_width, depth_height, depth_focal_length_x, depth_focal_length_y, rgb_focal_length_x, rgb_focal_length_y, rgb_center_x, rgb_center_y, Rmat, tvec);

	// unbind cuda
	cudaGLUnmapBufferObject(vboId);

	pointCloudCount++;
}

DLL_EXPORT void upload_chain_depth_data_to_vbo_cuda_cloud(std::vector<carmen_slam6d_kinect_fused_t> keyframe_chain, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, float rgb_center_x, float rgb_center_y)
{
	int vertice_size = pointCloudSize * pointCloudDim;
	int color_size = pointCloudSize * 3;
	int pos_vertice = 0;
	int pos_color = 0;

	pointCloudCount = 0;
	pointCloudDisplayCount = 0;

	for(unsigned int i = 0; i < keyframe_chain.size(); i++)
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

		// create point cloud with cuda
		create_point_cloud(keyframe_chain[i].depth, keyframe_chain[i].image, cuda_vbo_ptr + pos_vertice,
				cuda_vbo_ptr + pos_color, depth_width, depth_height, depth_focal_length_x, depth_focal_length_y, rgb_focal_length_x, rgb_focal_length_y, rgb_center_x, rgb_center_y, keyframe_chain[i].rotation, keyframe_chain[i].position);

		// unbind cuda
		cudaGLUnmapBufferObject(vboId);

		pointCloudCount++;
	}
}

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

	if(cuda_enable)
		cudaGLUnregisterBufferObject(vboId);
}

DLL_EXPORT void cleare_global_variables()
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

    glClearColor(0, 0, 0, 0);                   // background color
    glClearStencil(0);                          // clear stencil buffer
    glClearDepth(1.0f);                         // 0 is near, 1 is far
    glDepthFunc(GL_LEQUAL);

    initLights();
    setCamera(0, 0, 5, 0, 0, 0);
}

///////////////////////////////////////////////////////////////////////////////
// write 2d text using GLUT
// The projection matrix must be set to orthogonal before call this function.
///////////////////////////////////////////////////////////////////////////////
void drawString(const char *str, int x, int y, float color[4], void *font)
{
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
    glDisable(GL_LIGHTING);     // need to disable lighting for proper text color

    glColor4fv(color);          // set text color
    glRasterPos2i(x, y);        // place text position

    // loop all characters in the string
    while(*str)
    {
        glutBitmapCharacter(font, *str);
        ++str;
    }

    glEnable(GL_LIGHTING);
    glPopAttrib();
}

///////////////////////////////////////////////////////////////////////////////
// draw a string in 3D space
///////////////////////////////////////////////////////////////////////////////
void drawString3D(const char *str, float pos[3], float color[4], void *font)
{
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
    glDisable(GL_LIGHTING);     // need to disable lighting for proper text color

    glColor4fv(color);          // set text color
    glRasterPos3fv(pos);        // place text position

    // loop all characters in the string
    while(*str)
    {
        glutBitmapCharacter(font, *str);
        ++str;
    }

    glEnable(GL_LIGHTING);
    glPopAttrib();
}

void drawAxes()
{
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

///////////////////////////////////////////////////////////////////////////////
// display info messages
///////////////////////////////////////////////////////////////////////////////
void showInfo()
{
    // backup current model-view matrix
    glPushMatrix();                     // save current modelview matrix
    glLoadIdentity();                   // reset modelview matrix

    // set to 2D orthogonal projection
    glMatrixMode(GL_PROJECTION);     // switch to projection matrix
    glPushMatrix();                  // save current projection matrix
    glLoadIdentity();                // reset projection matrix
    gluOrtho2D(0, 400, 0, 300);  // set to orthogonal projection

    float color[4] = {1, 1, 1, 1};

    stringstream ss;
    ss << "VBO: " << (vboUsed ? "on" : "off") << ends;  // add 0(ends) at the end
    drawString(ss.str().c_str(), 1, 286, color, font);
    ss.str(""); // clear buffer

    ss << "Press SPACE key to toggle VBO on/off." << ends;
    drawString(ss.str().c_str(), 1, 1, color, font);

    // restore projection matrix
    glPopMatrix();                   // restore to previous projection matrix

    // restore modelview matrix
    glMatrixMode(GL_MODELVIEW);      // switch to modelview matrix
    glPopMatrix();                   // restore to previous modelview matrix
}

//=============================================================================
// CALLBACKS
//=============================================================================

void displayCB()
{
    // clear buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // save the initial ModelView matrix before modifying ModelView matrix
    glPushMatrix();

    // tramsform camera
	glTranslatef(cameraTranslateX, cameraTranslateY, cameraDistance);
    glRotatef(cameraAngleX, 1, 0, 0);   // pitch
    glRotatef(cameraAngleY, 0, 1, 0);   // heading

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

    // draw info messages
    showInfo();

    glPopMatrix();

    glutSwapBuffers();
}

void reshapeCB(int w, int h)
{
    // set viewport to be the entire window
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    // set perspective viewing frustum
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //glFrustum(-aspectRatio, aspectRatio, -1, 1, 1, 100);
    gluPerspective(60.0f, (float)(w)/h, 1.0f, 10000.0f); // FOV, AspectRatio, NearClip, FarClip

    // switch to modelview matrix in order to set scene
    glMatrixMode(GL_MODELVIEW);
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
    switch(key)
    {
    case 27: // ESCAPE
		exitCB();
        exit(0);
        break;

    case ' ':
        if(vboSupported)
            vboUsed = !vboUsed;
        break;

    case 'd': // switch rendering modes (fill -> wire -> point)
    case 'D':
    	drawMode += 1;
        drawMode = drawMode % 3;
        if(drawMode == 0)        // fill mode
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_CULL_FACE);
        }
        else if(drawMode == 1)  // wireframe mode
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_CULL_FACE);
        }
        else                    // point mode
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_CULL_FACE);
        }
        break;

    default:
        ;
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
        }
        else if(state == GLUT_UP)
            mouseLeftDown = false;
    }

    else if(button == GLUT_RIGHT_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseRightDown = true;
        }
        else if(state == GLUT_UP)
            mouseRightDown = false;
    }

	else if(button == GLUT_MIDDLE_BUTTON)
	{
		if(state == GLUT_DOWN)
        {
            mouseMiddleDown = true;
        }
        else if(state == GLUT_UP)
            mouseMiddleDown = false;
	}
}

void mouseMotionCB(int x, int y)
{
    if(mouseLeftDown)
    {
        cameraAngleY += (x - mouseX);
        cameraAngleX += (y - mouseY);
        mouseX = x;
        mouseY = y;
    }

    if(mouseRightDown)
    {
        cameraDistance += (y - mouseY) * 0.2f;
        mouseY = y;
    }

	if(mouseMiddleDown)
	{
		cameraTranslateX += (x - mouseX) * 0.2f;
		cameraTranslateY += -(y - mouseY) * 0.2f;

		mouseX = x;
		mouseY = y;
	}

    //glutPostRedisplay();
}

void exitCB()
{
    cleare_global_variables();
}
