#include <carmen/carmen.h>
#include "viewer_3D.h"
#include "Window.h"


int
processWindow (window* w, void (*mouseFunc)(int type, int button, int x, int y), void (*keyPress)(int code), void (*keyRelease)(int code), void (*resizeFunc)(int width, int height))
{
    XEvent event;

    while (XPending (w->g_pDisplay) > 0)
    {
        XNextEvent (w->g_pDisplay, &event);

        switch (event.type)
        {
        case KeyPress:
        {
            int code = event.xbutton.button;

            keyPress (code);
        }
        break;

        case KeyRelease:
        {
            int code = event.xbutton.button;

            keyRelease (code);
        }
        break;

        case ButtonPress:
        case ButtonRelease:
        {
            XButtonEvent* mouseEvent = (XButtonEvent*) (&event);

            int type = mouseEvent->type;
            int button = mouseEvent->button;
            int x = mouseEvent->x;
            int y = mouseEvent->y;

            mouseFunc (type, button, x, y);
        }
        break;

        case MotionNotify:
        {
            int type = 0;
            int button = 0;
            int x = event.xmotion.x;
            int y = event.xmotion.y;

            mouseFunc (type, button, x, y);
        }
        break;

        case ConfigureNotify:
        {
            glViewport (0, 0, event.xconfigure.width, event.xconfigure.height);
            resizeFunc(event.xconfigure.width, event.xconfigure.height);
        }
        break;

        case DestroyNotify:
        {
            return 0;
        }
        break;

        case ClientMessage:
        {
            if ((unsigned int) event.xclient.data.l[0] == w->wmDeleteMessage)
            {
                XDestroyWindow (w->g_pDisplay, event.xclient.window);
                return 0;
            }
        }
        break;

        }

    }

    return 1;

}

int
showWindow (window* w)
{
    if (w->g_bDoubleBuffered)
        glXSwapBuffers (w->g_pDisplay, w->g_window); // Buffer swap does implicit glFlush
    else
        glFlush ();

    return 1;
}

window*
initWindow (int width, int height)
{
    //int tela;

    window* w = (window*) malloc (sizeof (window));

    XSetWindowAttributes windowAttributes;
    XVisualInfo *visualInfo = NULL;
    Colormap colorMap;
    GLXContext glxContext;
    int errorBase;
    int eventBase;

    w->g_bDoubleBuffered = GL_TRUE;

    // Open a connection to the X server
    w->g_pDisplay = XOpenDisplay (NULL);

    if (w->g_pDisplay == NULL)
    {
        fprintf (stderr, "glxsimple: %s\n", "could not open display");
        exit (1);
    }

    // Make sure OpenGL's GLX extension supported
    if (!glXQueryExtension (w->g_pDisplay, &errorBase, &eventBase))
    {
        fprintf (stderr, "glxsimple: %s\n", "X server has no OpenGL GLX extension");
        exit (1);
    }

    //tela = DefaultScreen(w->g_pDisplay);

    //int res_x = DisplayWidth(w->g_pDisplay, tela);
    //int	res_y = DisplayHeight(w->g_pDisplay, tela);

    //printf("Resolucao do sistema: %dx%d\n", res_x, res_y);

    // Find an appropriate visual

    int doubleBufferVisual[] = {
        GLX_RGBA, // Needs to support OpenGL
        GLX_DEPTH_SIZE, 16, // Needs to support a 16 bit depth buffer
        GLX_DOUBLEBUFFER, // Needs to support double-buffering
        None // end of list
    };

    int singleBufferVisual[] = {
        GLX_RGBA, // Needs to support OpenGL
        GLX_DEPTH_SIZE, 16, // Needs to support a 16 bit depth buffer
        None // end of list
    };

    // Try for the double-bufferd visual first
    visualInfo = glXChooseVisual (w->g_pDisplay, DefaultScreen (w->g_pDisplay), doubleBufferVisual);

    if (visualInfo == NULL)
    {
        // If we can't find a double-bufferd visual, try for a single-buffered visual...
        visualInfo = glXChooseVisual (w->g_pDisplay, DefaultScreen (w->g_pDisplay), singleBufferVisual);

        if (visualInfo == NULL)
        {
            fprintf (stderr, "glxsimple: %s\n", "no RGB visual with depth buffer");
            exit (1);
        }

        w->g_bDoubleBuffered = 0;
    }

    // Create an OpenGL rendering context
    glxContext = glXCreateContext (w->g_pDisplay,
                                   visualInfo,
                                   NULL, // No sharing of display lists
                                   GL_TRUE); // Direct rendering if possible

    if (glxContext == NULL)
    {
        fprintf (stderr, "glxsimple: %s\n", "could not create rendering context");
        exit (1);
    }

    // Create an X colormap since we're probably not using the default visual
    colorMap = XCreateColormap (w->g_pDisplay,
                                RootWindow (w->g_pDisplay, visualInfo->screen),
                                visualInfo->visual,
                                AllocNone);

    w->wmDeleteMessage = XInternAtom (w->g_pDisplay, "WM_DELETE_WINDOW", False);


    windowAttributes.colormap = colorMap;
    windowAttributes.border_pixel = 0;
    windowAttributes.event_mask = ExposureMask |
            VisibilityChangeMask |
            KeyPressMask |
            KeyReleaseMask |
            ButtonPressMask |
            ButtonReleaseMask |
            PointerMotionMask |
            StructureNotifyMask |
            SubstructureNotifyMask |
            FocusChangeMask;

    // Create an X window with the selected visual
    w->g_window = XCreateWindow (w->g_pDisplay,
                                 RootWindow (w->g_pDisplay, visualInfo->screen),
                                 0, 0, // x/y position of top-left outside corner of the window
								 width, height, // Width and height of window
                                 0, // Border width
                                 visualInfo->depth,
                                 InputOutput,
                                 visualInfo->visual,
                                 CWBorderPixel | CWColormap | CWEventMask,
                                 &windowAttributes);

    XSetStandardProperties (w->g_pDisplay,
                            w->g_window,
                            "Viewer 3D",
                            "Viewer",
                            None,
                            NULL,
                            0,
                            NULL);

    // Bind the rendering context to the window
    glXMakeCurrent (w->g_pDisplay, w->g_window, glxContext);

    // Request the X window to be displayed on the screen
    XMapWindow (w->g_pDisplay, w->g_window);
    XSetWMProtocols (w->g_pDisplay, w->g_window, &(w->wmDeleteMessage), 1);

    XFree(visualInfo);

    return w;
}

void
destroyWindow (window* w)
{
    free (w);
}
