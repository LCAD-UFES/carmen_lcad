#include <carmen/carmen.h>

#include "viewer_3D.h"

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "interface_drawer.h"

struct button {
    double x, y;
    double width, height;

    int mouse_over;
    int visible;
    int state;
    int code;

    const char* text;
};
typedef struct button button;

struct interface_drawer {
    button* butt;
    int num_buttons;
};

static void init_buttons(interface_drawer* i_drawer);
static void handle_mouse_movement(interface_drawer* i_drawer, int x, int y);
static void handle_mouse_left_click(interface_drawer* i_drawer, int x, int y);
static void handle_mouse_right_click(interface_drawer* i_drawer, int x, int y);
static int test_mouse_over_button(button b, int x, int y);
static void draw_button(button b);
static void drawText(float x, float y, const char* msg, ...);

interface_drawer*
create_interface_drawer(void) {
    interface_drawer* i_drawer = (interface_drawer*) malloc(sizeof (interface_drawer));

    i_drawer->num_buttons = 56;
    i_drawer->butt = (button*) malloc(i_drawer->num_buttons * sizeof (button));

    init_buttons(i_drawer);

    return i_drawer;
}

void
destroy_interface_drawer(interface_drawer* i_drawer) {
    free(i_drawer);
}

static void
init_buttons(interface_drawer* i_drawer) {
    int i;
    for (i = 0; i < i_drawer->num_buttons; i++) {
        i_drawer->butt[i].code = i;
        i_drawer->butt[i].mouse_over = 0;
        i_drawer->butt[i].state = 0;
        i_drawer->butt[i].text = "\0";

        if (i < 8) {
            i_drawer->butt[i].x = 80 + i * 120;
            i_drawer->butt[i].y = 30;
            i_drawer->butt[i].width = 100;
            i_drawer->butt[i].height = 20;

            i_drawer->butt[i].visible = 0;
        } else if (i < 16) {
            i_drawer->butt[i].x = 80 + (i - 8)*120;
            i_drawer->butt[i].y = 60;
            i_drawer->butt[i].width = 100;
            i_drawer->butt[i].height = 20;

            i_drawer->butt[i].visible = 0;
        } else if (i < 24) {
            i_drawer->butt[i].x = 80 + (i - 16) * 120;
            i_drawer->butt[i].y = 90;
            i_drawer->butt[i].width = 100;
            i_drawer->butt[i].height = 20;

            i_drawer->butt[i].visible = 0;
        } else if (i < 32) {
            i_drawer->butt[i].x = 80 + (i - 24) * 120;
            i_drawer->butt[i].y = 120;
            i_drawer->butt[i].width = 100;
            i_drawer->butt[i].height = 20;

            i_drawer->butt[i].visible = 0;
        } else if (i < 40) {
            i_drawer->butt[i].x = 80 + (i - 32) * 120;
            i_drawer->butt[i].y = 60;
            i_drawer->butt[i].width = 100;
            i_drawer->butt[i].height = 20;

            i_drawer->butt[i].visible = 0;
        } else if (i < 48) {
            i_drawer->butt[i].x = 80 + (i - 40) * 120;
            i_drawer->butt[i].y = 90;
            i_drawer->butt[i].width = 100;
            i_drawer->butt[i].height = 20;

            i_drawer->butt[i].visible = 0;
        } else if (i < 56) {
            i_drawer->butt[i].x = 80 + (i - 48) * 120;
            i_drawer->butt[i].y = 90;
            i_drawer->butt[i].width = 100;
            i_drawer->butt[i].height = 20;

            i_drawer->butt[i].visible = 0;
        }
        if (i == 0) {
            i_drawer->butt[i].visible = 1;
        }
    }

    i_drawer->butt[0].text = "Options";
    i_drawer->butt[1].text = "Car";
    i_drawer->butt[2].text = "Particles";
    i_drawer->butt[3].text = "Fused Odometry";
    i_drawer->butt[4].text = "GPS XSENS";
    i_drawer->butt[5].text = "Follow Car";
    i_drawer->butt[6].text = "GPS";
    i_drawer->butt[7].text = "Google Image";

    i_drawer->butt[8].text = "Velodyne 360";
    i_drawer->butt[9].text = "Velodyne VBO";
    i_drawer->butt[10].text = "Velodyne";
    i_drawer->butt[11].text = "Var Velodyne";
    i_drawer->butt[12].text = "Map";
    i_drawer->butt[13].text = "Annotation";
    i_drawer->butt[14].text = "SICK";
    i_drawer->butt[15].text = "Show Rays";

    i_drawer->butt[16].text = "XSENS Axis";
    i_drawer->butt[17].text = "Global Pos";
    i_drawer->butt[18].text = "Intensity Vldyn";
    i_drawer->butt[19].text = "Motion Plan";
    i_drawer->butt[20].text = "Obstacle Plan";
    i_drawer->butt[21].text = "Trajectory Plan";
    i_drawer->butt[22].text = "Moving Objects";
    i_drawer->butt[23].text = "GPS Axis";

    i_drawer->butt[24].text = "Neural Localizer";
    i_drawer->butt[25].text = "Vldyn Remission";
    i_drawer->butt[26].text = "Empty";
    i_drawer->butt[27].text = "Empty";
    i_drawer->butt[28].text = "Empty";
    i_drawer->butt[29].text = "Empty";
    i_drawer->butt[30].text = "Empty";
    i_drawer->butt[31].text = "Empty";

    //Annotations
    i_drawer->butt[32].text = "Traffic Light";
    i_drawer->butt[33].text = "Traffic Signal";
    i_drawer->butt[34].text = "Pedestrian Track";
    i_drawer->butt[35].text = "Stop";
    i_drawer->butt[36].text = "Barrier";
    i_drawer->butt[37].text = "Bump";
    i_drawer->butt[38].text = "Speed";
    i_drawer->butt[39].text = "Delete";
    //Speed Annotations
    i_drawer->butt[40].text = "0 km/h";
    i_drawer->butt[41].text = "5 km/h";
    i_drawer->butt[42].text = "10 km/h";
    i_drawer->butt[43].text = "15 km/h";
    i_drawer->butt[44].text = "20 km/h";
    i_drawer->butt[45].text = "30 km/h";
    i_drawer->butt[46].text = "40 km/h";
    i_drawer->butt[47].text = "60 km/h";
    //Traffic Signals Annotations
    i_drawer->butt[48].text = "Bump";
    i_drawer->butt[49].text = "20 km/h";
    i_drawer->butt[50].text = "30 km/h";
    i_drawer->butt[51].text = "Empty";
    i_drawer->butt[52].text = "Empty";
    i_drawer->butt[53].text = "Empty";
    i_drawer->butt[54].text = "Empty";
    i_drawer->butt[55].text = "Empty";

}

void
interface_mouse_func(interface_drawer* i_drawer, int type, int button, int x, int y) {
    y = 600 - y;

    //printf("mouse - type: %d, button: %d, x: %d, y: %d\n", type, button, x, y);
    if (type == 0 && button == 0) // Mouse movement
    {
        handle_mouse_movement(i_drawer, x, y);
    } else if (type == 5 && button == 1) // Left Click
    {
        handle_mouse_left_click(i_drawer, x, y);
    } else if (type == 5 && button == 3) // Right Click
    {
        handle_mouse_right_click(i_drawer, x, y);
    }
}

static void
handle_mouse_movement(interface_drawer* i_drawer, int x, int y) {
    int i;
    for (i = 0; i < i_drawer->num_buttons; i++) {
        if (i_drawer->butt[i].visible) {
            i_drawer->butt[i].mouse_over = test_mouse_over_button(i_drawer->butt[i], x, y);
        }
    }
}

static void
handle_mouse_left_click(interface_drawer* i_drawer, int x, int y) {
    int i;
    for (i = 0; i < i_drawer->num_buttons; i++) {
        if (i_drawer->butt[i].visible) {
            if (test_mouse_over_button(i_drawer->butt[i], x, y)) {
                if (i_drawer->butt[i].code == 0) // option
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    int j;
                    for (j = 1; j < 32; j++) {
                        i_drawer->butt[j].visible = !i_drawer->butt[j].visible;
                    }

                    if (i_drawer->butt[13].state) {
                        i_drawer->butt[13].state = !(i_drawer->butt[13].state);
                        set_flag_viewer_3D(19, i_drawer->butt[13].state);
                        for (j = 32; j < 56; j++) {
                            i_drawer->butt[j].visible = 0;
                        }
                    }
                } else if (i_drawer->butt[i].code == 1) // draw car
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(4, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 2) // draw particles
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(0, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 3) // draw odometry
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(9, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 4) // draw gps xsens
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(10, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 5) // follow car
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(11, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 6) // draw gps
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(8, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 7) // draw map image
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(6, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 8) // velodyne 360
                {
                    if (i_drawer->butt[i].state == 0)
                        i_drawer->butt[i].state = 3;
                    else
                        i_drawer->butt[i].state = 0;

                    set_flag_viewer_3D(2, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 9) // velodyne VBO
                {
                    if (i_drawer->butt[i].state == 0)
                        i_drawer->butt[i].state = 2;
                    else
                        i_drawer->butt[i].state = 0;

                    set_flag_viewer_3D(2, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 10) // velodyne
                {
                    if (i_drawer->butt[i].state == 0)
                        i_drawer->butt[i].state = 1;
                    else
                        i_drawer->butt[i].state = 0;

                    set_flag_viewer_3D(2, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 11) // variable velodyne
                {
//                    if (i_drawer->butt[i].state == 0)
//                        i_drawer->butt[i].state = 4;
//                    else
//                        i_drawer->butt[i].state = 0;

                	i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(3, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 12) // MAP
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(12, i_drawer->butt[i].state);
                    set_flag_viewer_3D(13, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 13) // Annotation
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);
                    set_flag_viewer_3D(19, i_drawer->butt[i].state);


                    if (i_drawer->butt[38].state == 1) {
                        for (int j = 40; j < 48; j++) {
                            i_drawer->butt[j].visible = 0;
                        }
                    }
                    if (i_drawer->butt[37].state == 1) {
                        for (int j = 48; j < 56; j++) {
                            i_drawer->butt[j].visible = 0;
                        }
                    }

                    for (int j = 32; j < 40; j++) {
                        i_drawer->butt[j].visible = i_drawer->butt[i].state;
                    }
                    for (int j = 1; j < 32; j++) {
                        i_drawer->butt[j].visible = !i_drawer->butt[i].state;
                    }

                } else if (i_drawer->butt[i].code == 14) // SICK
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(1, i_drawer->butt[i].state);
                    set_flag_viewer_3D(13, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 15) // SICK Rays
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(5, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 16) // XSENS Orientation
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(15, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 17) // Localize Ackerman
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(16, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 18) // Velodyne Intensity
                {
                    if (i_drawer->butt[i].state == 0)
                        i_drawer->butt[i].state = 5;
                    else
                        i_drawer->butt[i].state = 0;

                    set_flag_viewer_3D(2, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 19) // Trajectory Plan
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(17, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 20) // Trajectory Plan
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(18, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 21) // Trajectory Plan
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(14, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 22) // Moving Objects
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(28, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 23) // GPS Axis
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(29, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 24) // Localize Neural
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(30, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 25) // Remission (works on pointcloud from Velodyne VBO)
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(31, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 32) // Traffic Light
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(20, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 33) // Traffic Signal
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    for (int j = 48; j < 56; j++) {
                        i_drawer->butt[j].visible = i_drawer->butt[i].state;
                    }
                    for (int j = 40; j < 48; j++) {
                        i_drawer->butt[j].visible = 0;
                    }
                } else if (i_drawer->butt[i].code == 34) // Pedestrian Track
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(22, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 35) // Stop
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(23, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 36) // Barrier
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(24, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 37) // Bump
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(25, i_drawer->butt[i].state);
                } else if (i_drawer->butt[i].code == 38) // Speed
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    for (int j = 40; j < 48; j++) {
                        i_drawer->butt[j].visible = i_drawer->butt[i].state;
                    }
                    for (int j = 48; j < 56; j++) {
                        i_drawer->butt[j].visible = 0;
                    }
                } else if (i_drawer->butt[i].code == 39) // Delete Annotation
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(27, i_drawer->butt[i].state);
                }
                else if (i_drawer->butt[i].code == 40) // Speed 0
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(26, 0);
                } else if (i_drawer->butt[i].code == 41) // Speed 5
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(26, 5);
                } else if (i_drawer->butt[i].code == 42) // Speed 10
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(26, 10);
                } else if (i_drawer->butt[i].code == 43) // Speed 15
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(26, 15);
                } else if (i_drawer->butt[i].code == 44) // Speed 20
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(26, 20);
                } else if (i_drawer->butt[i].code == 45) // Speed 30
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(26, 30);
                } else if (i_drawer->butt[i].code == 46) // Speed 40
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(26, 40);
                } else if (i_drawer->butt[i].code == 47) // Speed 60
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(26, 60);
                } else if (i_drawer->butt[i].code == 48) // Bump Traffic Signal
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(21, 0);
                } else if (i_drawer->butt[i].code == 49) // Speed 20 Traffic Signal
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(21, 20);
                } else if (i_drawer->butt[i].code == 50) // Speed 30 Traffic Signal
                {
                    i_drawer->butt[i].state = !(i_drawer->butt[i].state);

                    set_flag_viewer_3D(21, 30);
                }
            }
        }
    }
}

static void
handle_mouse_right_click(interface_drawer* i_drawer, int x, int y) {
    i_drawer = i_drawer;
    x = x;
    y = y;
}

static int
test_mouse_over_button(button b, int x, int y) {
    if (x > b.x - b.width / 2 && x < b.x + b.width / 2 && y > b.y - b.height / 2 && y < b.y + b.height / 2) {
        return 1;
    }

    return 0;
}

void
draw_interface(interface_drawer * i_drawer) {
    glPushMatrix();

    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, 1000.0, 0, 600.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    int i;
    for (i = 0; i < i_drawer->num_buttons; i++) {
        if (i_drawer->butt[i].visible) {
            draw_button(i_drawer->butt[i]);
        }
    }

    glPopMatrix();
}

static void
draw_button(button b) {
    glPushMatrix();

    if (b.mouse_over) {
        glColor3d(1.0, 1.0, 0.0);
    } else {
        glColor3d(1.0, 1.0, 1.0);
    }

    glTranslated(b.x, b.y, 0.0);

    glBegin(GL_QUADS);

    glVertex3d(-b.width / 2, -b.height / 2, 0.0);
    glVertex3d(b.width / 2, -b.height / 2, 0.0);
    glVertex3d(b.width / 2, b.height / 2, 0.0);
    glVertex3d(-b.width / 2, b.height / 2, 0.0);

    glEnd();

    glColor3d(0.0, 0.0, 0.0);
    drawText(-b.width / 2 + 5, -5, b.text);

    glPopMatrix();
}

static void
drawText(float x, float y, const char* msg, ...) {
    char buf[1024];
    va_list args;
    va_start(args, msg);
    vsprintf(buf, msg, args);
    va_end(args);

    int l, i;

    l = strlen(buf);
    glRasterPos2f(x, y);
    for (i = 0; i < l; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, buf[i]);
    }

}
