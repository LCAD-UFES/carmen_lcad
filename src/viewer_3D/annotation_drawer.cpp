#include <carmen/carmen.h>
#include <carmen/rddf_messages.h>
#include <carmen/glm.h>
#include <iostream>
#include <vector>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "annotation_drawer.h"

struct AnnotationDrawer
{
    // std::vector<carmen_rddf_annotation_message> annotations;
};

int
has_annotation(carmen_rddf_annotation_message msg, std::vector<carmen_rddf_annotation_message> annotations)
{
    for (uint i = 0; i < annotations.size(); i++)
    {
        if (annotations.at(i).annotation_code == msg.annotation_code &&
            annotations.at(i).annotation_description == msg.annotation_description &&
            annotations.at(i).annotation_point.x == msg.annotation_point.x &&
            annotations.at(i).annotation_point.y == msg.annotation_point.y &&
            annotations.at(i).annotation_point.z == msg.annotation_point.z)
        {
            return 1;
        }
    }
    return 0;
}

//AnnotationDrawer*
//addAnnotation(carmen_rddf_annotation_message msg, AnnotationDrawer *annotationDrawer) {
//    if (!has_annotation(msg, annotationDrawer->annotations)) {
//        annotationDrawer->annotations.push_back(msg);
//    }
//    return annotationDrawer;
//}

AnnotationDrawer*
createAnnotationDrawer(int argc, char** argv)
{
    argc = argc;
    argv = argv;
    AnnotationDrawer *annotationDrawer = NULL;
    annotationDrawer = (AnnotationDrawer *) malloc(sizeof (AnnotationDrawer));

    return annotationDrawer;
}

void
destroyAnnotationDrawer(AnnotationDrawer *annotationDrawer)
{
    free(annotationDrawer);
}

int
distance(carmen_vector_3D_t annotation_pose, carmen_vector_3D_t car_pose)
{
    return (sqrt((annotation_pose.x - car_pose.x) * (annotation_pose.x - car_pose.x) +
                 (annotation_pose.y - car_pose.y) * (annotation_pose.y - car_pose.y)+
                 (annotation_pose.z - car_pose.z) * (annotation_pose.z - car_pose.z)));
}

void
draw_annotations(std::vector<carmen_rddf_annotation_message> annotations, carmen_vector_3D_t car_pose, carmen_vector_3D_t offset)
{
    for (std::vector<carmen_rddf_annotation_message>::iterator it = annotations.begin(); it != annotations.end(); ++it)
    {
        carmen_vector_3D_t point_annotation;
        point_annotation.x = it.base()->annotation_point.x - offset.x;
        point_annotation.y = it.base()->annotation_point.y - offset.y;
        point_annotation.z = it.base()->annotation_point.z; // - offset.z;

        if (distance(point_annotation, car_pose) < 1000)
        {
            glColor3f(0.0f, 1.0f, 0.0f);
            glPushMatrix();
            glTranslatef(point_annotation.x, point_annotation.y, point_annotation.z);
            glutSolidSphere(0.55, 8, 8);
            glPopMatrix();
        }
    }
}
