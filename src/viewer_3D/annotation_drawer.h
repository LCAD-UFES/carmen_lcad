#ifndef ANNOTATION_DRAWER_H
#define ANNOTATION_DRAWER_H

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct AnnotationDrawer AnnotationDrawer;                       

    AnnotationDrawer* createAnnotationDrawer(int argc, char** argv);
    void destroyAnnotationDrawer(AnnotationDrawer *annotationDrawer);
    AnnotationDrawer* addAnnotation(carmen_rddf_annotation_message msg, AnnotationDrawer *annotationDrawer);
    void draw_annotations(std::vector<carmen_annotation_t> annotations, carmen_vector_3D_t car_pose, carmen_vector_3D_t offset);

#ifdef __cplusplus
}
#endif

#endif
