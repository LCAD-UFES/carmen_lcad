/*! @file ObjRec/test-faceDet.C test face detection */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-faceDet.C $
// $Id: test-faceDet.C 13716 2010-07-28 22:07:03Z itti $
//


#include "Image/OpenCVUtil.H"
#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Transforms.H"
#include "Image/DrawOps.H"
#include "Image/Rectangle.H"
#include "Image/MathOps.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "GUI/DebugWin.H"
#include "Neuro/BeoHeadBrain.H"
#include "Util/Timer.H"

static CvMemStorage* storage = 0;
static CvHaarClassifierCascade* cascade = 0;

#include <stdio.h>


 //these settings affect the quality of detection: change with care
#define CV_ADJUST_FEATURES 1
#define CV_ADJUST_WEIGHTS  0

typedef int sumtype;
typedef double sqsumtype;

void* cvAlignPtr( const void* ptr, int align=32 )
{
    assert( (align & (align-1)) == 0 );
    return (void*)( ((size_t)ptr + align - 1) & ~(size_t)(align-1) );
}

typedef struct CvHidHaarFeature
{
    struct
    {
        sumtype *p0, *p1, *p2, *p3;
        float weight;
    }
    rect[CV_HAAR_FEATURE_MAX];
}
CvHidHaarFeature;


typedef struct CvHidHaarTreeNode
{
    CvHidHaarFeature feature;
    float threshold;
    int left;
    int right;
}
CvHidHaarTreeNode;


typedef struct CvHidHaarClassifier
{
    int count;
    //CvHaarFeature* orig_feature;
    CvHidHaarTreeNode* node;
    float* alpha;
}
CvHidHaarClassifier;


typedef struct CvHidHaarStageClassifier
{
    int  count;
    float threshold;
    CvHidHaarClassifier* classifier;
    int two_rects;

    struct CvHidHaarStageClassifier* next;
    struct CvHidHaarStageClassifier* child;
    struct CvHidHaarStageClassifier* parent;
}
CvHidHaarStageClassifier;


struct CvHidHaarClassifierCascade
{
    int  count;
    int  is_stump_based;
    int  has_tilted_features;
    int  is_tree;
    double inv_window_area;
    CvMat sum, sqsum, tilted;
    CvHidHaarStageClassifier* stage_classifier;
    sqsumtype *pq0, *pq1, *pq2, *pq3;
    sumtype *p0, *p1, *p2, *p3;

    void** ipp_stages;
};


// IPP functions for object detection
int icvHaarClassifierInitAlloc_32f_p = 0;
int icvHaarClassifierFree_32f_p = 0;
int icvApplyHaarClassifier_32s32f_C1R_p = 0;
int icvRectStdDev_32s32f_C1R_p = 0;

const int icv_object_win_border = 1;
const float icv_stage_threshold_bias = 0.0001f;

//static CvHaarClassifierCascade*
//icvCreateHaarClassifierCascade( int stage_count )
//{
//    CvHaarClassifierCascade* cascade = 0;
//
//    CV_FUNCNAME( "icvCreateHaarClassifierCascade" );
//
//    __BEGIN__;
//
//    int block_size = sizeof(*cascade) + stage_count*sizeof(*cascade->stage_classifier);
//
//    if( stage_count <= 0 )
//        CV_ERROR( CV_StsOutOfRange, "Number of stages should be positive" );
//
//    CV_CALL( cascade = (CvHaarClassifierCascade*)cvAlloc( block_size ));
//    memset( cascade, 0, block_size );
//
//    cascade->stage_classifier = (CvHaarStageClassifier*)(cascade + 1);
//    cascade->flags = CV_HAAR_MAGIC_VAL;
//    cascade->count = stage_count;
//
//    __END__;
//
//    return cascade;
//}

static void
icvReleaseHidHaarClassifierCascade( CvHidHaarClassifierCascade** _cascade )
{
    if( _cascade && *_cascade )
    {
        CvHidHaarClassifierCascade* cascade = *_cascade;
        if( cascade->ipp_stages && icvHaarClassifierFree_32f_p )
        {
            int i;
            for( i = 0; i < cascade->count; i++ )
            {
                //if( cascade->ipp_stages[i] )
                //    icvHaarClassifierFree_32f_p( cascade->ipp_stages[i] );
            }
        }
        //cvFree( &cascade->ipp_stages );
        //cvFree((void**) _cascade );
    }
}

// create more efficient internal representation of haar classifier cascade
static CvHidHaarClassifierCascade*
icvCreateHidHaarClassifierCascade( CvHaarClassifierCascade* cascade )
{
    CvRect* ipp_features = 0;
    float *ipp_weights = 0, *ipp_thresholds = 0, *ipp_val1 = 0, *ipp_val2 = 0;
    int* ipp_counts = 0;

    CvHidHaarClassifierCascade* out = 0;

    CV_FUNCNAME( "icvCreateHidHaarClassifierCascade" );

    __BEGIN__;

    int i, j, k, l;
    int datasize;
    int total_classifiers = 0;
    int total_nodes = 0;
    char errorstr[100];
    CvHidHaarClassifier* haar_classifier_ptr;
    CvHidHaarTreeNode* haar_node_ptr;
    CvSize orig_window_size;
    int has_tilted_features = 0;
    int max_count = 0;

    if( !CV_IS_HAAR_CLASSIFIER(cascade) )
        CV_ERROR( !cascade ? CV_StsNullPtr : CV_StsBadArg, "Invalid classifier pointer" );

    if( cascade->hid_cascade )
        CV_ERROR( CV_StsError, "hid_cascade has been already created" );

    if( !cascade->stage_classifier )
        CV_ERROR( CV_StsNullPtr, "" );

    if( cascade->count <= 0 )
        CV_ERROR( CV_StsOutOfRange, "Negative number of cascade stages" );

    orig_window_size = cascade->orig_window_size;

    // check input structure correctness and calculate total memory size needed for
    // internal representation of the classifier cascade
    for( i = 0; i < cascade->count; i++ )
    {
        CvHaarStageClassifier* stage_classifier = cascade->stage_classifier + i;

        if( !stage_classifier->classifier ||
            stage_classifier->count <= 0 )
        {
            sprintf( errorstr, "header of the stage classifier #%d is invalid "
                     "(has null pointers or non-positive classfier count)", i );
            CV_ERROR( CV_StsError, errorstr );
        }

        max_count = MAX( max_count, stage_classifier->count );
        total_classifiers += stage_classifier->count;

        for( j = 0; j < stage_classifier->count; j++ )
        {
            CvHaarClassifier* classifier = stage_classifier->classifier + j;

            total_nodes += classifier->count;
            for( l = 0; l < classifier->count; l++ )
            {
                for( k = 0; k < CV_HAAR_FEATURE_MAX; k++ )
                {
                    if( classifier->haar_feature[l].rect[k].r.width )
                    {
                        CvRect r = classifier->haar_feature[l].rect[k].r;
                        int tilted = classifier->haar_feature[l].tilted;
                        has_tilted_features |= tilted != 0;
                        if( r.width < 0 || r.height < 0 || r.y < 0 ||
                            r.x + r.width > orig_window_size.width
                            ||
                            (!tilted &&
                            (r.x < 0 || r.y + r.height > orig_window_size.height))
                            ||
                            (tilted && (r.x - r.height < 0 ||
                            r.y + r.width + r.height > orig_window_size.height)))
                        {
                            sprintf( errorstr, "rectangle #%d of the classifier #%d of "
                                     "the stage classifier #%d is not inside "
                                     "the reference (original) cascade window", k, j, i );
                            CV_ERROR( CV_StsNullPtr, errorstr );
                        }
                    }
                }
            }
        }
    }

    // this is an upper boundary for the whole hidden cascade size
    datasize = sizeof(CvHidHaarClassifierCascade) +
               sizeof(CvHidHaarStageClassifier)*cascade->count +
               sizeof(CvHidHaarClassifier) * total_classifiers +
               sizeof(CvHidHaarTreeNode) * total_nodes +
               sizeof(void*)*(total_nodes + total_classifiers);

    CV_CALL( out = (CvHidHaarClassifierCascade*)cvAlloc( datasize ));
    memset( out, 0, sizeof(*out) );

    // init header
    out->count = cascade->count;
    out->stage_classifier = (CvHidHaarStageClassifier*)(out + 1);
    haar_classifier_ptr = (CvHidHaarClassifier*)(out->stage_classifier + cascade->count);
    haar_node_ptr = (CvHidHaarTreeNode*)(haar_classifier_ptr + total_classifiers);

    out->is_stump_based = 1;
    out->has_tilted_features = has_tilted_features;
    out->is_tree = 0;

    // initialize internal representation
    for( i = 0; i < cascade->count; i++ )
    {
        CvHaarStageClassifier* stage_classifier = cascade->stage_classifier + i;
        CvHidHaarStageClassifier* hid_stage_classifier = out->stage_classifier + i;

        hid_stage_classifier->count = stage_classifier->count;
        hid_stage_classifier->threshold = stage_classifier->threshold - icv_stage_threshold_bias;
        hid_stage_classifier->classifier = haar_classifier_ptr;
        hid_stage_classifier->two_rects = 1;
        haar_classifier_ptr += stage_classifier->count;

        hid_stage_classifier->parent = (stage_classifier->parent == -1)
            ? NULL : out->stage_classifier + stage_classifier->parent;
        hid_stage_classifier->next = (stage_classifier->next == -1)
            ? NULL : out->stage_classifier + stage_classifier->next;
        hid_stage_classifier->child = (stage_classifier->child == -1)
            ? NULL : out->stage_classifier + stage_classifier->child;

        out->is_tree |= hid_stage_classifier->next != NULL;

        for( j = 0; j < stage_classifier->count; j++ )
        {
            CvHaarClassifier* classifier = stage_classifier->classifier + j;
            CvHidHaarClassifier* hid_classifier = hid_stage_classifier->classifier + j;
            int node_count = classifier->count;
            float* alpha_ptr = (float*)(haar_node_ptr + node_count);

            hid_classifier->count = node_count;
            hid_classifier->node = haar_node_ptr;
            hid_classifier->alpha = alpha_ptr;

            for( l = 0; l < node_count; l++ )
            {
                CvHidHaarTreeNode* node = hid_classifier->node + l;
                CvHaarFeature* feature = classifier->haar_feature + l;
                memset( node, -1, sizeof(*node) );
                node->threshold = classifier->threshold[l];
                node->left = classifier->left[l];
                node->right = classifier->right[l];

                if( fabs(feature->rect[2].weight) < DBL_EPSILON ||
                    feature->rect[2].r.width == 0 ||
                    feature->rect[2].r.height == 0 )
                    memset( &(node->feature.rect[2]), 0, sizeof(node->feature.rect[2]) );
                else
                    hid_stage_classifier->two_rects = 0;
            }

            memcpy( alpha_ptr, classifier->alpha, (node_count+1)*sizeof(alpha_ptr[0]));
            haar_node_ptr =
                (CvHidHaarTreeNode*)cvAlignPtr(alpha_ptr+node_count+1, sizeof(void*));

            out->is_stump_based &= node_count == 1;
        }
    }

    //
    // NOTE: Currently, OpenMP is implemented and IPP modes are incompatible.
    //
#ifndef _OPENMP
    {
    int can_use_ipp = icvHaarClassifierInitAlloc_32f_p != 0 &&
        icvHaarClassifierFree_32f_p != 0 &&
                      icvApplyHaarClassifier_32s32f_C1R_p != 0 &&
                      icvRectStdDev_32s32f_C1R_p != 0 &&
                      !out->has_tilted_features && !out->is_tree && out->is_stump_based;

    if( can_use_ipp )
    {
        int ipp_datasize = cascade->count*sizeof(out->ipp_stages[0]);
        float ipp_weight_scale=(float)(1./((orig_window_size.width-icv_object_win_border*2)*
            (orig_window_size.height-icv_object_win_border*2)));

        CV_CALL( out->ipp_stages = (void**)cvAlloc( ipp_datasize ));
        memset( out->ipp_stages, 0, ipp_datasize );

        CV_CALL( ipp_features = (CvRect*)cvAlloc( max_count*3*sizeof(ipp_features[0]) ));
        CV_CALL( ipp_weights = (float*)cvAlloc( max_count*3*sizeof(ipp_weights[0]) ));
        CV_CALL( ipp_thresholds = (float*)cvAlloc( max_count*sizeof(ipp_thresholds[0]) ));
        CV_CALL( ipp_val1 = (float*)cvAlloc( max_count*sizeof(ipp_val1[0]) ));
        CV_CALL( ipp_val2 = (float*)cvAlloc( max_count*sizeof(ipp_val2[0]) ));
        CV_CALL( ipp_counts = (int*)cvAlloc( max_count*sizeof(ipp_counts[0]) ));

        for( i = 0; i < cascade->count; i++ )
        {
            CvHaarStageClassifier* stage_classifier = cascade->stage_classifier + i;
            for( j = 0, k = 0; j < stage_classifier->count; j++ )
            {
                CvHaarClassifier* classifier = stage_classifier->classifier + j;
                int rect_count = 2 + (classifier->haar_feature->rect[2].r.width != 0);

                ipp_thresholds[j] = classifier->threshold[0];
                ipp_val1[j] = classifier->alpha[0];
                ipp_val2[j] = classifier->alpha[1];
                ipp_counts[j] = rect_count;

                for( l = 0; l < rect_count; l++, k++ )
                {
                    ipp_features[k] = classifier->haar_feature->rect[l].r;
                    //ipp_features[k].y = orig_window_size.height - ipp_features[k].y - ipp_features[k].height;
                    ipp_weights[k] = classifier->haar_feature->rect[l].weight*ipp_weight_scale;
                }
            }

            //if( icvHaarClassifierInitAlloc_32f_p( &out->ipp_stages[i],
            //    ipp_features, ipp_weights, ipp_thresholds,
            //    ipp_val1, ipp_val2, ipp_counts, stage_classifier->count ) < 0 )
            //    break;
        }

        if( i < cascade->count )
        {
            //for( j = 0; j < i; j++ )
            //   // if( icvHaarClassifierFree_32f_p && out->ipp_stages[i] )
            //   //     icvHaarClassifierFree_32f_p( out->ipp_stages[i] );
            //cvFree( &out->ipp_stages );
        }
    }
    }
#endif

    cascade->hid_cascade = out;
    assert( (char*)haar_node_ptr - (char*)out <= datasize );

    __END__;

    if( cvGetErrStatus() < 0 )
        icvReleaseHidHaarClassifierCascade( &out );

    //cvFree( &ipp_features );
    //cvFree( &ipp_weights );
    //cvFree( &ipp_thresholds );
    //cvFree( &ipp_val1 );
    //cvFree( &ipp_val2 );
    //cvFree( &ipp_counts );

    return out;
}


#define sum_elem_ptr(sum,row,col)  \
    ((sumtype*)CV_MAT_ELEM_PTR_FAST((sum),(row),(col),sizeof(sumtype)))

#define sqsum_elem_ptr(sqsum,row,col)  \
    ((sqsumtype*)CV_MAT_ELEM_PTR_FAST((sqsum),(row),(col),sizeof(sqsumtype)))

#define calc_sum(rect,offset) \
    ((rect).p0[offset] - (rect).p1[offset] - (rect).p2[offset] + (rect).p3[offset])


void
cvSetImagesForHaarClassifierCascade( CvHaarClassifierCascade* _cascade,
                                     const CvArr* _sum,
                                     const CvArr* _sqsum,
                                     const CvArr* _tilted_sum,
                                     double scale )
{
    CV_FUNCNAME("cvSetImagesForHaarClassifierCascade");

    __BEGIN__;

    CvMat sum_stub, *sum = (CvMat*)_sum;
    CvMat sqsum_stub, *sqsum = (CvMat*)_sqsum;
    CvMat tilted_stub, *tilted = (CvMat*)_tilted_sum;
    CvHidHaarClassifierCascade* cascade;
    int coi0 = 0, coi1 = 0;
    int i;
    CvRect equ_rect;
    double weight_scale;

    if( !CV_IS_HAAR_CLASSIFIER(_cascade) )
        CV_ERROR( !_cascade ? CV_StsNullPtr : CV_StsBadArg, "Invalid classifier pointer" );

    if( scale <= 0 )
        CV_ERROR( CV_StsOutOfRange, "Scale must be positive" );

    CV_CALL( sum = cvGetMat( sum, &sum_stub, &coi0 ));
    CV_CALL( sqsum = cvGetMat( sqsum, &sqsum_stub, &coi1 ));

    if( coi0 || coi1 )
        CV_ERROR( CV_BadCOI, "COI is not supported" );

    if( !CV_ARE_SIZES_EQ( sum, sqsum ))
        CV_ERROR( CV_StsUnmatchedSizes, "All integral images must have the same size" );

    if( CV_MAT_TYPE(sqsum->type) != CV_64FC1 ||
        CV_MAT_TYPE(sum->type) != CV_32SC1 )
        CV_ERROR( CV_StsUnsupportedFormat,
        "Only (32s, 64f, 32s) combination of (sum,sqsum,tilted_sum) formats is allowed" );

    if( !_cascade->hid_cascade )
        CV_CALL( icvCreateHidHaarClassifierCascade(_cascade) );

    cascade = _cascade->hid_cascade;

    if( cascade->has_tilted_features )
    {
        CV_CALL( tilted = cvGetMat( tilted, &tilted_stub, &coi1 ));

        if( CV_MAT_TYPE(tilted->type) != CV_32SC1 )
            CV_ERROR( CV_StsUnsupportedFormat,
            "Only (32s, 64f, 32s) combination of (sum,sqsum,tilted_sum) formats is allowed" );

        if( sum->step != tilted->step )
            CV_ERROR( CV_StsUnmatchedSizes,
            "Sum and tilted_sum must have the same stride (step, widthStep)" );

        if( !CV_ARE_SIZES_EQ( sum, tilted ))
            CV_ERROR( CV_StsUnmatchedSizes, "All integral images must have the same size" );
        cascade->tilted = *tilted;
    }

    _cascade->scale = scale;
    _cascade->real_window_size.width = cvRound( _cascade->orig_window_size.width * scale );
    _cascade->real_window_size.height = cvRound( _cascade->orig_window_size.height * scale );

    cascade->sum = *sum;
    cascade->sqsum = *sqsum;

    equ_rect.x = equ_rect.y = cvRound(scale);
    equ_rect.width = cvRound((_cascade->orig_window_size.width-2)*scale);
    equ_rect.height = cvRound((_cascade->orig_window_size.height-2)*scale);
    weight_scale = 1./(equ_rect.width*equ_rect.height);
    cascade->inv_window_area = weight_scale;

    cascade->p0 = sum_elem_ptr(*sum, equ_rect.y, equ_rect.x);
    cascade->p1 = sum_elem_ptr(*sum, equ_rect.y, equ_rect.x + equ_rect.width );
    cascade->p2 = sum_elem_ptr(*sum, equ_rect.y + equ_rect.height, equ_rect.x );
    cascade->p3 = sum_elem_ptr(*sum, equ_rect.y + equ_rect.height,
                                     equ_rect.x + equ_rect.width );

    cascade->pq0 = sqsum_elem_ptr(*sqsum, equ_rect.y, equ_rect.x);
    cascade->pq1 = sqsum_elem_ptr(*sqsum, equ_rect.y, equ_rect.x + equ_rect.width );
    cascade->pq2 = sqsum_elem_ptr(*sqsum, equ_rect.y + equ_rect.height, equ_rect.x );
    cascade->pq3 = sqsum_elem_ptr(*sqsum, equ_rect.y + equ_rect.height,
                                          equ_rect.x + equ_rect.width );

    // init pointers in haar features according to real window size and
    //   given image pointers
    {
#ifdef _OPENMP
    int max_threads = cvGetNumThreads();
    #pragma omp parallel for num_threads(max_threads), schedule(dynamic)
#endif // _OPENMP
    for( i = 0; i < _cascade->count; i++ )
    {
        int j, k, l;
        for( j = 0; j < cascade->stage_classifier[i].count; j++ )
        {
            for( l = 0; l < cascade->stage_classifier[i].classifier[j].count; l++ )
            {
                CvHaarFeature* feature =
                    &_cascade->stage_classifier[i].classifier[j].haar_feature[l];
                // CvHidHaarClassifier* classifier =
                //    cascade->stage_classifier[i].classifier + j;
                CvHidHaarFeature* hidfeature =
                    &cascade->stage_classifier[i].classifier[j].node[l].feature;
                double sum0 = 0, area0 = 0;
                CvRect r[3];
#if CV_ADJUST_FEATURES
                int base_w = -1, base_h = -1;
                int new_base_w = 0, new_base_h = 0;
                int kx, ky;
                int flagx = 0, flagy = 0;
                int x0 = 0, y0 = 0;
#endif
                int nr;

                // align blocks
                for( k = 0; k < CV_HAAR_FEATURE_MAX; k++ )
                {
                    if( !hidfeature->rect[k].p0 )
                        break;
#if CV_ADJUST_FEATURES
                    r[k] = feature->rect[k].r;
                    base_w = (int)CV_IMIN( (unsigned)base_w, (unsigned)(r[k].width-1) );
                    base_w = (int)CV_IMIN( (unsigned)base_w, (unsigned)(r[k].x - r[0].x-1) );
                    base_h = (int)CV_IMIN( (unsigned)base_h, (unsigned)(r[k].height-1) );
                    base_h = (int)CV_IMIN( (unsigned)base_h, (unsigned)(r[k].y - r[0].y-1) );
#endif
                }

                nr = k;

#if CV_ADJUST_FEATURES
                base_w += 1;
                base_h += 1;
                kx = r[0].width / base_w;
                ky = r[0].height / base_h;

                if( kx <= 0 )
                {
                    flagx = 1;
                    new_base_w = cvRound( r[0].width * scale ) / kx;
                    x0 = cvRound( r[0].x * scale );
                }

                if( ky <= 0 )
                {
                    flagy = 1;
                    new_base_h = cvRound( r[0].height * scale ) / ky;
                    y0 = cvRound( r[0].y * scale );
                }
#endif

                for( k = 0; k < nr; k++ )
                {
                    CvRect tr;
                    double correction_ratio;

#if CV_ADJUST_FEATURES
                    if( flagx )
                    {
                        tr.x = (r[k].x - r[0].x) * new_base_w / base_w + x0;
                        tr.width = r[k].width * new_base_w / base_w;
                    }
                    else
#endif
                    {
                        tr.x = cvRound( r[k].x * scale );
                        tr.width = cvRound( r[k].width * scale );
                    }

#if CV_ADJUST_FEATURES
                    if( flagy )
                    {
                        tr.y = (r[k].y - r[0].y) * new_base_h / base_h + y0;
                        tr.height = r[k].height * new_base_h / base_h;
                    }
                    else
#endif
                    {
                        tr.y = cvRound( r[k].y * scale );
                        tr.height = cvRound( r[k].height * scale );
                    }

#if CV_ADJUST_WEIGHTS
                    {
                    // RAINER START
                    const float orig_feature_size =  (float)(feature->rect[k].r.width)*feature->rect[k].r.height;
                    const float orig_norm_size = (float)(_cascade->orig_window_size.width)*(_cascade->orig_window_size.height);
                    const float feature_size = float(tr.width*tr.height);
                    //const float normSize    = float(equ_rect.width*equ_rect.height);
                    float target_ratio = orig_feature_size / orig_norm_size;
                    //float isRatio = featureSize / normSize;
                    //correctionRatio = targetRatio / isRatio / normSize;
                    correction_ratio = target_ratio / feature_size;
                    // RAINER END
                    }
#else
                    correction_ratio = weight_scale * (!feature->tilted ? 1 : 0.5);
#endif

                    if( !feature->tilted )
                    {
                        hidfeature->rect[k].p0 = sum_elem_ptr(*sum, tr.y, tr.x);
                        hidfeature->rect[k].p1 = sum_elem_ptr(*sum, tr.y, tr.x + tr.width);
                        hidfeature->rect[k].p2 = sum_elem_ptr(*sum, tr.y + tr.height, tr.x);
                        hidfeature->rect[k].p3 = sum_elem_ptr(*sum, tr.y + tr.height, tr.x + tr.width);
                    }
                    else
                    {
                        hidfeature->rect[k].p2 = sum_elem_ptr(*tilted, tr.y + tr.width, tr.x + tr.width);
                        hidfeature->rect[k].p3 = sum_elem_ptr(*tilted, tr.y + tr.width + tr.height,
                                                              tr.x + tr.width - tr.height);
                        hidfeature->rect[k].p0 = sum_elem_ptr(*tilted, tr.y, tr.x);
                        hidfeature->rect[k].p1 = sum_elem_ptr(*tilted, tr.y + tr.height, tr.x - tr.height);
                    }

                    hidfeature->rect[k].weight = (float)(feature->rect[k].weight * correction_ratio);

                    if( k == 0 )
                        area0 = tr.width * tr.height;
                    else
                        sum0 += hidfeature->rect[k].weight * tr.width * tr.height;
                }

                hidfeature->rect[0].weight = (float)(-sum0/area0);
            } // l
        } // j
    }
    }

    __END__;
}


CV_INLINE
double icvEvalHidHaarClassifier( CvHidHaarClassifier* classifier,
                                 double variance_norm_factor,
                                 size_t p_offset )
{
    int idx = 0;
    do
    {
        CvHidHaarTreeNode* node = classifier->node + idx;
        double t = node->threshold * variance_norm_factor;

        double sum = calc_sum(node->feature.rect[0],p_offset) * node->feature.rect[0].weight;
        sum += calc_sum(node->feature.rect[1],p_offset) * node->feature.rect[1].weight;

        if( node->feature.rect[2].p0 )
            sum += calc_sum(node->feature.rect[2],p_offset) * node->feature.rect[2].weight;

        idx = sum < t ? node->left : node->right;
    }
    while( idx > 0 );
    return classifier->alpha[-idx];
}


int
cvRunHaarClassifierCascade( CvHaarClassifierCascade* _cascade,
                            CvPoint pt, int start_stage )
{
    int result = -1;
    CV_FUNCNAME("cvRunHaarClassifierCascade");

    __BEGIN__;

    int p_offset, pq_offset;
    int i, j;
    double mean, variance_norm_factor;
    CvHidHaarClassifierCascade* cascade;

    if( !CV_IS_HAAR_CLASSIFIER(_cascade) )
        CV_ERROR( !_cascade ? CV_StsNullPtr : CV_StsBadArg, "Invalid cascade pointer" );

    cascade = _cascade->hid_cascade;
    if( !cascade )
        CV_ERROR( CV_StsNullPtr, "Hidden cascade has not been created.\n"
            "Use cvSetImagesForHaarClassifierCascade" );

    if( pt.x < 0 || pt.y < 0 ||
        pt.x + _cascade->real_window_size.width >= cascade->sum.width-2 ||
        pt.y + _cascade->real_window_size.height >= cascade->sum.height-2 )
        EXIT;

    p_offset = pt.y * (cascade->sum.step/sizeof(sumtype)) + pt.x;
    pq_offset = pt.y * (cascade->sqsum.step/sizeof(sqsumtype)) + pt.x;
    mean = calc_sum(*cascade,p_offset)*cascade->inv_window_area;
    variance_norm_factor = cascade->pq0[pq_offset] - cascade->pq1[pq_offset] -
                           cascade->pq2[pq_offset] + cascade->pq3[pq_offset];
    variance_norm_factor = variance_norm_factor*cascade->inv_window_area - mean*mean;
    if( variance_norm_factor >= 0. )
        variance_norm_factor = sqrt(variance_norm_factor);
    else
        variance_norm_factor = 1.;

    if( cascade->is_tree )
    {
        CvHidHaarStageClassifier* ptr;
        assert( start_stage == 0 );

        result = 1;
        ptr = cascade->stage_classifier;

        while( ptr )
        {
            double stage_sum = 0;

            for( j = 0; j < ptr->count; j++ )
            {
                stage_sum += icvEvalHidHaarClassifier( ptr->classifier + j,
                    variance_norm_factor, p_offset );
            }

            if( stage_sum >= ptr->threshold )
            {
                ptr = ptr->child;
            }
            else
            {
                while( ptr && ptr->next == NULL ) ptr = ptr->parent;
                if( ptr == NULL )
                {
                    result = 0;
                    EXIT;
                }
                ptr = ptr->next;
            }
        }
    }
    else if( cascade->is_stump_based )
    {
        for( i = start_stage; i < cascade->count; i++ )
        {
            double stage_sum = 0;

            if( cascade->stage_classifier[i].two_rects )
            {
                for( j = 0; j < cascade->stage_classifier[i].count; j++ )
                {
                    CvHidHaarClassifier* classifier = cascade->stage_classifier[i].classifier + j;
                    CvHidHaarTreeNode* node = classifier->node;
                    double sum, t = node->threshold*variance_norm_factor, a, b;

                    sum = calc_sum(node->feature.rect[0],p_offset) * node->feature.rect[0].weight;
                    sum += calc_sum(node->feature.rect[1],p_offset) * node->feature.rect[1].weight;

                    a = classifier->alpha[0];
                    b = classifier->alpha[1];
                    stage_sum += sum < t ? a : b;
                }
            }
            else
            {
                for( j = 0; j < cascade->stage_classifier[i].count; j++ )
                {
                    CvHidHaarClassifier* classifier = cascade->stage_classifier[i].classifier + j;
                    CvHidHaarTreeNode* node = classifier->node;
                    double sum, t = node->threshold*variance_norm_factor, a, b;

                    sum = calc_sum(node->feature.rect[0],p_offset) * node->feature.rect[0].weight;
                    sum += calc_sum(node->feature.rect[1],p_offset) * node->feature.rect[1].weight;

                    if( node->feature.rect[2].p0 )
                        sum += calc_sum(node->feature.rect[2],p_offset) * node->feature.rect[2].weight;

                    a = classifier->alpha[0];
                    b = classifier->alpha[1];
                    stage_sum += sum < t ? a : b;
                }
            }

            if( stage_sum < cascade->stage_classifier[i].threshold )
            {
                result = -i;
                EXIT;
            }
        }
    }
    else
    {
        for( i = start_stage; i < cascade->count; i++ )
        {
            double stage_sum = 0;

            for( j = 0; j < cascade->stage_classifier[i].count; j++ )
            {
                stage_sum += icvEvalHidHaarClassifier(
                    cascade->stage_classifier[i].classifier + j,
                    variance_norm_factor, p_offset );
            }

            if( stage_sum < cascade->stage_classifier[i].threshold )
            {
                result = -i;
                EXIT;
            }
        }
    }

    result = 1;

    __END__;

    return result;
}


static int is_equal( const void* _r1, const void* _r2, void* )
{
    const CvRect* r1 = (const CvRect*)_r1;
    const CvRect* r2 = (const CvRect*)_r2;
    int distance = cvRound(r1->width*0.2);

    return r2->x <= r1->x + distance &&
           r2->x >= r1->x - distance &&
           r2->y <= r1->y + distance &&
           r2->y >= r1->y - distance &&
           r2->width <= cvRound( r1->width * 1.2 ) &&
           cvRound( r2->width * 1.2 ) >= r1->width;
}

CvSeq*
cvHaarDetectObjects2( const CvArr* _img,
                     CvHaarClassifierCascade* cascade,
                     CvMemStorage* storage, double scale_factor,
                     int min_neighbors, int flags, CvSize min_size )
{
    int split_stage = 2;

    IplImage* iplImg = (IplImage*)_img;
    CvMat stub, *img = (CvMat*)_img;
    CvMat *temp = 0, *sum = 0, *tilted = 0, *sqsum = 0, *norm_img = 0, *sumcanny = 0, *img_small = 0;
    CvSeq* seq = 0;
    CvSeq* seq2 = 0;
    CvSeq* idx_seq = 0;
    CvSeq* result_seq = 0;
    CvMemStorage* temp_storage = 0;
    CvAvgComp* comps = 0;
    int i;


    double factor;
    int npass = 2, coi;
    int do_canny_pruning = flags & CV_HAAR_DO_CANNY_PRUNING;

    img = cvGetMat(_img, &stub, &coi );

    if( !CV_IS_HAAR_CLASSIFIER(cascade) )
        LFATAL( "Invalid classifier cascade" );

    //if( !storage )
    //    CV_ERROR( CV_StsNullPtr, "Null storage pointer" );

    if( coi )
      LFATAL("COI is not supported" );

    if( CV_MAT_DEPTH(img->type) != CV_8U )
        LFATAL("Only 8-bit images are supported" );

    temp = cvCreateMat( iplImg->height, iplImg->width, CV_8UC1 );
    sum = cvCreateMat( iplImg->height + 1, iplImg->width + 1, CV_32SC1 );
    sqsum = cvCreateMat( iplImg->height + 1, iplImg->width + 1, CV_64FC1 );
    temp_storage = cvCreateChildMemStorage( storage );

#ifdef _OPENMP
    max_threads = cvGetNumThreads();
    for( i = 0; i < max_threads; i++ )
    {
        CvMemStorage* temp_storage_thread;
        temp_storage_thread = cvCreateMemStorage(0);
        CV_CALL( seq_thread[i] = cvCreateSeq( 0, sizeof(CvSeq),
                        sizeof(CvRect), temp_storage_thread ));
    }
#endif

    if( !cascade->hid_cascade )
        icvCreateHidHaarClassifierCascade(cascade);

    if( cascade->hid_cascade->has_tilted_features )
        tilted = cvCreateMat( iplImg->height + 1, iplImg->width + 1, CV_32SC1 );

    seq = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvRect), temp_storage );
    seq2 = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvAvgComp), temp_storage );
    result_seq = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvAvgComp), storage );

    if( min_neighbors == 0 )
        seq = result_seq;

    if( CV_MAT_CN(img->type) > 1 )
    {
        cvCvtColor( img, temp, CV_BGR2GRAY );
        img = temp;
    }

    if( flags & CV_HAAR_SCALE_IMAGE )
    {
        CvSize win_size0 = cascade->orig_window_size;
        int use_ipp = cascade->hid_cascade->ipp_stages != 0 &&
                    icvApplyHaarClassifier_32s32f_C1R_p != 0;

        img_small = cvCreateMat( iplImg->height + 1, iplImg->width + 1, CV_8UC1 );

        for( factor = 1; ; factor *= scale_factor )
        {
            int positive = 0;
            int x, y;
            CvSize win_size = { cvRound(win_size0.width*factor),
                                cvRound(win_size0.height*factor) };
            CvSize sz = { cvRound( img->cols/factor ), cvRound( img->rows/factor ) };
            CvSize sz1 = { sz.width - win_size0.width, sz.height - win_size0.height };
            //CvRect rect1 = { icv_object_win_border, icv_object_win_border,
            //    win_size0.width - icv_object_win_border*2,
            //    win_size0.height - icv_object_win_border*2 };
            CvMat img1, sum1, sqsum1, norm1, tilted1, mask1;
            CvMat* _tilted = 0;

            if( sz1.width <= 0 || sz1.height <= 0 )
                break;
            if( win_size.width < min_size.width || win_size.height < min_size.height )
                continue;

            img1 = cvMat( sz.height, sz.width, CV_8UC1, img_small->data.ptr );
            sum1 = cvMat( sz.height+1, sz.width+1, CV_32SC1, sum->data.ptr );
            sqsum1 = cvMat( sz.height+1, sz.width+1, CV_64FC1, sqsum->data.ptr );
            if( tilted )
            {
                tilted1 = cvMat( sz.height+1, sz.width+1, CV_32SC1, tilted->data.ptr );
                _tilted = &tilted1;
            }
            norm1 = cvMat( sz1.height, sz1.width, CV_32FC1, norm_img ? norm_img->data.ptr : 0 );
            mask1 = cvMat( sz1.height, sz1.width, CV_8UC1, temp->data.ptr );

            cvResize( img, &img1, CV_INTER_LINEAR );
            cvIntegral( &img1, &sum1, &sqsum1, _tilted );

           // if( use_ipp && icvRectStdDev_32s32f_C1R_p( sum1.data.i, sum1.step,
           //     sqsum1.data.db, sqsum1.step, norm1.data.fl, norm1.step, sz1, rect1 ) < 0 )
           //     use_ipp = 0;
           use_ipp = 0;

            if( use_ipp )
            {
                positive = mask1.cols*mask1.rows;
                cvSet( &mask1, cvScalarAll(255) );
                for( i = 0; i < cascade->count; i++ )
                {
                    //if( icvApplyHaarClassifier_32s32f_C1R_p(sum1.data.i, sum1.step,
                    //    norm1.data.fl, norm1.step, mask1.data.ptr, mask1.step,
                    //    sz1, &positive, cascade->hid_cascade->stage_classifier[i].threshold,
                    //    cascade->hid_cascade->ipp_stages[i]) < 0 )
                    //{
                    //    use_ipp = 0;
                    //    break;
                    //}
                    if( positive <= 0 )
                        break;
                }
            }

            if( !use_ipp )
            {
                cvSetImagesForHaarClassifierCascade( cascade, &sum1, &sqsum1, 0, 1. );
                for( y = 0, positive = 0; y < sz1.height; y++ )
                    for( x = 0; x < sz1.width; x++ )
                    {
                        mask1.data.ptr[mask1.step*y + x] =
                            cvRunHaarClassifierCascade( cascade, cvPoint(x,y), 0 ) > 0;
                        positive += mask1.data.ptr[mask1.step*y + x];
                    }
            }

            if( positive > 0 )
            {
                for( y = 0; y < sz1.height; y++ )
                    for( x = 0; x < sz1.width; x++ )
                        if( mask1.data.ptr[mask1.step*y + x] != 0 )
                        {
                            CvRect obj_rect = { cvRound(y*factor), cvRound(x*factor),
                                                win_size.width, win_size.height };
                            cvSeqPush( seq, &obj_rect );
                        }
            }
        }
    }
    else
    {
        cvIntegral(img, sum, sqsum, tilted );

        if( do_canny_pruning )
        {
            sumcanny = cvCreateMat( img->rows + 1, img->cols + 1, CV_32SC1 );
            cvCanny( img, temp, 0, 50, 3 );
            cvIntegral( temp, sumcanny );
        }

        if( (unsigned)split_stage >= (unsigned)cascade->count ||
            cascade->hid_cascade->is_tree )
        {
            split_stage = cascade->count;
            npass = 1;
        }

        LINFO("1");
        for( factor = 1; factor*cascade->orig_window_size.width < img->cols - 10 &&
                         factor*cascade->orig_window_size.height < img->rows - 10;
             factor *= scale_factor )
        {
            const double ystep = MAX( 2, factor );
            CvSize win_size = { cvRound( cascade->orig_window_size.width * factor ),
                                cvRound( cascade->orig_window_size.height * factor )};
            CvRect equ_rect = { 0, 0, 0, 0 };
            int *p0 = 0, *p1 = 0, *p2 = 0, *p3 = 0;
            int *pq0 = 0, *pq1 = 0, *pq2 = 0, *pq3 = 0;
            int pass, stage_offset = 0;
            int stop_height = cvRound((img->rows - win_size.height) / ystep);

            if( win_size.width < min_size.width || win_size.height < min_size.height )
                continue;

            LINFO("1.1");
            cvSetImagesForHaarClassifierCascade( cascade, sum, sqsum, tilted, factor );
            LINFO("1.2");
            cvZero( temp );

            if( do_canny_pruning )
            {
                equ_rect.x = cvRound(win_size.width*0.15);
                equ_rect.y = cvRound(win_size.height*0.15);
                equ_rect.width = cvRound(win_size.width*0.7);
                equ_rect.height = cvRound(win_size.height*0.7);

                p0 = (int*)(sumcanny->data.ptr + equ_rect.y*sumcanny->step) + equ_rect.x;
                p1 = (int*)(sumcanny->data.ptr + equ_rect.y*sumcanny->step)
                            + equ_rect.x + equ_rect.width;
                p2 = (int*)(sumcanny->data.ptr + (equ_rect.y + equ_rect.height)*sumcanny->step) + equ_rect.x;
                p3 = (int*)(sumcanny->data.ptr + (equ_rect.y + equ_rect.height)*sumcanny->step)
                            + equ_rect.x + equ_rect.width;

                pq0 = (int*)(sum->data.ptr + equ_rect.y*sum->step) + equ_rect.x;
                pq1 = (int*)(sum->data.ptr + equ_rect.y*sum->step)
                            + equ_rect.x + equ_rect.width;
                pq2 = (int*)(sum->data.ptr + (equ_rect.y + equ_rect.height)*sum->step) + equ_rect.x;
                pq3 = (int*)(sum->data.ptr + (equ_rect.y + equ_rect.height)*sum->step)
                            + equ_rect.x + equ_rect.width;
            }

            cascade->hid_cascade->count = split_stage;

            for( pass = 0; pass < npass; pass++ )
            {
#ifdef _OPENMP
    #pragma omp parallel for num_threads(max_threads), schedule(dynamic)
#endif
                for( int _iy = 0; _iy < stop_height; _iy++ )
                {
                    int iy = cvRound(_iy*ystep);
                    int _ix, _xstep = 1;
                    int stop_width = cvRound((img->cols - win_size.width) / ystep);
                    uchar* mask_row = temp->data.ptr + temp->step * iy;

                    for( _ix = 0; _ix < stop_width; _ix += _xstep )
                    {
                        int ix = cvRound(_ix*ystep); // it really should be ystep

                        if( pass == 0 )
                        {
                            int result;
                            _xstep = 2;

                            if( do_canny_pruning )
                            {
                                int offset;
                                int s, sq;

                                offset = iy*(sum->step/sizeof(p0[0])) + ix;
                                s = p0[offset] - p1[offset] - p2[offset] + p3[offset];
                                sq = pq0[offset] - pq1[offset] - pq2[offset] + pq3[offset];
                                if( s < 100 || sq < 20 )
                                    continue;
                            }

                            result = cvRunHaarClassifierCascade( cascade, cvPoint(ix,iy), 0 );
                            if( result > 0 )
                            {
                                if( pass < npass - 1 )
                                    mask_row[ix] = 1;
                                else
                                {
                                    CvRect rect = cvRect(ix,iy,win_size.width,win_size.height);
#ifndef _OPENMP
                                    cvSeqPush( seq, &rect );
#else
                                    cvSeqPush( seq_thread[omp_get_thread_num()], &rect );
#endif
                                }
                            }
                            if( result < 0 )
                                _xstep = 1;
                        }
                        else if( mask_row[ix] )
                        {
                            int result = cvRunHaarClassifierCascade( cascade, cvPoint(ix,iy),
                                                                     stage_offset );
                            if( result > 0 )
                            {
                                if( pass == npass - 1 )
                                {
                                    CvRect rect = cvRect(ix,iy,win_size.width,win_size.height);
#ifndef _OPENMP
                                    cvSeqPush( seq, &rect );
#else
                                    cvSeqPush( seq_thread[omp_get_thread_num()], &rect );
#endif
                                }
                            }
                            else
                                mask_row[ix] = 0;
                        }
                    }
                }
                stage_offset = cascade->hid_cascade->count;
                cascade->hid_cascade->count = cascade->count;
            }
        }
        LINFO("end");
    }

#ifdef _OPENMP
        // gather the results
        for( i = 0; i < max_threads; i++ )
        {
                CvSeq* s = seq_thread[i];
        int j, total = s->total;
        CvSeqBlock* b = s->first;
        for( j = 0; j < total; j += b->count, b = b->next )
            cvSeqPushMulti( seq, b->data, b->count );
        }
#endif

    if( min_neighbors != 0 )
    {
        // group retrieved rectangles in order to filter out noise
        int ncomp = cvSeqPartition( seq, 0, &idx_seq, is_equal, 0 );
        comps = (CvAvgComp*)cvAlloc( (ncomp+1)*sizeof(comps[0]));
        memset( comps, 0, (ncomp+1)*sizeof(comps[0]));

        // count number of neighbors
        for( i = 0; i < seq->total; i++ )
        {
            CvRect r1 = *(CvRect*)cvGetSeqElem( seq, i );
            int idx = *(int*)cvGetSeqElem( idx_seq, i );
            assert( (unsigned)idx < (unsigned)ncomp );

            comps[idx].neighbors++;

            comps[idx].rect.x += r1.x;
            comps[idx].rect.y += r1.y;
            comps[idx].rect.width += r1.width;
            comps[idx].rect.height += r1.height;
        }

        // calculate average bounding box
        for( i = 0; i < ncomp; i++ )
        {
            int n = comps[i].neighbors;
            if( n >= min_neighbors )
            {
                CvAvgComp comp;
                comp.rect.x = (comps[i].rect.x*2 + n)/(2*n);
                comp.rect.y = (comps[i].rect.y*2 + n)/(2*n);
                comp.rect.width = (comps[i].rect.width*2 + n)/(2*n);
                comp.rect.height = (comps[i].rect.height*2 + n)/(2*n);
                comp.neighbors = comps[i].neighbors;

                cvSeqPush( seq2, &comp );
            }
        }

        // filter out small face rectangles inside large face rectangles
        for( i = 0; i < seq2->total; i++ )
        {
            CvAvgComp r1 = *(CvAvgComp*)cvGetSeqElem( seq2, i );
            int j, flag = 1;

            for( j = 0; j < seq2->total; j++ )
            {
                CvAvgComp r2 = *(CvAvgComp*)cvGetSeqElem( seq2, j );
                int distance = cvRound( r2.rect.width * 0.2 );

                if( i != j &&
                    r1.rect.x >= r2.rect.x - distance &&
                    r1.rect.y >= r2.rect.y - distance &&
                    r1.rect.x + r1.rect.width <= r2.rect.x + r2.rect.width + distance &&
                    r1.rect.y + r1.rect.height <= r2.rect.y + r2.rect.height + distance &&
                    (r2.neighbors > MAX( 3, r1.neighbors ) || r1.neighbors < 3) )
                {
                    flag = 0;
                    break;
                }
            }

            if( flag )
            {
                cvSeqPush( result_seq, &r1 );
                // cvSeqPush( result_seq, &r1.rect );
            }
        }
    }


#ifdef _OPENMP
        for( i = 0; i < max_threads; i++ )
        {
                if( seq_thread[i] )
            cvReleaseMemStorage( &seq_thread[i]->storage );
        }
#endif

    cvReleaseMemStorage( &temp_storage );
    cvReleaseMat( &sum );
    cvReleaseMat( &sqsum );
    cvReleaseMat( &tilted );
    cvReleaseMat( &temp );
    cvReleaseMat( &sumcanny );
    cvReleaseMat( &norm_img );
    cvReleaseMat( &img_small );
    //cvFree( &comps );

    return result_seq;
}



void detect_and_draw( IplImage* img , std::vector<Rectangle> &facesRec)
{

  double scale = 1.3;
  IplImage* gray = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
  IplImage* small_img = cvCreateImage( cvSize( cvRound (img->width/scale),
        cvRound (img->height/scale)),
      8, 1 );
  int i;

  cvCvtColor( img, gray, CV_BGR2GRAY );

  cvResize( gray, small_img, CV_INTER_LINEAR );
  cvEqualizeHist( small_img, small_img );

  cvClearMemStorage( storage );

  int coi;
  CvMat stub;
  CvMat *iimg = cvGetMat(small_img, &stub, &coi );
  LINFO("Tracei %ix%i", iimg->width, iimg->height);

  if( cascade )
  {
    double t = (double)cvGetTickCount();
    LINFO("Size %ix%i", small_img->width, small_img->height);
    CvSeq* faces = cvHaarDetectObjects2( small_img, cascade, storage,
        1.1, 2, 0/*CV_HAAR_DO_CANNY_PRUNING*/,
        cvSize(30, 30) );
    t = (double)cvGetTickCount() - t;
    //printf( "detection time = %gms\n", t/((double)cvGetTickFrequency()*1000.) );
    for( i = 0; i < (faces ? faces->total : 0); i++ )
    {
      CvRect* r = (CvRect*)cvGetSeqElem( faces, i );

      Rectangle faceRect(Point2D<int>((int)(r->x*scale), (int)(r->y*scale)),
          Dims((int)(r->width*scale), (int)(r->height*scale)));
      facesRec.push_back(faceRect);
    }
  }

  cvReleaseImage( &gray );
  cvReleaseImage( &small_img );
}


int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Test ObjRec");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*mgr));
  mgr->addSubComponent(ofs);

  nub::soft_ref<BeoHeadBrain> beoHeadBrain(new BeoHeadBrain(*mgr));
  mgr->addSubComponent(beoHeadBrain);


  mgr->exportOptions(MC_RECURSE);

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "", 0, 0) == false)
    return 1;

  mgr->start();

  const char* cascade_name = "/usr/local/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml";
  cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );

  if( !cascade )
  {
    fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
    return -1;
  }
  storage = cvCreateMemStorage(0);

  Image<float> avgFace(256,256,ZEROS);

  // do post-command-line configs:

  beoHeadBrain->initHead();

  unsigned long imgNum  = 0;
  uint64 avgtime = 0; int avgn = 0;
  float faceDetRate = 0.0F;
  Timer timer;

  while(1)
  {
    Image< PixRGB<byte> > inputImg = beoHeadBrain->getLeftEyeImg();

    if (!inputImg.initialized())
      continue;

    Image<byte> greyInput = luminance(inputImg);

    std::vector<Rectangle> faces;
    detect_and_draw(img2ipl(inputImg), faces );

    Point2D<int> targetLoc(-1,-1);
    for(uint i=0; i<faces.size(); i++)
    {
      Point2D<int> faceCenter = Point2D<int>(
          faces[i].topLeft().i + faces[i].width()/2,
          faces[i].topLeft().j + faces[i].height()/2);

      beoHeadBrain->setTarget(faceCenter);
      targetLoc = faceCenter;

      Image<PixRGB<byte> > faceImg = crop(inputImg, faces[i]);

      ofs->writeRGB(faceImg, "faceImg", FrameInfo("faceImg", SRC_POS));

      char filename[255];
      sprintf(filename, "faces/face%.6lu.pnm", imgNum++);
      Raster::WriteRGB(faceImg, filename);

      ofs->updateNext();

      drawRect(inputImg, faces[i], PixRGB<byte>(255,0,0));
      avgn++;
    }


    targetLoc = beoHeadBrain->getTargetLoc();

    if (targetLoc.isValid())
      drawCircle(inputImg, targetLoc, 3, PixRGB<byte>(255,0,0));

    ofs->writeRGB(inputImg, "inputImg", FrameInfo("inputImg", SRC_POS));

                //compute the face detection Rate
    avgtime += timer.getReset();
    if (avgtime > 1000.0F)
    {
                        faceDetRate = avgn;
            avgtime = 0;
            avgn = 0;
    }


                //if (faceDetRate > 10)
                //{
                //        LINFO("Waking up");
                //        beoHeadBrain->wakeUp();
                //} else {
                //        //move head to relax position. aka sleep
                //        if (!beoHeadBrain->isSleeping())
                //                beoHeadBrain->gotoSleep();
                //}

  }
  mgr->stop();

  return 0;

}

