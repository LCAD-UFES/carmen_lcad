/*! @file Neuro/gistParams.H -- Gist parameters */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/gistParams.H $
// $Id: gistParams.H 6092 2006-01-13 19:59:17Z siagian $

// ####################################################################################
// Std implementations
// ####################################################################################

#define NUM_GIST_SUB_CHAN                7 // 4 (in ori) + 2 (in color) + 1 (in intens)
#define NUM_GIST_LEV                     4 // 4 pyramid levels for Orientation channel
#define NUM_GIST_FEAT                    21 // 1 + 4 + 16
#define NUM_GIST_CTR                     3
#define NUM_GIST_CTR_S                   2
#define NUM_GIST_SURR_DELTA              2
#define NUM_GIST_SURR_DELTA_S            3
#define NUM_GIST_CS                      NUM_GIST_CTR * NUM_GIST_SURR_DELTA
#define NUM_GIST_COL                     (4 * NUM_GIST_LEV +  2 * NUM_GIST_CS + 1 * NUM_GIST_CS)
#define NUM_GIST_OC_FEAT_SIZE            4 * NUM_GIST_LEV * NUM_GIST_FEAT // 4 in ori
#define NUM_GIST_CC_FEAT_SIZE            2 * NUM_GIST_CS * NUM_GIST_FEAT // 2 in ori
#define NUM_GIST_IC_FEAT_SIZE            1 * NUM_GIST_CS * NUM_GIST_FEAT // 1 in ori
#define NUM_GIST_CHAN                    (4 * NUM_GIST_LEV + 2 * NUM_GIST_CS + 1 * NUM_GIST_CS)
#define NUM_GIST_FEAT_SIZE               (NUM_GIST_OC_FEAT_SIZE + NUM_GIST_CC_FEAT_SIZE + NUM_GIST_IC_FEAT_SIZE)


// ####################################################################################
// FFT implementations
// ####################################################################################

#define NUM_GIST_FEAT_SIZE_FFT           16 * 16
