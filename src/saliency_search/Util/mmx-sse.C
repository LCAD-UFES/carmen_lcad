/*!@file Util/mmx-sse.C -- Optimized implementations of low-level functions
  for MMX/SSE */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Nitin Dhavale <dhavale@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/mmx-sse.C $
// $Id: mmx-sse.C 10118 2008-08-18 23:51:38Z ilab24 $
//

#include "Util/mmx-sse.H"
#include "Util/log.H"

// specific types only to the code that is in this file
typedef int int32;
typedef unsigned  char byte;
typedef float float32;

#ifdef INVT_CPU_OPTERON

#ifdef INVT_USE_SSE

//######################################################################
void sse_absDiff(const double *a, const double *b, double *diff, const int32 sz)
{
  static int32 rcx= sz>>2;
  static int32 rdx= sz & 0x3;

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .AG2;\n\t"
       ".AG1:;\n\t"
       "movupd  0(%%rsi), %%xmm0;\n\t" // xmm0 <- a3 a2 a1 a0
       "movupd  0(%%rdi), %%xmm1;\n\t" // xmm1 <- b3 b2 b1 b0
       "movupd  16(%%rsi), %%xmm2;\n\t"// xmm2 <- a7 a6 a5 a4
       "movupd  16(%%rdi), %%xmm3;\n\t"// xmm3 <- b7 b6 b5 b4
       "movupd  %%xmm0, %%xmm4;\n\t"   // xmm4 <- a3 a2 a1 a0
       "movupd  %%xmm1, %%xmm5;\n\t"   // xmm5 <- b3 b2 b1 b0
       "movupd  %%xmm2, %%xmm6;\n\t"   // xmm6 <- a7 a6 a5 a4
       "movupd  %%xmm3, %%xmm7;\n\t"   // xmm7 <- b7 b6 b5 b4
       "subpd   %%xmm1, %%xmm0;\n\t"   // xmm0 <- (a3-b3) .. (a1-b1) (a0-b0)
       "subpd   %%xmm3, %%xmm2;\n\t"   // xmm2 <- (a7-b7) .. (a5-b5) (a4-b4)
       "subpd   %%xmm4, %%xmm5;\n\t"   // xmm5 <- (b3-a3) .. (b1-a1) (b0-a0)
       "subpd   %%xmm6, %%xmm7;\n\t"   // xmm7 <- (b7-a7) .. (b5-a5) (b4-a4)
       "maxpd   %%xmm0, %%xmm5;\n\t"   // xmm5 <- max(xmm0,xmm5)
       "maxpd   %%xmm2, %%xmm7;\n\t"   // xmm7 <- max(xmm2,xmm7)
       "movupd  %%xmm5, 0(%%rbx);\n\t"
       "movupd  %%xmm7, 16(%%rbx);\n\t"
       "add $32, %%rsi;\n\t"
       "add $32, %%rdi;\n\t"
       "add $32, %%rbx;\n\t"
       "loop  .AG1;\n\t"
       ".AG2:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz .AG4;\n\t"
       ".AG3:;\n\t"
       "movsd 0(%%rsi), %%xmm0;\n\t"
       "movsd 0(%%rdi), %%xmm1;\n\t"
       "movsd %%xmm0, %%xmm2;\n\t"
       "movsd %%xmm1, %%xmm3;\n\t"
       "subsd %%xmm3, %%xmm2;\n\t"
       "subsd %%xmm0, %%xmm1;\n\t"
       "maxsd %%xmm2, %%xmm1;\n\t"
       "movsd %%xmm1, 0(%%rbx);\n\t"
       "add $8, %%rsi;\n\t"
       "add $8, %%rdi;\n\t"
       "add $8, %%rbx;\n\t"
       "loop .AG3;\n\t"
       ".AG4:;\n\t"
       :
       :"S"(a),"D"(b),"b"(diff), "c"(rcx), "d"(rdx)
       :"memory"
       );
}
#endif

#ifdef INVT_USE_MMXSSE2
//######################################################################
// speedup ~= 2.1
void sse2_absDiff(const float *a, const float *b, float *diff, const int32 sz)
{
  static int32 rcx= sz>>3;
  static int32 rdx= sz & 0x7;

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .AE2;\n\t"
       ".AE1:;\n\t"
       "movups  0(%%rsi), %%xmm0;\n\t" // xmm0 <- a3 a2 a1 a0
       "movups  0(%%rdi), %%xmm1;\n\t" // xmm1 <- b3 b2 b1 b0
       "movups  16(%%rsi), %%xmm2;\n\t"// xmm2 <- a7 a6 a5 a4
       "movups  16(%%rdi), %%xmm3;\n\t"// xmm3 <- b7 b6 b5 b4
       "movups  %%xmm0, %%xmm4;\n\t"   // xmm4 <- a3 a2 a1 a0
       "movups  %%xmm1, %%xmm5;\n\t"   // xmm5 <- b3 b2 b1 b0
       "movups  %%xmm2, %%xmm6;\n\t"   // xmm6 <- a7 a6 a5 a4
       "movups  %%xmm3, %%xmm7;\n\t"   // xmm7 <- b7 b6 b5 b4
       "subps   %%xmm1, %%xmm0;\n\t"   // xmm0 <- (a3-b3) .. (a1-b1) (a0-b0)
       "subps   %%xmm3, %%xmm2;\n\t"   // xmm2 <- (a7-b7) .. (a5-b5) (a4-b4)
       "subps   %%xmm4, %%xmm5;\n\t"   // xmm5 <- (b3-a3) .. (b1-a1) (b0-a0)
       "subps   %%xmm6, %%xmm7;\n\t"   // xmm7 <- (b7-a7) .. (b5-a5) (b4-a4)
       "maxps   %%xmm0, %%xmm5;\n\t"   // xmm5 <- max(xmm0,xmm5)
       "maxps   %%xmm2, %%xmm7;\n\t"   // xmm7 <- max(xmm2,xmm7)
       "movups  %%xmm5, 0(%%rbx);\n\t"
       "movups  %%xmm7, 16(%%rbx);\n\t"
       "add $32, %%rsi;\n\t"
       "add $32, %%rdi;\n\t"
       "add $32, %%rbx;\n\t"
       "loop  .AE1;\n\t"
       ".AE2:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz .AE4;\n\t"
       ".AE3:;\n\t"
       "movss 0(%%rsi), %%xmm0;\n\t"
       "movss 0(%%rdi), %%xmm1;\n\t"
       "movss %%xmm0, %%xmm2;\n\t"
       "movss %%xmm1, %%xmm3;\n\t"
       "subss %%xmm3, %%xmm2;\n\t"
       "subss %%xmm0, %%xmm1;\n\t"
       "maxss %%xmm2, %%xmm1;\n\t"
       "movss %%xmm1, 0(%%rbx);\n\t"
       "add $4, %%rsi;\n\t"
       "add $4, %%rdi;\n\t"
       "add $4, %%rbx;\n\t"
       "loop .AE3;\n\t"
       ".AE4:;\n\t"
       "emms;\n\t"
       :
       :"S"(a),"D"(b),"b"(diff), "c"(rcx), "d"(rdx)
       :"memory"
       );
}



//######################################################################
// speedup ~= 3.4
void sse2_absDiff(const int32 *a, const int32 *b, int32 *diff, const int32 sz)
{
  static int32 rcx= sz>>3;
  static int32 rdx= sz&0x7;

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .AF2;\n\t"
       ".AF1:;\n\t"
       "movdqu  0(%%rsi), %%xmm0;\n\t"
       "movdqu  0(%%rdi), %%xmm1;\n\t"
       "movdqu  16(%%rsi), %%xmm2;\n\t"
       "movdqu  16(%%rdi), %%xmm3;\n\t"
       "movdqu  %%xmm0, %%xmm4;\n\t"
       "movdqu  %%xmm1, %%xmm5;\n\t"
       "movdqu  %%xmm2, %%xmm6;\n\t"
       "movdqu  %%xmm3, %%xmm7;\n\t"
       "psubusw %%xmm1, %%xmm0;\n\t"
       "psubusw %%xmm3, %%xmm2;\n\t"
       "psubusw %%xmm4, %%xmm5;\n\t"
       "psubusw %%xmm6, %%xmm7;\n\t"
       "pmaxsw  %%xmm0, %%xmm5;\n\t"
       "pmaxsw  %%xmm2, %%xmm7;\n\t"
       "movdqu  %%xmm5, 0(%%rbx);\n\t"
       "movdqu  %%xmm7, 16(%%rbx);\n\t"
       "add $32, %%rsi;\n\t"
       "add $32, %%rdi;\n\t"
       "add $32, %%rbx;\n\t"
       "loop  .AF1;\n\t"
       ".AF2:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz .AF4;\n\t"
       ".AF3:;\n\t"
       "mov (%%rsi), %%rax;\n\t"
       "mov (%%rdi), %%rdx;\n\t"
       "cmp %%rdx, %%rax;\n\t"
       "ja .AF5;\n\t"
       "xchg %%rax, %%rdx;\n\t"
       ".AF5:;\n\t"
       "sub %%rdx, %%rax;\n\t"
       "mov %%rax, (%%rbx);\n\t"
       "add $4, %%rsi;\n\t"
       "add $4, %%rdi;\n\t"
       "add $4, %%rbx;\n\t"
       "loop .AF3;\n\t"
       ".AF4:;\n\t"
       "emms;\n\t"
       :
       :"S"(a),"D"(b),"b"(diff), "c"(rcx), "d"(rdx)
       :"memory"
       );
}


//######################################################################
// speedup ~=10.0!
void sse2_absDiff(const byte *a, const byte *b, byte *diff, const int32 sz)
{
  static int32 rcx= sz>>5;
  static int32 rdx= sz&0x1f;

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .AD2;\n\t"
       ".AD1:;\n\t"
       "movdqu  0(%%rsi), %%xmm0;\n\t" // xmm0<- a15 ... a3 a2 a1 a0
       "movdqu  0(%%rdi), %%xmm1;\n\t" // xmm1<- b15 ... b3 b2 b1 b0
       "movdqu  16(%%rsi), %%xmm2;\n\t"// xmm2<- a31 ... a18 a17 a16
       "movdqu  16(%%rdi), %%xmm3;\n\t"// xmm3<- b31 ... b18 b17 b16
       "movdqu  %%xmm0, %%xmm4;\n\t"   // xmm4<- a15 ... a3 a2 a1 a0
       "movdqu  %%xmm1, %%xmm5;\n\t"   // xmm5<- b15 ... b3 b2 b1 b0
       "movdqu  %%xmm2, %%xmm6;\n\t"   // xmm6<- a31 ... a18 a17 a16
       "movdqu  %%xmm3, %%xmm7;\n\t"   // xmm7<- b31 ... b18 b17 b16
       "psubusb %%xmm1, %%xmm0;\n\t"   // xmm0<-(a15-b15)...( a1-b1 )(a0-b0)
       "psubusb %%xmm3, %%xmm2;\n\t"   // xmm2<-(a31-b31)...(a17-b17)(a16-b16)
       "psubusb %%xmm4, %%xmm5;\n\t"   // xmm5<-(b15-a15)...(b17-a17)(b16-a16)
       "psubusb %%xmm6, %%xmm7;\n\t"   // xmm7<-(b31-a31)...(b17-a17)(b16-a16)
       "pmaxub  %%xmm0, %%xmm5;\n\t"   // xmm5<- max(xmm0,xmm5)
       "pmaxub  %%xmm2, %%xmm7;\n\t"   // xmm7<- max(xmm2,xmm7)
       "movdqu  %%xmm5, 0(%%rbx);\n\t"
       "movdqu  %%xmm7, 16(%%rbx);\n\t"
       "add $32, %%rsi;\n\t"
       "add $32, %%rdi;\n\t"
       "add $32, %%rbx;\n\t"
       "loop  .AD1;\n\t"
       ".AD2:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz .AD4;\n\t"
       ".AD3:;\n\t"
       "movb (%%rsi), %%al;\n\t"
       "movb (%%rdi), %%dl;\n\t"
       "cmpb %%dl, %%al;\n\t"
       "ja .AD5;\n\t"
       "xchgb %%al, %%dl;\n\t"
       ".AD5:;\n\t"
       "subb %%dl, %%al;\n\t"
       "movb %%al, (%%rbx);\n\t"
       "inc %%rbx;\n\t"
       "inc %%rsi;\n\t"
       "inc %%rdi;\n\t"
       "loop .AD3;\n\t"
       ".AD4:;\n\t"
       "emms;\n\t"
       :
       :"S"(a),"D"(b),"b"(diff), "c"(rcx), "d"(rdx)
       :"memory"
       );
}
#endif

#ifdef INVT_USE_SSE
//######################################################################
// speedup ~= 2.0
void sse_sum(const double *a, double *sum, const int32 sz)
{
  static int32 rcx = sz>>3;
  static int32 rdx = sz&0x7;

  asm (
       "pxor %%xmm4, %%xmm4;\n\t"
       "pxor %%xmm5, %%xmm5;\n\t"
       "pxor %%xmm6, %%xmm6;\n\t"
       "pxor %%xmm7, %%xmm7;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz  BE1;\n\t"
       ".BE0:\n\t"
       "movupd     0(%%rsi), %%xmm0;\n\t"
       "movupd  16(%%rsi), %%xmm1;\n\t"
       "movupd  32(%%rsi), %%xmm2;\n\t"
       "movupd  48(%%rsi), %%xmm3;\n\t"
       "addpd %%xmm0, %%xmm4;\n\t"
       "addpd %%xmm1, %%xmm5;\n\t"
       "addpd %%xmm2, %%xmm6;\n\t"
       "addpd %%xmm3, %%xmm7;\n\t"
       "add $64, %%rsi;\n\t"
       "loop .BE0;\n\t"
       "BE1:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "pxor %%xmm0, %%xmm0;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz BE2;\n\t"
       "BE3:;\n\t"
       "movupd 0(%%rsi), %%xmm1;\n\t"
       "addpd %%xmm1, %%xmm0;\n\t"
       "add $16, %%rsi;\n\t"
       "loop BE3;\n\t"
       "BE2:;\n\t"
       "addpd %%xmm4, %%xmm7;\n\t"
       "addpd %%xmm5, %%xmm7;\n\t"
       "addpd %%xmm6, %%xmm7;\n\t"
       "addpd %%xmm7, %%xmm0;\n\t"
       "movhpd %%xmm0, (%%rbx);\n\t"
       "addsd  (%%rbx), %%xmm0;\n\t"
       "movsd %%xmm0, (%%rbx);\n\t"
       "emms;\n\t"
       :
       :"S"(a), "b"(sum), "c"(rcx), "d"(rdx)
       :"memory"
       );
}
#endif

#ifdef INVT_USE_MMXSSE2
//######################################################################
//speedup ~= 4
void sse2_sum(const float *a, double *sum, const int32 sz)
{
  static int32 rcx = sz>>3;
  static int32 rdx = sz & 0x7;

  asm (
       "pxor %%xmm4, %%xmm4;\n\t"
       "pxor %%xmm5, %%xmm5;\n\t"
       "pxor %%xmm6, %%xmm6;\n\t"
       "pxor %%xmm7, %%xmm7;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz  BA1;\n\t"
       ".BA0:\n\t"
       "cvtps2pd  0(%%rsi), %%xmm0;\n\t"
       "cvtps2pd  8(%%rsi), %%xmm1;\n\t"
       "cvtps2pd  16(%%rsi), %%xmm2;\n\t"
       "cvtps2pd 24(%%rsi), %%xmm3;\n\t"
       "addpd %%xmm0, %%xmm4;\n\t"
       "addpd %%xmm1, %%xmm5;\n\t"
       "addpd %%xmm2, %%xmm6;\n\t"
       "addpd %%xmm3, %%xmm7;\n\t"
       "add $32, %%rsi;\n\t"
       "loop .BA0;\n\t"
       "BA1:;\n\t"
       "pxor %%xmm0, %%xmm0;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz BA2;\n\t"
       "BA3:;\n\t"
       "cvtps2pd 0(%%rsi), %%xmm1;\n\t"
       "addpd %%xmm1, %%xmm0;\n\t"
       "add $8, %%rsi;\n\t"
       "loop BA3;\n\t"
       "BA2:;\n\t"
       "addpd %%xmm4, %%xmm7;\n\t"
       "addpd %%xmm5, %%xmm7;\n\t"
       "addpd %%xmm6, %%xmm7;\n\t"
       "addpd %%xmm7, %%xmm0;\n\t"
       "movhpd %%xmm0, (%%rbx);\n\t"
       "addsd  (%%rbx), %%xmm0;\n\t"
       "movsd %%xmm0, (%%rbx);\n\t"
       "emms;\n\t"
       :
       :"S"(a), "b"(sum), "c"(rcx), "d"(rdx)
       :"memory"
       );
}


//######################################################################
// speedup ~= 4.0
void sse2_sum(const int32 *a, double *sum, const int32 sz)
{
  static int32 rcx = sz>>3;
  static int32 rdx = sz & 0x7;

  asm (
       "pxor %%xmm4, %%xmm4;\n\t"
       "pxor %%xmm5, %%xmm5;\n\t"
       "pxor %%xmm6, %%xmm6;\n\t"
       "pxor %%xmm7, %%xmm7;\n\t"
       "or %%rcx, %%rcx;\n\t"
       ".BC0:\n\t"
       "cvtdq2pd  0(%%rsi), %%xmm0;\n\t"
       "cvtdq2pd  8(%%rsi), %%xmm1;\n\t"
       "cvtdq2pd  16(%%rsi), %%xmm2;\n\t"
       "cvtdq2pd 24(%%rsi), %%xmm3;\n\t"
       "addpd %%xmm0, %%xmm4;\n\t"
       "addpd %%xmm1, %%xmm5;\n\t"
       "addpd %%xmm2, %%xmm6;\n\t"
       "addpd %%xmm3, %%xmm7;\n\t"
       "add $32, %%rsi;\n\t"
       "loop .BC0;\n\t"
       "BC1:;\n\t"
       "pxor %%xmm0, %%xmm0;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz BC2;\n\t"
       "BC3:;\n\t"
       "cvtdq2pd 0(%%rsi), %%xmm1;\n\t"
       "addpd %%xmm1, %%xmm0;\n\t"
       "add $8, %%rsi;\n\t"
       "loop BC3;\n\t"
       "BC2:;\n\t"
       "addpd %%xmm4, %%xmm7;\n\t"
       "addpd %%xmm5, %%xmm7;\n\t"
       "addpd %%xmm6, %%xmm7;\n\t"
       "addpd %%xmm7, %%xmm0;\n\t"
       "movhpd %%xmm0, (%%rbx);\n\t"
       "addsd  (%%rbx), %%xmm0;\n\t"
       "movsd %%xmm0, (%%rbx);\n\t"
       "emms;\n\t"
       :
       :"S"(a), "b"(sum), "c"(rcx), "d"(rdx)
       :"memory"
       );
}



//######################################################################
void sse2_sum(const byte *a, double *sum, const int32 sz)
{
  static int rcx = sz>>5;
  static int rdx = sz & 0x1f;

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz  BB1;\n\t"
       "pxor %%xmm7, %%xmm7;\n\t"
       "push %%rbx;\n\t"
       "push %%rdx;\n\t"
       "BB3:;\n\t"
       "pxor %%xmm5, %%xmm5;\n\t"
       "pxor %%xmm6, %%xmm6;\n\t"
       "movdqu (%%rsi), %%xmm0;\n\t"
       "movdqu 16(%%rsi), %%xmm1;\n\t"
       "psadbw %%xmm0, %%xmm5;\n\t"
       "psadbw %%xmm1, %%xmm6;\n\t"
       "pextrw $0, %%xmm5, %%rax;\n\t"
       "cvtsi2sd %%rax, %%xmm0;\n\t"
       "pextrw $4, %%xmm5, %%rbx;\n\t"
       "cvtsi2sd %%rbx, %%xmm1;\n\t"
       "pextrw $0, %%xmm6, %%rdx;\n\t"
       "cvtsi2sd %%rdx, %%xmm2;\n\t"
       "pextrw $4, %%xmm6, %%rdi;\n\t"
       "cvtsi2sd %%rdi, %%xmm3;\n\t"
       "addsd %%xmm0, %%xmm1;\n\t"
       "addsd %%xmm2, %%xmm3;\n\t"
       "addsd %%xmm1, %%xmm7;\n\t"
       "addsd %%xmm3, %%xmm7;\n\t"
       "add $32, %%rsi;\n\t"
       "loop BB3;\n\t"
       "pop %%rdx;\n\t"
       "pop %%rbx;\n\t"
       "BB1:;\n\t"
       "xor %%rdi, %%rdi;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz BB2;\n\t"
       "BB5:;\n\t"
       "xor %%rax, %%rax;\n\t"
       "movb (%%rsi), %%al;\n\t"
       "add %%rax, %%rdi;\n\t"
       "inc %%rsi;\n\t"
       "loop BB5;\n\t"
       "BB2:\n\t"
       "cvtsi2sd %%rdi, %%xmm0;\n\t"
       "addsd %%xmm0, %%xmm7;\n\t"
       "movhpd %%xmm7, (%%rbx);\n\t"
       "addsd  (%%rbx), %%xmm7;\n\t"
       "movsd %%xmm7, (%%rbx);\n\t"
       "BB6:;\n\t"
       "emms;\n\t"
       :
       :"S"(a), "c"(rcx),"b"(sum),"d"(rdx)
       :"memory","rax","rdi"
       );
}
#endif

#ifdef INVT_USE_SSE
//######################################################################
// speedup ~= 10 !
void sse_clampedDiff(const byte *a, const byte *b, byte *result, const int32 sz)
{
  int rcx = sz >> 6;
  int rdx = sz & 0x7f;

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .DA0;\n\t"
       ".DA1:;\n\t"
       "movdqu (%%rsi), %%xmm0;\n\t"
       "movdqu (%%rdi), %%xmm4;\n\t"
       "movdqu 16(%%rsi), %%xmm1;\n\t"
       "movdqu 16(%%rdi), %%xmm5;\n\t"
       "movdqu 32(%%rsi), %%xmm2;\n\t"
       "movdqu 32(%%rdi), %%xmm6;\n\t"
       "movdqu 48(%%rsi), %%xmm3;\n\t"
       "movdqu 48(%%rdi), %%xmm7;\n\t"
       "psubusb %%xmm4, %%xmm0;\n\t"
       "psubusb %%xmm5, %%xmm1;\n\t"
       "psubusb %%xmm6, %%xmm2;\n\t"
       "psubusb %%xmm7, %%xmm3;\n\t"
       "movdqu  %%xmm0, 0(%%rbx);\n\t"
       "movdqu  %%xmm1, 16(%%rbx);\n\t"
       "movdqu  %%xmm2, 32(%%rbx);\n\t"
       "movdqu  %%xmm3, 48(%%rbx);\n\t"
       "add $64, %%rsi;\n\t"
       "add $64, %%rdi;\n\t"
       "add $64, %%rbx;\n\t"
       "loop .DA1;\n\t"
       ".DA0:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz .DA2;\n\t"
       ".DA3:;\n\t"
       "movb (%%rsi), %%al;\n\t"
       "movb (%%rdi), %%dl;\n\t"
       "cmpb %%bl, %%al;\n\t"
       "ja .DA4;\n\t"
       "xchg %%al, %%bl;\n\t"
       ".DA4:;\n\t"
       "subb %%bl, %%al;\n\t"
       "movb %%al, (%%rbx);\n\t"
       "inc %%rsi;\n\t"
       "inc %%rdi;\n\t"
       "inc %%rbx;\n\t"
       "loop .DA3;\n\t"
       ".DA2:;\n\t"
       "emms;\n\t"
       :
       :"S"(a),"D"(b),"c"(rcx),"d"(rdx),"b"(result)
       );
}


//######################################################################
// speedup ~= 20 !
void sse_clampedDiff(const float32 *a, const float32 *b, float32 *result,
                        const int32 sz)
{
  int32 rcx=sz>>5;
  int32 rdx=sz&0x1f;

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .DB0;\n\t"
       ".DB1:;\n\t"
       "movups  0(%%rsi), %%xmm0;\n\t"
       "movups  0(%%rdi), %%xmm1;\n\t"
       "movups 16(%%rsi), %%xmm2;\n\t"
       "movups 16(%%rdi), %%xmm3;\n\t"
       "movups %%xmm1, %%xmm6;\n\t"
       "movups %%xmm3, %%xmm7;\n\t"
       "cmpps  $1, %%xmm0, %%xmm6;\n\t"
       "cmpps  $1, %%xmm2, %%xmm7;\n\t"
       "subps  %%xmm1, %%xmm0;\n\t"
       "subps  %%xmm3, %%xmm2;\n\t"
       "andps  %%xmm6, %%xmm0;\n\t"
       "andps  %%xmm7, %%xmm2;\n\t"
       "movups %%xmm0, (%%rbx);\n\t"
       "movups %%xmm2, 16(%%rbx);\n\t"
       "add  $32, %%rsi;\n\t"
       "add  $32, %%rdi;\n\t"
       "add  $32, %%rbx;\n\t"
       "loop .DB1;\n\t"
       ".DB0:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz .DB2;\n\t"
       ".DB3:;\n\t"
       "movss (%%rsi), %%xmm0;\n\t"
       "movss (%%rdi), %%xmm1;\n\t"
       "movss %%xmm1, %%xmm2;\n\t"
       "cmpss $1, %%xmm0,  %%xmm2;\n\t"
       "andps %%xmm2, %%xmm0;\n\t"
       "andps %%xmm2, %%xmm1;\n\t"
       "subss %%xmm1,  %%xmm0;\n\t"
       "movss %%xmm0,  (%%rbx);\n\t"
       "add $4, %%rsi;\n\t"
       "add $4, %%rdi;\n\t"
       "add $4, %%rbx;\n\t"
       "loop .DB3;\n\t"
       ".DB2:;\n\t"
       :
       :"S"(a), "D"(b), "b"(result), "c"(rcx), "d"(rdx)
       :"memory"
       );
}


//######################################################################
// speedup ~= 3
void sse_clampedDiff(const int32 *a, const int32 *b, int32 *c, const int32 sz)
{
  int32 rcx=sz>>3;
  int32 rdx=sz&0x7;
  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .DC0;\n\t"
       ".DC1:;\n\t"
       "movdqu 0(%%rsi), %%xmm0;\n\t" //xmm0=  a3     a2     a1     a0
       "movdqu 0(%%rdi), %%xmm1;\n\t" //xmm1=  b3     b2     b1     b0
       "movdqu 16(%%rsi), %%xmm3;\n\t"//xmm3=  a7     a6     a5     a4
       "movdqu 16(%%rdi), %%xmm4;\n\t"//xmm4=  b7     b6     b5     b4
       "movdqu  %%xmm0, %%xmm2;\n\t"  //xmm2=  a3     a2     a1     a0
       "movdqu  %%xmm3, %%xmm5;\n\t"  //xmm5=  a7     a6     a5     a4
       "pcmpgtd %%xmm1, %%xmm2;\n\t"  //xmm2=(a3>b3)(a2>b2)(a1>b1)(a0>b0)
       "pcmpgtd %%xmm4, %%xmm5;\n\t"  //xmm5=(a7>b7)(a6>b6)(b5>a5)(a4>b4)
       "psubd   %%xmm1, %%xmm0;\n\t"  //xmm0=(a3-b3)(a2-b2)(a1-b1)(a0-b0)
       "psubd   %%xmm4, %%xmm3;\n\t"  //xmm3=(a7-b7)(a6-b6)(a5-b5)(a4-b4)
       "pand    %%xmm2, %%xmm0;\n\t"
       "pand    %%xmm5, %%xmm3;\n\t"
       "movdqu  %%xmm0, (%%rbx);\n\t"
       "movdqu  %%xmm3, 16(%%rbx);\n\t"
       "add $32, %%rsi;\n\t"
       "add $32, %%rdi;\n\t"
       "add $32, %%rbx;\n\t"
       "loop .DC1;\n\t"
       ".DC0:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or  %%rcx, %%rcx;\n\t"
       "jz .DC2;\n\t"
       ".DC3:;\n\t"
       "movsd 0(%%rsi), %%xmm0;\n\t"
       "movsd 0(%%rdi), %%xmm1;\n\t"
       "movdqu %%xmm0, %%xmm2;\n\t"
       "pcmpgtd %%xmm1, %%xmm2;\n\t"
       "psubd   %%xmm1, %%xmm0;\n\t"
       "pand    %%xmm2, %%xmm0;\n\t"
       "movsd    %%xmm0, (%%rbx);\n\t"
       "add $4, %%rsi;\n\t"
       "add $4, %%rdi;\n\t"
       "add $4, %%rbx;\n\t"
       "loop .DC3;\n\t"
       ".DC2:;\n\t"
       :
       :"S"(a), "D"(b), "c"(rcx), "d"(rdx), "b"(c)
       :"memory"
       );
}


//######################################################################
// speedup ~= 4-5
void sse_binaryReverse(const byte *a, byte *result, const byte val, const
                                int32 sz)
{
  static unsigned int rcx=(sz>>7);
  static unsigned int rdx=sz&0x7f;

  byte pVal[16];

  memset(result, val, 16);

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .FA0;\n\t"
       ".FA1:;\n\t"
       "movdqu  0(%%rbx), %%xmm0;\n\t"
       "movdqu  0(%%rbx), %%xmm1;\n\t"
       "movdqu  %%xmm0, %%xmm2;\n\t"
       "movdqu  %%xmm1, %%xmm3;\n\t"
       "movdqu  %%xmm0, %%xmm4;\n\t"
       "movdqu  %%xmm1, %%xmm5;\n\t"
       "movdqu  %%xmm0, %%xmm6;\n\t"
       "movdqu  %%xmm1, %%xmm7;\n\t"
       "psubb (%%rsi), %%xmm0;\n\t"
       "psubb 16(%%rsi), %%xmm1;\n\t"
       "psubb 32(%%rsi), %%xmm2;\n\t"
       "psubb 48(%%rsi), %%xmm3;\n\t"
       "psubb 64(%%rsi), %%xmm4;\n\t"
       "psubb 80(%%rsi), %%xmm5;\n\t"
       "psubb 96(%%rsi), %%xmm6;\n\t"
       "psubb 112(%%rsi), %%xmm7;\n\t"
       "movdqu %%xmm0, (%%rdi);\n\t"
       "movdqu %%xmm1, 16(%%rdi);\n\t"
       "movdqu %%xmm2, 32(%%rdi);\n\t"
       "movdqu %%xmm3, 48(%%rdi);\n\t"
       "movdqu %%xmm4, 64(%%rdi);\n\t"
       "movdqu %%xmm5, 80(%%rdi);\n\t"
       "movdqu %%xmm6, 96(%%rdi);\n\t"
       "movdqu %%xmm7, 112(%%rdi);\n\t"
       "add $128, %%rdi;\n\t"
       "add $128, %%rsi;\n\t"
       "loop .FA1;\n\t"
       ".FA0:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz .FA2;\n\t"
       "movb (%%rbx), %%dl;\n\t"
       ".FA3:;\n\t"
       "movb %%dl, %%dh;\n\t"
       "movb (%%rsi), %%al;\n\t"
       "subb %%al, %%dh;\n\t"
       "movb %%dh, (%%rdi);\n\t"
       "inc %%rsi;\n\t"
       "inc %%rdi;\n\t"
       "loop .FA3;\n\t"
       ".FA2:;\n\t"
       :
       :"S"(a), "D"(result), "b"(pVal),"c"(rcx),"d"(rdx)
       :"memory","rax"
       );
}


//######################################################################
// speedup ~= 2
void sse_binaryReverse(const float *a, float *result, const float val,
                                const int sz)
{
  static unsigned int rcx = sz>>5;
  static unsigned int rdx = sz&0x1f;
  int i;
  float pVal[16];

  for(i=0;i<16;++i)
    pVal[i] = val;


  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .FB4;\n\t"
       ".FB2:;\n\t"
       "movups (%%rbx), %%xmm0;\n\t"
       "movups (%%rbx), %%xmm1;\n\t"
       "movups %%xmm0, %%xmm2;\n\t"
       "movups %%xmm1, %%xmm3;\n\t"
       "movups %%xmm0, %%xmm4;\n\t"
       "movups %%xmm1, %%xmm5;\n\t"
       "movups %%xmm0, %%xmm6;\n\t"
       "movups %%xmm1, %%xmm7;\n\t"
       "psubq (%%rsi), %%xmm0;\n\t"
       "psubq 16(%%rsi), %%xmm1;\n\t"
       "psubq 32(%%rsi), %%xmm2;\n\t"
       "psubq 48(%%rsi), %%xmm3;\n\t"
       "psubq 64(%%rsi), %%xmm4;\n\t"
       "psubq 80(%%rsi), %%xmm5;\n\t"
       "psubq 96(%%rsi), %%xmm6;\n\t"
       "psubq 112(%%rsi), %%xmm7;\n\t"
       "movups %%xmm0,  0(%%rdi);\n\t"
       "movups %%xmm1, 16(%%rdi);\n\t"
       "movups %%xmm2, 32(%%rdi);\n\t"
       "movups %%xmm3, 48(%%rdi);\n\t"
       "movups %%xmm4, 64(%%rdi);\n\t"
       "movups %%xmm5, 80(%%rdi);\n\t"
       "movups %%xmm6, 96(%%rdi);\n\t"
       "movups %%xmm7,112(%%rdi);\n\t"
       "add $128, %%rsi;\n\t"
       "add $128, %%rdi;\n\t"
       "loop .FB2;\n\t"
       ".FB4:\n\t"
       "or  %%rdx, %%rdx;\n\t"
       "jz .FB1;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       ".FB3:;\n\t"
       "movss 0(%%rbx), %%xmm0;\n\t"
       "subss (%%rsi), %%xmm0;\n\t"
       "movups %%xmm0, (%%rdi);\n\t"
       "add $16, %%rsi;\n\t"
       "add $16, %%rdi;\n\t"
       "loop .FB3;\n\t"
       ".FB1:;\n\t"
       :
       :"S"(a), "D"(result), "b"(pVal),"c"(rcx),"d"(rdx)
       :"memory","rax"
       );
}



//######################################################################

void sse_binaryReverse(const int32 *a, int32 *result, const int32 val,
                        const int32 sz)
{
  int32 rcx=sz>>5;
  int32 rdx=sz&31;
  int32 pVal[16];
  int i;

  for(i=0;i<16;++i) pVal[i] = val;

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .FC4;\n\t"
       ".FC2:;\n\t"
       "movdqu (%%rbx), %%xmm0;\n\t"
       "movdqu (%%rbx), %%xmm1;\n\t"
       "movdqu %%xmm0, %%xmm2;\n\t"
       "movdqu %%xmm1, %%xmm3;\n\t"
       "movdqu %%xmm0, %%xmm4;\n\t"
       "movdqu %%xmm1, %%xmm5;\n\t"
       "movdqu %%xmm0, %%xmm6;\n\t"
       "movdqu %%xmm1, %%xmm7;\n\t"
       "psubd  (%%rsi), %%xmm0;\n\t"
       "psubd  16(%%rsi), %%xmm1;\n\t"
       "psubd  32(%%rsi), %%xmm2;\n\t"
       "psubd  48(%%rsi), %%xmm3;\n\t"
       "psubd  64(%%rsi), %%xmm4;\n\t"
       "psubd  80(%%rsi), %%xmm5;\n\t"
       "psubd  96(%%rsi), %%xmm6;\n\t"
       "psubd  112(%%rsi), %%xmm7;\n\t"
       "movdqu %%xmm0,  0(%%rdi);\n\t"
       "movdqu %%xmm1, 16(%%rdi);\n\t"
       "movdqu %%xmm2, 32(%%rdi);\n\t"
       "movdqu %%xmm3, 48(%%rdi);\n\t"
       "movdqu %%xmm4, 64(%%rdi);\n\t"
       "movdqu %%xmm5, 80(%%rdi);\n\t"
       "movdqu %%xmm6, 96(%%rdi);\n\t"
       "movdqu %%xmm7,112(%%rdi);\n\t"
       "add $128, %%rsi;\n\t"
       "add $128, %%rdi;\n\t"
       "loop .FC2;\n\t"
       ".FC4:;\n\t"
       "or  %%rdx, %%rdx;\n\t"
       "jz .FC1;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       ".FC3:;\n\t"
       "movdqu 0(%%rbx), %%xmm0;\n\t"
       "psubd (%%rsi), %%xmm0;\n\t"
       "movups %%xmm0, (%%rdi);\n\t"
       "add $16, %%rsi;\n\t"
       "add $16, %%rdi;\n\t"
       "loop .FC3;\n\t"
       ".FC1:;\n\t"
       :
       :"S"(a), "D"(result), "b"(pVal),"c"(rcx),"d"(rdx)
       :"memory","rax"
       );
}



//######################################################################

void sse_cvt_byte_to_int(const byte *a, int32 *b, const int32 sz)
{
  int32 rcx=sz>>4;
  int32 rdx=sz&0xf;

  asm(
      "or %%rcx, %%rcx;\n\t"
      "jz .GA4;\n\t"
      "pxor %%xmm0, %%xmm0;\n\t"
      ".GA2:;\n\t"
      "movdqu 0(%%rsi), %%xmm1;\n\t"
      "movdqa %%xmm1, %%xmm2;\n\t"
      "movdqa %%xmm1, %%xmm3;\n\t"
      "movdqa %%xmm1, %%xmm4;\n\t"
      "psrldq $4, %%xmm2;\n\t"
      "psrldq $8, %%xmm3;\n\t"
      "psrldq $12, %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "movdqu %%xmm1, (%%rdi);\n\t"
      "movdqu %%xmm2, 16(%%rdi);\n\t"
      "movdqu %%xmm3, 32(%%rdi);\n\t"
      "movdqu %%xmm4, 48(%%rdi);\n\t"
      "add $16, %%rsi;\n\t"
      "add $64, %%rdi;\n\t"
      "loop .GA2;\n\t"
      ".GA4:;\n\t"
      "or %%rdx, %%rdx;\n\t"
      "jz .GA1;\n\t"
      "mov %%rdx, %%rcx;\n\t"
      ".GA3:;\n\t"
      "xor %%rax, %%rax;\n\t"
      "movb (%%rsi), %%al;\n\t"
      "mov %%rax, (%%rdi);\n\t"
      "inc %%rsi;\n\t"
      "add $4, %%rdi;\n\t"
      "loop .GA3;\n\t"
      ".GA1:;"
      :
      :"S"(a), "D"(b), "c"(rcx),"d"(rdx)
      :"memory"
      );


}

#endif

#ifdef INVT_USE_MMXSSE2

//######################################################################
// speedup ~= 1.5
void sse2_cvt_byte_to_float(const byte *a, float32 *b, const int32 sz)
{
  int32 rcx=sz>>4;
  int32 rdx=sz&0xf;

  asm(
      "or %%rcx, %%rcx;\n\t"
      "jz .GB4;\n\t"
      ".GB2:;\n\t"
      "pxor %%xmm0, %%xmm0;\n\t"
      "movdqu 0(%%rsi), %%xmm1;\n\t"
      "movdqu 4(%%rsi), %%xmm2;\n\t"
      "movdqu 8(%%rsi), %%xmm3;\n\t"
      "movdqu 12(%%rsi), %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "cvtdq2ps %%xmm1, %%xmm1;\n\t"
      "cvtdq2ps %%xmm2, %%xmm2;\n\t"
      "movups  %%xmm1, (%%rdi);\n\t"
      "movups  %%xmm2, 16(%%rdi);\n\t"
      "cvtdq2ps %%xmm3, %%xmm3;\n\t"
      "cvtdq2ps %%xmm4, %%xmm4;\n\t"
      "movups  %%xmm3, 32(%%rdi);\n\t"
      "movups  %%xmm4, 48(%%rdi);\n\t"
      "add $16, %%rsi;\n\t"
      "add $64, %%rdi;\n\t"
      "loop .GB2;\n\t"
      ".GB4:;\n\t"
      "or %%rdx, %%rdx;\n\t"
      "jz .GB1;\n\t"
      "mov %%rdx, %%rcx;\n\t"
      ".GB3:;\n\t"
      "xor %%rax, %%rax;\n\t"
      "movb (%%rsi), %%al;\n\t"
      "movd %%rax, %%xmm0;\n\t"
      "cvtdq2ps %%xmm0, %%xmm1;\n\t"
      "movss %%xmm1, (%%rdi);\n\t"
      "inc %%rsi;\n\t"
      "add $4, %%rdi;\n\t"
      "loop .GB3;\n\t"
      ".GB1:;"
      :
      :"S"(a), "D"(b), "c"(rcx),"d"(rdx)
      :"memory"
      );
}



//######################################################################
// speedup ~= 1.15
void sse2_cvt_byte_to_double(const byte *a, double *b, int32 sz)
{
  int32 rcx=sz>>3;
  int32 rdx=sz&0x7;

  asm(
      "or %%rcx, %%rcx;\n\t"
      "jz .GC4;\n\t"
      ".GC2:;\n\t"
      "pxor %%xmm0, %%xmm0;\n\t"
      "movdqu 0(%%rsi), %%xmm1;\n\t"
      "movdqu 2(%%rsi), %%xmm2;\n\t"
      "movdqu 4(%%rsi), %%xmm3;\n\t"
      "movdqu 6(%%rsi), %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "cvtdq2pd %%xmm1, %%xmm1;\n\t"
      "cvtdq2pd %%xmm2, %%xmm2;\n\t"
      "movupd  %%xmm1, (%%rdi);\n\t"
      "movupd  %%xmm2, 16(%%rdi);\n\t"
      "cvtdq2pd %%xmm3, %%xmm3;\n\t"
      "cvtdq2pd %%xmm4, %%xmm4;\n\t"
      "movupd  %%xmm3, 32(%%rdi);\n\t"
      "movupd  %%xmm4, 48(%%rdi);\n\t"
      "add $8, %%rsi;\n\t"
      "add $64, %%rdi;\n\t"
      "loop .GC2;\n\t"
      ".GC4:;\n\t"
      "or %%rdx, %%rdx;\n\t"
      "jz .GC1;\n\t"
      "mov %%rdx, %%rcx;\n\t"
      ".GC3:;\n\t"
      "xor %%rax, %%rax;\n\t"
      "movb (%%rsi), %%al;\n\t"
      "movd %%rax, %%xmm0;\n\t"
      "cvtdq2pd %%xmm0, %%xmm1;\n\t"
      "movsd %%xmm1, (%%rdi);\n\t"
      "inc %%rsi;\n\t"
      "add $8, %%rdi;\n\t"
      "loop .GC3;\n\t"
      ".GC1:;"
      :
      :"S"(a), "D"(b), "c"(rcx),"d"(rdx)
      :"memory"
      );

}



//######################################################################

void sse2_cvt_int_to_float(const int32 *a, float *b, const int32 sz)
{
  int32 rcx=sz>>5;
  int32 rdx=sz&0x1f;

  asm(
      "or %%rcx, %%rcx;\n\t"
      "jz .GD4;\n\t"
      ".GD2:;\n\t"
      "movdqu 0(%%rsi), %%xmm0;\n\t"
      "movdqu 16(%%rsi), %%xmm1;\n\t"
      "movdqu 32(%%rsi), %%xmm2;\n\t"
      "movdqu 48(%%rsi), %%xmm3;\n\t"
      "movdqu 64(%%rsi), %%xmm4;\n\t"
      "movdqu 80(%%rsi), %%xmm5;\n\t"
      "movdqu 96(%%rsi), %%xmm6;\n\t"
      "movdqu 112(%%rsi), %%xmm7;\n\t"
      "cvtdq2ps %%xmm0, %%xmm0;\n\t"
      "cvtdq2ps %%xmm1, %%xmm1;\n\t"
      "cvtdq2ps %%xmm2, %%xmm2;\n\t"
      "cvtdq2ps %%xmm3, %%xmm3;\n\t"
      "cvtdq2ps %%xmm4, %%xmm4;\n\t"
      "cvtdq2ps %%xmm5, %%xmm5;\n\t"
      "cvtdq2ps %%xmm6, %%xmm6;\n\t"
      "cvtdq2ps %%xmm7, %%xmm7;\n\t"
      "movups %%xmm0, 0(%%rdi);\n\t"
      "movups %%xmm1, 16(%%rdi);\n\t"
      "movups %%xmm2, 32(%%rdi);\n\t"
      "movups %%xmm3, 48(%%rdi);\n\t"
      "movups %%xmm4, 64(%%rdi);\n\t"
      "movups %%xmm5, 80(%%rdi);\n\t"
      "movups %%xmm6, 96(%%rdi);\n\t"
      "movups %%xmm7, 112(%%rdi);\n\t"
      "add $128, %%rsi;\n\t"
      "add $128, %%rdi;\n\t"
      "dec %%rcx;\n\t"
      "jnz .GD2;\n\t"
      ".GD4:;\n\t"
      "or %%rdx, %%rdx;\n\t"
      "jz .GD1;\n\t"
      "mov %%rdx, %%rcx;\n\t"
      ".GD3:;\n\t"
      "movsd (%%rsi), %%xmm0;\n\t"
      "cvtdq2ps %%xmm0, %%xmm0;\n\t"
      "movss %%xmm0, (%%rdi);\n\t"
      "add $4, %%rsi;\n\t"
      "add $4, %%rdi;\n\t"
      "loop .GD3;\n\t"
      ".GD1:;"
      :
      :"S"(a), "D"(b), "c"(rcx),"d"(rdx)
      :"memory"
      );

}

//######################################################################
// speedup ~= 1.2
void sse2_cvt_int_to_double(const int32 *a, double *b, const int32 sz)
{
  int32 rcx=sz>>4;
  int32 rdx=sz&0xf;

  asm(
      "or %%rcx, %%rcx;\n\t"
      "jz .GE4;\n\t"
      ".GE2:;\n\t"
      "movdqu 0(%%rsi), %%xmm0;\n\t"
      "movdqu  8(%%rsi), %%xmm1;\n\t"
      "movdqu 16(%%rsi), %%xmm2;\n\t"
      "movdqu 24(%%rsi), %%xmm3;\n\t"
      "movdqu 32(%%rsi), %%xmm4;\n\t"
      "movdqu 40(%%rsi), %%xmm5;\n\t"
      "movdqu 48(%%rsi), %%xmm6;\n\t"
      "movdqu 56(%%rsi), %%xmm7;\n\t"
      "cvtdq2pd %%xmm0, %%xmm0;\n\t"
      "cvtdq2pd %%xmm1, %%xmm1;\n\t"
      "cvtdq2pd %%xmm2, %%xmm2;\n\t"
      "cvtdq2pd %%xmm3, %%xmm3;\n\t"
      "cvtdq2pd %%xmm4, %%xmm4;\n\t"
      "cvtdq2pd %%xmm5, %%xmm5;\n\t"
      "cvtdq2pd %%xmm6, %%xmm6;\n\t"
      "cvtdq2pd %%xmm7, %%xmm7;\n\t"
      "movups %%xmm0, 0(%%rdi);\n\t"
      "movups %%xmm1, 16(%%rdi);\n\t"
      "movups %%xmm2, 32(%%rdi);\n\t"
      "movups %%xmm3, 48(%%rdi);\n\t"
      "movups %%xmm4, 64(%%rdi);\n\t"
      "movups %%xmm5, 80(%%rdi);\n\t"
      "movups %%xmm6, 96(%%rdi);\n\t"
      "movups %%xmm7, 112(%%rdi);\n\t"
      "add $64, %%rsi;\n\t"
      "add $128, %%rdi;\n\t"
      "dec %%rcx;\n\t"
      "jnz .GE2;\n\t"
      ".GE4:;\n\t"
      "or %%rdx, %%rdx;\n\t"
      "jz .GE1;\n\t"
      "mov %%rdx, %%rcx;\n\t"
      ".GE3:;\n\t"
      "movsd (%%rsi), %%xmm0;\n\t"
      "cvtdq2pd %%xmm0, %%xmm0;\n\t"
      "movsd %%xmm0, (%%rdi);\n\t"
      "add $4, %%rsi;\n\t"
      "add $8, %%rdi;\n\t"
      "loop .GE3;\n\t"
      ".GE1:;"
      :
      :"S"(a), "D"(b), "c"(rcx),"d"(rdx)
      :"memory"
      );

}

//######################################################################
void sse2_cvt_float_to_int(const float *a, int *b, const int32 sz)
{
  int32 rcx=sz;
  int32 rdx=sz;

  asm (
       "or %%rcx, %%rcx;\n\t"
       "jz .GF1;\n\t"
       ".GF2:;\n\t"
       "movdqu 0(%%rsi), %%xmm0;\n\t"
       "movdqu  8(%%rsi), %%xmm1;\n\t"
       "movdqu 16(%%rsi), %%xmm2;\n\t"
       "movdqu 24(%%rsi), %%xmm3;\n\t"
       "movdqu 32(%%rsi), %%xmm4;\n\t"
       "movdqu 40(%%rsi), %%xmm5;\n\t"
       "movdqu 48(%%rsi), %%xmm6;\n\t"
       "movdqu 56(%%rsi), %%xmm7;\n\t"
       "cvtps2dq %%xmm0, %%xmm0;\n\t"
       "cvtps2dq %%xmm1, %%xmm1;\n\t"
       "cvtps2dq %%xmm2, %%xmm2;\n\t"
       "cvtps2dq %%xmm3, %%xmm3;\n\t"
       "cvtps2dq %%xmm4, %%xmm4;\n\t"
       "cvtps2dq %%xmm5, %%xmm5;\n\t"
       "cvtps2dq %%xmm6, %%xmm6;\n\t"
       "cvtps2dq %%xmm7, %%xmm7;\n\t"
       "movdqu %%xmm0, 0(%%rdi);\n\t"
       "movdqu %%xmm1, 16(%%rdi);\n\t"
       "movdqu %%xmm2, 32(%%rdi);\n\t"
       "movdqu %%xmm3, 48(%%rdi);\n\t"
       "movdqu %%xmm4, 64(%%rdi);\n\t"
       "movdqu %%xmm5, 80(%%rdi);\n\t"
       "movdqu %%xmm6, 96(%%rdi);\n\t"
       "movdqu %%xmm7, 112(%%rdi);\n\t"
       "add $64, %%rsi;\n\t"
       "add $128, %%rdi;\n\t"
       "dec %%rcx;\n\t"
       "jnz .GF2;\n\t"
       ".GF4:;\n\t"
       "or %%rdx, %%rdx;\n\t"
       "jz .GF1;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       ".GF3:;\n\t"
       "movsd (%%rsi), %%xmm0;\n\t"
       "cvtps2dq %%xmm0, %%xmm0;\n\t"
       "movsd  %%xmm0, (%%rdi);\n\t"
       "add $4, %%rsi;\n\t"
       "add $8, %%rdi;\n\t"
       "loop .GF3;\n\t"
       ".GF1:;"
       :
       :"S"(a), "D"(b), "c"(rcx),"d"(rdx)
       :"memory"
       );

}



//######################################################################
void sse2_cvt_float_to_double(const float *a, double *b, const int32 sz)
{
  int32 rcx=sz>>4;
  int32 rdx=sz&0xf;

  asm(
      "or %%rcx, %%rcx;\n\t"
      "jz .GG4;\n\t"
      ".GG2:;\n\t"
      "movups 0(%%rsi), %%xmm0;\n\t"
      "movups  8(%%rsi), %%xmm1;\n\t"
      "movups 16(%%rsi), %%xmm2;\n\t"
      "movups 24(%%rsi), %%xmm3;\n\t"
      "movups 32(%%rsi), %%xmm4;\n\t"
      "movups 40(%%rsi), %%xmm5;\n\t"
      "movups 48(%%rsi), %%xmm6;\n\t"
      "movups 56(%%rsi), %%xmm7;\n\t"
      "cvtps2pd %%xmm0, %%xmm0;\n\t"
      "cvtps2pd %%xmm1, %%xmm1;\n\t"
      "cvtps2pd %%xmm2, %%xmm2;\n\t"
      "cvtps2pd %%xmm3, %%xmm3;\n\t"
      "cvtps2pd %%xmm4, %%xmm4;\n\t"
      "cvtps2pd %%xmm5, %%xmm5;\n\t"
      "cvtps2pd %%xmm6, %%xmm6;\n\t"
      "cvtps2pd %%xmm7, %%xmm7;\n\t"
      "movupd %%xmm0, 0(%%rdi);\n\t"
      "movupd %%xmm1, 16(%%rdi);\n\t"
      "movupd %%xmm2, 32(%%rdi);\n\t"
      "movupd %%xmm3, 48(%%rdi);\n\t"
      "movupd %%xmm4, 64(%%rdi);\n\t"
      "movupd %%xmm5, 80(%%rdi);\n\t"
      "movupd %%xmm6, 96(%%rdi);\n\t"
      "movupd %%xmm7, 112(%%rdi);\n\t"
      "add $64, %%rsi;\n\t"
      "add $128, %%rdi;\n\t"
      "dec %%rcx;\n\t"
      "jnz .GG2;\n\t"
      ".GG4:;\n\t"
      "or %%rdx, %%rdx;\n\t"
      "jz .GG1;\n\t"
      "mov %%rdx, %%rcx;\n\t"
      ".GG3:;\n\t"
      "movsd (%%rsi), %%xmm0;\n\t"
      "cvtps2pd %%xmm0, %%xmm0;\n\t"
      "movsd %%xmm0, (%%rdi);\n\t"
      "add $4, %%rsi;\n\t"
      "add $8, %%rdi;\n\t"
      "loop .GG3;\n\t"
      ".GG1:;"
      :
      :"S"(a), "D"(b), "c"(rcx),"d"(rdx)
      :"memory"
      );
}

#endif

#ifdef INVT_USE_SSE

//######################################################################
void sse_lowPass3x(const float *a, float *b, const int h, const int w)
{
  const float coeffs[] = { 3.0, 1.0, 1.0, 1.0, 4.0, 4.0, 4.0, 4.0};
  int rdx = (w-2)/12;
  int rax = (w-2)%12;

  asm (
       //       "movups 16(%%rbx), %%xmm7;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz  .HA1;\n\t"
       ".HA2:;\n\t"

       // *dptr++ = (sptr[0]+sptr[0]+sptr[1])/3.0
       "movss 0(%%rsi), %%xmm1;\n\t"  // xmm1 <- sptr[0]
       "movss 4(%%rsi), %%xmm2;\n\t" // xmm2 <- sptr[1]
       "addss %%xmm1, %%xmm1;\n\t"   // xmm2 <- sptr[0] + sptr[0]
       "addss %%xmm1, %%xmm2;\n\t"   // xmm2 <- xmm2 + sptr[1]
       "divss (%%rbx), %%xmm2;\n\t" // xmm2 <- xmm2/3.0
       "movss %%xmm2, (%%rdi);\n\t"  // *dptr <- xmm2
       "add  $4, %%rdi;\n\t"        // ++dptr

       //  for (int i = 0; i < w - 2; i ++)
       "or %%rdx, %%rdx;\n\t"
       "jz .HA4;\n\t"

       "push %%rdx;\n\t"
       ".HA3:;\n\t"
       "movups 00(%%rsi),  %%xmm0;\n\t"
       "movups 04(%%rsi),  %%xmm1;\n\t"
       "movups 8(%%rsi),  %%xmm2;\n\t"
       "movups 16(%%rsi),  %%xmm3;\n\t"
       "movups 20(%%rsi),  %%xmm4;\n\t"
       "movups 24(%%rsi),  %%xmm5;\n\t"
       "movups 32(%%rsi),  %%xmm6;\n\t"
       "movups 36(%%rsi),  %%xmm7;\n\t"
       "addps  %%xmm1, %%xmm0;\n\t"
       "addps  %%xmm4, %%xmm3;\n\t"
       "addps  %%xmm1, %%xmm0;\n\t"
       "addps  %%xmm4, %%xmm3;\n\t"
       "movups 40(%%rsi), %%xmm1;\n\t"
       "addps  %%xmm7, %%xmm6;\n\t"
       "addps  %%xmm2, %%xmm0;\n\t"
       "addps  %%xmm1, %%xmm6;\n\t"
       "addps  %%xmm5, %%xmm3;\n\t"
       "addps  %%xmm7, %%xmm6;\n\t"
       "divps  16(%%rbx ), %%xmm0;\n\t"
       "divps  16(%%rbx ), %%xmm3;\n\t"
       "divps  16(%%rbx ), %%xmm6;\n\t"
       "movups %%xmm0, (%%rdi);\n\t"
       "movups %%xmm3, 16(%%rdi);\n\t"
       "movups %%xmm6, 32(%%rdi);\n\t"
       "add   $48, %%rsi;\n\t"
       "add   $48, %%rdi;\n\t"
       "dec   %%rdx;\n\t"
       "jnz  .HA3;\n\t"
       "pop %%rdx;\n\t"
       ".HA4:;\n\t"

       "or  %%rax, %%rax;\n\t"
       "jz .HA6;\n\t"
       "push %%rax;\n\t"
       ".HA5:;\n\t"
       "movss  00(%%rsi),  %%xmm0;\n\t"
       "movss  04(%%rsi),  %%xmm1;\n\t"
       "movss  8(%%rsi),  %%xmm2;\n\t"
       "addps  %%xmm1, %%xmm0;\n\t"
       "addps  %%xmm1, %%xmm2;\n\t"
       "addps  %%xmm2, %%xmm0;\n\t"
       "divss  16(%%rbx ), %%xmm0;\n\t"
       "movss  %%xmm0, (%%rdi);\n\t"
       "add   $4, %%rsi;\n\t"
       "add   $4, %%rdi;\n\t"
       "dec %%rax;\n\t"
       "jnz .HA5;\n\t"
       "pop %%rax;\n\t"

       ".HA6:;\n\t"
       "movss (%%rsi), %%xmm1;\n\t"  // xmm1 <- sptr[0]
       "movss 4(%%rsi), %%xmm2;\n\t" // xmm2 <- sptr[1]
       "addss %%xmm2, %%xmm2;\n\t"   // xmm2 <- sptr[0] + sptr[1]
       "addss %%xmm1, %%xmm2;\n\t"   // xmm2 <- xmm2 + sptr[0]
       "divss 0(%%rbx), %%xmm2;\n\t" // xmm2 <- xmm2/3.0

       "movss %%xmm2, (%%rdi);\n\t"     // *dptr <- xmm2
       "add  $4, %%rdi;\n\t"        // ++dptr
       "add  $8, %%rsi;\n\t"        // sptr += 2
       "dec %%rcx;\n\t"
       "jnz .HA2;\n\t"
       ".HA1:;\n\t"
       :
       :"S"(a), "D"(b),"c"(h),"a"(rax),"d"(rdx),"b"(coeffs)
       :"memory"
       );

}




//######################################################################

void sse_lowPass3y(const float *a, float *b, const int h, const int w)
{
  const float coeffs[] = { 3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0, 4.0};

  if (h < 2){
    memcpy(b, a, w*h*sizeof(b[0]));
    return; // nothing to smooth
  }

  if (h < 2){
    memcpy(b, a, w*h*sizeof(b[0]));
    return; // nothing to smooth
  }

  asm (
       // top row
       "mov %%rdx, %%rcx;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz .HU1;\n\t"
       "push %%rsi;\n\t"
       ".HU0:;\n\t"
       "movss (%%rsi), %%xmm0;\n\t" // xmm0 <- sptr[0]
       "movss (%%rsi, %%rdx, 4), %%xmm1;\n\t" //xmm1 <- sptr[w]
       "addss %%xmm0, %%xmm0;\n\t"
       "addss %%xmm1, %%xmm0;\n\t"
       "divss (%%rbx), %%xmm0;\n\t"
       "add $4, %%rsi;\n\t"
       "movss %%xmm0, (%%rdi);\n\t"
       "add  $4, %%rdi;\n\t"
       "dec %%rcx;\n\t"
       "jnz .HU0;\n\t"
       "pop %%rsi;\n\t"
       ".HU1:;\n\t"
       "cmp $2, %%rax;\n\t"
       "jle .HU5;\n\t"

       "push %%rax;\n\t"
       "sub $2, %%rax;\n\t"
       "jle .HU4;\n\t"
       ".HU2:;\n\t"
       "mov %%rdx, %%rcx;\n\t"
       "push %%rdx;\n\t"
       ".HU3:;\n\t"
       "movss (%%rsi), %%xmm0;\n\t" //xmm0 <- sptr[0]
       "movss (%%rsi,%%rdx,4), %%xmm1;\n\t" //xmm1 <- sptr[w]
       "movss (%%rsi,%%rdx,8), %%xmm2;\n\t" //xmm2 <- sptr[2*w]
       "addss %%xmm1, %%xmm0;\n\t"
       "addss %%xmm1, %%xmm2;\n\t"
       "addss %%xmm2, %%xmm0;\n\t"
       "divss 16(%%rbx), %%xmm0;\n\t"
       "movss %%xmm0, (%%rdi);\n\t"
       "add  $4, %%rsi;\n\t"
       "add  $4, %%rdi;\n\t"
       "dec  %%rcx;\n\t"
       "jnz .HU3;\n\t"
       "pop %%rdx;\n\t"
       "dec %%rax;\n\t"
       "jnz .HU2;\n\t"

       ".HU4:;\n\t"
       "pop %%rax;\n\t"
       ".HU5:;\n\t"
       "or %%rdx, %%rdx;\n\t"
       "jz .HU7;\n\t"
       "push %%rdx;\n\t"
       "mov  %%rdx, %%rcx;\n\t"
       ".HU6:;\n\t"
       "movss (%%rsi), %%xmm0;\n\t" //xmm0 <- sptr[0]
       "movss (%%rsi,%%rcx,4), %%xmm1;\n\t" //xmm1 <- sptr[w]
       "addss %%xmm1, %%xmm1;\n\t"
       "addss %%xmm1, %%xmm0;\n\t"
       "divss (%%rbx), %%xmm0;\n\t"
       "movss %%xmm0, (%%rdi);\n\t"
       "add $4, %%rsi;\n\t"
       "add $4, %%rdi;\n\t"
       "dec %%rdx;\n\t"
       "jnz .HU6;\n\t"
       "pop %%rdx;\n\t"
       ".HU7:;\n\t"
       :
       :"S"(a),"D"(b),"a"(h),"d"(w),"b"(coeffs)
       );

}


//######################################################################

void sse_lowPass5x(const float *src, float *dest, const int h, const int w)
{
  const float *sptr= src;
  float *dptr= dest;

  if(w<2)
    {
      memcpy(dest,src,h*w*sizeof(dest[0]));
      return;
    }

  if (w == 2) //////////////////////////////////////////////////
    for (int j = 0; j < h; j ++)
      {
        // leftmost point  [ (6^) 4 ] / 10
        *dptr++ = sptr[0] * (6.0F / 10.0F) + sptr[1] * (4.0F / 10.0F);

        // rightmost point  [ 4^ (6) ] / 10
        *dptr++ = sptr[0] * (4.0F / 10.0F) + sptr[1] * (6.0F / 10.0F);

        sptr += 2;  // sptr back to same position as dptr
      }
  else if (w == 3) //////////////////////////////////////////////////
    for (int j = 0; j < h; j ++)
      {
        // leftmost point  [ (6^) 4 1 ] / 11
        *dptr++ = sptr[0] * (6.0F / 11.0F) +
          sptr[1] * (4.0F / 11.0F) +
          sptr[2] * (1.0F / 11.0F);

        // middle point    [ 4^ (6) 4 ] / 14
        *dptr++ = (sptr[0] + sptr[2]) * (4.0F / 14.0F) +
          sptr[1] * (6.0F / 14.0F);

        // rightmost point  [ 1^ 4 (6) ] / 11
        *dptr++ = sptr[0] * (1.0F / 11.0F) +
          sptr[1] * (4.0F / 11.0F) +
          sptr[2] * (6.0F / 11.0F);

        sptr += 3;  // sptr back to same position as dptr
      }
  else
    if(w>3)
      {
        const float coeffs[] = {6.0/11.0, 4.0/11.0, 1.0/11.0, 4.0/15.0,
                                4.0/15.0, 6.0/15.0, 1.0/15.0, 1.0/16.0,
                                1.0/16.0, 1.0/16.0, 1.0/16.0, 1.0/16.0,
                                4.0/16.0, 4.0/16.0, 4.0/16.0, 4.0/16.0,
                                6.0/16.0, 6.0/16.0, 6.0/16.0, 6.0/16.0,
                                1.0/15.0, 4.0/15.0, 6.0/15.0, 1.0/15.0,
                                1.0/11.0, 4.0/11.0, 6.0/11.0, 1.0/11.0
        };

        int rax= (w-4)&3;
        int rdx= (w-4)>>2;

        asm(
            "or %%rcx, %%rcx;\n\t"  // rcx <- h
            "jz .HG6;\n\t"
            ".HG0:;\n\t"
            "movss   (%%rsi), %%xmm0;\n\t" // xmm0 <- s[0]
            "movss  4(%%rsi), %%xmm2;\n\t" // xmm2 <- s[1]
            "movss  8(%%rsi), %%xmm4;\n\t" // xmm4 <- s[2]
            "movss 12(%%rsi), %%xmm6;\n\t" // xmm6 <- s[3]
            "movss  %%xmm0, %%xmm1;\n\t"   // xmm1 <- s[0]
            "movss  %%xmm2, %%xmm3;\n\t"   // xmm3 <- s[1]
            "movss  %%xmm4, %%xmm5;\n\t"   // xmm5 <- s[2]
            "mulss   (%%rbx), %%xmm0;\n\t" // xmm0 <- 6.0/11.0*s[0]
            "mulss  4(%%rbx), %%xmm2;\n\t" // xmm2 <- 4.0/11.0*s[1]
            "mulss  8(%%rbx), %%xmm4;\n\t" // xmm4 <- 1.0/11.0*s[2]
            "addss  %%xmm5, %%xmm1;\n\t"   // xmm1 <- s[2]+s[0]
            "mulss 16(%%rbx), %%xmm1;\n\t" // xmm1 <- (s2+s0)*4.0/15.0
            "mulss 20(%%rbx), %%xmm3;\n\t"
            "mulss 24(%%rbx), %%xmm6;\n\t"
            "addss %%xmm2, %%xmm0;\n\t"
            "addss %%xmm3, %%xmm1;\n\t"
            "addss %%xmm4, %%xmm0;\n\t"
            "addss %%xmm6, %%xmm1;\n\t"
            "movss %%xmm0,   (%%rdi);\n\t"
            "movss %%xmm1,  4(%%rdi);\n\t"
            "add  $8, %%rdi;\n\t"

            "or   %%rdx, %%rdx;\n\t"
            "jz .HG5;\n\t"

            "push %%rdx;\n\t"   // rdx <- (w-4)/4
            "movups  32(%%rbx), %%xmm5;\n\t" // xmm5 <- 1.0/16.0 1.0/16.0 1.0/16 1.0/16
            "movups  48(%%rbx), %%xmm6;\n\t" // xmm6 <- 4.0/16.0 ......................
            "movups  64(%%rbx), %%xmm7;\n\t" // xmm7 <- 6.0/16.0 ......................
            ".HG1:;\n\t"
            "movups   0(%%rsi), %%xmm0;\n\t" // xmm0 <- s0  s1  s2  s3
            "movups 04(%%rsi), %%xmm1;\n\t" // xmm1 <- s1  s2  s3  s4
            "movups  8(%%rsi), %%xmm2;\n\t" // xmm2 <- s2  s3  s4  s5
            "movups 12(%%rsi), %%xmm3;\n\t" // xmm3 <- s3  s4  s5  s6
             "movups 16(%%rsi), %%xmm4;\n\t" // xmm4 <- s4  s5  s6  s7
            "addps  %%xmm4, %%xmm0;\n\t"
            "addps  %%xmm3, %%xmm1;\n\t"
            "mulps  %%xmm5, %%xmm0;\n\t"
            "mulps  %%xmm6, %%xmm1;\n\t"
            "mulps  %%xmm7, %%xmm2;\n\t"
            "addps  %%xmm1, %%xmm0;\n\t"
            "addps  %%xmm2, %%xmm0;\n\t"
            "movups %%xmm0, (%%rdi);\n\t"
            "add   $16, %%rsi;\n\t"
            "add   $16, %%rdi;\n\t"
            "dec   %%rdx;\n\t"
            "jnz .HG1;\n\t"
            "pop %%rdx;\n\t"

            ".HG5:;\n\t"
            "or  %%rax, %%rax;\n\t"
            "jz  .HG3;\n\t"
            "push %%rax;\n\t"       // rax <- (w-4)%4
            "movups 32(%%rbx), %%xmm5;\n\t"
            "movups 48(%%rbx), %%xmm6;\n\t"
            "movups 64(%%rbx), %%xmm7;\n\t"
            ".HG2:;\n\t"
            "movss    (%%rsi), %%xmm0;\n\t"
            "movss   4(%%rsi), %%xmm1;\n\t"
            "movss   8(%%rsi), %%xmm2;\n\t"
            "movss  12(%%rsi), %%xmm3;\n\t"
            "movss  16(%%rsi), %%xmm4;\n\t"
            "mulss  %%xmm5   , %%xmm0;\n\t"
            "mulss  %%xmm6   , %%xmm1;\n\t"
            "mulss  %%xmm7   , %%xmm2;\n\t"
            "mulss  %%xmm6   , %%xmm3;\n\t"
            "mulss  %%xmm5   , %%xmm4;\n\t"
            "addss  %%xmm1, %%xmm0;\n\t"
            "addss  %%xmm3, %%xmm2;\n\t"
            "addss  %%xmm4, %%xmm0;\n\t"
            "addss  %%xmm2, %%xmm0;\n\t"
            "add   $4, %%rsi;\n\t"
            "movss  %%xmm0, (%%rdi);\n\t"
            "add   $4, %%rdi;\n\t"
            "dec   %%rax;\n\t"
            "jnz .HG2;\n\t"
            "pop  %%rax;\n\t"
            ".HG3:;\n\t"
            "movss  (%%rsi), %%xmm0;\n\t"  // xmm0 <- s0
            "movss 4(%%rsi), %%xmm1;\n\t"  // xmm1 <- s1
            "movss 8(%%rsi), %%xmm2;\n\t"  // xmm2 <- s2
            "movss 12(%%rsi), %%xmm3;\n\t" // xmm3 <- s3
            "movss %%xmm1, %%xmm4;\n\t"    // xmm4 <- s1
            "movss %%xmm2, %%xmm5;\n\t"    // xmm5 <- s2
            "movss %%xmm3, %%xmm6;\n\t"    // xmm6 <- s3
            "addps %%xmm1, %%xmm3;\n\t"    // xmm3 <- s1+s3
            "mulss 80(%%rbx), %%xmm0;\n\t" // xmm0 <- 1.0/15.0*s0
            "mulss 84(%%rbx), %%xmm3;\n\t" // xmm3 <- 4.0/15.0*(s1+s3)
            "mulss 88(%%rbx), %%xmm2;\n\t" // xmm2 <- 6.0/15.0*s2
            "addss %%xmm3, %%xmm0;\n\t"
            "addss %%xmm2, %%xmm0;\n\t"
            "movss %%xmm0, (%%rdi);\n\t"
            "mulss 96(%%rbx), %%xmm4;\n\t"
            "mulss 100(%%rbx), %%xmm5;\n\t"
            "mulss 104(%%rbx), %%xmm6;\n\t"
            "addss %%xmm5, %%xmm4;\n\t"
            "addss %%xmm6, %%xmm4;\n\t"
            "movss %%xmm4, 4(%%rdi);\n\t"
            "add $16, %%rsi;\n\t"
            "add $8, %%rdi;\n\t"
            "dec %%rcx;\n\t"
            "jnz .HG0;\n\t"
            ".HG6:;\n\t"
            :
            :"S"(sptr),"D"(dptr),"a"(rax),"b"(coeffs),"c"(h),"d"(rdx)
            :"memory"
            );
      }

}



//######################################################################

void sse_lowPass5y(const float *src, float *dest, const int h,
                       const int w)
{

  /*
  if (h < 2){
    memcpy(dest, src, h*w*sizeof(dest[0]));
    return; // nothing to smooth
  }

  const float *sptr= src;
  float *dptr= dest;

  // ########## vertical pass  (even though we scan horiz for speedup)
  const int w2 = w * 2; // speedup


  if (h == 2) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 ] / 10 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[0] * (6.0F / 10.0F) +
            sptr[w] * (4.0F / 10.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // bottommost points  ( [ 4^ (6) ] / 10 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[0] * (4.0F / 10.0F) +
            sptr[w] * (6.0F / 10.0F);
          sptr++;
        }
    }
  else if (h == 3) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (6.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (1.0F / 11.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // middle points  ( [ 4^ (6) 4 ] / 14 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = (sptr[ 0] + sptr[w2]) * (4.0F / 14.0F) +
            sptr[ w] * (6.0F / 14.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // bottommost points  ( [ 1^ 4 (6) ] / 11 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (1.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (6.0F / 11.0F);
          sptr++;
        }
    }
  else  ///////////////////////////////// general case for height >= 4
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T

      static const float coeffs[] = {
        6.0/11.0, 6.0/11.0, 6.0/11.0, 6.0/11.0, //0
        4.0/11.0, 4.0/11.0, 4.0/11.0, 4.0/11.0, //16
        1.0/11.0, 1.0/11.0, 1.0/11.0, 1.0/11.0, //32
        4.0F/15.0F, 4.0F/15.0F, 4.0F/15.0F, 4.0F/15.0F, //48
        6.0F/15.0F, 6.0F/15.0F, 6.0F/15.0F, 6.0F/15.0F, //64
        1.0F/15.0F, 1.0F/15.0F, 1.0F/15.0F, 1.0F/15.0F, //80
        1.0/16.0, 1.0/16.0, 1.0/16.0, 1.0/16.0, //96
        4.0F/16.0F, 4.0F/16.0F, 4.0F/16.0F, 4.0F/16.0F, //112
        6.0F/16.0F, 6.0F/16.0F, 6.0F/16.0F, 6.0F/16.0F  //128
      };

      int rcx=h-4;
      int rdx=w>>2;
      int rax=w&3;

       asm (
            "push %%rbp;\n\t"
            "mov %0, %%rbp;\n\t"
            "add %%rbp, %%rbp;\n\t"
            "add %%rbp, %%rbp;\n\t"

            // 1st loop
            "movups (%%rbx), %%xmm4;\n\t"          //xmm4 <- 6.0/11.0 ...
            "movups 16(%%rbx), %%xmm5;\n\t"        //xmm5 <- 4.0/11.0
            "movups 32(%%rbx), %%xmm6;\n\t"        //xmm6 <- 1.0/11.0
            "push %%rsi;\n\t"
            "or  %%rdx, %%rdx;\n\t"
            "jz .IA1;\n\t"
            ".align 4;\n\t"
            "push %%rdx;\n\t"
            ".IA0:;\n\t"
            ".align 4;\n\t"
            "movups (%%rsi), %%xmm0;\n\t"          //xmm0 <- s0   s0   s0   s0
            "movups (%%rsi,%%rbp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
            "movups (%%rsi,%%rbp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
            "mulps  %%xmm4, %%xmm0;\n\t"
            "mulps  %%xmm5, %%xmm1;\n\t"
            "mulps  %%xmm6, %%xmm2;\n\t"
            "addps  %%xmm1, %%xmm0;\n\t"
            "addps  %%xmm2, %%xmm0;\n\t"
            "movups %%xmm0, (%%rdi);\n\t"
            "add $16, %%rsi;\n\t"
            "add $16, %%rdi;\n\t"
            "dec %%rdx;\n\t"
            "jnz .IA0;\n\t"
            "pop %%rdx;\n\t"
            ".IA1:;\n\t"
            ".align 4;\n\t"
            "or %%rax, %%rax;\n\t"
            "jz .IA3;\n\t"
            "push %%rax;\n\t"
            ".IA2:;\n\t"
            ".align 4;\n\t"
            "movss  (%%rsi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
            "movss  (%%rsi,%%rbp,1), %%xmm1;\n\t"  //xmm1 <- sW+3 sW+2 sW+1 sW
            "movss  (%%rsi,%%rbp,2), %%xmm2;\n\t"  //xmm2 <- sP+3 sP+3 sP+1 sP
            "mulss  %%xmm4, %%xmm0;\n\t"
            "mulss  %%xmm5, %%xmm1;\n\t"
            "mulss  %%xmm6, %%xmm2;\n\t"
            "addss  %%xmm1, %%xmm0;\n\t"
            "addss  %%xmm2, %%xmm0;\n\t"
            "movss  %%xmm0, (%%rdi);\n\t"
            "add $4, %%rsi;\n\t"
            "add $4, %%rdi;\n\t"
            "dec %%rax;\n\t"
            "jnz .IA2;\n\t"
            "pop %%rax;\n\t"
            ".IA3:;\n\t"
            "pop %%rsi;\n\t"  // restore sptr

            // 2nd loop
            "movups 48(%%rbx), %%xmm4;\n\t" //xmm4 <- 4.0/15.0
            "movups 64(%%rbx), %%xmm5;\n\t" //xmm5 <- 6.0/15.0
            "movups 80(%%rbx), %%xmm6;\n\t" //xmm6 <- 1.0/15.0
            "push %%rsi;\n\t"
            "or   %%rdx, %%rdx;\n\t"
            "jz .IA5;\n\t"
            "push %%rdx;\n\t"
            "push %%rax;\n\t"
            "mov  %%rbp, %%rax;\n\t"
            "add  %%rbp, %%rax;\n\t"
            "add  %%rbp, %%rax;\n\t"
            ".IA4:;\n\t"
            "movups (%%rsi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
            "movups (%%rsi,%%rbp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
            "movups (%%rsi,%%rbp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
            "movups (%%rsi,%%rax,1), %%xmm3;\n\t"  //xmm3 <- sW3  sW3  sW3  sW3
            "addps  %%xmm2, %%xmm0;\n\t"
            "mulps  %%xmm4, %%xmm0;\n\t"
            "mulps  %%xmm5, %%xmm1;\n\t"
            "mulps  %%xmm6, %%xmm3;\n\t"
            "addps  %%xmm1, %%xmm0;\n\t"
            "addps  %%xmm3, %%xmm0;\n\t"
            "movups %%xmm0, (%%rdi);\n\t"
            "add $16, %%rsi;\n\t"
            "add $16, %%rdi;\n\t"
            "dec %%rdx;\n\t"
            "jnz .IA4;\n\t"
            "pop %%rax;\n\t"
            "pop %%rdx;\n\t"
            ".IA5:;\n\t"
            "or %%rax, %%rax;\n\t"
            "jz .IA7;\n\t"
            "push %%rax;\n\t"
            "push %%rdx;\n\t"
            "mov  %%rbp, %%rdx;\n\t"
            "add  %%rbp, %%rdx;\n\t"
            "add  %%rbp, %%rdx;\n\t"
            ".IA6:;\n\t"
            "movss  (%%rsi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
            "movss  (%%rsi,%%rbp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
            "movss  (%%rsi,%%rbp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
            "movss  (%%rsi,%%rdx,1), %%xmm3;\n\t" //xmm3 <- sW3  sW3  sW3  sW3
            "addss  %%xmm2, %%xmm0;\n\t"
            "mulss  %%xmm4, %%xmm0;\n\t"
            "mulss  %%xmm5, %%xmm1;\n\t"
            "mulss  %%xmm6, %%xmm3;\n\t"
            "addss  %%xmm1, %%xmm0;\n\t"
            "addss  %%xmm3, %%xmm0;\n\t"
            "movss  %%xmm0, (%%rdi);\n\t"
            "add $4, %%rsi;\n\t"
            "add $4, %%rdi;\n\t"
            "dec %%rax;\n\t"
            "jnz .IA6;\n\t"
            "pop %%rdx;\n\t"
            "pop %%rax;\n\t"
            ".IA7:;\n\t"
            "pop %%rsi;\n\t"  // restore sptr


            //            the double loops
            "or %%rcx, %%rcx;\n\t"
            "jz .IA29;\n\t"
            "push %%rcx;\n\t"
            "movups 96(%%rbx), %%xmm5;\n\t"    // xmm5 <- 1.0/16.0
            "movups 112(%%rbx), %%xmm6;\n\t"   // xmm6 <- 4.0/16.0
            "movups 128(%%rbx), %%xmm7;\n\t"   // xmm7 <- 6.0/16.0
            ".IA8:;\n\t"
            "or  %%rdx, %%rdx;\n\t"
            "jz .IA10;\n\t"
            "push %%rdx;\n\t"
            "push %%rax;\n\t"
            "mov  %%rbp, %%rax;\n\t"
            "add  %%rbp, %%rax;\n\t"
            "add  %%rbp, %%rax;\n\t"                 // rax <- 3*W
            ".IA9:;\n\t"
            "movups  (%%rsi),  %%xmm0;\n\t"          // xmm0 <- s    s    s    s
            "movups  (%%rsi,%%rbp,1),  %%xmm1;\n\t"  // xmm1 <- sW   sW   sW   sW
            "movups  (%%rsi,%%rbp,2),  %%xmm2;\n\t"  // xmm2 <- sW2  sW2  sW2  sW2
            "movups  (%%rsi,%%rax,1), %%xmm3;\n\t"   // xmm3 <- sW3  sW3  sW3  sW3
            "movups  (%%rsi,%%rbp,4), %%xmm4;\n\t"   // xmm4 <- sW4  sW4  sW4  sW4
            "addps   %%xmm3, %%xmm1;\n\t"            // xmm1 <- sW3 + sW1
            "addps   %%xmm4, %%xmm0;\n\t"            // xmm0 <- s0  + sW4
            "mulps   %%xmm6, %%xmm1;\n\t"            // xmm1 <- 4.0/16.0*(sW3+sW1)
            "mulps   %%xmm5, %%xmm0;\n\t"            // xmm0 <- 1.0/16.08(s0 +sW4)
            "mulps   %%xmm7, %%xmm2;\n\t"            // xmm2 <- 6.0/16.0*sW2
            "addps   %%xmm1, %%xmm0;\n\t"
            "addps   %%xmm2, %%xmm0;\n\t"
            "add    $16, %%rsi;\n\t"
            "movups  %%xmm0, (%%rdi);\n\t"
            "add    $16, %%rdi;\n\t"
            "dec   %%rdx;\n\t"
            "jnz .IA9;\n\t"
            "pop   %%rax;\n\t"
            "pop   %%rdx;\n\t"
            ".IA10:;\n\t"
            "or  %%rax, %%rax;\n\t"
            "jz .IA12;\n\t"
            "push %%rax;\n\t"
            "push %%rdx;\n\t"
            "mov  %%rbp, %%rdx;\n\t"
            "add  %%rbp, %%rdx;\n\t"
            "add  %%rbp, %%rdx;\n\t"
            ".IA11:;\n\t"
            "movss   (%%rsi),  %%xmm0;\n\t"          // xmm0 <- s    s    s    s
            "movss   (%%rsi,%%rbp,1),  %%xmm1;\n\t"  // xmm1 <- sW   sW   sW   sW
            "movss   (%%rsi,%%rbp,2),  %%xmm2;\n\t"  // xmm2 <- sW2  sW2  sW2  sW2
            "movss   (%%rsi,%%rdx,1), %%xmm3;\n\t"   // xmm3 <- sW3  sW3  sW3  sW3
            "movss   (%%rsi,%%rbp,4), %%xmm4;\n\t"   // xmm4 <- sW4  sW4  sW4  sW4
            "addss   %%xmm3, %%xmm1;\n\t"
            "addss   %%xmm4, %%xmm0;\n\t"
            "mulss   %%xmm6, %%xmm1;\n\t"
            "mulss   %%xmm5, %%xmm0;\n\t"
            "mulss   %%xmm7, %%xmm2;\n\t"
            "addss   %%xmm1, %%xmm0;\n\t"
            "addss   %%xmm2, %%xmm0;\n\t"
            "add    $4, %%rsi;\n\t"
            "movss   %%xmm0, (%%rdi);\n\t"
            "add    $4, %%rdi;\n\t"
            "dec  %%rax;\n\t"
            "jnz .IA11;\n\t"
            "pop %%rdx;\n\t"
            "pop %%rax;\n\t"
            ".IA12:;\n\t"
            "dec %%rcx;\n\t"
            "jnz .IA8;\n\t"
            "pop %%rcx;\n\t"
            ".IA29:;\n\t"

            // fourth loop
            "movups 48(%%rbx), %%xmm4;\n\t"  //xmm4 <- 4.0/15.0
            "movups 64(%%rbx), %%xmm5;\n\t"  //xmm5 <- 6.0/15.0
            "movups 80(%%rbx), %%xmm6;\n\t"  //xmm6 <- 1.0/15.0
            "or  %%rdx, %%rdx;\n\t"
            "jz .IA14;\n\t"
            "push %%rdx;\n\t"
            "push %%rax;\n\t"
            "mov  %%rbp, %%rax;\n\t"
            "add  %%rbp, %%rax;\n\t"
            "add  %%rbp, %%rax;\n\t"
            ".IA13:;\n\t"
            "movups (%%rsi), %%xmm0;\n\t"          //xmm0 <- s0   s0   s0   s0
            "movups (%%rsi,%S%rbp,1), %%xmm1;\n\t" //xmm1 <- sW1  sW1  sW1  sW1
            "movups (%%rsi,%%rbp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
            "movups (%%rsi,%%rax,1),%%xmm3;\n\t"  //xmm3 <- sW3  sW3  sW3  sW3
            "addps  %%xmm3, %%xmm1;\n\t"          //xmm1 <- sW3 + sW1
            "mulps  %%xmm6, %%xmm0;\n\t"          //xmm0 <- 1.0/15.0 * s0
            "mulps  %%xmm5, %%xmm2;\n\t"          //xmm2 <- 6.0/15.0 * sW2
            "mulps  %%xmm4, %%xmm1;\n\t"          //xmm4 <- 4.0/15.0 * (sW3+sW1)
            "addps  %%xmm2, %%xmm0;\n\t"
            "addps  %%xmm1, %%xmm0;\n\t"
            "movups %%xmm0, (%%rdi);\n\t"
            "add $16, %%rsi;\n\t"
            "add $16, %%rdi;\n\t"
            "dec %%rdx;\n\t"
            "jnz .IA13;\n\t"
            "pop %%rax;\n\t"
            "pop %%rdx;\n\t"
            ".IA14:;\n\t"
            "or %%rax, %%rax;\n\t"
            "jz .IA16;\n\t"
            "push %%rax;\n\t"
            "push %%rdx;\n\t"
            "mov %%rbp, %%rdx;\n\t"
            "add %%rbp, %%rdx;\n\t"
            "add %%rbp, %%rdx;\n\t"
            ".IA15:;\n\t"
            "movss  (%%rsi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
            "movss  (%%rsi, %%rbp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
            "movss  (%%rsi, %%rbp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
            "movss  (%%rsi, %%rdx,1), %%xmm3;\n\t" //xmm3 <- sW3  sW3  sW3  sW3
            "addss  %%xmm3, %%xmm1;\n\t"
            "mulss  %%xmm6, %%xmm0;\n\t"
            "mulss  %%xmm5, %%xmm2;\n\t"
            "mulss  %%xmm4, %%xmm1;\n\t"
            "addss  %%xmm2, %%xmm0;\n\t"
            "addss  %%xmm1, %%xmm0;\n\t"
            "movss  %%xmm0, (%%rdi);\n\t"
            "add $4, %%rsi;\n\t"
            "add $4, %%rdi;\n\t"
            "dec %%rax;\n\t"
            "jnz .IA15;\n\t"
            "pop %%rdx;\n\t"
            "pop %%rax;\n\t"
            ".IA16:;\n\t"

             // final loop
            "movups 32(%%rbx), %%xmm4;\n\t"
            "movups 16(%%rbx), %%xmm5;\n\t"
            "movups   (%%rbx), %%xmm6;\n\t"
            "or  %%rdx, %%rdx;\n\t"
            "jz .IA18;\n\t"
            "push %%rdx;\n\t"
            ".IA17:;\n\t"
            "movups (%%rsi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
            "movups (%%rsi,%%rbp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
            "movups (%%rsi,%%rbp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
            "mulps  %%xmm4, %%xmm0;\n\t"
            "mulps  %%xmm5, %%xmm1;\n\t"
            "mulps  %%xmm6, %%xmm2;\n\t"
            "addps  %%xmm1, %%xmm0;\n\t"
            "addps  %%xmm2, %%xmm0;\n\t"
            "movups %%xmm0, (%%rdi);\n\t"
            "add $16, %%rsi;\n\t"
            "add $16, %%rdi;\n\t"
            "dec %%rdx;\n\t"
            "jnz .IA17;\n\t"
            "pop %%rdx;\n\t"
            ".IA18:;\n\t"
            "or %%rax, %%rax;\n\t"
            "jz .IA20;\n\t"
            "push %%rax;\n\t"
            ".IA19:;\n\t"
            "movss  (%%rsi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
            "movss  (%%rsi,%%rbp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
            "movss  (%%rsi,%%rbp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
            "mulss  %%xmm4, %%xmm0;\n\t"
            "mulss  %%xmm5, %%xmm1;\n\t"
            "mulss  %%xmm6, %%xmm2;\n\t"
            "addss  %%xmm1, %%xmm0;\n\t"
            "addss  %%xmm2, %%xmm0;\n\t"
            "movss  %%xmm0, (%%rdi);\n\t"
            "add $4, %%rsi;\n\t"
            "add $4, %%rdi;\n\t"
            "dec %%rax;\n\t"
            "jnz .IA19;\n\t"
            "pop %%rax;\n\t"
            ".IA20:;\n\t"

            "pop %%rbp;\n\t"
            :
            :"m"(w),"S"(sptr),"D"(dptr),"a"(rax),"b"(coeffs),"c"(rcx),"d"(rdx)
            :
            );

    }
  */
}


// ######################################################################

void sse_yuv411_to_rgb_mode_640x480(const byte *src, byte *dest,
                                    const int nbpix2)
{
  int rcx=nbpix2/6;

  const float coeffs[] = {
    0.0F,       -0.198242F,   1.014648F,     0.0F,  // R  G   B  xx  -> u
    0.700195F,  -0.29052F,    0.0F,          0.0F,  // R  G   B  xx  -> v
    128.0F,        128.0F,    128.0F,      128.0F   // division factor
  };

  asm (
       ".JA0:;\n\t"
       "or %%rcx, %%rcx;\n\t"
       "jz .JA1;\n\t"
       "pxor  %%mm7, %%mm7;\n\t"    //mm7 <-  00 00 00 00
       "xor  %%rax, %%rax;\n\t"
       "xor  %%rbx, %%rbx;\n\t"
       "mov  (%%rsi),   %%rax;\n\t" // rax <-   v   y1  y0 u
       "movw 4(%%rsi),  %%bx;\n\t"   // rbx <-   xx  xx  y3 y2
       "movd %%rax, %%mm0;\n\t"        // mm0<- xx xx xx xx v  y1  y0  u
       "movd %%rax, %%mm1;\n\t"        // mm1<- xx xx xx xx v  y1  y0  u
       "movd %%rbx, %%mm2;\n\t"        // mm2<- xx xx xx xx xx xx  y3  y2
       "psrlq $16,  %%mm1;\n\t"        // mm1<- xx xx xx xx xx xx  v   y1
       "punpcklbw %%mm7, %%mm0;\n\t"   // mm0<- xx xx xx xx 0  y0  0   u
       "punpcklbw %%mm7, %%mm1;\n\t"   // mm1<- xx xx xx xx 00 v   00  y1
       "punpcklbw %%mm7, %%mm2;\n\t"   // mm2<- xx xx xx xx 00 y3  00  y2
       "punpcklwd %%mm7, %%mm0;\n\t"   // mm0<- 00 00 00 y0 00 00  00  u
       "punpcklwd %%mm7, %%mm1;\n\t"   // mm1<- 00 00 00 v  00 00  00  y1
       "punpcklwd %%mm7, %%mm2;\n\t"   // mm2<- 00 00 00 y3 00 00  00  y2

       "cvtpi2ps %%mm0, %%xmm0;\n\t"   // xmm0 <- 00 00 y0 u
       "cvtpi2ps %%mm1, %%xmm1;\n\t"   // xmm1 <- 00 00 v  y1
       "cvtpi2ps %%mm2, %%xmm2;\n\t"   // xmm2 <- 00 00 y3 y2

       // 01 01 01 01
       "movaps %%xmm0, %%xmm3;\n\t"

       // 00 00 00 00
       "movaps %%xmm1, %%xmm4;\n\t"

       // 00 00 00 00
       "movaps %%xmm2, %%xmm5;\n\t"

       // 01 01 01 01
       "movaps %%xmm2, %%xmm6;\n\t"

       "shufps $0x55, %%xmm3, %%xmm3;\n\t"// xmm3 <- y0 y0 y0 y0
       "shufps $00, %%xmm4, %%xmm4;\n\t"  // xmm4 <- y1 y1 y1 y1
       "shufps $0x00, %%xmm5, %%xmm5;\n\t"// xmm5 <- y2 y2 y2 y2
       "shufps $0x55, %%xmm6, %%xmm6;\n\t"// xmm6 <- y3 y3 y3 y3

       // 00 00 00 00
       "shufps $0, %%xmm0, %%xmm0;\n\t"  // xmm0 <- u  u  u  u
       // 01 01 01 01
       "shufps $0x55, %%xmm1, %%xmm1;\n\t" // xmm1 <- v  v  v  v

       "subps  32(%%rdx), %%xmm0;\n\t"
       "subps  32(%%rdx), %%xmm1;\n\t"

       "mulps (%%rdx), %%xmm0;\n\t"
       "mulps 16(%%rdx),%%xmm1;\n\t"

       "addps %%xmm0, %%xmm3;\n\t"
       "addps %%xmm0, %%xmm4;\n\t"
       "addps %%xmm0, %%xmm5;\n\t"
       "addps %%xmm0, %%xmm6;\n\t"

       "addps %%xmm1, %%xmm3;\n\t"    // xmm3 <- xx b0 g0 r0
       "addps %%xmm1, %%xmm4;\n\t"    // xmm4 <- xx b1 g1 r1
       "addps %%xmm1, %%xmm5;\n\t"    // xmm5 <- xx b2 g2 r2
       "addps %%xmm1, %%xmm6;\n\t"    // xmm6 <- xx b3 g3 r3

       "cvtps2pi %%xmm3, %%mm0;\n\t"  //mm0  <- g0 r0
       "movhlps  %%xmm3, %%xmm3;\n\t" //xmm3 <- g0 r0 xx b0
       "cvtps2pi %%xmm3, %%mm1;\n\t"  //mm1  <- xx b0
       "packssdw %%mm1, %%mm0;\n\t"   //mm0<- xx b0 g0 r0

       "cvtps2pi %%xmm4, %%mm2;\n\t"  //mm2  <- g1 r1
       "movhlps  %%xmm4, %%xmm4;\n\t" //xmm4 <- g1 r1 xx b1
       "cvtps2pi %%xmm4, %%mm3;\n\t"  //mm3  <- xx b1
       "packssdw %%mm3, %%mm2;\n\t"   //mm2<- xx b1 g1 r1

       "cvtps2pi %%xmm5, %%mm4;\n\t"  //mm4  <- g2 r2
       "movhlps  %%xmm5, %%xmm5;\n\t" //xmm5 <- g2 r2 xx b2
       "cvtps2pi %%xmm5, %%mm5;\n\t"  //mm5  <- xx b2
       "packssdw %%mm5, %%mm4;\n\t"   //mm4<- xx b2 g2 r2

       "cvtps2pi %%xmm6, %%mm6;\n\t"  //mm6  <- g3 r3
       "movhlps  %%xmm6, %%xmm6;\n\t" //xmm3 <- g3 r3 xx b3
       "cvtps2pi %%xmm6, %%mm7;\n\t"  //mm7  <- xx b3
       "packssdw %%mm7, %%mm6;\n\t"   //mm6<- xx b3 g3 r3

       "pxor %%mm1, %%mm1;\n\t"
       "pcmpgtw %%mm0, %%mm1;\n\t"
       "pandn %%mm0, %%mm1;\n\t"

       "pxor %%mm3, %%mm3;\n\t"
       "pcmpgtw %%mm2, %%mm3;\n\t"
       "pandn %%mm2, %%mm3;\n\t"

       "pxor %%mm5, %%mm5;\n\t"
       "pcmpgtw %%mm4, %%mm5;\n\t"
       "pandn %%mm4, %%mm5;\n\t"

       "pxor %%mm7, %%mm7;\n\t"
       "pcmpgtw %%mm6, %%mm7;\n\t"
       "pandn %%mm6, %%mm7;\n\t"

       "packuswb %%mm1, %%mm1;\n\t"   //mm0<- xx xx xx xx xx b0 g0 r0
       "packuswb %%mm3, %%mm3;\n\t"   //mm2<- xx xx xx xx xx b1 g1 r1
       "packuswb %%mm5, %%mm5;\n\t"   //mm4<- xx xx xx xx xx b2 g2 r2
       "packuswb %%mm7, %%mm7;\n\t"   //mm6<- xx xx xx xx xx b3 g3 r3

       "push %%rcx;\n\t"
       "push %%rdx;\n\t"
       "movd %%mm1, %%rax;\n\t"  // rax <- xx b0 g0 r0
       "movd %%mm3, %%rbx;\n\t"  // rbx <- xx b1 g1 r1
       "movd %%mm5, %%rcx;\n\t"  // rcx <- xx b2 g2 r2
       "movd %%mm7, %%rdx;\n\t"  // rdx <- xx b3 g3 r3
       "movw %%ax, (%%rdi);\n\t"
       "movw %%bx,3(%%rdi);\n\t"
       "movw %%cx,6(%%rdi);\n\t"
       "movw %%dx,9(%%rdi);\n\t"
       "shr $8, %%rax;\n\t"
       "shr $8, %%rbx;\n\t"
       "shr $8, %%rcx;\n\t"
       "shr $8, %%rdx;\n\t"
       "movb %%ah, 2(%%rdi);\n\t"
       "movb %%bh, 5(%%rdi);\n\t"
       "movb %%ch, 8(%%rdi);\n\t"
       "movb %%dh,11(%%rdi);\n\t"
       "pop %%rdx;\n\t"
       "pop %%rcx;\n\t"

       "add $12,%%rdi;\n\t"
       "dec %%rcx;\n\t"
       "add $6, %%rsi;\n\t"
       "jmp .JA0;\n\t"
       ".JA1:;\n\t"
       "emms;\n\t"
       :
       :"S"(src),"D"(dest),"c"(rcx),"d"(coeffs)
       :"rax","rbx","memory"
       );

}




void sse_lowPass9x(const float *sptr, float *dptr, const int h, const int w)
{

 for (int j = 0; j < h; j ++)
    {
      // leftmost points
      *dptr++ = sptr[0] * (70.0F / 163.0F) +
        sptr[1] * (56.0F / 163.0F) +
        sptr[2] * (28.0F / 163.0F) +
        sptr[3] * ( 8.0F / 163.0F) +
        sptr[4] * ( 1.0F / 163.0F);
      *dptr++ = (sptr[0] + sptr[2]) * (56.0F / 219.0F) +
        sptr[1] * (70.0F / 219.0F) +
        sptr[3] * (28.0F / 219.0F) +
        sptr[4] * ( 8.0F / 219.0F) +
        sptr[5] * ( 1.0F / 219.0F);
      *dptr++ = (sptr[0] + sptr[4]) * (28.0F / 247.0F) +
        (sptr[1] + sptr[3]) * (56.0F / 247.0F) +
        sptr[2] * (70.0F / 247.0F) +
        sptr[5] * ( 8.0F / 247.0F) +
        sptr[6] * ( 1.0F / 247.0F);
      *dptr++ = (sptr[0] + sptr[6]) * ( 8.0F / 255.0F) +
        (sptr[1] + sptr[5]) * (28.0F / 255.0F) +
        (sptr[2] + sptr[4]) * (56.0F / 255.0F) +
        sptr[3] * (70.0F / 255.0F) +
        sptr[7] * ( 1.0F / 255.0F);

      // far from the borders
      for (int i = 0; i < w - 8; i ++)
        {
          *dptr++ = (sptr[0] + sptr[8]) * ( 1.0F / 256.0F) +
            (sptr[1] + sptr[7]) * ( 8.0F / 256.0F) +
            (sptr[2] + sptr[6]) * (28.0F / 256.0F) +
            (sptr[3] + sptr[5]) * (56.0F / 256.0F) +
            sptr[4] * (70.0F / 256.0F);
          sptr ++;
        }

      // rightmost points
      *dptr++ = sptr[0] * ( 1.0F / 255.0F) +
        (sptr[1] + sptr[7]) * ( 8.0F / 255.0F) +
        (sptr[2] + sptr[6]) * (28.0F / 255.0F) +
        (sptr[3] + sptr[5]) * (56.0F / 255.0F) +
        sptr[4] * (70.0F / 255.0F);
      sptr ++;
      *dptr++ = sptr[0] * ( 1.0F / 247.0F) +
        sptr[1] * ( 8.0F / 247.0F) +
        (sptr[2] + sptr[6]) * (28.0F / 247.0F) +
        (sptr[3] + sptr[5]) * (56.0F / 247.0F) +
        sptr[4] * (70.0F / 247.0F);
      sptr ++;
      *dptr++ = sptr[0] * ( 1.0F / 219.0F) +
        sptr[1] * ( 8.0F / 219.0F) +
        sptr[2] * (28.0F / 219.0F) +
        (sptr[3] + sptr[5]) * (56.0F / 219.0F) +
        sptr[4] * (70.0F / 219.0F);
      sptr ++;
      *dptr++ = sptr[0] * ( 1.0F / 163.0F) +
        sptr[1] * ( 8.0F / 163.0F) +
        sptr[2] * (28.0F / 163.0F) +
        sptr[3] * (56.0F / 163.0F) +
        sptr[4] * (70.0F / 163.0F);
      sptr += 5;  // sptr back to same as dptr (start of next line)
    }
}
#endif

//############################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif

#ifndef INVT_CPU_OPTERON

#ifdef INVT_USE_SSE

//######################################################################
void sse_absDiff(const double *a, const double *b, double *diff, const int32 sz)
{
  static int32 ecx= sz>>2;
  static int32 edx= sz & 0x3;

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .AG2;\n\t"
       ".AG1:;\n\t"
       "movupd  0(%%esi), %%xmm0;\n\t" // xmm0 <- a3 a2 a1 a0
       "movupd  0(%%edi), %%xmm1;\n\t" // xmm1 <- b3 b2 b1 b0
       "movupd  16(%%esi), %%xmm2;\n\t"// xmm2 <- a7 a6 a5 a4
       "movupd  16(%%edi), %%xmm3;\n\t"// xmm3 <- b7 b6 b5 b4
       "movupd  %%xmm0, %%xmm4;\n\t"   // xmm4 <- a3 a2 a1 a0
       "movupd  %%xmm1, %%xmm5;\n\t"   // xmm5 <- b3 b2 b1 b0
       "movupd  %%xmm2, %%xmm6;\n\t"   // xmm6 <- a7 a6 a5 a4
       "movupd  %%xmm3, %%xmm7;\n\t"   // xmm7 <- b7 b6 b5 b4
       "subpd   %%xmm1, %%xmm0;\n\t"   // xmm0 <- (a3-b3) .. (a1-b1) (a0-b0)
       "subpd   %%xmm3, %%xmm2;\n\t"   // xmm2 <- (a7-b7) .. (a5-b5) (a4-b4)
       "subpd   %%xmm4, %%xmm5;\n\t"   // xmm5 <- (b3-a3) .. (b1-a1) (b0-a0)
       "subpd   %%xmm6, %%xmm7;\n\t"   // xmm7 <- (b7-a7) .. (b5-a5) (b4-a4)
       "maxpd   %%xmm0, %%xmm5;\n\t"   // xmm5 <- max(xmm0,xmm5)
       "maxpd   %%xmm2, %%xmm7;\n\t"   // xmm7 <- max(xmm2,xmm7)
       "movupd  %%xmm5, 0(%%ebx);\n\t"
       "movupd  %%xmm7, 16(%%ebx);\n\t"
       "addl $32, %%esi;\n\t"
       "addl $32, %%edi;\n\t"
       "addl $32, %%ebx;\n\t"
       "loop  .AG1;\n\t"
       ".AG2:;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz .AG4;\n\t"
       ".AG3:;\n\t"
       "movsd 0(%%esi), %%xmm0;\n\t"
       "movsd 0(%%edi), %%xmm1;\n\t"
       "movsd %%xmm0, %%xmm2;\n\t"
       "movsd %%xmm1, %%xmm3;\n\t"
       "subsd %%xmm3, %%xmm2;\n\t"
       "subsd %%xmm0, %%xmm1;\n\t"
       "maxsd %%xmm2, %%xmm1;\n\t"
       "movsd %%xmm1, 0(%%ebx);\n\t"
       "addl $8, %%esi;\n\t"
       "addl $8, %%edi;\n\t"
       "addl $8, %%ebx;\n\t"
       "loop .AG3;\n\t"
       ".AG4:;\n\t"
       :
       :"S"(a),"D"(b),"b"(diff), "c"(ecx), "d"(edx)
       :"memory"
       );
}
#endif

#ifdef INVT_USE_MMXSSE2
//######################################################################
// speedup ~= 2.1
void sse2_absDiff(const float *a, const float *b, float *diff, const int32 sz)
{
  static int32 ecx= sz>>3;
  static int32 edx= sz & 0x7;

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .AE2;\n\t"
       ".AE1:;\n\t"
       "movups  0(%%esi), %%xmm0;\n\t" // xmm0 <- a3 a2 a1 a0
       "movups  0(%%edi), %%xmm1;\n\t" // xmm1 <- b3 b2 b1 b0
       "movups  16(%%esi), %%xmm2;\n\t"// xmm2 <- a7 a6 a5 a4
       "movups  16(%%edi), %%xmm3;\n\t"// xmm3 <- b7 b6 b5 b4
       "movups  %%xmm0, %%xmm4;\n\t"   // xmm4 <- a3 a2 a1 a0
       "movups  %%xmm1, %%xmm5;\n\t"   // xmm5 <- b3 b2 b1 b0
       "movups  %%xmm2, %%xmm6;\n\t"   // xmm6 <- a7 a6 a5 a4
       "movups  %%xmm3, %%xmm7;\n\t"   // xmm7 <- b7 b6 b5 b4
       "subps   %%xmm1, %%xmm0;\n\t"   // xmm0 <- (a3-b3) .. (a1-b1) (a0-b0)
       "subps   %%xmm3, %%xmm2;\n\t"   // xmm2 <- (a7-b7) .. (a5-b5) (a4-b4)
       "subps   %%xmm4, %%xmm5;\n\t"   // xmm5 <- (b3-a3) .. (b1-a1) (b0-a0)
       "subps   %%xmm6, %%xmm7;\n\t"   // xmm7 <- (b7-a7) .. (b5-a5) (b4-a4)
       "maxps   %%xmm0, %%xmm5;\n\t"   // xmm5 <- max(xmm0,xmm5)
       "maxps   %%xmm2, %%xmm7;\n\t"   // xmm7 <- max(xmm2,xmm7)
       "movups  %%xmm5, 0(%%ebx);\n\t"
       "movups  %%xmm7, 16(%%ebx);\n\t"
       "addl $32, %%esi;\n\t"
       "addl $32, %%edi;\n\t"
       "addl $32, %%ebx;\n\t"
       "loop  .AE1;\n\t"
       ".AE2:;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz .AE4;\n\t"
       ".AE3:;\n\t"
       "movss 0(%%esi), %%xmm0;\n\t"
       "movss 0(%%edi), %%xmm1;\n\t"
       "movss %%xmm0, %%xmm2;\n\t"
       "movss %%xmm1, %%xmm3;\n\t"
       "subss %%xmm3, %%xmm2;\n\t"
       "subss %%xmm0, %%xmm1;\n\t"
       "maxss %%xmm2, %%xmm1;\n\t"
       "movss %%xmm1, 0(%%ebx);\n\t"
       "addl $4, %%esi;\n\t"
       "addl $4, %%edi;\n\t"
       "addl $4, %%ebx;\n\t"
       "loop .AE3;\n\t"
       ".AE4:;\n\t"
       "emms;\n\t"
       :
       :"S"(a),"D"(b),"b"(diff), "c"(ecx), "d"(edx)
       :"memory"
       );
}



//######################################################################
// speedup ~= 3.4
void sse2_absDiff(const int32 *a, const int32 *b, int32 *diff, const int32 sz)
{
  static int32 ecx= sz>>3;
  static int32 edx= sz&0x7;

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .AF2;\n\t"
       ".AF1:;\n\t"
       "movdqu  0(%%esi), %%xmm0;\n\t"
       "movdqu  0(%%edi), %%xmm1;\n\t"
       "movdqu  16(%%esi), %%xmm2;\n\t"
       "movdqu  16(%%edi), %%xmm3;\n\t"
       "movdqu  %%xmm0, %%xmm4;\n\t"
       "movdqu  %%xmm1, %%xmm5;\n\t"
       "movdqu  %%xmm2, %%xmm6;\n\t"
       "movdqu  %%xmm3, %%xmm7;\n\t"
       "psubusw %%xmm1, %%xmm0;\n\t"
       "psubusw %%xmm3, %%xmm2;\n\t"
       "psubusw %%xmm4, %%xmm5;\n\t"
       "psubusw %%xmm6, %%xmm7;\n\t"
       "pmaxsw  %%xmm0, %%xmm5;\n\t"
       "pmaxsw  %%xmm2, %%xmm7;\n\t"
       "movdqu  %%xmm5, 0(%%ebx);\n\t"
       "movdqu  %%xmm7, 16(%%ebx);\n\t"
       "addl $32, %%esi;\n\t"
       "addl $32, %%edi;\n\t"
       "addl $32, %%ebx;\n\t"
       "loop  .AF1;\n\t"
       ".AF2:;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz .AF4;\n\t"
       ".AF3:;\n\t"
       "movl (%%esi), %%eax;\n\t"
       "movl (%%edi), %%edx;\n\t"
       "cmpl %%edx, %%eax;\n\t"
       "ja .AF5;\n\t"
       "xchgl %%eax, %%edx;\n\t"
       ".AF5:;\n\t"
       "subl %%edx, %%eax;\n\t"
       "movl %%eax, (%%ebx);\n\t"
       "addl $4, %%esi;\n\t"
       "addl $4, %%edi;\n\t"
       "addl $4, %%ebx;\n\t"
       "loop .AF3;\n\t"
       ".AF4:;\n\t"
       "emms;\n\t"
       :
       :"S"(a),"D"(b),"b"(diff), "c"(ecx), "d"(edx)
       :"memory"
       );
}


//######################################################################
// speedup ~=10.0!
void sse2_absDiff(const byte *a, const byte *b, byte *diff, const int32 sz)
{
  static int32 ecx= sz>>5;
  static int32 edx= sz&0x1f;

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .AD2;\n\t"
       ".AD1:;\n\t"
       "movdqu  0(%%esi), %%xmm0;\n\t" // xmm0<- a15 ... a3 a2 a1 a0
       "movdqu  0(%%edi), %%xmm1;\n\t" // xmm1<- b15 ... b3 b2 b1 b0
       "movdqu  16(%%esi), %%xmm2;\n\t"// xmm2<- a31 ... a18 a17 a16
       "movdqu  16(%%edi), %%xmm3;\n\t"// xmm3<- b31 ... b18 b17 b16
       "movdqu  %%xmm0, %%xmm4;\n\t"   // xmm4<- a15 ... a3 a2 a1 a0
       "movdqu  %%xmm1, %%xmm5;\n\t"   // xmm5<- b15 ... b3 b2 b1 b0
       "movdqu  %%xmm2, %%xmm6;\n\t"   // xmm6<- a31 ... a18 a17 a16
       "movdqu  %%xmm3, %%xmm7;\n\t"   // xmm7<- b31 ... b18 b17 b16
       "psubusb %%xmm1, %%xmm0;\n\t"   // xmm0<-(a15-b15)...( a1-b1 )(a0-b0)
       "psubusb %%xmm3, %%xmm2;\n\t"   // xmm2<-(a31-b31)...(a17-b17)(a16-b16)
       "psubusb %%xmm4, %%xmm5;\n\t"   // xmm5<-(b15-a15)...(b17-a17)(b16-a16)
       "psubusb %%xmm6, %%xmm7;\n\t"   // xmm7<-(b31-a31)...(b17-a17)(b16-a16)
       "pmaxub  %%xmm0, %%xmm5;\n\t"   // xmm5<- max(xmm0,xmm5)
       "pmaxub  %%xmm2, %%xmm7;\n\t"   // xmm7<- max(xmm2,xmm7)
       "movdqu  %%xmm5, 0(%%ebx);\n\t"
       "movdqu  %%xmm7, 16(%%ebx);\n\t"
       "addl $32, %%esi;\n\t"
       "addl $32, %%edi;\n\t"
       "addl $32, %%ebx;\n\t"
       "loop  .AD1;\n\t"
       ".AD2:;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz .AD4;\n\t"
       ".AD3:;\n\t"
       "movb (%%esi), %%al;\n\t"
       "movb (%%edi), %%dl;\n\t"
       "cmpb %%dl, %%al;\n\t"
       "ja .AD5;\n\t"
       "xchgb %%al, %%dl;\n\t"
       ".AD5:;\n\t"
       "subb %%dl, %%al;\n\t"
       "movb %%al, (%%ebx);\n\t"
       "incl %%ebx;\n\t"
       "incl %%esi;\n\t"
       "incl %%edi;\n\t"
       "loop .AD3;\n\t"
       ".AD4:;\n\t"
       "emms;\n\t"
       :
       :"S"(a),"D"(b),"b"(diff), "c"(ecx), "d"(edx)
       :"memory"
       );
}
#endif

#ifdef INVT_USE_SSE
//######################################################################
// speedup ~= 2.0
void sse_sum(const double *a, double *sum, const int32 sz)
{
  static int32 ecx = sz>>3;
  static int32 edx = sz&0x7;

  asm (
       "pxor %%xmm4, %%xmm4;\n\t"
       "pxor %%xmm5, %%xmm5;\n\t"
       "pxor %%xmm6, %%xmm6;\n\t"
       "pxor %%xmm7, %%xmm7;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz  BE1;\n\t"
       ".BE0:\n\t"
       "movupd     0(%%esi), %%xmm0;\n\t"
       "movupd  16(%%esi), %%xmm1;\n\t"
       "movupd  32(%%esi), %%xmm2;\n\t"
       "movupd  48(%%esi), %%xmm3;\n\t"
       "addpd %%xmm0, %%xmm4;\n\t"
       "addpd %%xmm1, %%xmm5;\n\t"
       "addpd %%xmm2, %%xmm6;\n\t"
       "addpd %%xmm3, %%xmm7;\n\t"
       "addl $64, %%esi;\n\t"
       "loop .BE0;\n\t"
       "BE1:;\n\t"
       "mov %%edx, %%ecx;\n\t"
       "pxor %%xmm0, %%xmm0;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz BE2;\n\t"
       "BE3:;\n\t"
       "movupd 0(%%esi), %%xmm1;\n\t"
       "addpd %%xmm1, %%xmm0;\n\t"
       "addl $16, %%esi;\n\t"
       "loop BE3;\n\t"
       "BE2:;\n\t"
       "addpd %%xmm4, %%xmm7;\n\t"
       "addpd %%xmm5, %%xmm7;\n\t"
       "addpd %%xmm6, %%xmm7;\n\t"
       "addpd %%xmm7, %%xmm0;\n\t"
       "movhpd %%xmm0, (%%ebx);\n\t"
       "addsd  (%%ebx), %%xmm0;\n\t"
       "movlpd %%xmm0, (%%ebx);\n\t"
       "emms;\n\t"
       :
       :"S"(a), "b"(sum), "c"(ecx), "d"(edx)
       :"memory"
       );
}
#endif

#ifdef INVT_USE_MMXSSE2
//######################################################################
//speedup ~= 4
void sse2_sum(const float *a, double *sum, const int32 sz)
{
  static int32 ecx = sz>>3;
  static int32 edx = sz & 0x7;

  asm (
       "pxor %%xmm4, %%xmm4;\n\t"
       "pxor %%xmm5, %%xmm5;\n\t"
       "pxor %%xmm6, %%xmm6;\n\t"
       "pxor %%xmm7, %%xmm7;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz  BA1;\n\t"
       ".BA0:\n\t"
       "cvtps2pd  0(%%esi), %%xmm0;\n\t"
       "cvtps2pd  8(%%esi), %%xmm1;\n\t"
       "cvtps2pd  16(%%esi), %%xmm2;\n\t"
       "cvtps2pd 24(%%esi), %%xmm3;\n\t"
       "addpd %%xmm0, %%xmm4;\n\t"
       "addpd %%xmm1, %%xmm5;\n\t"
       "addpd %%xmm2, %%xmm6;\n\t"
       "addpd %%xmm3, %%xmm7;\n\t"
       "addl $32, %%esi;\n\t"
       "loop .BA0;\n\t"
       "BA1:;\n\t"
       "pxor %%xmm0, %%xmm0;\n\t"
       "mov %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz BA2;\n\t"
       "BA3:;\n\t"
       "cvtps2pd 0(%%esi), %%xmm1;\n\t"
       "addpd %%xmm1, %%xmm0;\n\t"
       "addl $8, %%esi;\n\t"
       "loop BA3;\n\t"
       "BA2:;\n\t"
       "addpd %%xmm4, %%xmm7;\n\t"
       "addpd %%xmm5, %%xmm7;\n\t"
       "addpd %%xmm6, %%xmm7;\n\t"
       "addpd %%xmm7, %%xmm0;\n\t"
       "movhpd %%xmm0, (%%ebx);\n\t"
       "addsd  (%%ebx), %%xmm0;\n\t"
       "movlpd %%xmm0, (%%ebx);\n\t"
       "emms;\n\t"
       :
       :"S"(a), "b"(sum), "c"(ecx), "d"(edx)
       :"memory"
       );
}


//######################################################################
// speedup ~= 4.0
void sse2_sum(const int32 *a, double *sum, const int32 sz)
{
  static int32 ecx = sz>>3;
  static int32 edx = sz & 0x7;

  asm (
       "pxor %%xmm4, %%xmm4;\n\t"
       "pxor %%xmm5, %%xmm5;\n\t"
       "pxor %%xmm6, %%xmm6;\n\t"
       "pxor %%xmm7, %%xmm7;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       ".BC0:\n\t"
       "cvtdq2pd  0(%%esi), %%xmm0;\n\t"
       "cvtdq2pd  8(%%esi), %%xmm1;\n\t"
       "cvtdq2pd  16(%%esi), %%xmm2;\n\t"
       "cvtdq2pd 24(%%esi), %%xmm3;\n\t"
       "addpd %%xmm0, %%xmm4;\n\t"
       "addpd %%xmm1, %%xmm5;\n\t"
       "addpd %%xmm2, %%xmm6;\n\t"
       "addpd %%xmm3, %%xmm7;\n\t"
       "addl $32, %%esi;\n\t"
       "loop .BC0;\n\t"
       "BC1:;\n\t"
       "pxor %%xmm0, %%xmm0;\n\t"
       "mov %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz BC2;\n\t"
       "BC3:;\n\t"
       "cvtdq2pd 0(%%esi), %%xmm1;\n\t"
       "addpd %%xmm1, %%xmm0;\n\t"
       "addl $8, %%esi;\n\t"
       "loop BC3;\n\t"
       "BC2:;\n\t"
       "addpd %%xmm4, %%xmm7;\n\t"
       "addpd %%xmm5, %%xmm7;\n\t"
       "addpd %%xmm6, %%xmm7;\n\t"
       "addpd %%xmm7, %%xmm0;\n\t"
       "movhpd %%xmm0, (%%ebx);\n\t"
       "addsd  (%%ebx), %%xmm0;\n\t"
       "movlpd %%xmm0, (%%ebx);\n\t"
       "emms;\n\t"
       :
       :"S"(a), "b"(sum), "c"(ecx), "d"(edx)
       :"memory"
       );
}



//######################################################################
void sse2_sum(const byte *a, double *sum, const int32 sz)
{
  static int ecx = sz>>5;
  static int edx = sz & 0x1f;

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz  BB1;\n\t"
       "pxor %%xmm7, %%xmm7;\n\t"
       "pushl %%ebx;\n\t"
       "pushl %%edx;\n\t"
       "BB3:;\n\t"
       "pxor %%xmm5, %%xmm5;\n\t"
       "pxor %%xmm6, %%xmm6;\n\t"
       "movdqu (%%esi), %%xmm0;\n\t"
       "movdqu 16(%%esi), %%xmm1;\n\t"
       "psadbw %%xmm0, %%xmm5;\n\t"
       "psadbw %%xmm1, %%xmm6;\n\t"
       "pextrw $0, %%xmm5, %%eax;\n\t"
       "cvtsi2sd %%eax, %%xmm0;\n\t"
       "pextrw $4, %%xmm5, %%ebx;\n\t"
       "cvtsi2sd %%ebx, %%xmm1;\n\t"
       "pextrw $0, %%xmm6, %%edx;\n\t"
       "cvtsi2sd %%edx, %%xmm2;\n\t"
       "pextrw $4, %%xmm6, %%edi;\n\t"
       "cvtsi2sd %%edi, %%xmm3;\n\t"
       "addsd %%xmm0, %%xmm1;\n\t"
       "addsd %%xmm2, %%xmm3;\n\t"
       "addsd %%xmm1, %%xmm7;\n\t"
       "addsd %%xmm3, %%xmm7;\n\t"
       "addl $32, %%esi;\n\t"
       "loop BB3;\n\t"
       "popl %%edx;\n\t"
       "popl %%ebx;\n\t"
       "BB1:;\n\t"
       "xorl %%edi, %%edi;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz BB2;\n\t"
       "BB5:;\n\t"
       "xorl %%eax, %%eax;\n\t"
       "movb (%%esi), %%al;\n\t"
       "addl %%eax, %%edi;\n\t"
       "incl %%esi;\n\t"
       "loop BB5;\n\t"
       "BB2:\n\t"
       "cvtsi2sd %%edi, %%xmm0;\n\t"
       "addsd %%xmm0, %%xmm7;\n\t"
       "movhpd %%xmm7, (%%ebx);\n\t"
       "addsd  (%%ebx), %%xmm7;\n\t"
       "movlpd %%xmm7, (%%ebx);\n\t"
       "BB6:;\n\t"
       "emms;\n\t"
       :
       :"S"(a), "c"(ecx),"b"(sum),"d"(edx)
       :"memory","eax","edi"
       );
}
#endif

#ifdef INVT_USE_SSE
//######################################################################
// speedup ~= 10 !
void sse_clampedDiff(const byte *a, const byte *b, byte *result, const int32 sz)
{
  int ecx = sz >> 6;
  int edx = sz & 0x7f;

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .DA0;\n\t"
       ".DA1:;\n\t"
       "movdqu (%%esi), %%xmm0;\n\t"
       "movdqu (%%edi), %%xmm4;\n\t"
       "movdqu 16(%%esi), %%xmm1;\n\t"
       "movdqu 16(%%edi), %%xmm5;\n\t"
       "movdqu 32(%%esi), %%xmm2;\n\t"
       "movdqu 32(%%edi), %%xmm6;\n\t"
       "movdqu 48(%%esi), %%xmm3;\n\t"
       "movdqu 48(%%edi), %%xmm7;\n\t"
       "psubusb %%xmm4, %%xmm0;\n\t"
       "psubusb %%xmm5, %%xmm1;\n\t"
       "psubusb %%xmm6, %%xmm2;\n\t"
       "psubusb %%xmm7, %%xmm3;\n\t"
       "movdqu  %%xmm0, 0(%%ebx);\n\t"
       "movdqu  %%xmm1, 16(%%ebx);\n\t"
       "movdqu  %%xmm2, 32(%%ebx);\n\t"
       "movdqu  %%xmm3, 48(%%ebx);\n\t"
       "addl $64, %%esi;\n\t"
       "addl $64, %%edi;\n\t"
       "addl $64, %%ebx;\n\t"
       "loop .DA1;\n\t"
       ".DA0:;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz .DA2;\n\t"
       ".DA3:;\n\t"
       "movb (%%esi), %%al;\n\t"
       "movb (%%edi), %%dl;\n\t"
       "cmpb %%bl, %%al;\n\t"
       "ja .DA4;\n\t"
       "xchg %%al, %%bl;\n\t"
       ".DA4:;\n\t"
       "subb %%bl, %%al;\n\t"
       "movb %%al, (%%ebx);\n\t"
       "incl %%esi;\n\t"
       "incl %%edi;\n\t"
       "incl %%ebx;\n\t"
       "loop .DA3;\n\t"
       ".DA2:;\n\t"
       "emms;\n\t"
       :
       :"S"(a),"D"(b),"c"(ecx),"d"(edx),"b"(result)
       );
}


//######################################################################
// speedup ~= 20 !
void sse_clampedDiff(const float32 *a, const float32 *b, float32 *result,
                        const int32 sz)
{
  int32 ecx=sz>>5;
  int32 edx=sz&0x1f;

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .DB0;\n\t"
       ".DB1:;\n\t"
       "movups  0(%%esi), %%xmm0;\n\t"
       "movups  0(%%edi), %%xmm1;\n\t"
       "movups 16(%%esi), %%xmm2;\n\t"
       "movups 16(%%edi), %%xmm3;\n\t"
       "movups %%xmm1, %%xmm6;\n\t"
       "movups %%xmm3, %%xmm7;\n\t"
       "cmpps  $1, %%xmm0, %%xmm6;\n\t"
       "cmpps  $1, %%xmm2, %%xmm7;\n\t"
       "subps  %%xmm1, %%xmm0;\n\t"
       "subps  %%xmm3, %%xmm2;\n\t"
       "andps  %%xmm6, %%xmm0;\n\t"
       "andps  %%xmm7, %%xmm2;\n\t"
       "movups %%xmm0, (%%ebx);\n\t"
       "movups %%xmm2, 16(%%ebx);\n\t"
       "addl  $32, %%esi;\n\t"
       "addl  $32, %%edi;\n\t"
       "addl  $32, %%ebx;\n\t"
       "loop .DB1;\n\t"
       ".DB0:;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz .DB2;\n\t"
       ".DB3:;\n\t"
       "movss (%%esi), %%xmm0;\n\t"
       "movss (%%edi), %%xmm1;\n\t"
       "movss %%xmm1, %%xmm2;\n\t"
       "cmpss $1, %%xmm0,  %%xmm2;\n\t"
       "andps %%xmm2, %%xmm0;\n\t"
       "andps %%xmm2, %%xmm1;\n\t"
       "subss %%xmm1,  %%xmm0;\n\t"
       "movss %%xmm0,  (%%ebx);\n\t"
       "addl $4, %%esi;\n\t"
       "addl $4, %%edi;\n\t"
       "addl $4, %%ebx;\n\t"
       "loop .DB3;\n\t"
       ".DB2:;\n\t"
       :
       :"S"(a), "D"(b), "b"(result), "c"(ecx), "d"(edx)
       :"memory"
       );
}


//######################################################################
// speedup ~= 3
void sse_clampedDiff(const int32 *a, const int32 *b, int32 *c, const int32 sz)
{
  int32 ecx=sz>>3;
  int32 edx=sz&0x7;
  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .DC0;\n\t"
       ".DC1:;\n\t"
       "movdqu 0(%%esi), %%xmm0;\n\t" //xmm0=  a3     a2     a1     a0
       "movdqu 0(%%edi), %%xmm1;\n\t" //xmm1=  b3     b2     b1     b0
       "movdqu 16(%%esi), %%xmm3;\n\t"//xmm3=  a7     a6     a5     a4
       "movdqu 16(%%edi), %%xmm4;\n\t"//xmm4=  b7     b6     b5     b4
       "movdqu  %%xmm0, %%xmm2;\n\t"  //xmm2=  a3     a2     a1     a0
       "movdqu  %%xmm3, %%xmm5;\n\t"  //xmm5=  a7     a6     a5     a4
       "pcmpgtd %%xmm1, %%xmm2;\n\t"  //xmm2=(a3>b3)(a2>b2)(a1>b1)(a0>b0)
       "pcmpgtd %%xmm4, %%xmm5;\n\t"  //xmm5=(a7>b7)(a6>b6)(b5>a5)(a4>b4)
       "psubd   %%xmm1, %%xmm0;\n\t"  //xmm0=(a3-b3)(a2-b2)(a1-b1)(a0-b0)
       "psubd   %%xmm4, %%xmm3;\n\t"  //xmm3=(a7-b7)(a6-b6)(a5-b5)(a4-b4)
       "pand    %%xmm2, %%xmm0;\n\t"
       "pand    %%xmm5, %%xmm3;\n\t"
       "movdqu  %%xmm0, (%%ebx);\n\t"
       "movdqu  %%xmm3, 16(%%ebx);\n\t"
       "addl $32, %%esi;\n\t"
       "addl $32, %%edi;\n\t"
       "addl $32, %%ebx;\n\t"
       "loop .DC1;\n\t"
       ".DC0:;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "orl  %%ecx, %%ecx;\n\t"
       "jz .DC2;\n\t"
       ".DC3:;\n\t"
       "movd 0(%%esi), %%xmm0;\n\t"
       "movd 0(%%edi), %%xmm1;\n\t"
       "movdqu %%xmm0, %%xmm2;\n\t"
       "pcmpgtd %%xmm1, %%xmm2;\n\t"
       "psubd   %%xmm1, %%xmm0;\n\t"
       "pand    %%xmm2, %%xmm0;\n\t"
       "movd    %%xmm0, (%%ebx);\n\t"
       "addl $4, %%esi;\n\t"
       "addl $4, %%edi;\n\t"
       "addl $4, %%ebx;\n\t"
       "loop .DC3;\n\t"
       ".DC2:;\n\t"
       :
       :"S"(a), "D"(b), "c"(ecx), "d"(edx), "b"(c)
       :"memory"
       );
}


//######################################################################
// speedup ~= 4-5
void sse_binaryReverse(const byte *a, byte *result, const byte val, const
                                int32 sz)
{
  static unsigned int ecx=(sz>>7);
  static unsigned int edx=sz&0x7f;

  byte pVal[16];

  memset(result, val, 16);

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .FA0;\n\t"
       ".FA1:;\n\t"
       "movdqu  0(%%ebx), %%xmm0;\n\t"
       "movdqu  0(%%ebx), %%xmm1;\n\t"
       "movdqu  %%xmm0, %%xmm2;\n\t"
       "movdqu  %%xmm1, %%xmm3;\n\t"
       "movdqu  %%xmm0, %%xmm4;\n\t"
       "movdqu  %%xmm1, %%xmm5;\n\t"
       "movdqu  %%xmm0, %%xmm6;\n\t"
       "movdqu  %%xmm1, %%xmm7;\n\t"
       "psubb (%%esi), %%xmm0;\n\t"
       "psubb 16(%%esi), %%xmm1;\n\t"
       "psubb 32(%%esi), %%xmm2;\n\t"
       "psubb 48(%%esi), %%xmm3;\n\t"
       "psubb 64(%%esi), %%xmm4;\n\t"
       "psubb 80(%%esi), %%xmm5;\n\t"
       "psubb 96(%%esi), %%xmm6;\n\t"
       "psubb 112(%%esi), %%xmm7;\n\t"
       "movdqu %%xmm0, (%%edi);\n\t"
       "movdqu %%xmm1, 16(%%edi);\n\t"
       "movdqu %%xmm2, 32(%%edi);\n\t"
       "movdqu %%xmm3, 48(%%edi);\n\t"
       "movdqu %%xmm4, 64(%%edi);\n\t"
       "movdqu %%xmm5, 80(%%edi);\n\t"
       "movdqu %%xmm6, 96(%%edi);\n\t"
       "movdqu %%xmm7, 112(%%edi);\n\t"
       "addl $128, %%edi;\n\t"
       "addl $128, %%esi;\n\t"
       "loop .FA1;\n\t"
       ".FA0:;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz .FA2;\n\t"
       "movb (%%ebx), %%dl;\n\t"
       ".FA3:;\n\t"
       "movb %%dl, %%dh;\n\t"
       "movb (%%esi), %%al;\n\t"
       "subb %%al, %%dh;\n\t"
       "movb %%dh, (%%edi);\n\t"
       "incl %%esi;\n\t"
       "incl %%edi;\n\t"
       "loop .FA3;\n\t"
       ".FA2:;\n\t"
       :
       :"S"(a), "D"(result), "b"(pVal),"c"(ecx),"d"(edx)
       :"memory","eax"
       );
}


//######################################################################
// speedup ~= 2
void sse_binaryReverse(const float *a, float *result, const float val,
                                const int sz)
{
  static unsigned int ecx = sz>>5;
  static unsigned int edx = sz&0x1f;
  int i;
  float pVal[16];

  for(i=0;i<16;++i)
    pVal[i] = val;


  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .FB4;\n\t"
       ".FB2:;\n\t"
       "movups (%%ebx), %%xmm0;\n\t"
       "movups (%%ebx), %%xmm1;\n\t"
       "movups %%xmm0, %%xmm2;\n\t"
       "movups %%xmm1, %%xmm3;\n\t"
       "movups %%xmm0, %%xmm4;\n\t"
       "movups %%xmm1, %%xmm5;\n\t"
       "movups %%xmm0, %%xmm6;\n\t"
       "movups %%xmm1, %%xmm7;\n\t"
       "psubq (%%esi), %%xmm0;\n\t"
       "psubq 16(%%esi), %%xmm1;\n\t"
       "psubq 32(%%esi), %%xmm2;\n\t"
       "psubq 48(%%esi), %%xmm3;\n\t"
       "psubq 64(%%esi), %%xmm4;\n\t"
       "psubq 80(%%esi), %%xmm5;\n\t"
       "psubq 96(%%esi), %%xmm6;\n\t"
       "psubq 112(%%esi), %%xmm7;\n\t"
       "movups %%xmm0,  0(%%edi);\n\t"
       "movups %%xmm1, 16(%%edi);\n\t"
       "movups %%xmm2, 32(%%edi);\n\t"
       "movups %%xmm3, 48(%%edi);\n\t"
       "movups %%xmm4, 64(%%edi);\n\t"
       "movups %%xmm5, 80(%%edi);\n\t"
       "movups %%xmm6, 96(%%edi);\n\t"
       "movups %%xmm7,112(%%edi);\n\t"
       "addl $128, %%esi;\n\t"
       "addl $128, %%edi;\n\t"
       "loop .FB2;\n\t"
       ".FB4:\n\t"
       "orl  %%edx, %%edx;\n\t"
       "jz .FB1;\n\t"
       "movl %%edx, %%ecx;\n\t"
       ".FB3:;\n\t"
       "movss 0(%%ebx), %%xmm0;\n\t"
       "subss (%%esi), %%xmm0;\n\t"
       "movups %%xmm0, (%%edi);\n\t"
       "addl $16, %%esi;\n\t"
       "addl $16, %%edi;\n\t"
       "loop .FB3;\n\t"
       ".FB1:;\n\t"
       :
       :"S"(a), "D"(result), "b"(pVal),"c"(ecx),"d"(edx)
       :"memory","eax"
       );
}



//######################################################################

void sse_binaryReverse(const int32 *a, int32 *result, const int32 val,
                        const int32 sz)
{
  int32 ecx=sz>>5;
  int32 edx=sz&31;
  int32 pVal[16];
  int i;

  for(i=0;i<16;++i) pVal[i] = val;

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .FC4;\n\t"
       ".FC2:;\n\t"
       "movdqu (%%ebx), %%xmm0;\n\t"
       "movdqu (%%ebx), %%xmm1;\n\t"
       "movdqu %%xmm0, %%xmm2;\n\t"
       "movdqu %%xmm1, %%xmm3;\n\t"
       "movdqu %%xmm0, %%xmm4;\n\t"
       "movdqu %%xmm1, %%xmm5;\n\t"
       "movdqu %%xmm0, %%xmm6;\n\t"
       "movdqu %%xmm1, %%xmm7;\n\t"
       "psubd  (%%esi), %%xmm0;\n\t"
       "psubd  16(%%esi), %%xmm1;\n\t"
       "psubd  32(%%esi), %%xmm2;\n\t"
       "psubd  48(%%esi), %%xmm3;\n\t"
       "psubd  64(%%esi), %%xmm4;\n\t"
       "psubd  80(%%esi), %%xmm5;\n\t"
       "psubd  96(%%esi), %%xmm6;\n\t"
       "psubd  112(%%esi), %%xmm7;\n\t"
       "movdqu %%xmm0,  0(%%edi);\n\t"
       "movdqu %%xmm1, 16(%%edi);\n\t"
       "movdqu %%xmm2, 32(%%edi);\n\t"
       "movdqu %%xmm3, 48(%%edi);\n\t"
       "movdqu %%xmm4, 64(%%edi);\n\t"
       "movdqu %%xmm5, 80(%%edi);\n\t"
       "movdqu %%xmm6, 96(%%edi);\n\t"
       "movdqu %%xmm7,112(%%edi);\n\t"
       "addl $128, %%esi;\n\t"
       "addl $128, %%edi;\n\t"
       "loop .FC2;\n\t"
       ".FC4:;\n\t"
       "orl  %%edx, %%edx;\n\t"
       "jz .FC1;\n\t"
       "movl %%edx, %%ecx;\n\t"
       ".FC3:;\n\t"
       "movdqu 0(%%ebx), %%xmm0;\n\t"
       "psubd (%%esi), %%xmm0;\n\t"
       "movups %%xmm0, (%%edi);\n\t"
       "addl $16, %%esi;\n\t"
       "addl $16, %%edi;\n\t"
       "loop .FC3;\n\t"
       ".FC1:;\n\t"
       :
       :"S"(a), "D"(result), "b"(pVal),"c"(ecx),"d"(edx)
       :"memory","eax"
       );
}



//######################################################################

void sse_cvt_byte_to_int(const byte *a, int32 *b, const int32 sz)
{
  int32 ecx=sz>>4;
  int32 edx=sz&0xf;

  asm(
      "orl %%ecx, %%ecx;\n\t"
      "jz .GA4;\n\t"
      "pxor %%xmm0, %%xmm0;\n\t"
      ".GA2:;\n\t"
      "movdqu 0(%%esi), %%xmm1;\n\t"
      "movdqa %%xmm1, %%xmm2;\n\t"
      "movdqa %%xmm1, %%xmm3;\n\t"
      "movdqa %%xmm1, %%xmm4;\n\t"
      "psrldq $4, %%xmm2;\n\t"
      "psrldq $8, %%xmm3;\n\t"
      "psrldq $12, %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "movdqu %%xmm1, (%%edi);\n\t"
      "movdqu %%xmm2, 16(%%edi);\n\t"
      "movdqu %%xmm3, 32(%%edi);\n\t"
      "movdqu %%xmm4, 48(%%edi);\n\t"
      "addl $16, %%esi;\n\t"
      "addl $64, %%edi;\n\t"
      "loop .GA2;\n\t"
      ".GA4:;\n\t"
      "orl %%edx, %%edx;\n\t"
      "jz .GA1;\n\t"
      "mov %%edx, %%ecx;\n\t"
      ".GA3:;\n\t"
      "xorl %%eax, %%eax;\n\t"
      "movb (%%esi), %%al;\n\t"
      "movl %%eax, (%%edi);\n\t"
      "incl %%esi;\n\t"
      "addl $4, %%edi;\n\t"
      "loop .GA3;\n\t"
      ".GA1:;"
      :
      :"S"(a), "D"(b), "c"(ecx),"d"(edx)
      :"memory"
      );


}

#endif

#ifdef INVT_USE_MMXSSE2

//######################################################################
// speedup ~= 1.5
void sse2_cvt_byte_to_float(const byte *a, float32 *b, const int32 sz)
{
  int32 ecx=sz>>4;
  int32 edx=sz&0xf;

  asm(
      "orl %%ecx, %%ecx;\n\t"
      "jz .GB4;\n\t"
      ".GB2:;\n\t"
      "pxor %%xmm0, %%xmm0;\n\t"
      "movdqu 0(%%esi), %%xmm1;\n\t"
      "movdqu 4(%%esi), %%xmm2;\n\t"
      "movdqu 8(%%esi), %%xmm3;\n\t"
      "movdqu 12(%%esi), %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "cvtdq2ps %%xmm1, %%xmm1;\n\t"
      "cvtdq2ps %%xmm2, %%xmm2;\n\t"
      "movups  %%xmm1, (%%edi);\n\t"
      "movups  %%xmm2, 16(%%edi);\n\t"
      "cvtdq2ps %%xmm3, %%xmm3;\n\t"
      "cvtdq2ps %%xmm4, %%xmm4;\n\t"
      "movups  %%xmm3, 32(%%edi);\n\t"
      "movups  %%xmm4, 48(%%edi);\n\t"
      "addl $16, %%esi;\n\t"
      "addl $64, %%edi;\n\t"
      "loop .GB2;\n\t"
      ".GB4:;\n\t"
      "orl %%edx, %%edx;\n\t"
      "jz .GB1;\n\t"
      "movl %%edx, %%ecx;\n\t"
      ".GB3:;\n\t"
      "xorl %%eax, %%eax;\n\t"
      "movb (%%esi), %%al;\n\t"
      "movd %%eax, %%xmm0;\n\t"
      "cvtdq2ps %%xmm0, %%xmm1;\n\t"
      "movss %%xmm1, (%%edi);\n\t"
      "incl %%esi;\n\t"
      "addl $4, %%edi;\n\t"
      "loop .GB3;\n\t"
      ".GB1:;"
      :
      :"S"(a), "D"(b), "c"(ecx),"d"(edx)
      :"memory"
      );
}



//######################################################################
// speedup ~= 1.15
void sse2_cvt_byte_to_double(const byte *a, double *b, int32 sz)
{
  int32 ecx=sz>>3;
  int32 edx=sz&0x7;

  asm(
      "orl %%ecx, %%ecx;\n\t"
      "jz .GC4;\n\t"
      ".GC2:;\n\t"
      "pxor %%xmm0, %%xmm0;\n\t"
      "movdqu 0(%%esi), %%xmm1;\n\t"
      "movdqu 2(%%esi), %%xmm2;\n\t"
      "movdqu 4(%%esi), %%xmm3;\n\t"
      "movdqu 6(%%esi), %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "punpcklbw %%xmm0, %%xmm1;\n\t"
      "punpcklbw %%xmm0, %%xmm2;\n\t"
      "punpcklbw %%xmm0, %%xmm3;\n\t"
      "punpcklbw %%xmm0, %%xmm4;\n\t"
      "cvtdq2pd %%xmm1, %%xmm1;\n\t"
      "cvtdq2pd %%xmm2, %%xmm2;\n\t"
      "movupd  %%xmm1, (%%edi);\n\t"
      "movupd  %%xmm2, 16(%%edi);\n\t"
      "cvtdq2pd %%xmm3, %%xmm3;\n\t"
      "cvtdq2pd %%xmm4, %%xmm4;\n\t"
      "movupd  %%xmm3, 32(%%edi);\n\t"
      "movupd  %%xmm4, 48(%%edi);\n\t"
      "addl $8, %%esi;\n\t"
      "addl $64, %%edi;\n\t"
      "loop .GC2;\n\t"
      ".GC4:;\n\t"
      "orl %%edx, %%edx;\n\t"
      "jz .GC1;\n\t"
      "movl %%edx, %%ecx;\n\t"
      ".GC3:;\n\t"
      "xorl %%eax, %%eax;\n\t"
      "movb (%%esi), %%al;\n\t"
      "movd %%eax, %%xmm0;\n\t"
      "cvtdq2pd %%xmm0, %%xmm1;\n\t"
      "movsd %%xmm1, (%%edi);\n\t"
      "incl %%esi;\n\t"
      "addl $8, %%edi;\n\t"
      "loop .GC3;\n\t"
      ".GC1:;"
      :
      :"S"(a), "D"(b), "c"(ecx),"d"(edx)
      :"memory"
      );

}



//######################################################################

void sse2_cvt_int_to_float(const int32 *a, float *b, const int32 sz)
{
  int32 ecx=sz>>5;
  int32 edx=sz&0x1f;

  asm(
      "orl %%ecx, %%ecx;\n\t"
      "jz .GD4;\n\t"
      ".GD2:;\n\t"
      "movdqu 0(%%esi), %%xmm0;\n\t"
      "movdqu 16(%%esi), %%xmm1;\n\t"
      "movdqu 32(%%esi), %%xmm2;\n\t"
      "movdqu 48(%%esi), %%xmm3;\n\t"
      "movdqu 64(%%esi), %%xmm4;\n\t"
      "movdqu 80(%%esi), %%xmm5;\n\t"
      "movdqu 96(%%esi), %%xmm6;\n\t"
      "movdqu 112(%%esi), %%xmm7;\n\t"
      "cvtdq2ps %%xmm0, %%xmm0;\n\t"
      "cvtdq2ps %%xmm1, %%xmm1;\n\t"
      "cvtdq2ps %%xmm2, %%xmm2;\n\t"
      "cvtdq2ps %%xmm3, %%xmm3;\n\t"
      "cvtdq2ps %%xmm4, %%xmm4;\n\t"
      "cvtdq2ps %%xmm5, %%xmm5;\n\t"
      "cvtdq2ps %%xmm6, %%xmm6;\n\t"
      "cvtdq2ps %%xmm7, %%xmm7;\n\t"
      "movups %%xmm0, 0(%%edi);\n\t"
      "movups %%xmm1, 16(%%edi);\n\t"
      "movups %%xmm2, 32(%%edi);\n\t"
      "movups %%xmm3, 48(%%edi);\n\t"
      "movups %%xmm4, 64(%%edi);\n\t"
      "movups %%xmm5, 80(%%edi);\n\t"
      "movups %%xmm6, 96(%%edi);\n\t"
      "movups %%xmm7, 112(%%edi);\n\t"
      "addl $128, %%esi;\n\t"
      "addl $128, %%edi;\n\t"
      "decl %%ecx;\n\t"
      "jnz .GD2;\n\t"
      ".GD4:;\n\t"
      "orl %%edx, %%edx;\n\t"
      "jz .GD1;\n\t"
      "movl %%edx, %%ecx;\n\t"
      ".GD3:;\n\t"
      "movd (%%esi), %%xmm0;\n\t"
      "cvtdq2ps %%xmm0, %%xmm0;\n\t"
      "movss %%xmm0, (%%edi);\n\t"
      "addl $4, %%esi;\n\t"
      "addl $4, %%edi;\n\t"
      "loop .GD3;\n\t"
      ".GD1:;"
      :
      :"S"(a), "D"(b), "c"(ecx),"d"(edx)
      :"memory"
      );

}

//######################################################################
// speedup ~= 1.2
void sse2_cvt_int_to_double(const int32 *a, double *b, const int32 sz)
{
  int32 ecx=sz>>4;
  int32 edx=sz&0xf;

  asm(
      "orl %%ecx, %%ecx;\n\t"
      "jz .GE4;\n\t"
      ".GE2:;\n\t"
      "movdqu 0(%%esi), %%xmm0;\n\t"
      "movdqu  8(%%esi), %%xmm1;\n\t"
      "movdqu 16(%%esi), %%xmm2;\n\t"
      "movdqu 24(%%esi), %%xmm3;\n\t"
      "movdqu 32(%%esi), %%xmm4;\n\t"
      "movdqu 40(%%esi), %%xmm5;\n\t"
      "movdqu 48(%%esi), %%xmm6;\n\t"
      "movdqu 56(%%esi), %%xmm7;\n\t"
      "cvtdq2pd %%xmm0, %%xmm0;\n\t"
      "cvtdq2pd %%xmm1, %%xmm1;\n\t"
      "cvtdq2pd %%xmm2, %%xmm2;\n\t"
      "cvtdq2pd %%xmm3, %%xmm3;\n\t"
      "cvtdq2pd %%xmm4, %%xmm4;\n\t"
      "cvtdq2pd %%xmm5, %%xmm5;\n\t"
      "cvtdq2pd %%xmm6, %%xmm6;\n\t"
      "cvtdq2pd %%xmm7, %%xmm7;\n\t"
      "movups %%xmm0, 0(%%edi);\n\t"
      "movups %%xmm1, 16(%%edi);\n\t"
      "movups %%xmm2, 32(%%edi);\n\t"
      "movups %%xmm3, 48(%%edi);\n\t"
      "movups %%xmm4, 64(%%edi);\n\t"
      "movups %%xmm5, 80(%%edi);\n\t"
      "movups %%xmm6, 96(%%edi);\n\t"
      "movups %%xmm7, 112(%%edi);\n\t"
      "addl $64, %%esi;\n\t"
      "addl $128, %%edi;\n\t"
      "decl %%ecx;\n\t"
      "jnz .GE2;\n\t"
      ".GE4:;\n\t"
      "orl %%edx, %%edx;\n\t"
      "jz .GE1;\n\t"
      "movl %%edx, %%ecx;\n\t"
      ".GE3:;\n\t"
      "movd (%%esi), %%xmm0;\n\t"
      "cvtdq2pd %%xmm0, %%xmm0;\n\t"
      "movsd %%xmm0, (%%edi);\n\t"
      "addl $4, %%esi;\n\t"
      "addl $8, %%edi;\n\t"
      "loop .GE3;\n\t"
      ".GE1:;"
      :
      :"S"(a), "D"(b), "c"(ecx),"d"(edx)
      :"memory"
      );

}

//######################################################################
void sse2_cvt_float_to_int(const float *a, int *b, const int32 sz)
{
  int32 ecx=sz;
  int32 edx=sz;

  asm (
       "orl %%ecx, %%ecx;\n\t"
       "jz .GF1;\n\t"
       ".GF2:;\n\t"
       "movdqu 0(%%esi), %%xmm0;\n\t"
       "movdqu  8(%%esi), %%xmm1;\n\t"
       "movdqu 16(%%esi), %%xmm2;\n\t"
       "movdqu 24(%%esi), %%xmm3;\n\t"
       "movdqu 32(%%esi), %%xmm4;\n\t"
       "movdqu 40(%%esi), %%xmm5;\n\t"
       "movdqu 48(%%esi), %%xmm6;\n\t"
       "movdqu 56(%%esi), %%xmm7;\n\t"
       "cvtps2dq %%xmm0, %%xmm0;\n\t"
       "cvtps2dq %%xmm1, %%xmm1;\n\t"
       "cvtps2dq %%xmm2, %%xmm2;\n\t"
       "cvtps2dq %%xmm3, %%xmm3;\n\t"
       "cvtps2dq %%xmm4, %%xmm4;\n\t"
       "cvtps2dq %%xmm5, %%xmm5;\n\t"
       "cvtps2dq %%xmm6, %%xmm6;\n\t"
       "cvtps2dq %%xmm7, %%xmm7;\n\t"
       "movdqu %%xmm0, 0(%%edi);\n\t"
       "movdqu %%xmm1, 16(%%edi);\n\t"
       "movdqu %%xmm2, 32(%%edi);\n\t"
       "movdqu %%xmm3, 48(%%edi);\n\t"
       "movdqu %%xmm4, 64(%%edi);\n\t"
       "movdqu %%xmm5, 80(%%edi);\n\t"
       "movdqu %%xmm6, 96(%%edi);\n\t"
       "movdqu %%xmm7, 112(%%edi);\n\t"
       "addl $64, %%esi;\n\t"
       "addl $128, %%edi;\n\t"
       "decl %%ecx;\n\t"
       "jnz .GF2;\n\t"
       ".GF4:;\n\t"
       "orl %%edx, %%edx;\n\t"
       "jz .GF1;\n\t"
       "movl %%edx, %%ecx;\n\t"
       ".GF3:;\n\t"
       "movd (%%esi), %%xmm0;\n\t"
       "cvtps2dq %%xmm0, %%xmm0;\n\t"
       "movd  %%xmm0, (%%edi);\n\t"
       "addl $4, %%esi;\n\t"
       "addl $8, %%edi;\n\t"
       "loop .GF3;\n\t"
       ".GF1:;"
       :
       :"S"(a), "D"(b), "c"(ecx),"d"(edx)
       :"memory"
       );

}



//######################################################################
void sse2_cvt_float_to_double(const float *a, double *b, const int32 sz)
{
  int32 ecx=sz>>4;
  int32 edx=sz&0xf;

  asm(
      "orl %%ecx, %%ecx;\n\t"
      "jz .GG4;\n\t"
      ".GG2:;\n\t"
      "movups 0(%%esi), %%xmm0;\n\t"
      "movups  8(%%esi), %%xmm1;\n\t"
      "movups 16(%%esi), %%xmm2;\n\t"
      "movups 24(%%esi), %%xmm3;\n\t"
      "movups 32(%%esi), %%xmm4;\n\t"
      "movups 40(%%esi), %%xmm5;\n\t"
      "movups 48(%%esi), %%xmm6;\n\t"
      "movups 56(%%esi), %%xmm7;\n\t"
      "cvtps2pd %%xmm0, %%xmm0;\n\t"
      "cvtps2pd %%xmm1, %%xmm1;\n\t"
      "cvtps2pd %%xmm2, %%xmm2;\n\t"
      "cvtps2pd %%xmm3, %%xmm3;\n\t"
      "cvtps2pd %%xmm4, %%xmm4;\n\t"
      "cvtps2pd %%xmm5, %%xmm5;\n\t"
      "cvtps2pd %%xmm6, %%xmm6;\n\t"
      "cvtps2pd %%xmm7, %%xmm7;\n\t"
      "movupd %%xmm0, 0(%%edi);\n\t"
      "movupd %%xmm1, 16(%%edi);\n\t"
      "movupd %%xmm2, 32(%%edi);\n\t"
      "movupd %%xmm3, 48(%%edi);\n\t"
      "movupd %%xmm4, 64(%%edi);\n\t"
      "movupd %%xmm5, 80(%%edi);\n\t"
      "movupd %%xmm6, 96(%%edi);\n\t"
      "movupd %%xmm7, 112(%%edi);\n\t"
      "addl $64, %%esi;\n\t"
      "addl $128, %%edi;\n\t"
      "decl %%ecx;\n\t"
      "jnz .GG2;\n\t"
      ".GG4:;\n\t"
      "orl %%edx, %%edx;\n\t"
      "jz .GG1;\n\t"
      "movl %%edx, %%ecx;\n\t"
      ".GG3:;\n\t"
      "movd (%%esi), %%xmm0;\n\t"
      "cvtps2pd %%xmm0, %%xmm0;\n\t"
      "movsd %%xmm0, (%%edi);\n\t"
      "addl $4, %%esi;\n\t"
      "addl $8, %%edi;\n\t"
      "loop .GG3;\n\t"
      ".GG1:;"
      :
      :"S"(a), "D"(b), "c"(ecx),"d"(edx)
      :"memory"
      );
}

#endif

#ifdef INVT_USE_SSE

//######################################################################
void sse_lowPass3x(const float *a, float *b, const int h, const int w)
{
  const float coeffs[] = { 3.0, 1.0, 1.0, 1.0, 4.0, 4.0, 4.0, 4.0};
  int edx = (w-2)/12;
  int eax = (w-2)%12;

  asm (
       //       "movups 16(%%ebx), %%xmm7;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz  .HA1;\n\t"
       ".HA2:;\n\t"

       // *dptr++ = (sptr[0]+sptr[0]+sptr[1])/3.0
       "movss 0(%%esi), %%xmm1;\n\t"  // xmm1 <- sptr[0]
       "movss 4(%%esi), %%xmm2;\n\t" // xmm2 <- sptr[1]
       "addss %%xmm1, %%xmm1;\n\t"   // xmm2 <- sptr[0] + sptr[0]
       "addss %%xmm1, %%xmm2;\n\t"   // xmm2 <- xmm2 + sptr[1]
       "divss (%%ebx), %%xmm2;\n\t" // xmm2 <- xmm2/3.0
       "movss %%xmm2, (%%edi);\n\t"  // *dptr <- xmm2
       "addl  $4, %%edi;\n\t"        // ++dptr

       //  for (int i = 0; i < w - 2; i ++)
       "orl %%edx, %%edx;\n\t"
       "jz .HA4;\n\t"

       "pushl %%edx;\n\t"
       ".HA3:;\n\t"
       "movups 00(%%esi),  %%xmm0;\n\t"
       "movups 04(%%esi),  %%xmm1;\n\t"
       "movups 8(%%esi),  %%xmm2;\n\t"
       "movups 16(%%esi),  %%xmm3;\n\t"
       "movups 20(%%esi),  %%xmm4;\n\t"
       "movups 24(%%esi),  %%xmm5;\n\t"
       "movups 32(%%esi),  %%xmm6;\n\t"
       "movups 36(%%esi),  %%xmm7;\n\t"
       "addps  %%xmm1, %%xmm0;\n\t"
       "addps  %%xmm4, %%xmm3;\n\t"
       "addps  %%xmm1, %%xmm0;\n\t"
       "addps  %%xmm4, %%xmm3;\n\t"
       "movups 40(%%esi), %%xmm1;\n\t"
       "addps  %%xmm7, %%xmm6;\n\t"
       "addps  %%xmm2, %%xmm0;\n\t"
       "addps  %%xmm1, %%xmm6;\n\t"
       "addps  %%xmm5, %%xmm3;\n\t"
       "addps  %%xmm7, %%xmm6;\n\t"
       "divps  16(%%ebx ), %%xmm0;\n\t"
       "divps  16(%%ebx ), %%xmm3;\n\t"
       "divps  16(%%ebx ), %%xmm6;\n\t"
       "movups %%xmm0, (%%edi);\n\t"
       "movups %%xmm3, 16(%%edi);\n\t"
       "movups %%xmm6, 32(%%edi);\n\t"
       "addl   $48, %%esi;\n\t"
       "addl   $48, %%edi;\n\t"
       "decl   %%edx;\n\t"
       "jnz  .HA3;\n\t"
       "popl %%edx;\n\t"
       ".HA4:;\n\t"

       "orl  %%eax, %%eax;\n\t"
       "jz .HA6;\n\t"
       "pushl %%eax;\n\t"
       ".HA5:;\n\t"
       "movss  00(%%esi),  %%xmm0;\n\t"
       "movss  04(%%esi),  %%xmm1;\n\t"
       "movss  8(%%esi),  %%xmm2;\n\t"
       "addps  %%xmm1, %%xmm0;\n\t"
       "addps  %%xmm1, %%xmm2;\n\t"
       "addps  %%xmm2, %%xmm0;\n\t"
       "divss  16(%%ebx ), %%xmm0;\n\t"
       "movss  %%xmm0, (%%edi);\n\t"
       "addl   $4, %%esi;\n\t"
       "addl   $4, %%edi;\n\t"
       "decl %%eax;\n\t"
       "jnz .HA5;\n\t"
       "popl %%eax;\n\t"

       ".HA6:;\n\t"
       "movss (%%esi), %%xmm1;\n\t"  // xmm1 <- sptr[0]
       "movss 4(%%esi), %%xmm2;\n\t" // xmm2 <- sptr[1]
       "addss %%xmm2, %%xmm2;\n\t"   // xmm2 <- sptr[0] + sptr[1]
       "addss %%xmm1, %%xmm2;\n\t"   // xmm2 <- xmm2 + sptr[0]
       "divss 0(%%ebx), %%xmm2;\n\t" // xmm2 <- xmm2/3.0

       "movss %%xmm2, (%%edi);\n\t"     // *dptr <- xmm2
       "addl  $4, %%edi;\n\t"        // ++dptr
       "addl  $8, %%esi;\n\t"        // sptr += 2
       "decl %%ecx;\n\t"
       "jnz .HA2;\n\t"
       ".HA1:;\n\t"
       :
       :"S"(a), "D"(b),"c"(h),"a"(eax),"d"(edx),"b"(coeffs)
       :"memory"
       );

}




//######################################################################

void sse_lowPass3y(const float *a, float *b, const int h, const int w)
{
  const float coeffs[] = { 3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0, 4.0};

  if (h < 2){
    memcpy(b, a, w*h*sizeof(b[0]));
    return; // nothing to smooth
  }

  if (h < 2){
    memcpy(b, a, w*h*sizeof(b[0]));
    return; // nothing to smooth
  }

  asm (
       // top row
       "movl %%edx, %%ecx;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz .HU1;\n\t"
       "push %%esi;\n\t"
       ".HU0:;\n\t"
       "movss (%%esi), %%xmm0;\n\t" // xmm0 <- sptr[0]
       "movss (%%esi, %%edx, 4), %%xmm1;\n\t" //xmm1 <- sptr[w]
       "addss %%xmm0, %%xmm0;\n\t"
       "addss %%xmm1, %%xmm0;\n\t"
       "divss (%%ebx), %%xmm0;\n\t"
       "addl $4, %%esi;\n\t"
       "movss %%xmm0, (%%edi);\n\t"
       "addl  $4, %%edi;\n\t"
       "decl %%ecx;\n\t"
       "jnz .HU0;\n\t"
       "popl %%esi;\n\t"
       ".HU1:;\n\t"
       "cmpl $2, %%eax;\n\t"
       "jle .HU5;\n\t"

       "pushl %%eax;\n\t"
       "subl $2, %%eax;\n\t"
       "jle .HU4;\n\t"
       ".HU2:;\n\t"
       "movl %%edx, %%ecx;\n\t"
       "pushl %%edx;\n\t"
       ".HU3:;\n\t"
       "movss (%%esi), %%xmm0;\n\t" //xmm0 <- sptr[0]
       "movss (%%esi,%%edx,4), %%xmm1;\n\t" //xmm1 <- sptr[w]
       "movss (%%esi,%%edx,8), %%xmm2;\n\t" //xmm2 <- sptr[2*w]
       "addss %%xmm1, %%xmm0;\n\t"
       "addss %%xmm1, %%xmm2;\n\t"
       "addss %%xmm2, %%xmm0;\n\t"
       "divss 16(%%ebx), %%xmm0;\n\t"
       "movss %%xmm0, (%%edi);\n\t"
       "addl  $4, %%esi;\n\t"
       "addl  $4, %%edi;\n\t"
       "decl  %%ecx;\n\t"
       "jnz .HU3;\n\t"
       "popl %%edx;\n\t"
       "decl %%eax;\n\t"
       "jnz .HU2;\n\t"

       ".HU4:;\n\t"
       "popl %%eax;\n\t"
       ".HU5:;\n\t"
       "orl %%edx, %%edx;\n\t"
       "jz .HU7;\n\t"
       "pushl %%edx;\n\t"
       "movl  %%edx, %%ecx;\n\t"
       ".HU6:;\n\t"
       "movss (%%esi), %%xmm0;\n\t" //xmm0 <- sptr[0]
       "movss (%%esi,%%ecx,4), %%xmm1;\n\t" //xmm1 <- sptr[w]
       "addss %%xmm1, %%xmm1;\n\t"
       "addss %%xmm1, %%xmm0;\n\t"
       "divss (%%ebx), %%xmm0;\n\t"
       "movss %%xmm0, (%%edi);\n\t"
       "addl $4, %%esi;\n\t"
       "addl $4, %%edi;\n\t"
       "decl %%edx;\n\t"
       "jnz .HU6;\n\t"
       "popl %%edx;\n\t"
       ".HU7:;\n\t"
       :
       :"S"(a),"D"(b),"a"(h),"d"(w),"b"(coeffs)
       );

}


//######################################################################

void sse_lowPass5x(const float *src, float *dest, const int h, const int w)
{
  const float *sptr= src;
  float *dptr= dest;

  if(w<2)
    {
      memcpy(dest,src,h*w*sizeof(dest[0]));
      return;
    }

  if (w == 2) //////////////////////////////////////////////////
    for (int j = 0; j < h; j ++)
      {
        // leftmost point  [ (6^) 4 ] / 10
        *dptr++ = sptr[0] * (6.0F / 10.0F) + sptr[1] * (4.0F / 10.0F);

        // rightmost point  [ 4^ (6) ] / 10
        *dptr++ = sptr[0] * (4.0F / 10.0F) + sptr[1] * (6.0F / 10.0F);

        sptr += 2;  // sptr back to same position as dptr
      }
  else if (w == 3) //////////////////////////////////////////////////
    for (int j = 0; j < h; j ++)
      {
        // leftmost point  [ (6^) 4 1 ] / 11
        *dptr++ = sptr[0] * (6.0F / 11.0F) +
          sptr[1] * (4.0F / 11.0F) +
          sptr[2] * (1.0F / 11.0F);

        // middle point    [ 4^ (6) 4 ] / 14
        *dptr++ = (sptr[0] + sptr[2]) * (4.0F / 14.0F) +
          sptr[1] * (6.0F / 14.0F);

        // rightmost point  [ 1^ 4 (6) ] / 11
        *dptr++ = sptr[0] * (1.0F / 11.0F) +
          sptr[1] * (4.0F / 11.0F) +
          sptr[2] * (6.0F / 11.0F);

        sptr += 3;  // sptr back to same position as dptr
      }
  else
    if(w>3)
      {
        const float coeffs[] = {6.0/11.0, 4.0/11.0, 1.0/11.0, 4.0/15.0,
                                4.0/15.0, 6.0/15.0, 1.0/15.0, 1.0/16.0,
                                1.0/16.0, 1.0/16.0, 1.0/16.0, 1.0/16.0,
                                4.0/16.0, 4.0/16.0, 4.0/16.0, 4.0/16.0,
                                6.0/16.0, 6.0/16.0, 6.0/16.0, 6.0/16.0,
                                1.0/15.0, 4.0/15.0, 6.0/15.0, 1.0/15.0,
                                1.0/11.0, 4.0/11.0, 6.0/11.0, 1.0/11.0
        };

        int eax= (w-4)&3;
        int edx= (w-4)>>2;

        asm(
            "orl %%ecx, %%ecx;\n\t"  // ecx <- h
            "jz .HG6;\n\t"
            ".HG0:;\n\t"
            "movss   (%%esi), %%xmm0;\n\t" // xmm0 <- s[0]
            "movss  4(%%esi), %%xmm2;\n\t" // xmm2 <- s[1]
            "movss  8(%%esi), %%xmm4;\n\t" // xmm4 <- s[2]
            "movss 12(%%esi), %%xmm6;\n\t" // xmm6 <- s[3]
            "movss  %%xmm0, %%xmm1;\n\t"   // xmm1 <- s[0]
            "movss  %%xmm2, %%xmm3;\n\t"   // xmm3 <- s[1]
            "movss  %%xmm4, %%xmm5;\n\t"   // xmm5 <- s[2]
            "mulss   (%%ebx), %%xmm0;\n\t" // xmm0 <- 6.0/11.0*s[0]
            "mulss  4(%%ebx), %%xmm2;\n\t" // xmm2 <- 4.0/11.0*s[1]
            "mulss  8(%%ebx), %%xmm4;\n\t" // xmm4 <- 1.0/11.0*s[2]
            "addss  %%xmm5, %%xmm1;\n\t"   // xmm1 <- s[2]+s[0]
            "mulss 16(%%ebx), %%xmm1;\n\t" // xmm1 <- (s2+s0)*4.0/15.0
            "mulss 20(%%ebx), %%xmm3;\n\t"
            "mulss 24(%%ebx), %%xmm6;\n\t"
            "addss %%xmm2, %%xmm0;\n\t"
            "addss %%xmm3, %%xmm1;\n\t"
            "addss %%xmm4, %%xmm0;\n\t"
            "addss %%xmm6, %%xmm1;\n\t"
            "movss %%xmm0,   (%%edi);\n\t"
            "movss %%xmm1,  4(%%edi);\n\t"
            "addl  $8, %%edi;\n\t"

            "orl   %%edx, %%edx;\n\t"
            "jz .HG5;\n\t"

            "pushl %%edx;\n\t"   // edx <- (w-4)/4
            "movups  32(%%ebx), %%xmm5;\n\t" // xmm5 <- 1.0/16.0 1.0/16.0 1.0/16 1.0/16
            "movups  48(%%ebx), %%xmm6;\n\t" // xmm6 <- 4.0/16.0 ......................
            "movups  64(%%ebx), %%xmm7;\n\t" // xmm7 <- 6.0/16.0 ......................
            ".HG1:;\n\t"
            "movups   0(%%esi), %%xmm0;\n\t" // xmm0 <- s0  s1  s2  s3
            "movups 04(%%esi), %%xmm1;\n\t" // xmm1 <- s1  s2  s3  s4
            "movups  8(%%esi), %%xmm2;\n\t" // xmm2 <- s2  s3  s4  s5
            "movups 12(%%esi), %%xmm3;\n\t" // xmm3 <- s3  s4  s5  s6
             "movups 16(%%esi), %%xmm4;\n\t" // xmm4 <- s4  s5  s6  s7
            "addps  %%xmm4, %%xmm0;\n\t"
            "addps  %%xmm3, %%xmm1;\n\t"
            "mulps  %%xmm5, %%xmm0;\n\t"
            "mulps  %%xmm6, %%xmm1;\n\t"
            "mulps  %%xmm7, %%xmm2;\n\t"
            "addps  %%xmm1, %%xmm0;\n\t"
            "addps  %%xmm2, %%xmm0;\n\t"
            "movups %%xmm0, (%%edi);\n\t"
            "addl   $16, %%esi;\n\t"
            "addl   $16, %%edi;\n\t"
            "decl   %%edx;\n\t"
            "jnz .HG1;\n\t"
            "popl %%edx;\n\t"

            ".HG5:;\n\t"
            "orl  %%eax, %%eax;\n\t"
            "jz  .HG3;\n\t"
            "pushl %%eax;\n\t"       // eax <- (w-4)%4
            "movups 32(%%ebx), %%xmm5;\n\t"
            "movups 48(%%ebx), %%xmm6;\n\t"
            "movups 64(%%ebx), %%xmm7;\n\t"
            ".HG2:;\n\t"
            "movss    (%%esi), %%xmm0;\n\t"
            "movss   4(%%esi), %%xmm1;\n\t"
            "movss   8(%%esi), %%xmm2;\n\t"
            "movss  12(%%esi), %%xmm3;\n\t"
            "movss  16(%%esi), %%xmm4;\n\t"
            "mulss  %%xmm5   , %%xmm0;\n\t"
            "mulss  %%xmm6   , %%xmm1;\n\t"
            "mulss  %%xmm7   , %%xmm2;\n\t"
            "mulss  %%xmm6   , %%xmm3;\n\t"
            "mulss  %%xmm5   , %%xmm4;\n\t"
            "addss  %%xmm1, %%xmm0;\n\t"
            "addss  %%xmm3, %%xmm2;\n\t"
            "addss  %%xmm4, %%xmm0;\n\t"
            "addss  %%xmm2, %%xmm0;\n\t"
            "addl   $4, %%esi;\n\t"
            "movss  %%xmm0, (%%edi);\n\t"
            "addl   $4, %%edi;\n\t"
            "decl   %%eax;\n\t"
            "jnz .HG2;\n\t"
            "popl  %%eax;\n\t"
            ".HG3:;\n\t"
            "movss  (%%esi), %%xmm0;\n\t"  // xmm0 <- s0
            "movss 4(%%esi), %%xmm1;\n\t"  // xmm1 <- s1
            "movss 8(%%esi), %%xmm2;\n\t"  // xmm2 <- s2
            "movss 12(%%esi), %%xmm3;\n\t" // xmm3 <- s3
            "movss %%xmm1, %%xmm4;\n\t"    // xmm4 <- s1
            "movss %%xmm2, %%xmm5;\n\t"    // xmm5 <- s2
            "movss %%xmm3, %%xmm6;\n\t"    // xmm6 <- s3
            "addps %%xmm1, %%xmm3;\n\t"    // xmm3 <- s1+s3
            "mulss 80(%%ebx), %%xmm0;\n\t" // xmm0 <- 1.0/15.0*s0
            "mulss 84(%%ebx), %%xmm3;\n\t" // xmm3 <- 4.0/15.0*(s1+s3)
            "mulss 88(%%ebx), %%xmm2;\n\t" // xmm2 <- 6.0/15.0*s2
            "addss %%xmm3, %%xmm0;\n\t"
            "addss %%xmm2, %%xmm0;\n\t"
            "movss %%xmm0, (%%edi);\n\t"
            "mulss 96(%%ebx), %%xmm4;\n\t"
            "mulss 100(%%ebx), %%xmm5;\n\t"
            "mulss 104(%%ebx), %%xmm6;\n\t"
            "addss %%xmm5, %%xmm4;\n\t"
            "addss %%xmm6, %%xmm4;\n\t"
            "movss %%xmm4, 4(%%edi);\n\t"
            "addl $16, %%esi;\n\t"
            "addl $8, %%edi;\n\t"
            "decl %%ecx;\n\t"
            "jnz .HG0;\n\t"
            ".HG6:;\n\t"
            :
            :"S"(sptr),"D"(dptr),"a"(eax),"b"(coeffs),"c"(h),"d"(edx)
            :"memory"
            );
      }

}



//######################################################################

void sse_lowPass5y(const float *src, float *dest, const int h,
                       const int w)
{
  if (h < 2){
    memcpy(dest, src, h*w*sizeof(dest[0]));
    return; // nothing to smooth
  }

  const float *sptr= src;
  float *dptr= dest;

  // ########## vertical pass  (even though we scan horiz for speedup)
  const int w2 = w * 2; // speedup


  if (h == 2) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 ] / 10 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[0] * (6.0F / 10.0F) +
            sptr[w] * (4.0F / 10.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // bottommost points  ( [ 4^ (6) ] / 10 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[0] * (4.0F / 10.0F) +
            sptr[w] * (6.0F / 10.0F);
          sptr++;
        }
    }
  else if (h == 3) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (6.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (1.0F / 11.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // middle points  ( [ 4^ (6) 4 ] / 14 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = (sptr[ 0] + sptr[w2]) * (4.0F / 14.0F) +
            sptr[ w] * (6.0F / 14.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // bottommost points  ( [ 1^ 4 (6) ] / 11 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (1.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (6.0F / 11.0F);
          sptr++;
        }
    }
  else  ///////////////////////////////// general case for height >= 4
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T

      static const float coeffs[] = {
        6.0/11.0, 6.0/11.0, 6.0/11.0, 6.0/11.0, //0
        4.0/11.0, 4.0/11.0, 4.0/11.0, 4.0/11.0, //16
        1.0/11.0, 1.0/11.0, 1.0/11.0, 1.0/11.0, //32
        4.0F/15.0F, 4.0F/15.0F, 4.0F/15.0F, 4.0F/15.0F, //48
        6.0F/15.0F, 6.0F/15.0F, 6.0F/15.0F, 6.0F/15.0F, //64
        1.0F/15.0F, 1.0F/15.0F, 1.0F/15.0F, 1.0F/15.0F, //80
        1.0/16.0, 1.0/16.0, 1.0/16.0, 1.0/16.0, //96
        4.0F/16.0F, 4.0F/16.0F, 4.0F/16.0F, 4.0F/16.0F, //112
        6.0F/16.0F, 6.0F/16.0F, 6.0F/16.0F, 6.0F/16.0F  //128
      };

      int ecx=h-4;
      int edx=w>>2;
      int eax=w&3;

      asm (
           "pushl %%ebp;\n\t"
           "movl %0, %%ebp;\n\t"
           "addl %%ebp, %%ebp;\n\t"
           "addl %%ebp, %%ebp;\n\t"

           // 1st loop
           "movups (%%ebx), %%xmm4;\n\t"          //xmm4 <- 6.0/11.0 ...
           "movups 16(%%ebx), %%xmm5;\n\t"        //xmm5 <- 4.0/11.0
           "movups 32(%%ebx), %%xmm6;\n\t"        //xmm6 <- 1.0/11.0
           "pushl %%esi;\n\t"
           "orl  %%edx, %%edx;\n\t"
           "jz .IA1;\n\t"
           ".align 4;\n\t"
           "pushl %%edx;\n\t"
           ".IA0:;\n\t"
           ".align 4;\n\t"
           "movups (%%esi), %%xmm0;\n\t"          //xmm0 <- s0   s0   s0   s0
           "movups (%%esi,%%ebp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
           "movups (%%esi,%%ebp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
           "mulps  %%xmm4, %%xmm0;\n\t"
           "mulps  %%xmm5, %%xmm1;\n\t"
           "mulps  %%xmm6, %%xmm2;\n\t"
           "addps  %%xmm1, %%xmm0;\n\t"
           "addps  %%xmm2, %%xmm0;\n\t"
           "movups %%xmm0, (%%edi);\n\t"
           "addl $16, %%esi;\n\t"
           "addl $16, %%edi;\n\t"
           "decl %%edx;\n\t"
           "jnz .IA0;\n\t"
           "popl %%edx;\n\t"
           ".IA1:;\n\t"
           ".align 4;\n\t"
           "orl %%eax, %%eax;\n\t"
           "jz .IA3;\n\t"
           "pushl %%eax;\n\t"
           ".IA2:;\n\t"
           ".align 4;\n\t"
           "movss  (%%esi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
           "movss  (%%esi,%%ebp,1), %%xmm1;\n\t"  //xmm1 <- sW+3 sW+2 sW+1 sW
           "movss  (%%esi,%%ebp,2), %%xmm2;\n\t"  //xmm2 <- sP+3 sP+3 sP+1 sP
           "mulss  %%xmm4, %%xmm0;\n\t"
           "mulss  %%xmm5, %%xmm1;\n\t"
           "mulss  %%xmm6, %%xmm2;\n\t"
           "addss  %%xmm1, %%xmm0;\n\t"
           "addss  %%xmm2, %%xmm0;\n\t"
           "movss  %%xmm0, (%%edi);\n\t"
           "addl $4, %%esi;\n\t"
           "addl $4, %%edi;\n\t"
           "decl %%eax;\n\t"
           "jnz .IA2;\n\t"
           "popl %%eax;\n\t"
           ".IA3:;\n\t"
           "popl %%esi;\n\t"  // restore sptr

           // 2nd loop
           "movups 48(%%ebx), %%xmm4;\n\t" //xmm4 <- 4.0/15.0
           "movups 64(%%ebx), %%xmm5;\n\t" //xmm5 <- 6.0/15.0
           "movups 80(%%ebx), %%xmm6;\n\t" //xmm6 <- 1.0/15.0
           "pushl %%esi;\n\t"
           "orl   %%edx, %%edx;\n\t"
           "jz .IA5;\n\t"
           "pushl %%edx;\n\t"
           "pushl %%eax;\n\t"
           "movl  %%ebp, %%eax;\n\t"
           "addl  %%ebp, %%eax;\n\t"
           "addl  %%ebp, %%eax;\n\t"
           ".IA4:;\n\t"
           "movups (%%esi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
           "movups (%%esi,%%ebp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
           "movups (%%esi,%%ebp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
           "movups (%%esi,%%eax,1), %%xmm3;\n\t"  //xmm3 <- sW3  sW3  sW3  sW3
           "addps  %%xmm2, %%xmm0;\n\t"
           "mulps  %%xmm4, %%xmm0;\n\t"
           "mulps  %%xmm5, %%xmm1;\n\t"
           "mulps  %%xmm6, %%xmm3;\n\t"
           "addps  %%xmm1, %%xmm0;\n\t"
           "addps  %%xmm3, %%xmm0;\n\t"
           "movups %%xmm0, (%%edi);\n\t"
           "addl $16, %%esi;\n\t"
           "addl $16, %%edi;\n\t"
           "decl %%edx;\n\t"
           "jnz .IA4;\n\t"
           "popl %%eax;\n\t"
           "popl %%edx;\n\t"
           ".IA5:;\n\t"
           "orl %%eax, %%eax;\n\t"
           "jz .IA7;\n\t"
           "pushl %%eax;\n\t"
           "pushl %%edx;\n\t"
           "movl  %%ebp, %%edx;\n\t"
           "addl  %%ebp, %%edx;\n\t"
           "addl  %%ebp, %%edx;\n\t"
           ".IA6:;\n\t"
           "movss  (%%esi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
           "movss  (%%esi,%%ebp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
           "movss  (%%esi,%%ebp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
           "movss  (%%esi,%%edx,1), %%xmm3;\n\t" //xmm3 <- sW3  sW3  sW3  sW3
           "addss  %%xmm2, %%xmm0;\n\t"
           "mulss  %%xmm4, %%xmm0;\n\t"
           "mulss  %%xmm5, %%xmm1;\n\t"
           "mulss  %%xmm6, %%xmm3;\n\t"
           "addss  %%xmm1, %%xmm0;\n\t"
           "addss  %%xmm3, %%xmm0;\n\t"
           "movss  %%xmm0, (%%edi);\n\t"
           "addl $4, %%esi;\n\t"
           "addl $4, %%edi;\n\t"
           "decl %%eax;\n\t"
           "jnz .IA6;\n\t"
           "popl %%edx;\n\t"
           "popl %%eax;\n\t"
           ".IA7:;\n\t"
           "popl %%esi;\n\t"  // restore sptr


           //            the double loops
           "orl %%ecx, %%ecx;\n\t"
           "jz .IA29;\n\t"
           "pushl %%ecx;\n\t"
           "movups 96(%%ebx), %%xmm5;\n\t"    // xmm5 <- 1.0/16.0
           "movups 112(%%ebx), %%xmm6;\n\t"   // xmm6 <- 4.0/16.0
           "movups 128(%%ebx), %%xmm7;\n\t"   // xmm7 <- 6.0/16.0
           ".IA8:;\n\t"
           "orl  %%edx, %%edx;\n\t"
           "jz .IA10;\n\t"
           "pushl %%edx;\n\t"
           "pushl %%eax;\n\t"
           "movl  %%ebp, %%eax;\n\t"
           "addl  %%ebp, %%eax;\n\t"
           "addl  %%ebp, %%eax;\n\t"                // eax <- 3*W
           ".IA9:;\n\t"
           "movups  (%%esi),  %%xmm0;\n\t"          // xmm0 <- s    s    s    s
           "movups  (%%esi,%%ebp,1),  %%xmm1;\n\t"  // xmm1 <- sW   sW   sW   sW
           "movups  (%%esi,%%ebp,2),  %%xmm2;\n\t"  // xmm2 <- sW2  sW2  sW2  sW2
           "movups  (%%esi,%%eax,1), %%xmm3;\n\t"   // xmm3 <- sW3  sW3  sW3  sW3
           "movups  (%%esi,%%ebp,4), %%xmm4;\n\t"   // xmm4 <- sW4  sW4  sW4  sW4
           "addps   %%xmm3, %%xmm1;\n\t"            // xmm1 <- sW3 + sW1
           "addps   %%xmm4, %%xmm0;\n\t"            // xmm0 <- s0  + sW4
           "mulps   %%xmm6, %%xmm1;\n\t"            // xmm1 <- 4.0/16.0*(sW3+sW1)
           "mulps   %%xmm5, %%xmm0;\n\t"            // xmm0 <- 1.0/16.08(s0 +sW4)
           "mulps   %%xmm7, %%xmm2;\n\t"            // xmm2 <- 6.0/16.0*sW2
           "addps   %%xmm1, %%xmm0;\n\t"
           "addps   %%xmm2, %%xmm0;\n\t"
           "addl    $16, %%esi;\n\t"
           "movups  %%xmm0, (%%edi);\n\t"
           "addl    $16, %%edi;\n\t"
           "decl   %%edx;\n\t"
           "jnz .IA9;\n\t"
           "popl   %%eax;\n\t"
           "popl   %%edx;\n\t"
           ".IA10:;\n\t"
           "orl  %%eax, %%eax;\n\t"
           "jz .IA12;\n\t"
           "pushl %%eax;\n\t"
           "pushl %%edx;\n\t"
           "movl  %%ebp, %%edx;\n\t"
           "addl  %%ebp, %%edx;\n\t"
           "addl  %%ebp, %%edx;\n\t"
           ".IA11:;\n\t"
           "movss   (%%esi),  %%xmm0;\n\t"          // xmm0 <- s    s    s    s
           "movss   (%%esi,%%ebp,1),  %%xmm1;\n\t"  // xmm1 <- sW   sW   sW   sW
           "movss   (%%esi,%%ebp,2),  %%xmm2;\n\t"  // xmm2 <- sW2  sW2  sW2  sW2
           "movss   (%%esi,%%edx,1), %%xmm3;\n\t"   // xmm3 <- sW3  sW3  sW3  sW3
           "movss   (%%esi,%%ebp,4), %%xmm4;\n\t"   // xmm4 <- sW4  sW4  sW4  sW4
           "addss   %%xmm3, %%xmm1;\n\t"
           "addss   %%xmm4, %%xmm0;\n\t"
           "mulss   %%xmm6, %%xmm1;\n\t"
           "mulss   %%xmm5, %%xmm0;\n\t"
           "mulss   %%xmm7, %%xmm2;\n\t"
           "addss   %%xmm1, %%xmm0;\n\t"
           "addss   %%xmm2, %%xmm0;\n\t"
           "addl    $4, %%esi;\n\t"
           "movss   %%xmm0, (%%edi);\n\t"
           "addl    $4, %%edi;\n\t"
           "decl  %%eax;\n\t"
           "jnz .IA11;\n\t"
           "popl %%edx;\n\t"
           "popl %%eax;\n\t"
           ".IA12:;\n\t"
           "decl %%ecx;\n\t"
           "jnz .IA8;\n\t"
           "popl %%ecx;\n\t"
           ".IA29:;\n\t"

           // fourth loop
           "movups 48(%%ebx), %%xmm4;\n\t"  //xmm4 <- 4.0/15.0
           "movups 64(%%ebx), %%xmm5;\n\t"  //xmm5 <- 6.0/15.0
           "movups 80(%%ebx), %%xmm6;\n\t"  //xmm6 <- 1.0/15.0
           "orl  %%edx, %%edx;\n\t"
           "jz .IA14;\n\t"
           "pushl %%edx;\n\t"
           "pushl %%eax;\n\t"
           "movl  %%ebp, %%eax;\n\t"
           "addl  %%ebp, %%eax;\n\t"
           "addl  %%ebp, %%eax;\n\t"
           ".IA13:;\n\t"
           "movups (%%esi), %%xmm0;\n\t"          //xmm0 <- s0   s0   s0   s0
           "movups (%%esi,%%ebp,1), %%xmm1;\n\t"  //xmm1 <- sW1  sW1  sW1  sW1
           "movups (%%esi,%%ebp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
           "movups (%%esi,%%eax,1),%%xmm3;\n\t"  //xmm3 <- sW3  sW3  sW3  sW3
           "addps  %%xmm3, %%xmm1;\n\t"          //xmm1 <- sW3 + sW1
           "mulps  %%xmm6, %%xmm0;\n\t"          //xmm0 <- 1.0/15.0 * s0
           "mulps  %%xmm5, %%xmm2;\n\t"          //xmm2 <- 6.0/15.0 * sW2
           "mulps  %%xmm4, %%xmm1;\n\t"          //xmm4 <- 4.0/15.0 * (sW3+sW1)
           "addps  %%xmm2, %%xmm0;\n\t"
           "addps  %%xmm1, %%xmm0;\n\t"
           "movups %%xmm0, (%%edi);\n\t"
           "addl $16, %%esi;\n\t"
           "addl $16, %%edi;\n\t"
           "decl %%edx;\n\t"
           "jnz .IA13;\n\t"
           "popl %%eax;\n\t"
           "popl %%edx;\n\t"
           ".IA14:;\n\t"
           "orl %%eax, %%eax;\n\t"
           "jz .IA16;\n\t"
           "pushl %%eax;\n\t"
           "pushl %%edx;\n\t"
           "movl %%ebp, %%edx;\n\t"
           "addl %%ebp, %%edx;\n\t"
           "addl %%ebp, %%edx;\n\t"
           ".IA15:;\n\t"
           "movss  (%%esi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
           "movss  (%%esi, %%ebp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
           "movss  (%%esi, %%ebp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
           "movss  (%%esi, %%edx,1), %%xmm3;\n\t" //xmm3 <- sW3  sW3  sW3  sW3
           "addss  %%xmm3, %%xmm1;\n\t"
           "mulss  %%xmm6, %%xmm0;\n\t"
           "mulss  %%xmm5, %%xmm2;\n\t"
           "mulss  %%xmm4, %%xmm1;\n\t"
           "addss  %%xmm2, %%xmm0;\n\t"
           "addss  %%xmm1, %%xmm0;\n\t"
           "movss  %%xmm0, (%%edi);\n\t"
           "addl $4, %%esi;\n\t"
           "addl $4, %%edi;\n\t"
           "decl %%eax;\n\t"
           "jnz .IA15;\n\t"
           "popl %%edx;\n\t"
           "popl %%eax;\n\t"
           ".IA16:;\n\t"

            // final loop
           "movups 32(%%ebx), %%xmm4;\n\t"
           "movups 16(%%ebx), %%xmm5;\n\t"
           "movups   (%%ebx), %%xmm6;\n\t"
           "orl  %%edx, %%edx;\n\t"
           "jz .IA18;\n\t"
           "pushl %%edx;\n\t"
           ".IA17:;\n\t"
           "movups (%%esi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
           "movups (%%esi,%%ebp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
           "movups (%%esi,%%ebp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
           "mulps  %%xmm4, %%xmm0;\n\t"
           "mulps  %%xmm5, %%xmm1;\n\t"
           "mulps  %%xmm6, %%xmm2;\n\t"
           "addps  %%xmm1, %%xmm0;\n\t"
           "addps  %%xmm2, %%xmm0;\n\t"
           "movups %%xmm0, (%%edi);\n\t"
           "addl $16, %%esi;\n\t"
           "addl $16, %%edi;\n\t"
           "decl %%edx;\n\t"
           "jnz .IA17;\n\t"
           "popl %%edx;\n\t"
           ".IA18:;\n\t"
           "orl %%eax, %%eax;\n\t"
           "jz .IA20;\n\t"
           "pushl %%eax;\n\t"
           ".IA19:;\n\t"
           "movss  (%%esi), %%xmm0;\n\t"          //xmm0 <- s3   s2   s1   s0
           "movss  (%%esi,%%ebp,1), %%xmm1;\n\t"  //xmm1 <- sW   sW   sW   sW
           "movss  (%%esi,%%ebp,2), %%xmm2;\n\t"  //xmm2 <- sW2  sW2  sW2  sW2
           "mulss  %%xmm4, %%xmm0;\n\t"
           "mulss  %%xmm5, %%xmm1;\n\t"
           "mulss  %%xmm6, %%xmm2;\n\t"
           "addss  %%xmm1, %%xmm0;\n\t"
           "addss  %%xmm2, %%xmm0;\n\t"
           "movss  %%xmm0, (%%edi);\n\t"
           "addl $4, %%esi;\n\t"
           "addl $4, %%edi;\n\t"
           "decl %%eax;\n\t"
           "jnz .IA19;\n\t"
           "popl %%eax;\n\t"
           ".IA20:;\n\t"

           "popl %%ebp;\n\t"
           :
           :"m"(w),"S"(sptr),"D"(dptr),"a"(eax),"b"(coeffs),"c"(ecx),"d"(edx)
           );

    }
}


// ######################################################################

void sse_yuv411_to_rgb_mode_640x480(const byte *src, byte *dest,
                                    const int nbpix2)
{
  int ecx=nbpix2/6;

  const float coeffs[] = {
    0.0F,       -0.198242F,   1.014648F,     0.0F,  // R  G   B  xx  -> u
    0.700195F,  -0.29052F,    0.0F,          0.0F,  // R  G   B  xx  -> v
    128.0F,        128.0F,    128.0F,      128.0F   // division factor
  };

  asm (
       ".JA0:;\n\t"
       "orl %%ecx, %%ecx;\n\t"
       "jz .JA1;\n\t"
       "pxor  %%mm7, %%mm7;\n\t"    //mm7 <-  00 00 00 00
       "xorl  %%eax, %%eax;\n\t"
       "xorl  %%ebx, %%ebx;\n\t"
       "movl  (%%esi),   %%eax;\n\t" // eax <-   v   y1  y0 u
       "movw 4(%%esi),  %%bx;\n\t"   // ebx <-   xx  xx  y3 y2
       "movd %%eax, %%mm0;\n\t"        // mm0<- xx xx xx xx v  y1  y0  u
       "movd %%eax, %%mm1;\n\t"        // mm1<- xx xx xx xx v  y1  y0  u
       "movd %%ebx, %%mm2;\n\t"        // mm2<- xx xx xx xx xx xx  y3  y2
       "psrlq $16,  %%mm1;\n\t"        // mm1<- xx xx xx xx xx xx  v   y1
       "punpcklbw %%mm7, %%mm0;\n\t"   // mm0<- xx xx xx xx 0  y0  0   u
       "punpcklbw %%mm7, %%mm1;\n\t"   // mm1<- xx xx xx xx 00 v   00  y1
       "punpcklbw %%mm7, %%mm2;\n\t"   // mm2<- xx xx xx xx 00 y3  00  y2
       "punpcklwd %%mm7, %%mm0;\n\t"   // mm0<- 00 00 00 y0 00 00  00  u
       "punpcklwd %%mm7, %%mm1;\n\t"   // mm1<- 00 00 00 v  00 00  00  y1
       "punpcklwd %%mm7, %%mm2;\n\t"   // mm2<- 00 00 00 y3 00 00  00  y2

       "cvtpi2ps %%mm0, %%xmm0;\n\t"   // xmm0 <- 00 00 y0 u
       "cvtpi2ps %%mm1, %%xmm1;\n\t"   // xmm1 <- 00 00 v  y1
       "cvtpi2ps %%mm2, %%xmm2;\n\t"   // xmm2 <- 00 00 y3 y2

       // 01 01 01 01
       "movaps %%xmm0, %%xmm3;\n\t"

       // 00 00 00 00
       "movaps %%xmm1, %%xmm4;\n\t"

       // 00 00 00 00
       "movaps %%xmm2, %%xmm5;\n\t"

       // 01 01 01 01
       "movaps %%xmm2, %%xmm6;\n\t"

       "shufps $0x55, %%xmm3, %%xmm3;\n\t"// xmm3 <- y0 y0 y0 y0
       "shufps $00, %%xmm4, %%xmm4;\n\t"  // xmm4 <- y1 y1 y1 y1
       "shufps $0x00, %%xmm5, %%xmm5;\n\t"// xmm5 <- y2 y2 y2 y2
       "shufps $0x55, %%xmm6, %%xmm6;\n\t"// xmm6 <- y3 y3 y3 y3

       // 00 00 00 00
       "shufps $0, %%xmm0, %%xmm0;\n\t"  // xmm0 <- u  u  u  u
       // 01 01 01 01
       "shufps $0x55, %%xmm1, %%xmm1;\n\t" // xmm1 <- v  v  v  v

       "subps  32(%%edx), %%xmm0;\n\t"
       "subps  32(%%edx), %%xmm1;\n\t"

       "mulps (%%edx), %%xmm0;\n\t"
       "mulps 16(%%edx),%%xmm1;\n\t"

       "addps %%xmm0, %%xmm3;\n\t"
       "addps %%xmm0, %%xmm4;\n\t"
       "addps %%xmm0, %%xmm5;\n\t"
       "addps %%xmm0, %%xmm6;\n\t"

       "addps %%xmm1, %%xmm3;\n\t"    // xmm3 <- xx b0 g0 r0
       "addps %%xmm1, %%xmm4;\n\t"    // xmm4 <- xx b1 g1 r1
       "addps %%xmm1, %%xmm5;\n\t"    // xmm5 <- xx b2 g2 r2
       "addps %%xmm1, %%xmm6;\n\t"    // xmm6 <- xx b3 g3 r3

       "cvtps2pi %%xmm3, %%mm0;\n\t"  //mm0  <- g0 r0
       "movhlps  %%xmm3, %%xmm3;\n\t" //xmm3 <- g0 r0 xx b0
       "cvtps2pi %%xmm3, %%mm1;\n\t"  //mm1  <- xx b0
       "packssdw %%mm1, %%mm0;\n\t"   //mm0<- xx b0 g0 r0

       "cvtps2pi %%xmm4, %%mm2;\n\t"  //mm2  <- g1 r1
       "movhlps  %%xmm4, %%xmm4;\n\t" //xmm4 <- g1 r1 xx b1
       "cvtps2pi %%xmm4, %%mm3;\n\t"  //mm3  <- xx b1
       "packssdw %%mm3, %%mm2;\n\t"   //mm2<- xx b1 g1 r1

       "cvtps2pi %%xmm5, %%mm4;\n\t"  //mm4  <- g2 r2
       "movhlps  %%xmm5, %%xmm5;\n\t" //xmm5 <- g2 r2 xx b2
       "cvtps2pi %%xmm5, %%mm5;\n\t"  //mm5  <- xx b2
       "packssdw %%mm5, %%mm4;\n\t"   //mm4<- xx b2 g2 r2

       "cvtps2pi %%xmm6, %%mm6;\n\t"  //mm6  <- g3 r3
       "movhlps  %%xmm6, %%xmm6;\n\t" //xmm3 <- g3 r3 xx b3
       "cvtps2pi %%xmm6, %%mm7;\n\t"  //mm7  <- xx b3
       "packssdw %%mm7, %%mm6;\n\t"   //mm6<- xx b3 g3 r3

       "pxor %%mm1, %%mm1;\n\t"
       "pcmpgtw %%mm0, %%mm1;\n\t"
       "pandn %%mm0, %%mm1;\n\t"

       "pxor %%mm3, %%mm3;\n\t"
       "pcmpgtw %%mm2, %%mm3;\n\t"
       "pandn %%mm2, %%mm3;\n\t"

       "pxor %%mm5, %%mm5;\n\t"
       "pcmpgtw %%mm4, %%mm5;\n\t"
       "pandn %%mm4, %%mm5;\n\t"

       "pxor %%mm7, %%mm7;\n\t"
       "pcmpgtw %%mm6, %%mm7;\n\t"
       "pandn %%mm6, %%mm7;\n\t"

       "packuswb %%mm1, %%mm1;\n\t"   //mm0<- xx xx xx xx xx b0 g0 r0
       "packuswb %%mm3, %%mm3;\n\t"   //mm2<- xx xx xx xx xx b1 g1 r1
       "packuswb %%mm5, %%mm5;\n\t"   //mm4<- xx xx xx xx xx b2 g2 r2
       "packuswb %%mm7, %%mm7;\n\t"   //mm6<- xx xx xx xx xx b3 g3 r3

       "pushl %%ecx;\n\t"
       "pushl %%edx;\n\t"
       "movd %%mm1, %%eax;\n\t"  // eax <- xx b0 g0 r0
       "movd %%mm3, %%ebx;\n\t"  // ebx <- xx b1 g1 r1
       "movd %%mm5, %%ecx;\n\t"  // ecx <- xx b2 g2 r2
       "movd %%mm7, %%edx;\n\t"  // edx <- xx b3 g3 r3
       "movw %%ax, (%%edi);\n\t"
       "movw %%bx,3(%%edi);\n\t"
       "movw %%cx,6(%%edi);\n\t"
       "movw %%dx,9(%%edi);\n\t"
       "shrl $8, %%eax;\n\t"
       "shrl $8, %%ebx;\n\t"
       "shrl $8, %%ecx;\n\t"
       "shrl $8, %%edx;\n\t"
       "movb %%ah, 2(%%edi);\n\t"
       "movb %%bh, 5(%%edi);\n\t"
       "movb %%ch, 8(%%edi);\n\t"
       "movb %%dh,11(%%edi);\n\t"
       "popl %%edx;\n\t"
       "popl %%ecx;\n\t"

       "addl $12,%%edi;\n\t"
       "decl %%ecx;\n\t"
       "addl $6, %%esi;\n\t"
       "jmp .JA0;\n\t"
       ".JA1:;\n\t"
       "emms;\n\t"
       :
       :"S"(src),"D"(dest),"c"(ecx),"d"(coeffs)
       :"eax","ebx","memory"
       );

}




void sse_lowPass9x(const float *sptr, float *dptr, const int h, const int w)
{

 for (int j = 0; j < h; j ++)
    {
      // leftmost points
      *dptr++ = sptr[0] * (70.0F / 163.0F) +
        sptr[1] * (56.0F / 163.0F) +
        sptr[2] * (28.0F / 163.0F) +
        sptr[3] * ( 8.0F / 163.0F) +
        sptr[4] * ( 1.0F / 163.0F);
      *dptr++ = (sptr[0] + sptr[2]) * (56.0F / 219.0F) +
        sptr[1] * (70.0F / 219.0F) +
        sptr[3] * (28.0F / 219.0F) +
        sptr[4] * ( 8.0F / 219.0F) +
        sptr[5] * ( 1.0F / 219.0F);
      *dptr++ = (sptr[0] + sptr[4]) * (28.0F / 247.0F) +
        (sptr[1] + sptr[3]) * (56.0F / 247.0F) +
        sptr[2] * (70.0F / 247.0F) +
        sptr[5] * ( 8.0F / 247.0F) +
        sptr[6] * ( 1.0F / 247.0F);
      *dptr++ = (sptr[0] + sptr[6]) * ( 8.0F / 255.0F) +
        (sptr[1] + sptr[5]) * (28.0F / 255.0F) +
        (sptr[2] + sptr[4]) * (56.0F / 255.0F) +
        sptr[3] * (70.0F / 255.0F) +
        sptr[7] * ( 1.0F / 255.0F);

      // far from the borders
      for (int i = 0; i < w - 8; i ++)
        {
          *dptr++ = (sptr[0] + sptr[8]) * ( 1.0F / 256.0F) +
            (sptr[1] + sptr[7]) * ( 8.0F / 256.0F) +
            (sptr[2] + sptr[6]) * (28.0F / 256.0F) +
            (sptr[3] + sptr[5]) * (56.0F / 256.0F) +
            sptr[4] * (70.0F / 256.0F);
          sptr ++;
        }

      // rightmost points
      *dptr++ = sptr[0] * ( 1.0F / 255.0F) +
        (sptr[1] + sptr[7]) * ( 8.0F / 255.0F) +
        (sptr[2] + sptr[6]) * (28.0F / 255.0F) +
        (sptr[3] + sptr[5]) * (56.0F / 255.0F) +
        sptr[4] * (70.0F / 255.0F);
      sptr ++;
      *dptr++ = sptr[0] * ( 1.0F / 247.0F) +
        sptr[1] * ( 8.0F / 247.0F) +
        (sptr[2] + sptr[6]) * (28.0F / 247.0F) +
        (sptr[3] + sptr[5]) * (56.0F / 247.0F) +
        sptr[4] * (70.0F / 247.0F);
      sptr ++;
      *dptr++ = sptr[0] * ( 1.0F / 219.0F) +
        sptr[1] * ( 8.0F / 219.0F) +
        sptr[2] * (28.0F / 219.0F) +
        (sptr[3] + sptr[5]) * (56.0F / 219.0F) +
        sptr[4] * (70.0F / 219.0F);
      sptr ++;
      *dptr++ = sptr[0] * ( 1.0F / 163.0F) +
        sptr[1] * ( 8.0F / 163.0F) +
        sptr[2] * (28.0F / 163.0F) +
        sptr[3] * (56.0F / 163.0F) +
        sptr[4] * (70.0F / 163.0F);
      sptr += 5;  // sptr back to same as dptr (start of next line)
    }
}
#endif

//############################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif

