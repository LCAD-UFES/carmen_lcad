/*!@file Devices/VCC4cmddef.H  command definitions for the Canon VC-C4 camera */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/VCC4cmddef.H $
// $Id: VCC4cmddef.H 9308 2008-02-22 19:04:41Z rjpeters $
//

#ifndef __VCC4CMDDEF_H__
#define __VCC4CMDDEF_H__

#define VCC4_SetCameraPowerOFF                2   //
#define VCC4_SetCameraPowerON                     3
#define VCC4_SetFocusModeAF                        4
#define VCC4_SetFocusModeMANUAL                5
#define VCC4_SetFocusModeNEAR                6
#define VCC4_SetFocusModeFAR                7
#define VCC4_SetFocusPosition                8
#define VCC4_GetFoucsPosition                9
#define VCC4_SetOnePushAF                        10
#define VCC4_SetZoomingSTOP                        11
#define VCC4_SetZoomingWIDE                        12
#define VCC4_SetZoomingTELE                        13
#define VCC4_SetZoomingHIWIDE                14
#define VCC4_SetZoomingHITELE                15
#define VCC4_SetZoomPosition2                16
#define VCC4_GetZoomPosition2                17
#define VCC4_SetZoomPosition4                18
#define VCC4_GetZoomPosition4                19
#define VCC4_SetZoomSpeed                        20
#define VCC4_GetZoomSpeed                        21
#define VCC4_SetBackLightOFF                22
#define VCC4_SetBackLightON                        23
#define VCC4_SetExposedAUTO                        24
#define VCC4_SetExposedMANUAL                25
#define VCC4_SetShutterSpeedPRO                26
#define VCC4_SetShutterSpeed3                28
#define VCC4_SetShutterSpeed60                29
#define VCC4_SetShutterSpeed100                30
#define VCC4_SetAGCGain                                31
#define VCC4_GetAGCGain                                32
#define VCC4_SetIris                                33
#define VCC4_GetIris                                34
#define VCC4_SetAEVal                                35
#define VCC4_GetAEVal                                36
#define VCC4_SetGainSel                                37    // undocumented command
#define VCC4_GetGainSel                                38    // undocumented command
#define VCC4_SetWhiteBalance2                39
#define VCC4_SetWhiteBalance3                40
#define VCC4_SetWhiteBalance4                41
#define VCC4_GetWhiteBalance                42
#define VCC4_SetFading                                44
#define VCC4_WhiteFade                                45
#define VCC4_HiSpeedWhiteFade                46
#define VCC4_HiSpeedBlackFade                47
#define VCC4_CameraReset                        48
#define VCC4_GetZoomRatio                        49
#define VCC4_GetCCDSize                                50
#define VCC4_GetZoomMax                                53
#define VCC4_GetCameraVersion                57
#define VCC4_GetEEPROMVersion                58
#define VCC4_SetMotorSpeedPAN                59
#define VCC4_SetMotorSpeedTILT                60
#define VCC4_GetMotorSpeedPAN                61
#define VCC4_GetMotorSpeedTILT                62
#define VCC4_SetPanTilt1                        63
#define VCC4_SetPanTilt2                        64
#define VCC4_SetPanTilt3                        65
#define VCC4_SetPanTilt4                        66
#define VCC4_SetPanTilt5                        67
#define VCC4_GoHome                                        69
#define VCC4_PTinit1                                70
#define VCC4_PTinit2                                71
#define VCC4_GetPanMoter0                        72
#define VCC4_GetPanMoter1                        73
#define VCC4_GetTiltMotor0                        74
#define VCC4_GetTiltMotor1                        75
#define VCC4_GetPanGRatio                        76
#define VCC4_GetMotorSpeed                        77
#define VCC4_GetAngleRatio                        78
#define VCC4_GetAngleRatio2                        79
#define VCC4_GetAngleRatio0                        80
#define VCC4_GetAngleRatio1                        81
#define VCC4_PTStop                                        82
#define VCC4_TiltUp                                        83
#define VCC4_TiltDown                                84
#define VCC4_PanRigth                                85
#define VCC4_PanLeft                                86
#define VCC4_PanRightTiltUP                        87
#define VCC4_PanRightTiltDOWN                88
#define VCC4_PanLeftTiltUP                        89
#define VCC4_PanLeftTiltDOWN                90
#define VCC4_SetPTAngle2                        92
#define VCC4_GetPTAngle2                        93
#define VCC4_SetPTAngle0                        94
#define VCC4_SetPTAngle1                        95
#define VCC4_GetPTAngle0                        96
#define VCC4_GetPTAngle1                        97
#define VCC4_SerRCON                                102
#define VCC4_SetRCOFF                                103
#define VCC4_GetStatus                                104
#define VCC4_GetUnitName                        106
#define VCC4_GetRomVersion                        107
#define VCC4_SetPresetMemory                108
#define VCC4_MovePreset                                109
#define VCC4_GetPresetStatus                110
#define VCC4_RemoteCommandPass0                113
#define VCC4_RemoteCommandPassN                114
#define VCC4_SetCascadeON                        118
#define VCC4_SetCascadeOFF                        119
#define VCC4_EnterHostMode                        120
#define VCC4_EnterLocalMode                        121
#define VCC4_SetScreenOFF                        122
#define VCC4_SetScreenON                        123
#define VCC4_ScreenTitleOFF                        124
#define VCC4_ScreenTitleON                        125
#define VCC4_SetTimeMode0                        126
#define VCC4_SetTimeMode1                        127
#define VCC4_SetTimeMode2                        128
#define VCC4_SetDateMode0                        129
#define VCC4_SetDateMode1                        130
#define VCC4_SetDateMode2                        131
#define VCC4_SetTitleString                        132
#define VCC4_GetTitleString                        133
#define VCC4_SetDate                                134
#define VCC4_GetDate                                135
#define VCC4_SetTime                                136
#define VCC4_GetTime                                137
#define VCC4_GetCameraOnTime                138
#define VCC4_GetPedestalOnTime                139
#define VCC4_DefaultReset                        140
#define VCC4_GetExtMove                                147
#define VCC4_GetExtPreset                        148
#define VCC4_SpecifyShutterSpeed        149
#define VCC4_SetWhiteBalance0                150
#define VCC4_SetWhiteBalance1                151
#define VCC4_GetFocusRange                        152
#define VCC4_SetNotifyOFF                        155
#define VCC4_SetNotifyON                        156
#define VCC4_SetGlobalNotifyOFF                157
#define VCC4_SetGlobalNotifyON                158
#define VCC4_SetAELockOFF                        159
#define VCC4_SetAELockON                        160
#define VCC4_SetLedNormal                        164
#define VCC4_SetLedGreenOn                        165
#define VCC4_SetLedAllOff                        166
#define VCC4_SetLedRedOn                        167
#define VCC4_SetLedOrangeOn                        168
#define VCC4_GetPedestalModel                170
#define VCC4_GetCameraModel                        171

#define VCC4_CMDMAX 139

struct VCC4CMD {
  int cmdno;                      // command id number
  int cmdtyp;                     // command type:
                                  // 0 = no need to embed value
                                  // 1 = embed absolut number
                                  // 2 = embed angle. i.e. offset of 8000h
                                  // 3 = embed string, date or time
  int cmdlen;                     // command string length (FF to EF)
  int cmdparamlen;                // length of embeded parameter section
  int cmdparamstart;              // first byte position for paramaters
  char cmdstr[30];                // VCC4 command string
} ;

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
