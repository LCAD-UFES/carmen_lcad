{
/*!@file Robots/Beobot2/KVM/KVM.spin The KVM firmware */

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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL$
// $Id$
//
}
{
USB Chip Mapping

USB_0 = T3 , C2C1C0 = 000
USB_1 = B2 , C2C1C0 = 001
USB_2 = T4 , C2C1C0 = 010
USB_3 = B1 , C2C1C0 = 011
USB_4 = T1 , C2C1C0 = 100
USB_5 = B4 , C2C1C0 = 101
USB_6 = T2 , C2C1C0 = 110
USB_7 = B3 , C2C1C0 = 111

VGA Chip Mapping

VGA_1 = T3
VGA_2 = T4
VGA_3 = B4
VGA_4 = T2
VGA_5 = B2
VGA_6 = B1
VGA_7 = T1
VGA_8 = B3

7-Segment Display Mapping

T1 = 1
T2 = 2
T3 = 3
T4 = 4
B1 = 5
B2 = 6
B3 = 7
B4 = 8


+----+---------+-----+------------+---------+-----+------------+
|COM | Display | VGA | VGA_C3..C0 | VGA_Sel | USB | USB_C2..C0 |
| T1 |    1    |  7  |   1011     |    1    |  4  |    100     |
| T2 |    2    |  4  |   0111     |    0    |  6  |    110     |
| T3 |    3    |  1  |   1110     |    0    |  0  |    000     |
| T4 |    4    |  2  |   1101     |    0    |  2  |    010     |
| B1 |    5    |  6  |   1101     |    1    |  3  |    011     |
| B2 |    6    |  5  |   1110     |    1    |  1  |    001     |
| B3 |    7    |  8  |   0111     |    1    |  7  |    111     |
| B4 |    8    |  3  |   1011     |    0    |  5  |    101     |
+----+---------+-----+------------+---------+-----+------------+
}
CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
  '7-Seg Display will read BCD code to display 0~9,DP is direct drive 
  DISP_DA = 8
  DISP_DB = 9
  DISP_DC = 10
  DISP_DD = 11
  DISP_DP = 12   

  'Navi button UP/Down/Left/Right/Middle
  BTN_A = 0  
  BTN_B = 1
  BTN_C = 2
  BTN_D = 3
  BTN_E = 4 'Middle Click

  'Five MAX4889 VGA 1:2 Mux,
  'Four VGA in mux share same select line(VGA_IN_SEL)
  'with 4 Enable Pins(C0..C3) to choose VGA source from 8 computers 
  
  'One mux for select(VGA_OUT_SEL) which output VGA device(Touch Screen/External LCD),
  'This mux always Enable
  
  VGA_OUT_SEL = 19
  VGA_IN_SEL  = 21
  VGA_EN_C0   = 22
  VGA_EN_C1   = 23
  VGA_EN_C2   = 24
  VGA_EN_C3   = 25

  USB_EN_C0 = 16
  USB_EN_C1 = 17
  USB_EN_C2 = 18

VAR
  byte USB_table[9]' :=  0,4,6,0,2,3,1,7,5
  byte VGA_table[9]'     0,7,4,1,2,6,5,8,3  
OBJ
  Serial : "FullDuplexSerial"
  SevenSeg : "SevenSegmentFader"
     
PUB Main | KVM_Select, EXT_Select
  KVM_Select := 1 'T1  
  EXT_Select := 1 'External LCD
  
  USB_table[1] := 4
  USB_table[2] := 6
  USB_table[3] := 0    
  USB_table[4] := 2
  USB_table[5] := 3
  USB_table[6] := 1    
  USB_table[7] := 7
  USB_table[8] := 5
    
  VGA_table[1] := 7
  VGA_table[2] := 4
  VGA_table[3] := 1
  VGA_table[4] := 2
  VGA_table[5] := 6
  VGA_table[6] := 5
  VGA_table[7] := 8
  VGA_table[8] := 3
                 
  
  Serial.start(31,30,0,9600)
  SevenSeg.Start(13, 3_000_000, 50)

  'Setup the Button pins as inputs
  dira[BTN_A .. BTN_E] := 0
  'Setup the dot as an output
  dira[DISP_DP] := 1
  'Setup all of the VGA Select pins as outputs, and set them all as disabled (active low)
  dira[VGA_EN_C0..VGA_EN_C3] := $ff
  outa[VGA_EN_C0..VGA_EN_C3]   := $ff
  dira[USB_EN_C0..USB_EN_C2]   := $ff  
  outa[USB_EN_C0..USB_EN_C2]   := 0
  dira[VGA_IN_SEL]              :=1
  dira[VGA_OUT_SEL]             :=1

  repeat
    if KVM_Select > 8
      KVM_Select := 1
    if KVM_Select < 1
      KVM_SELECT := 8

    'Set the seven segment display
    SevenSeg.SetVal(KVM_Select)
    outa[DISP_DP] := EXT_SELECT' When 7-Seg Display showing DOT,it's external LCD
    outa[VGA_OUT_SEL]  := EXT_SELECT 'Switch between touch screen or external LCD     
    SetVgaVal(VGA_table[KVM_Select])
    SetUsbVal(USB_table[KVM_Select])

    if ina[BTN_A] == 0
      KVM_Select := KVM_Select+1 
    if ina[BTN_B] == 0
      KVM_Select := KVM_Select+1
    if ina[BTN_C] == 0
      KVM_Select := KVM_Select-1
    if ina[BTN_D] == 0
      KVM_Select := KVM_Select-1
    if ina[BTN_E] == 0
      !EXT_SELECT

    waitcnt(clkfreq/5 + cnt)

PUB SetVgaVal(vga_val)|VGA_EN_PINS, VGA_SEL_PIN

    'Set the VGA Select pins
     VGA_EN_PINS := !(|< ((vga_val-1)//4))
     VGA_SEL_PIN := ((vga_val) < 5)+1
     outa[VGA_EN_C3..VGA_EN_C0] := VGA_EN_PINS
     outa[VGA_IN_SEL]           := VGA_SEL_PIN                  
     Serial.str(string("C3.C2.C1.C0: "))
    ' Serial.bin(VGA_EN_PINS, 4)
     Serial.bin(ina[VGA_EN_C3..VGA_EN_C0],4)
     Serial.str(string(" S: "))
     Serial.dec(ina[VGA_IN_SEL])
     Serial.str(string("  Out1/Out2: "))
     Serial.dec(ina[VGA_OUT_SEL])
     Serial.str(string(10,13))

PUB SetUsbVal(usb_val)|USB_EN_PINS
    USB_EN_PINS := usb_val 
    outa[USB_EN_C2..USB_EN_C0] := USB_EN_PINS
    Serial.str(string("  USB: "))    
    Serial.bin(USB_EN_PINS, 8)
    Serial.str(string(10,13))    
             