Sonar Python & C Code
=====================

The Cheetah SPI device from Total Phase is used to sample three analog-to-
digital converters (ADCs) using the convert pins as slave selects.  The
sampling is done in a sequentially multiplexed fashion so that all three ADCs
are read an equal number of times.  Thus, the sampling frequency of the
Cheetah device must be three times the sampling frequency required to
satisfy the Nyquist criterion on each ADC respectively.


Files
=====


Makefile
--------

Custom Makefile for combined C & C++ usage


SonarDetect.C
-------------

The most current detection algorithm.  This program continuously samples 3
multiplexed ADCs using the TotalPhase Cheetah SPI device.  After sufficient
data is collected from the ADCs to perform a N=512 FFT, the spectral content
of the signals is analized to detect the presense of a sonar pinger peak.
The implementation does not provide visualization of the spectral data.

Usage:
  SonarDetect [device port] [bitrate] [target frequency]
  E.g. SonarDetect 0 10000 25000


VisualizeRealFFT.c
------------------

This shows a visualization of the real-half of the FFT spectral data obtained
from the ADC on slave-select 0 (SS0).  It shows the most recent 100 spectral
results.  The visualized data shows only the real-half [0, pi] of the FFT
output, excluding the first 10 low-frequency components.

Usage:
  VisualizeRealFFT [device port] [bitrate]
  E.g. VisualizeRealFFT 0 10000


CImg.h
------

This is the CImge library header file.  It provides everything required for
the C-based visualizations used by the Sonar code.


cheetah_py.py
-------------

This is the Python API for the Cheetah interface.  It is only required when
using other Python-based code in this folder.


read_adcs.py
------------

This script samples 3 multiplexed ADCs on the TotalPhase Cheetah SPI device.
The data is written as comma-separated data points to the file 'data.dat'.
Each column corresponds to SS0, SS1, and SS2 respectively. To visualize the
data, use the Matlab script 'plot_python_data_FFT.m'.

Usage: sudo python read_adcs.py 0 9000 2048 100

