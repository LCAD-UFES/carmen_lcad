 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <qobject.h>
#include <qapplication.h>
#include <qlayout.h>
#include <qslider.h>
#include <qpainter.h>
#include <qframe.h>
#include <qwidget.h>
#include <qpainter.h>
#include <qvbox.h>
#include <qlabel.h>
#include <qbuttongroup.h>
#include <qpushbutton.h>
#include <qstring.h>

#define MODE_START        0
#define MODE_REVPLAY      1
#define MODE_STOP         2
#define MODE_PLAY         3
#define MODE_END          4

class QLaserDisplay : public QWidget {
  Q_OBJECT
  
public:
  QLaserDisplay( QWidget *parent = 0, const char *name = 0 );
  int         mode;
  int         wait;
  int         recpos;
  QSlider   * slider;
  
private:
  int             use_grid;
  double          scale;
  int             dontrepaint;
  int             recreate;
  QPushButton   * gpb;
  QPixmap       * qpixmap;
  QFrame        * drawbox;
  QPainter      * qpainter;
  
protected:
  void closeEvent( QCloseEvent *ev );
  void paintEvent( QPaintEvent *ev );
  void resizeEvent( QResizeEvent * );
  void drawDynamicBoxes( void );
  
protected slots:
  void slotChangeValue( int value );
  void slotJumpToStart();
  void slotRevPlay();
  void slotStop();
  void slotPlay();
  void slotGrid();
  void slotJumpToEnd();
  void slotSpeedFaster();
  void slotSpeedSlower();
  void slotZoomOut();
  void slotZoomIn();
  void slotStepPrev();
  void slotStepNext();

};
