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

#include <qapplication.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include <carmen/logtools.h>

#ifdef __cplusplus
}
#endif

#include "display-laserdata.h"


#define MAX_STRING_LENGTH       256
#define MAX_NUM_LASER_VALUES    801

logtools_log_data_t rec;
int                 LASER_ID      = 0;
  

QLaserDisplay::QLaserDisplay( QWidget *parent, const char *name )
  : QWidget( parent, name )
{
  QVBoxLayout   * vbox;
  QHBoxLayout   * hbox;
  QPushButton   * pb;

  use_grid  = FALSE;
  recreate  = TRUE;
  
  setCaption( "QLaserDisplay" );

  vbox = new QVBoxLayout( this, 1);

  QFrame *buttons = new QFrame(this);
  buttons->setMaximumHeight(30);

  hbox = new QHBoxLayout(buttons, 1);
    
  pb = new QPushButton( "START ", buttons, "start" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotJumpToStart()) );
  
  pb = new QPushButton( "R-PLAY", buttons, "revplay" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotRevPlay()) );
  
  pb = new QPushButton( " STOP ", buttons, "stop" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotStop()) );
  
  pb = new QPushButton( " PLAY ", buttons, "play" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotPlay()) );
    
  pb = new QPushButton( " END  ", buttons, "end" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotJumpToEnd()) );
    
  gpb = new QPushButton( " GRID ", buttons, "grid" );
  hbox->addWidget( gpb );
  connect( gpb, SIGNAL(clicked()), SLOT(slotGrid()) );
  gpb->setToggleButton( TRUE );

  pb = new QPushButton( "SPEED -", buttons, "speed-slower" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotSpeedSlower()) );

  pb = new QPushButton( "SPEED +", buttons, "speed-faster" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotSpeedFaster()) );

  pb = new QPushButton( "ZOOM -", buttons, "zoom-out" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotZoomOut()) );

  pb = new QPushButton( "ZOOM +", buttons, "zoom-in" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotZoomIn()) );

  pb = new QPushButton( "STEP -", buttons, "step-prev" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotStepPrev()) );

  pb = new QPushButton( "STEP +", buttons, "step-next" );
  hbox->addWidget( pb );
  connect( pb, SIGNAL(clicked()), SLOT(slotStepNext()) );

  vbox->addWidget(buttons);

  drawbox = new QFrame( this );
  qpainter = new QPainter( drawbox );
  vbox->addWidget( drawbox );

  slider = new QSlider( 1, 100, 1, 1, Qt::Horizontal, this );
  vbox->addWidget(slider);

  connect( slider, SIGNAL( valueChanged( int ) ),
	   this, SLOT( slotChangeValue( int ) ) );

  mode         = MODE_STOP;
  recpos       = 0;
  scale        = 5.0;
  dontrepaint  = 0;
  wait         = 4;

  setBackgroundMode (NoBackground);
  resize( 700, 700 );
}
 
void
QLaserDisplay::resizeEvent (QResizeEvent *) {
  recreate = TRUE;
}
 
QString
createTimeString( struct timeval time, int num )
{
  char str[256];
  float tsec;
  double sec;
  struct tm *actual_date;
  sec = (double) (time.tv_sec+(double) time.tv_usec/1000000.0);
  actual_date = localtime( &time.tv_sec );
  tsec =  (float) (actual_date->tm_sec+time.tv_usec/1000000.0);
  snprintf( str, 256, "[%s%d-%s%d-%d %s%d:%s%d:%s%.2f (%f)] scan: %d",
	    actual_date->tm_mday<10?"0":"",
	    actual_date->tm_mday,
	    actual_date->tm_mon+1<10?"0":"",
	    actual_date->tm_mon+1,
	    1900+actual_date->tm_year,
	    actual_date->tm_hour<10?"0":"",
	    actual_date->tm_hour,
	    actual_date->tm_min<10?"0":"",
	    actual_date->tm_min,
	    tsec<10?"0":"",
	    tsec, sec, num );
  QString timestr = str;
  return(timestr);
}

QString
createPosString( logtools_rpos2_t pos )
{
  char chrstr[256];
  snprintf( chrstr, 256, "[pos=%.2f,%.2f,%.2f]",
	    pos.x, pos.y, rad2deg(pos.o) );
  QString str = chrstr;
  return(str);
}

QString
createPeopleString( int numpeople )
{
  char str[256];
  snprintf( str, 256, "num people = %d", numpeople );
  QString pstr = str;
  return(pstr);
}

QPoint
absPos2screenPos( double x, double y, logtools_rpos2_t rpos,
		  double sx, double sy, double scale )
{
  logtools_vector2_t v1;
  v1.x = x - rpos.x;
  v1.y = y - rpos.y;
  return( QPoint( (int) (sx + ( v1.x * cos(-M_PI_2+rpos.o) +
				v1.y * sin(-M_PI_2+rpos.o) )/ scale ),
		  (int) (sy + ( v1.x * sin(-M_PI_2+rpos.o) -
				v1.y * cos(-M_PI_2+rpos.o) )/scale )) );
}

#define      MIN_PIXEL_DIST                 50
#define      ADD_SIZE                       40.0

double
stretch( double angle, double val )
{
  double   b = 5.0;
  return( 0.04 * val * b / (M_PI*(b*b+pow((angle-M_PI_4),2)) ) );
}
  
void
QLaserDisplay::paintEvent( QPaintEvent *ev  )
{
  int i, ix, iy, sx, sy, s, s_2, grid_size, max_size;
  double val, angle;
  char str[10];
  QPointArray pa(4);
  QPen        pen;

  if (recreate) {
    qpainter->end();
    qpixmap = new QPixmap (width(), height());
    qpainter->begin (qpixmap, this);
    recreate = FALSE;
  }
  
  ev = NULL;
  
  if (rec.lsens[recpos].id==LASER_ID) {
    sx = (int) (width()/2.0);
    sy = height()-100;
    
    if (sx>sy/2)
      max_size = (int) (2* sx * scale);
    else
      max_size = (int) (sy * scale);
    
    qpainter->eraseRect( 0, 0, width(), height() );
   
    qpainter->drawText( 5, 20, createTimeString( rec.lsens[recpos].laser.time,
						recpos ));
    
    qpainter->drawText( 5, 40, createPosString( rec.lsens[recpos].estpos ));
    
    if (use_grid) {
      qpainter->setPen(gray);
      if (100.0/scale > MIN_PIXEL_DIST ) 
	grid_size = 100; 
      else if (200.0/scale > MIN_PIXEL_DIST ) 
	grid_size = 200; 
      else if (500.0/scale > MIN_PIXEL_DIST ) 
	grid_size = 500; 
      else
	grid_size = 1000; 
      
      for (i = grid_size;
	   i<= (int) (max_size);
	   i = i+grid_size ) {
	s    = (int) (2*i/scale);
	s_2  = (int) (s / 2.0);
	qpainter->drawArc( sx-s_2, sy-s_2, s, s, 0, 180*16 );
	sprintf( str, "%dm", (int) (i/100.0) );
	qpainter->drawText( sx-s_2-5, sy+15, str );
	  qpainter->drawText( sx-s_2+s-5, sy+15, str );
	}
      qpainter->drawLine( sx-10, sy, sx+10, sy );
      qpainter->drawLine( sx, sy-10, sx, sy+10 );
      pen.setStyle(Qt::DotLine);
      pen.setColor(gray);
      qpainter->setPen(pen);
      for (i = 0;
	   i<= (int) (max_size);
	   i = i+(grid_size/2) ) {
	s_2  = (int) (i/scale);
	qpainter->drawLine( 0, sy-s_2, 2*sx, sy-s_2 );
      }
      for (i = 0;
	   i<= (int) (max_size/2.0);
	   i = i+(grid_size/2) ) {
	s_2  = (int) (i/scale);
	qpainter->drawLine( sx-s_2, 0, sx-s_2, sy );
	qpainter->drawLine( sx+s_2, 0, sx+s_2, sy );
      }
      
    }
    
    qpainter->setPen(black);
    qpainter->setBrush(black);
      
    for (i=0; i<rec.lsens[recpos].laser.numvalues; i++) {
      val   = rec.lsens[recpos].laser.val[i] / scale;
      angle = rec.lsens[recpos].laser.angle[i];
      val += stretch( angle, val );
      ix  = (int) (sx-(val*sin(angle))); 
      iy  = (int) (sy-(val*cos(angle)));
      qpainter->drawEllipse(ix-1,iy-1,3,3);
    }
    //      slider->setValue(recpos+1);
  }
  bitBlt( drawbox, 0, 0, qpixmap );
  dontrepaint = true;
  slider->setValue(recpos);
}

void
QLaserDisplay::slotChangeValue( int value )
{
  if (!dontrepaint) {
    recpos = value;
    repaint();
  } else {
    dontrepaint = 0;
  }
}


void
QLaserDisplay::closeEvent( QCloseEvent *ev )
{
  ev = NULL;
  exit(0);
}

/***********************************************************/
/*  BUTTONS */
/***********************************************************/

void
QLaserDisplay::slotJumpToStart()
{
  recpos = 0;
  repaint();
  mode = MODE_STOP;
}

void
QLaserDisplay::slotRevPlay()
{
  mode = MODE_REVPLAY;
}

void
QLaserDisplay::slotStop()
{
  mode = MODE_STOP;
}

void
QLaserDisplay::slotPlay()
{
  mode = MODE_PLAY;
}

void
QLaserDisplay::slotJumpToEnd()
{
  recpos = rec.numlaserscans-1;
  repaint();
  mode = MODE_STOP;
}

void
QLaserDisplay::slotGrid()
{
  use_grid = gpb->isOn();
  repaint();
}

void
QLaserDisplay::slotSpeedSlower()
{
  if (wait<1000) {
    wait+=5;
  }
}

void
QLaserDisplay::slotSpeedFaster()
{
  if (wait>1) {
    wait--;
  }
}

void
QLaserDisplay::slotZoomIn()
{
  if (scale>1.0) {
    scale = scale-1.0;
  }  else if (scale>0.1) {
    scale = scale-0.1;
  }
  repaint();

}

void
QLaserDisplay::slotZoomOut()
{
  if (scale<30.0) { 
    if (scale>=1.0) {
      scale = scale+1.0;
    }  else {
      scale = scale+0.1;
    }
    repaint();
  }
}

void
QLaserDisplay::slotStepPrev()
{
  if (recpos>0) {
    recpos--;
    repaint();
  }
}

void
QLaserDisplay::slotStepNext()
{
  if (recpos<rec.numlaserscans-1) {
    recpos++;
    repaint();
  }
}

/***********************************************************/
/*  MAIN */
/***********************************************************/


void
print_usage( void )
{
  fprintf( stderr, "usage: display-laserdata [-num LASER_ID] <FILE>\n");
}

void
check_mode( QLaserDisplay *w )
{
  switch(w->mode) {
  case MODE_START:
    w->recpos = 0;
    w->mode   = MODE_STOP;
    w->repaint();
    break;
  case MODE_PLAY:
    if (w->recpos<rec.numlaserscans-1)
      (w->recpos)++;
    else
      w->mode = MODE_STOP;
    w->repaint();
    break;
  case MODE_REVPLAY:
    if (w->recpos>0)
      (w->recpos)--;
    else
      w->mode = MODE_STOP;
    w->repaint();
    break;
  case MODE_END:
    w->recpos = rec.numlaserscans-1;
    w->mode   = MODE_STOP;
    w->repaint();
    break;
  }
}
  
int
main( int argc, char *argv[] )
{
  QApplication         app( argc, argv );
  QLaserDisplay        window;
  int                  i, cnt=1;
  
  for (i=1; i<argc-1; i++) {
    if (!strcmp(argv[i],"-num") && (argc>i+1)) {
      LASER_ID = atoi( argv[++i] );
    } else {
      print_usage();
      exit(1);
    }
  }
  
  if (!logtools_read_logfile( &rec, argv[argc-1] ))
    exit(0);

  while (rec.lsens[window.recpos].id!=LASER_ID) {
    window.recpos++;
    if (window.recpos>=rec.numlaserscans) {
      fprintf( stderr, "# ERROR: no laser scans with id %d !!!\n",
	       LASER_ID );
      exit(0);
    }
  }

  if (rec.numlaserscans>1)
    window.slider->setMaxValue(rec.numlaserscans-2);
  else
    window.slider->setMaxValue(1);
  
  window.show();

  while(1) {
    if (cnt%window.wait==0) {
      check_mode( &window );
      cnt=1;
    }
    cnt++;
    app.processEvents();
    usleep(1);
  }
  
  return(0);
}

