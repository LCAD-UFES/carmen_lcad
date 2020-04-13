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

#include <q3frame.h>
#include <q3groupbox.h>
#include <qpushbutton.h>
#include <q3buttongroup.h>
#include <qlayout.h>
#include <qlabel.h>
#include <qcursor.h>
#include <qwidget.h>
#include <q3popupmenu.h>
#include <q3multilineedit.h>
#include <q3textview.h>
#include <qfont.h>
//Added by qt3to4:
#include <QCloseEvent>
#include <Q3HBoxLayout>

#define MAX_NUM_GROUPS   20
#define MAX_NUM_MODULES  40
#define NUM_STATES       4
#define MAX_NAME_LENGTH  256

typedef struct {
  char   group_name[MAX_NAME_LENGTH];
  int    group;
  char   module_name[MAX_NAME_LENGTH];
  int    module;
  int    active;
  int    requested_state;
  int    pid;
  int    output;
} process_type;

typedef struct {
  int           numprocesses;
  int           numgrps;
  int           procingrp[MAX_NUM_GROUPS];
  process_type  process[MAX_NUM_GROUPS][MAX_NUM_MODULES];
  int           output;
} process_table_type;


void  carmen_output( int state );


class QDisplay : public QWidget {
  Q_OBJECT

public:
  QDisplay( QWidget *parent = 0, const char *name = 0 );
  void showStatus( int group, int button, int status );
  void showStatus2( int group, int button, int status, char* module_name );
  void setGroup( int group, char * group_name );
  void setModule( int group, int module, char * module_name, int pid );
  void hideButton( int group, int module );
  void showLine( char * module_name, int pid, char * line );

private:
  QPushButton  *but[MAX_NUM_GROUPS][MAX_NUM_MODULES];
  Q3HBoxLayout  *box[MAX_NUM_GROUPS];
  Q3ButtonGroup *bgrp[MAX_NUM_GROUPS];
  Q3TextView *output;

private slots:
  void startClicked( int );
  void stopClicked( int );
  void showClicked( int );
  void noClicked( int );


protected:
  void closeEvent( QCloseEvent *ev );

protected slots:
};


