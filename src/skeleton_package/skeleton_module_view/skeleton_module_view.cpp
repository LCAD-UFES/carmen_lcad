#include "skeleton_module_view.h"
#include "ui_skeleton_module_view.h"
#include "qpainter.h"

skeleton_module_view::skeleton_module_view(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::skeleton_module_view)
{
  ui->setupUi(this);
  x = 0;
  y = 0;
  pixmap = new QPixmap(256, 256);
  pixmap->fill(Qt::white);

  painter = new QPainter(pixmap);
}

skeleton_module_view::~skeleton_module_view()
{
  delete ui;
}

int skeleton_module_view::updateIPC(int i)
{
  carmen_ipc_sleep(0.01);
  carmen_graphics_update_ipc_callbacks_qt(this, SLOT(updateIPC(int)));
  i = 1; // only to make the compiler happy

  return i;
}

void skeleton_module_view::update_char_flow_image(char c)
{
  if ((x > 255) && (y > 255))
  {
    x = y = 0;
    pixmap->fill( Qt::white );
  }
  else if (x > 255)
  {
    x = 0; y++;
  }
  else
      x++;

  painter->setPen(QColor(c, x, y, 255));
  painter->drawPoint(x, y);

  ui->char_flow_image->setPixmap(*pixmap);
  ui->char_flow_image->repaint();
}
