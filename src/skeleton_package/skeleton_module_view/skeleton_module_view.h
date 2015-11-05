#ifndef SKELETON_MODULE_VIEW_H
#define SKELETON_MODULE_VIEW_H

#include <QMainWindow>
#include <carmen/carmen.h>
#include <carmen/global_graphics_qt.h>
#include <carmen/skeleton_module_filter_interface.h>
#include <carmen/skeleton_module_filter_messages.h>

namespace Ui {
    class skeleton_module_view;
}

class skeleton_module_view : public QMainWindow
{
    Q_OBJECT

public:
    explicit skeleton_module_view(QWidget *parent = 0);
    ~skeleton_module_view();
    void update_char_flow_image(char c);
    QPixmap *pixmap;
    QPainter *painter;
    int x, y;

private:
    Ui::skeleton_module_view *ui;

public slots:
   int updateIPC(int i);

};

#endif // SKELETON_MODULE_VIEW_H
