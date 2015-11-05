Hey, so here's how I've been doing GUI stuffs:
1) Outline it in QtDesigner/QtCreater and save it as ui_X.ui (make sure to change the name of 
the form to X as uic is going to use this).
2) perform uic ui_X.ui > ui_X.qt.H
3) write a class (X.qt.C/X.qt.H) which inherits QMainWindow or QWidget and has a private Ui::X object.