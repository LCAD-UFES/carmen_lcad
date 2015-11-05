/********************************************************************************
** Form generated from reading ui file 'ui_VisionGUI.ui'
**
** Created: Sat Jan 30 22:39:28 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_UI_VISIONGUI_H
#define UI_UI_VISIONGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_VisionGUI
{
public:
    QAction *actionShapes_Recognized;
    QAction *actionObjects_Recognized;
    QAction *actionFreeze_Frame;
    QAction *actionSave_Frame_as_Image;
    QAction *actionExit;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QLabel *imageLabel;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuOverlay;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *VisionGUI)
    {
        if (VisionGUI->objectName().isEmpty())
            VisionGUI->setObjectName(QString::fromUtf8("VisionGUI"));
        VisionGUI->resize(800, 600);
        actionShapes_Recognized = new QAction(VisionGUI);
        actionShapes_Recognized->setObjectName(QString::fromUtf8("actionShapes_Recognized"));
        actionShapes_Recognized->setCheckable(true);
        actionShapes_Recognized->setChecked(true);
        actionObjects_Recognized = new QAction(VisionGUI);
        actionObjects_Recognized->setObjectName(QString::fromUtf8("actionObjects_Recognized"));
        actionObjects_Recognized->setCheckable(true);
        actionObjects_Recognized->setChecked(true);
        actionFreeze_Frame = new QAction(VisionGUI);
        actionFreeze_Frame->setObjectName(QString::fromUtf8("actionFreeze_Frame"));
        actionFreeze_Frame->setCheckable(true);
        actionSave_Frame_as_Image = new QAction(VisionGUI);
        actionSave_Frame_as_Image->setObjectName(QString::fromUtf8("actionSave_Frame_as_Image"));
        actionExit = new QAction(VisionGUI);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        centralwidget = new QWidget(VisionGUI);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        imageLabel = new QLabel(centralwidget);
        imageLabel->setObjectName(QString::fromUtf8("imageLabel"));
        imageLabel->setScaledContents(true);

        gridLayout->addWidget(imageLabel, 0, 0, 1, 1);

        VisionGUI->setCentralWidget(centralwidget);
        menubar = new QMenuBar(VisionGUI);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 25));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuOverlay = new QMenu(menubar);
        menuOverlay->setObjectName(QString::fromUtf8("menuOverlay"));
        VisionGUI->setMenuBar(menubar);
        statusbar = new QStatusBar(VisionGUI);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        VisionGUI->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuOverlay->menuAction());
        menuFile->addAction(actionFreeze_Frame);
        menuFile->addAction(actionSave_Frame_as_Image);
        menuFile->addAction(actionExit);
        menuOverlay->addAction(actionShapes_Recognized);
        menuOverlay->addAction(actionObjects_Recognized);

        retranslateUi(VisionGUI);

        QMetaObject::connectSlotsByName(VisionGUI);
    } // setupUi

    void retranslateUi(QMainWindow *VisionGUI)
    {
        VisionGUI->setWindowTitle(QApplication::translate("VisionGUI", "BeoHawk Vision System", 0, QApplication::UnicodeUTF8));
        actionShapes_Recognized->setText(QApplication::translate("VisionGUI", "Shapes Recognized", 0, QApplication::UnicodeUTF8));
        actionObjects_Recognized->setText(QApplication::translate("VisionGUI", "Objects Recognized", 0, QApplication::UnicodeUTF8));
        actionFreeze_Frame->setText(QApplication::translate("VisionGUI", "Freeze Frame", 0, QApplication::UnicodeUTF8));
        actionSave_Frame_as_Image->setText(QApplication::translate("VisionGUI", "Save Frame as Image", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("VisionGUI", "Exit", 0, QApplication::UnicodeUTF8));
        imageLabel->setText(QString());
        menuFile->setTitle(QApplication::translate("VisionGUI", "File", 0, QApplication::UnicodeUTF8));
        menuOverlay->setTitle(QApplication::translate("VisionGUI", "Overlay", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class VisionGUI: public Ui_VisionGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_UI_VISIONGUI_H
