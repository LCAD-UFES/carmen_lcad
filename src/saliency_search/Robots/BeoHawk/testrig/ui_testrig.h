/********************************************************************************
** Form generated from reading ui file 'ui_testrig.ui'
**
** Created: Fri Mar 5 12:01:09 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_TESTRIG_H
#define UI_TESTRIG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDial>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QWidget>
#include <qwt/qwt_plot.h>

QT_BEGIN_NAMESPACE

class Ui_testrig
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QFrame *line;
    QGroupBox *groupBox_3;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *label_18;
    QLabel *imu_up_per_sec;
    QLabel *cam_up_per_sec;
    QLabel *pitch_gyro_sd;
    QLabel *pitch_accel_sd;
    QLabel *roll_gyro_sd;
    QLabel *roll_accel_sd;
    QFrame *line_2;
    QTabWidget *tabWidget;
    QWidget *plotTab;
    QGridLayout *gridLayout_4;
    QGridLayout *gridLayout_3;
    QwtPlot *pitch_cmp;
    QwtPlot *roll_cmp;
    QwtPlot *pitch_raw;
    QwtPlot *roll_raw;
    QSpacerItem *horizontalSpacer;
    QSpacerItem *verticalSpacer;
    QPushButton *constRollAxis;
    QPushButton *constPitchAxis;
    QWidget *fftTab;
    QGridLayout *gridLayout_6;
    QComboBox *fft_select;
    QwtPlot *fft_plot;
    QLabel *label_24;
    QWidget *pidTab;
    QLabel *label_33;
    QComboBox *pidLoop;
    QLineEdit *pidKp;
    QLineEdit *pidKi;
    QLineEdit *pidKd;
    QLabel *label_34;
    QLabel *label_35;
    QLabel *label_36;
    QPushButton *pidUpdate;
    QWidget *controlTab;
    QDial *controlYaw;
    QSlider *controlVelocity;
    QLabel *label_19;
    QLabel *label_20;
    QLabel *label_21;
    QLabel *label_22;
    QLabel *label_23;
    QWidget *camTab;
    QGridLayout *gridLayout_5;
    QLabel *cameraLabel;
    QSpacerItem *horizontalSpacer_2;
    QSpacerItem *horizontalSpacer_3;
    QFrame *frame;
    QCheckBox *centerLines;
    QCheckBox *contours;
    QSpacerItem *verticalSpacer_2;
    QSpacerItem *verticalSpacer_3;
    QGroupBox *groupBox;
    QDoubleSpinBox *pitch_gyro_var;
    QLabel *label;
    QDoubleSpinBox *pitch_bias_var;
    QLabel *label_2;
    QDoubleSpinBox *pitch_accel_var;
    QLabel *label_3;
    QPushButton *reset_pitch;
    QPushButton *reset_roll;
    QPushButton *pauseKalmanUpdates;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *pitch_kalman_0;
    QLabel *label_27;
    QLabel *label_28;
    QLabel *label_29;
    QLabel *pitch_p_00;
    QLabel *pitch_p_10;
    QLabel *pitch_p_01;
    QLabel *pitch_p_11;
    QLabel *pitch_kalman_1;
    QLabel *roll_kalman_1;
    QLabel *label_7;
    QLabel *roll_p_01;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_30;
    QLabel *label_10;
    QLabel *roll_p_11;
    QDoubleSpinBox *roll_bias_var;
    QDoubleSpinBox *roll_gyro_var;
    QLabel *label_11;
    QDoubleSpinBox *roll_accel_var;
    QLabel *label_12;
    QLabel *roll_kalman_0;
    QLabel *roll_p_10;
    QLabel *label_31;
    QLabel *label_32;
    QLabel *roll_p_00;
    QFrame *line_3;
    QSpinBox *pitchSumCount;
    QSpinBox *rollSumCount;
    QLabel *label_25;
    QLabel *label_26;

    void setupUi(QMainWindow *testrig)
    {
        if (testrig->objectName().isEmpty())
            testrig->setObjectName(QString::fromUtf8("testrig"));
        testrig->resize(1060, 788);
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Base, brush);
        QBrush brush1(QColor(229, 221, 213, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        testrig->setPalette(palette);
        centralwidget = new QWidget(testrig);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        QPalette palette1;
        QBrush brush2(QColor(254, 0, 255, 255));
        brush2.setStyle(Qt::SolidPattern);
        palette1.setBrush(QPalette::Active, QPalette::LinkVisited, brush2);
        palette1.setBrush(QPalette::Inactive, QPalette::LinkVisited, brush2);
        palette1.setBrush(QPalette::Disabled, QPalette::LinkVisited, brush2);
        centralwidget->setPalette(palette1);
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShadow(QFrame::Sunken);
        line->setLineWidth(2);
        line->setMidLineWidth(0);
        line->setFrameShape(QFrame::VLine);

        gridLayout->addWidget(line, 0, 1, 4, 1);

        groupBox_3 = new QGroupBox(centralwidget);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_3->sizePolicy().hasHeightForWidth());
        groupBox_3->setSizePolicy(sizePolicy);
        groupBox_3->setMinimumSize(QSize(0, 125));
        groupBox_3->setMaximumSize(QSize(16777215, 125));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        groupBox_3->setFont(font);
        label_13 = new QLabel(groupBox_3);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(10, 30, 171, 21));
        QFont font1;
        font1.setBold(false);
        font1.setWeight(50);
        label_13->setFont(font1);
        label_14 = new QLabel(groupBox_3);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(320, 30, 171, 21));
        label_14->setFont(font1);
        label_15 = new QLabel(groupBox_3);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(10, 70, 201, 21));
        label_15->setFont(font1);
        label_16 = new QLabel(groupBox_3);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(10, 90, 211, 21));
        label_16->setFont(font1);
        label_17 = new QLabel(groupBox_3);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(320, 70, 201, 21));
        label_17->setFont(font1);
        label_18 = new QLabel(groupBox_3);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(320, 90, 201, 21));
        label_18->setFont(font1);
        imu_up_per_sec = new QLabel(groupBox_3);
        imu_up_per_sec->setObjectName(QString::fromUtf8("imu_up_per_sec"));
        imu_up_per_sec->setGeometry(QRect(230, 30, 62, 21));
        imu_up_per_sec->setFont(font1);
        cam_up_per_sec = new QLabel(groupBox_3);
        cam_up_per_sec->setObjectName(QString::fromUtf8("cam_up_per_sec"));
        cam_up_per_sec->setGeometry(QRect(540, 30, 62, 21));
        cam_up_per_sec->setFont(font1);
        pitch_gyro_sd = new QLabel(groupBox_3);
        pitch_gyro_sd->setObjectName(QString::fromUtf8("pitch_gyro_sd"));
        pitch_gyro_sd->setGeometry(QRect(230, 70, 62, 21));
        pitch_gyro_sd->setFont(font1);
        pitch_accel_sd = new QLabel(groupBox_3);
        pitch_accel_sd->setObjectName(QString::fromUtf8("pitch_accel_sd"));
        pitch_accel_sd->setGeometry(QRect(230, 90, 62, 21));
        pitch_accel_sd->setFont(font1);
        roll_gyro_sd = new QLabel(groupBox_3);
        roll_gyro_sd->setObjectName(QString::fromUtf8("roll_gyro_sd"));
        roll_gyro_sd->setGeometry(QRect(540, 70, 61, 21));
        roll_gyro_sd->setFont(font1);
        roll_accel_sd = new QLabel(groupBox_3);
        roll_accel_sd->setObjectName(QString::fromUtf8("roll_accel_sd"));
        roll_accel_sd->setGeometry(QRect(540, 90, 62, 21));
        roll_accel_sd->setFont(font1);

        gridLayout->addWidget(groupBox_3, 3, 2, 1, 1);

        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShadow(QFrame::Sunken);
        line_2->setLineWidth(2);
        line_2->setFrameShape(QFrame::HLine);

        gridLayout->addWidget(line_2, 2, 2, 1, 1);

        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QPalette palette2;
        QBrush brush3(QColor(222, 214, 207, 255));
        brush3.setStyle(Qt::SolidPattern);
        palette2.setBrush(QPalette::Active, QPalette::Button, brush3);
        palette2.setBrush(QPalette::Active, QPalette::Base, brush);
        palette2.setBrush(QPalette::Active, QPalette::Window, brush3);
        palette2.setBrush(QPalette::Inactive, QPalette::Button, brush3);
        palette2.setBrush(QPalette::Inactive, QPalette::Base, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::Window, brush3);
        palette2.setBrush(QPalette::Disabled, QPalette::Button, brush3);
        palette2.setBrush(QPalette::Disabled, QPalette::Base, brush3);
        palette2.setBrush(QPalette::Disabled, QPalette::Window, brush3);
        tabWidget->setPalette(palette2);
        tabWidget->setAutoFillBackground(false);
        plotTab = new QWidget();
        plotTab->setObjectName(QString::fromUtf8("plotTab"));
        gridLayout_4 = new QGridLayout(plotTab);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        pitch_cmp = new QwtPlot(plotTab);
        pitch_cmp->setObjectName(QString::fromUtf8("pitch_cmp"));
        pitch_cmp->setFrameShape(QFrame::NoFrame);

        gridLayout_3->addWidget(pitch_cmp, 0, 0, 1, 1);

        roll_cmp = new QwtPlot(plotTab);
        roll_cmp->setObjectName(QString::fromUtf8("roll_cmp"));

        gridLayout_3->addWidget(roll_cmp, 0, 2, 1, 1);

        pitch_raw = new QwtPlot(plotTab);
        pitch_raw->setObjectName(QString::fromUtf8("pitch_raw"));

        gridLayout_3->addWidget(pitch_raw, 2, 0, 1, 1);

        roll_raw = new QwtPlot(plotTab);
        roll_raw->setObjectName(QString::fromUtf8("roll_raw"));

        gridLayout_3->addWidget(roll_raw, 2, 2, 1, 1);

        horizontalSpacer = new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout_3->addItem(horizontalSpacer, 0, 1, 3, 1);

        verticalSpacer = new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Fixed);

        gridLayout_3->addItem(verticalSpacer, 1, 0, 1, 1);


        gridLayout_4->addLayout(gridLayout_3, 1, 0, 1, 2);

        constRollAxis = new QPushButton(plotTab);
        constRollAxis->setObjectName(QString::fromUtf8("constRollAxis"));
        constRollAxis->setCheckable(true);
        constRollAxis->setChecked(true);

        gridLayout_4->addWidget(constRollAxis, 0, 1, 1, 1);

        constPitchAxis = new QPushButton(plotTab);
        constPitchAxis->setObjectName(QString::fromUtf8("constPitchAxis"));
        constPitchAxis->setCheckable(true);
        constPitchAxis->setChecked(true);

        gridLayout_4->addWidget(constPitchAxis, 0, 0, 1, 1);

        tabWidget->addTab(plotTab, QString());
        fftTab = new QWidget();
        fftTab->setObjectName(QString::fromUtf8("fftTab"));
        gridLayout_6 = new QGridLayout(fftTab);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        fft_select = new QComboBox(fftTab);
        fft_select->setObjectName(QString::fromUtf8("fft_select"));

        gridLayout_6->addWidget(fft_select, 0, 1, 1, 1);

        fft_plot = new QwtPlot(fftTab);
        fft_plot->setObjectName(QString::fromUtf8("fft_plot"));
        fft_plot->setFrameShape(QFrame::NoFrame);

        gridLayout_6->addWidget(fft_plot, 1, 0, 1, 2);

        label_24 = new QLabel(fftTab);
        label_24->setObjectName(QString::fromUtf8("label_24"));
        label_24->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_6->addWidget(label_24, 0, 0, 1, 1);

        tabWidget->addTab(fftTab, QString());
        pidTab = new QWidget();
        pidTab->setObjectName(QString::fromUtf8("pidTab"));
        label_33 = new QLabel(pidTab);
        label_33->setObjectName(QString::fromUtf8("label_33"));
        label_33->setGeometry(QRect(5, 10, 399, 27));
        label_33->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        pidLoop = new QComboBox(pidTab);
        pidLoop->setObjectName(QString::fromUtf8("pidLoop"));
        pidLoop->setGeometry(QRect(410, 10, 398, 27));
        pidKp = new QLineEdit(pidTab);
        pidKp->setObjectName(QString::fromUtf8("pidKp"));
        pidKp->setGeometry(QRect(460, 100, 161, 26));
        pidKi = new QLineEdit(pidTab);
        pidKi->setObjectName(QString::fromUtf8("pidKi"));
        pidKi->setGeometry(QRect(460, 160, 161, 26));
        pidKd = new QLineEdit(pidTab);
        pidKd->setObjectName(QString::fromUtf8("pidKd"));
        pidKd->setGeometry(QRect(460, 220, 161, 26));
        label_34 = new QLabel(pidTab);
        label_34->setObjectName(QString::fromUtf8("label_34"));
        label_34->setGeometry(QRect(50, 100, 399, 27));
        label_34->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_35 = new QLabel(pidTab);
        label_35->setObjectName(QString::fromUtf8("label_35"));
        label_35->setGeometry(QRect(50, 160, 399, 27));
        label_35->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_36 = new QLabel(pidTab);
        label_36->setObjectName(QString::fromUtf8("label_36"));
        label_36->setGeometry(QRect(50, 220, 399, 27));
        label_36->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        pidUpdate = new QPushButton(pidTab);
        pidUpdate->setObjectName(QString::fromUtf8("pidUpdate"));
        pidUpdate->setGeometry(QRect(320, 320, 191, 81));
        tabWidget->addTab(pidTab, QString());
        controlTab = new QWidget();
        controlTab->setObjectName(QString::fromUtf8("controlTab"));
        controlYaw = new QDial(controlTab);
        controlYaw->setObjectName(QString::fromUtf8("controlYaw"));
        controlYaw->setGeometry(QRect(130, 200, 231, 191));
        controlYaw->setMinimum(-180);
        controlYaw->setMaximum(180);
        controlYaw->setPageStep(0);
        controlYaw->setValue(0);
        controlYaw->setSliderPosition(0);
        controlYaw->setTracking(false);
        controlYaw->setOrientation(Qt::Horizontal);
        controlYaw->setInvertedAppearance(false);
        controlYaw->setInvertedControls(false);
        controlYaw->setWrapping(true);
        controlYaw->setNotchTarget(30);
        controlYaw->setNotchesVisible(true);
        controlVelocity = new QSlider(controlTab);
        controlVelocity->setObjectName(QString::fromUtf8("controlVelocity"));
        controlVelocity->setGeometry(QRect(540, 200, 71, 201));
        controlVelocity->setMinimum(-10);
        controlVelocity->setMaximum(10);
        controlVelocity->setTracking(false);
        controlVelocity->setOrientation(Qt::Vertical);
        controlVelocity->setTickPosition(QSlider::TicksBothSides);
        controlVelocity->setTickInterval(10);
        label_19 = new QLabel(controlTab);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setGeometry(QRect(140, 140, 211, 41));
        QFont font2;
        font2.setPointSize(15);
        label_19->setFont(font2);
        label_19->setAlignment(Qt::AlignCenter);
        label_20 = new QLabel(controlTab);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setGeometry(QRect(480, 140, 191, 41));
        label_20->setFont(font2);
        label_20->setAlignment(Qt::AlignCenter);
        label_21 = new QLabel(controlTab);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setGeometry(QRect(620, 200, 111, 18));
        QFont font3;
        font3.setPointSize(12);
        label_21->setFont(font3);
        label_22 = new QLabel(controlTab);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setGeometry(QRect(620, 380, 121, 18));
        label_22->setFont(font3);
        label_23 = new QLabel(controlTab);
        label_23->setObjectName(QString::fromUtf8("label_23"));
        label_23->setGeometry(QRect(620, 290, 111, 18));
        label_23->setFont(font3);
        tabWidget->addTab(controlTab, QString());
        camTab = new QWidget();
        camTab->setObjectName(QString::fromUtf8("camTab"));
        gridLayout_5 = new QGridLayout(camTab);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        cameraLabel = new QLabel(camTab);
        cameraLabel->setObjectName(QString::fromUtf8("cameraLabel"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(cameraLabel->sizePolicy().hasHeightForWidth());
        cameraLabel->setSizePolicy(sizePolicy1);
        cameraLabel->setMinimumSize(QSize(640, 480));
        cameraLabel->setSizeIncrement(QSize(4, 3));
        cameraLabel->setScaledContents(true);

        gridLayout_5->addWidget(cameraLabel, 1, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(10, 10, QSizePolicy::MinimumExpanding, QSizePolicy::Minimum);

        gridLayout_5->addItem(horizontalSpacer_2, 1, 0, 1, 1);

        horizontalSpacer_3 = new QSpacerItem(10, 10, QSizePolicy::MinimumExpanding, QSizePolicy::Minimum);

        gridLayout_5->addItem(horizontalSpacer_3, 1, 2, 1, 1);

        frame = new QFrame(camTab);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setMinimumSize(QSize(640, 42));
        frame->setMaximumSize(QSize(640, 42));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        centerLines = new QCheckBox(frame);
        centerLines->setObjectName(QString::fromUtf8("centerLines"));
        centerLines->setGeometry(QRect(10, 10, 171, 23));
        contours = new QCheckBox(frame);
        contours->setObjectName(QString::fromUtf8("contours"));
        contours->setGeometry(QRect(200, 10, 171, 23));

        gridLayout_5->addWidget(frame, 3, 1, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);

        gridLayout_5->addItem(verticalSpacer_2, 4, 1, 1, 1);

        verticalSpacer_3 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);

        gridLayout_5->addItem(verticalSpacer_3, 0, 1, 1, 1);

        tabWidget->addTab(camTab, QString());

        gridLayout->addWidget(tabWidget, 1, 2, 1, 1);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy2);
        groupBox->setMinimumSize(QSize(200, 0));
        groupBox->setMaximumSize(QSize(200, 16777215));
        groupBox->setFont(font);
        pitch_gyro_var = new QDoubleSpinBox(groupBox);
        pitch_gyro_var->setObjectName(QString::fromUtf8("pitch_gyro_var"));
        pitch_gyro_var->setGeometry(QRect(9, 130, 62, 28));
        pitch_gyro_var->setFont(font1);
        pitch_gyro_var->setDecimals(5);
        pitch_gyro_var->setMinimum(0);
        pitch_gyro_var->setMaximum(100000);
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(9, 100, 81, 31));
        label->setFont(font1);
        label->setWordWrap(true);
        pitch_bias_var = new QDoubleSpinBox(groupBox);
        pitch_bias_var->setObjectName(QString::fromUtf8("pitch_bias_var"));
        pitch_bias_var->setGeometry(QRect(109, 130, 62, 28));
        pitch_bias_var->setFont(font1);
        pitch_bias_var->setDecimals(5);
        pitch_bias_var->setMinimum(0);
        pitch_bias_var->setMaximum(100000);
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(109, 100, 81, 31));
        label_2->setFont(font1);
        label_2->setWordWrap(true);
        pitch_accel_var = new QDoubleSpinBox(groupBox);
        pitch_accel_var->setObjectName(QString::fromUtf8("pitch_accel_var"));
        pitch_accel_var->setGeometry(QRect(110, 170, 62, 28));
        pitch_accel_var->setFont(font1);
        pitch_accel_var->setDecimals(5);
        pitch_accel_var->setMinimum(0);
        pitch_accel_var->setMaximum(100000);
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(9, 170, 81, 31));
        label_3->setFont(font1);
        label_3->setWordWrap(true);
        reset_pitch = new QPushButton(groupBox);
        reset_pitch->setObjectName(QString::fromUtf8("reset_pitch"));
        reset_pitch->setGeometry(QRect(30, 320, 131, 31));
        reset_pitch->setFont(font1);
        reset_roll = new QPushButton(groupBox);
        reset_roll->setObjectName(QString::fromUtf8("reset_roll"));
        reset_roll->setGeometry(QRect(30, 600, 131, 31));
        reset_roll->setFont(font1);
        pauseKalmanUpdates = new QPushButton(groupBox);
        pauseKalmanUpdates->setObjectName(QString::fromUtf8("pauseKalmanUpdates"));
        pauseKalmanUpdates->setGeometry(QRect(20, 40, 151, 41));
        pauseKalmanUpdates->setFont(font1);
        pauseKalmanUpdates->setCheckable(true);
        pauseKalmanUpdates->setChecked(false);
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 210, 141, 18));
        label_4->setFont(font);
        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(16, 230, 20, 20));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(180, 230, 16, 18));
        pitch_kalman_0 = new QLabel(groupBox);
        pitch_kalman_0->setObjectName(QString::fromUtf8("pitch_kalman_0"));
        pitch_kalman_0->setGeometry(QRect(30, 230, 62, 18));
        pitch_kalman_0->setFont(font1);
        label_27 = new QLabel(groupBox);
        label_27->setObjectName(QString::fromUtf8("label_27"));
        label_27->setGeometry(QRect(10, 250, 171, 18));
        label_27->setFont(font);
        label_28 = new QLabel(groupBox);
        label_28->setObjectName(QString::fromUtf8("label_28"));
        label_28->setGeometry(QRect(170, 270, 16, 41));
        QFont font4;
        font4.setPointSize(32);
        font4.setBold(false);
        font4.setWeight(50);
        label_28->setFont(font4);
        label_28->setScaledContents(false);
        label_29 = new QLabel(groupBox);
        label_29->setObjectName(QString::fromUtf8("label_29"));
        label_29->setGeometry(QRect(10, 270, 16, 41));
        label_29->setFont(font4);
        label_29->setScaledContents(false);
        pitch_p_00 = new QLabel(groupBox);
        pitch_p_00->setObjectName(QString::fromUtf8("pitch_p_00"));
        pitch_p_00->setGeometry(QRect(30, 270, 62, 18));
        pitch_p_00->setFont(font1);
        pitch_p_10 = new QLabel(groupBox);
        pitch_p_10->setObjectName(QString::fromUtf8("pitch_p_10"));
        pitch_p_10->setGeometry(QRect(30, 290, 62, 18));
        pitch_p_10->setFont(font1);
        pitch_p_01 = new QLabel(groupBox);
        pitch_p_01->setObjectName(QString::fromUtf8("pitch_p_01"));
        pitch_p_01->setGeometry(QRect(100, 270, 62, 18));
        pitch_p_01->setFont(font1);
        pitch_p_11 = new QLabel(groupBox);
        pitch_p_11->setObjectName(QString::fromUtf8("pitch_p_11"));
        pitch_p_11->setGeometry(QRect(100, 290, 62, 18));
        pitch_p_11->setFont(font1);
        pitch_kalman_1 = new QLabel(groupBox);
        pitch_kalman_1->setObjectName(QString::fromUtf8("pitch_kalman_1"));
        pitch_kalman_1->setGeometry(QRect(100, 230, 62, 18));
        pitch_kalman_1->setFont(font1);
        roll_kalman_1 = new QLabel(groupBox);
        roll_kalman_1->setObjectName(QString::fromUtf8("roll_kalman_1"));
        roll_kalman_1->setGeometry(QRect(100, 510, 62, 18));
        roll_kalman_1->setFont(font1);
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(9, 380, 81, 31));
        label_7->setFont(font1);
        label_7->setWordWrap(true);
        roll_p_01 = new QLabel(groupBox);
        roll_p_01->setObjectName(QString::fromUtf8("roll_p_01"));
        roll_p_01->setGeometry(QRect(100, 550, 62, 18));
        roll_p_01->setFont(font1);
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 490, 141, 18));
        label_8->setFont(font);
        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(9, 450, 81, 31));
        label_9->setFont(font1);
        label_9->setWordWrap(true);
        label_30 = new QLabel(groupBox);
        label_30->setObjectName(QString::fromUtf8("label_30"));
        label_30->setGeometry(QRect(10, 530, 171, 18));
        label_30->setFont(font);
        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(109, 380, 81, 31));
        label_10->setFont(font1);
        label_10->setWordWrap(true);
        roll_p_11 = new QLabel(groupBox);
        roll_p_11->setObjectName(QString::fromUtf8("roll_p_11"));
        roll_p_11->setGeometry(QRect(100, 570, 62, 18));
        roll_p_11->setFont(font1);
        roll_bias_var = new QDoubleSpinBox(groupBox);
        roll_bias_var->setObjectName(QString::fromUtf8("roll_bias_var"));
        roll_bias_var->setGeometry(QRect(109, 410, 62, 28));
        roll_bias_var->setFont(font1);
        roll_bias_var->setDecimals(5);
        roll_bias_var->setMinimum(0);
        roll_bias_var->setMaximum(100000);
        roll_gyro_var = new QDoubleSpinBox(groupBox);
        roll_gyro_var->setObjectName(QString::fromUtf8("roll_gyro_var"));
        roll_gyro_var->setGeometry(QRect(9, 410, 62, 28));
        roll_gyro_var->setFont(font1);
        roll_gyro_var->setDecimals(5);
        roll_gyro_var->setMinimum(0);
        roll_gyro_var->setMaximum(100000);
        label_11 = new QLabel(groupBox);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(180, 510, 16, 18));
        roll_accel_var = new QDoubleSpinBox(groupBox);
        roll_accel_var->setObjectName(QString::fromUtf8("roll_accel_var"));
        roll_accel_var->setGeometry(QRect(110, 450, 62, 28));
        roll_accel_var->setFont(font1);
        roll_accel_var->setDecimals(5);
        roll_accel_var->setMinimum(0);
        roll_accel_var->setMaximum(100000);
        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(16, 510, 20, 20));
        roll_kalman_0 = new QLabel(groupBox);
        roll_kalman_0->setObjectName(QString::fromUtf8("roll_kalman_0"));
        roll_kalman_0->setGeometry(QRect(30, 510, 62, 18));
        roll_kalman_0->setFont(font1);
        roll_p_10 = new QLabel(groupBox);
        roll_p_10->setObjectName(QString::fromUtf8("roll_p_10"));
        roll_p_10->setGeometry(QRect(30, 570, 62, 18));
        roll_p_10->setFont(font1);
        label_31 = new QLabel(groupBox);
        label_31->setObjectName(QString::fromUtf8("label_31"));
        label_31->setGeometry(QRect(10, 550, 16, 41));
        label_31->setFont(font4);
        label_31->setScaledContents(false);
        label_32 = new QLabel(groupBox);
        label_32->setObjectName(QString::fromUtf8("label_32"));
        label_32->setGeometry(QRect(170, 550, 16, 41));
        label_32->setFont(font4);
        label_32->setScaledContents(false);
        roll_p_00 = new QLabel(groupBox);
        roll_p_00->setObjectName(QString::fromUtf8("roll_p_00"));
        roll_p_00->setGeometry(QRect(30, 550, 62, 18));
        roll_p_00->setFont(font1);
        line_3 = new QFrame(groupBox);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(-3, 640, 201, 20));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        pitchSumCount = new QSpinBox(groupBox);
        pitchSumCount->setObjectName(QString::fromUtf8("pitchSumCount"));
        pitchSumCount->setGeometry(QRect(120, 670, 55, 28));
        pitchSumCount->setValue(1);
        rollSumCount = new QSpinBox(groupBox);
        rollSumCount->setObjectName(QString::fromUtf8("rollSumCount"));
        rollSumCount->setGeometry(QRect(120, 710, 55, 28));
        rollSumCount->setValue(1);
        label_25 = new QLabel(groupBox);
        label_25->setObjectName(QString::fromUtf8("label_25"));
        label_25->setGeometry(QRect(10, 670, 101, 31));
        label_25->setFont(font1);
        label_25->setWordWrap(true);
        label_26 = new QLabel(groupBox);
        label_26->setObjectName(QString::fromUtf8("label_26"));
        label_26->setGeometry(QRect(10, 710, 101, 31));
        label_26->setFont(font1);
        label_26->setWordWrap(true);

        gridLayout->addWidget(groupBox, 0, 0, 4, 1);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);

        testrig->setCentralWidget(centralwidget);

        retranslateUi(testrig);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(testrig);
    } // setupUi

    void retranslateUi(QMainWindow *testrig)
    {
        testrig->setWindowTitle(QApplication::translate("testrig", "BeoHawk Test-Rig Display", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("testrig", "Statistical Data", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("testrig", "IMU Updates Per Second:", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("testrig", "Cam Updates Per Second:", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("testrig", "Pitch Gyro Standard Deviation:", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("testrig", "Pitch Accel Standard Deviation:", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("testrig", "Roll Gyro Standard Deviation:", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("testrig", "Roll Accel Standard Deviation:", 0, QApplication::UnicodeUTF8));
        imu_up_per_sec->setText(QApplication::translate("testrig", "0", 0, QApplication::UnicodeUTF8));
        cam_up_per_sec->setText(QApplication::translate("testrig", "0", 0, QApplication::UnicodeUTF8));
        pitch_gyro_sd->setText(QApplication::translate("testrig", "0", 0, QApplication::UnicodeUTF8));
        pitch_accel_sd->setText(QApplication::translate("testrig", "0", 0, QApplication::UnicodeUTF8));
        roll_gyro_sd->setText(QApplication::translate("testrig", "0", 0, QApplication::UnicodeUTF8));
        roll_accel_sd->setText(QApplication::translate("testrig", "0", 0, QApplication::UnicodeUTF8));
        constRollAxis->setText(QApplication::translate("testrig", "Hold Axis Scale Constant", 0, QApplication::UnicodeUTF8));
        constPitchAxis->setText(QApplication::translate("testrig", "Hold Axis Scale Constant", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(plotTab), QApplication::translate("testrig", "Graph Stuff", 0, QApplication::UnicodeUTF8));
        label_24->setText(QApplication::translate("testrig", "Show Frequency Spectrum of ", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(fftTab), QApplication::translate("testrig", "FFT Stuff", 0, QApplication::UnicodeUTF8));
        label_33->setText(QApplication::translate("testrig", "Tune PID Loop:", 0, QApplication::UnicodeUTF8));
        label_34->setText(QApplication::translate("testrig", "PID Proportional Constant, Kp", 0, QApplication::UnicodeUTF8));
        label_35->setText(QApplication::translate("testrig", "PID Integral Constant, Ki", 0, QApplication::UnicodeUTF8));
        label_36->setText(QApplication::translate("testrig", "PID Derivative Constant, Kd", 0, QApplication::UnicodeUTF8));
        pidUpdate->setText(QApplication::translate("testrig", "Update", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(pidTab), QApplication::translate("testrig", "PID Stuff", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("testrig", "Orientation", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("testrig", "Forward Velocity", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("testrig", "Fast, Forward", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("testrig", "Fast, Backward", 0, QApplication::UnicodeUTF8));
        label_23->setText(QApplication::translate("testrig", "Stopped", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(controlTab), QApplication::translate("testrig", "Control Stuff", 0, QApplication::UnicodeUTF8));
        cameraLabel->setStyleSheet(QApplication::translate("testrig", "border-style: solid;\n"
"border-width: 2px;\n"
"border-color: black;", 0, QApplication::UnicodeUTF8));
        cameraLabel->setText(QString());
        centerLines->setText(QApplication::translate("testrig", "Show Centering Lines", 0, QApplication::UnicodeUTF8));
        contours->setText(QApplication::translate("testrig", "Show Square Contours", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(camTab), QApplication::translate("testrig", "Camera Stuff", 0, QApplication::UnicodeUTF8));
        groupBox->setStyleSheet(QString());
        groupBox->setTitle(QApplication::translate("testrig", "Controls", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("testrig", "Pitch Gyro Covariance:", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("testrig", "Pitch Bias Coviariance:", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("testrig", "Pitch Accel Covariance:", 0, QApplication::UnicodeUTF8));
        reset_pitch->setText(QApplication::translate("testrig", "Reset Pitch Filter", 0, QApplication::UnicodeUTF8));
        reset_roll->setText(QApplication::translate("testrig", "Reset Roll Filter", 0, QApplication::UnicodeUTF8));
        pauseKalmanUpdates->setText(QApplication::translate("testrig", "Pause Live Updates\n"
"to Filter Data", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("testrig", "Pitch Kalman Gain", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("testrig", "[", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("testrig", "]", 0, QApplication::UnicodeUTF8));
        pitch_kalman_0->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        label_27->setText(QApplication::translate("testrig", "Pitch Predict Error Cov", 0, QApplication::UnicodeUTF8));
        label_28->setText(QApplication::translate("testrig", "]", 0, QApplication::UnicodeUTF8));
        label_29->setText(QApplication::translate("testrig", "[", 0, QApplication::UnicodeUTF8));
        pitch_p_00->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        pitch_p_10->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        pitch_p_01->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        pitch_p_11->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        pitch_kalman_1->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        roll_kalman_1->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("testrig", "Roll Gyro Covariance:", 0, QApplication::UnicodeUTF8));
        roll_p_01->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("testrig", "Roll Kalman Gain", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("testrig", "Roll Accel Covariance:", 0, QApplication::UnicodeUTF8));
        label_30->setText(QApplication::translate("testrig", "Roll Predict Error Cov", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("testrig", "Roll Bias Coviariance:", 0, QApplication::UnicodeUTF8));
        roll_p_11->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("testrig", "]", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("testrig", "[", 0, QApplication::UnicodeUTF8));
        roll_kalman_0->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        roll_p_10->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        label_31->setText(QApplication::translate("testrig", "[", 0, QApplication::UnicodeUTF8));
        label_32->setText(QApplication::translate("testrig", "]", 0, QApplication::UnicodeUTF8));
        roll_p_00->setText(QApplication::translate("testrig", "0.0", 0, QApplication::UnicodeUTF8));
        label_25->setText(QApplication::translate("testrig", "Pitch Weighted Sum Count:", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("testrig", "Roll Weighted Sum Count:", 0, QApplication::UnicodeUTF8));
        Q_UNUSED(testrig);
    } // retranslateUi

};

namespace Ui {
    class testrig: public Ui_testrig {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TESTRIG_H
