#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include <QWidget>
#include <QSettings>
#include "ui_SettingsDialog.h"

#define BOUNDING_BOX_ENABLED QString("boundingBoxEnabled")
#define BOUNDING_BOX_ENABLED_DEF false
#define BOUNDING_BOX_AUTO_START QString("boundingBoxAutoStart")
#define BOUNDING_BOX_AUTO_START_DEF false
#define BOUNDING_BOX_X QString("boundingBoxX")
#define BOUNDING_BOX_X_DEF 0.00
#define BOUNDING_BOX_Y QString("boundingBoxY")
#define BOUNDING_BOX_Y_DEF 0.00
#define BOUNDING_BOX_WIDTH QString("boundingBoxWidth")
#define BOUNDING_BOX_WIDTH_DEF 0.00
#define BOUNDING_BOX_HEIGHT QString("boundingBoxHeight")
#define BOUNDING_BOX_HEIGHT_DEF 0.00
#define FPS_LIMIT_ENABLED QString("fpsLimitEnabled")
#define FPS_LIMIT_ENABLED_DEF false
#define FPS_LIMIT QString("fpsLimit")
#define FPS_LIMIT_DEF 0
#define GEOMETRY_REMEMBER_POSITION_ENABLED QString("geometryRememberPositionEnabled")
#define GEOMETRY_REMEMBER_POSITION_ENABLED_DEF false
#define GEOMETRY QString("geometry")
#define GEOMETRY_X_DEF 0
#define GEOMETRY_Y_DEF 0
#define GEOMETRY_WIDTH_DEF 0
#define GEOMETRY_HEIGHT_DEF 0
#define DETECTOR_BOUNDING_BOX_ENABLED "detectorBoundingBoxEnabled"
#define DETECTOR_BOUNDING_BOX_ENABLED_DEF false
#define DETECTOR_POINTS_ENABLED "detectorPointsEnabled"
#define DETECTOR_POINTS_ENABLED_DEF false
#define DETECTOR_CONFIDENCE_ENABLED "detectorConfidenceEnabled"
#define DETECTOR_CONFIDENCE_ENABLED_DEF false
#define DETECTOR_DETECTIONS_ENABLED "detectorDetectionsEnabled"
#define DETECTOR_DETECTIONS_ENABLED_DEF false
#define DETECTOR_POSITIVE_ENABLED "detectorPositiveEnabled"
#define DETECTOR_POSITIVE_ENABLED_DEF false
#define DETECTOR_NEGATIVE_ENABLED "detectorNegativeEnabled"
#define DETECTOR_NEGATIVE_ENABLED_DEF false
#define DETECTOR_FLIP_IMAGE_ENABLED "detectorFlipImageEnabled"
#define DETECTOR_FLIP_IMAGE_ENABLED_DEF false

class QPushButton;

class SettingsDialog : public QDialog
{
	Q_OBJECT

public:
	SettingsDialog(QSettings& settings, QWidget *parent = 0);
	~SettingsDialog();

signals:
	void boundingRect(QRectF rect, bool show);
	void autoStart(bool);
	void fpsLimit(int);
	void geometry(QByteArray);
	void brightness(int);
	void contrast(int);
	void detectorBoundingBoxEnabled(bool);
	void detectorPointsEnabled(bool);
	void detectorConfidenceEnabled(bool);
	void detectorDetectionsEnabled(bool);
	void detectorPositiveEnabled(bool);
	void detectorNegativeEnabled(bool);
	void detectorFlipImageEnabled(bool);

public slots:
	virtual void accept();
	void enableApply(bool enable = true);
	void apply();
	void reject();

private:
	void applySettings();

private:
	Ui::SettingsDialog ui;
	QSettings& m_settings;
	QPushButton* m_applyButton;
};

#endif // SETTINGSDIALOG_H
