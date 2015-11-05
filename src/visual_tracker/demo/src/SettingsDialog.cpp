#include "SettingsDialog.h"
#include <QPushButton>

SettingsDialog::SettingsDialog(QSettings& settings, QWidget *parent)
	: QDialog(parent)
	, m_settings(settings)
{
	ui.setupUi(this);
	m_applyButton = ui.buttonBox->button(QDialogButtonBox::Apply);
	setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	setFixedSize(width(), height());
	applySettings();
	enableApply(false);

	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), ui.InitialBoundingBoxAutoStartCheckBox, SLOT(setEnabled(bool)));
	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), ui.initialBoundingBoxXLabel, SLOT(setEnabled(bool)));
	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), ui.initialBoundingBoxYLabel, SLOT(setEnabled(bool)));
	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), ui.initialBoundingBoxWidthLabel, SLOT(setEnabled(bool)));
	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), ui.initialBoundingBoxHeightLabel, SLOT(setEnabled(bool)));
	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), ui.initialBoundingBoxXSpinBox, SLOT(setEnabled(bool)));
	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), ui.initialBoundingBoxYSpinBox, SLOT(setEnabled(bool)));
	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), ui.initialBoundingBoxWidthSpinBox, SLOT(setEnabled(bool)));
	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), ui.initialBoundingBoxHeightSpinBox, SLOT(setEnabled(bool)));
	connect(ui.fpsLimitEnablecheckBox, SIGNAL(toggled(bool)), ui.fpsLimitSpinBox, SLOT(setEnabled(bool)));
	connect(ui.fpsLimitSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(fpsLimit(int)));

	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.InitialBoundingBoxAutoStartCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.initialBoundingBoxXSpinBox, SIGNAL(valueChanged(double)), this, SLOT(enableApply()));
	connect(ui.initialBoundingBoxYSpinBox, SIGNAL(valueChanged(double)), this, SLOT(enableApply()));
	connect(ui.initialBoundingBoxWidthSpinBox, SIGNAL(valueChanged(double)), this, SLOT(enableApply()));
	connect(ui.initialBoundingBoxHeightSpinBox, SIGNAL(valueChanged(double)), this, SLOT(enableApply()));
	connect(ui.fpsLimitEnablecheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.fpsLimitSpinBox, SIGNAL(valueChanged(int)), this, SLOT(enableApply()));

	connect(ui.InitialBoundingBoxEnabledCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.InitialBoundingBoxAutoStartCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.initialBoundingBoxXSpinBox, SIGNAL(valueChanged(double)), this, SLOT(enableApply()));
	connect(ui.initialBoundingBoxYSpinBox, SIGNAL(valueChanged(double)), this, SLOT(enableApply()));
	connect(ui.initialBoundingBoxWidthSpinBox, SIGNAL(valueChanged(double)), this, SLOT(enableApply()));
	connect(ui.initialBoundingBoxHeightSpinBox, SIGNAL(valueChanged(double)), this, SLOT(enableApply()));
	connect(ui.fpsLimitEnablecheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.fpsLimitSpinBox, SIGNAL(valueChanged(int)), this, SLOT(enableApply()));

	connect(ui.geometryRememberPositionCheckBox, SIGNAL(toggled(bool)), ui.geometryXLabel, SLOT(setEnabled(bool)));
	connect(ui.geometryRememberPositionCheckBox, SIGNAL(toggled(bool)), ui.geometryYLabel, SLOT(setEnabled(bool)));
	connect(ui.geometryRememberPositionCheckBox, SIGNAL(toggled(bool)), ui.geometryWidthLabel, SLOT(setEnabled(bool)));
	connect(ui.geometryRememberPositionCheckBox, SIGNAL(toggled(bool)), ui.geometryHeightLabel, SLOT(setEnabled(bool)));
	connect(ui.geometryRememberPositionCheckBox, SIGNAL(toggled(bool)), ui.geometryXSpinBox, SLOT(setEnabled(bool)));
	connect(ui.geometryRememberPositionCheckBox, SIGNAL(toggled(bool)), ui.geometryYSpinBox, SLOT(setEnabled(bool)));
	connect(ui.geometryRememberPositionCheckBox, SIGNAL(toggled(bool)), ui.geometryWidthSpinBox, SLOT(setEnabled(bool)));
	connect(ui.geometryRememberPositionCheckBox, SIGNAL(toggled(bool)), ui.geometryHeightSpinBox, SLOT(setEnabled(bool)));

	connect(ui.cameraBrightnessSlider, SIGNAL(valueChanged(int)), this, SIGNAL(brightness(int)));
	connect(ui.cameraContrastSlider, SIGNAL(valueChanged(int)), this, SIGNAL(contrast(int)));

	connect(ui.displayBoundingBoxCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.displayPointsCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.displayConfidenceCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.displayDetectionsCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.displayPositiveCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.displayNegativeCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));
	connect(ui.flipCheckBox, SIGNAL(toggled(bool)), this, SLOT(enableApply()));

	connect(ui.displayBoundingBoxCheckBox, SIGNAL(toggled(bool)), this, SIGNAL(detectorBoundingBoxEnabled(bool)));
	connect(ui.displayPointsCheckBox, SIGNAL(toggled(bool)), this, SIGNAL(detectorPointsEnabled(bool)));
	connect(ui.displayConfidenceCheckBox, SIGNAL(toggled(bool)), this, SIGNAL(detectorConfidenceEnabled(bool)));
	connect(ui.displayDetectionsCheckBox, SIGNAL(toggled(bool)), this, SIGNAL(detectorDetectionsEnabled(bool)));
	connect(ui.displayPositiveCheckBox, SIGNAL(toggled(bool)), this, SIGNAL(detectorPositiveEnabled(bool)));
	connect(ui.displayNegativeCheckBox, SIGNAL(toggled(bool)), this, SIGNAL(detectorNegativeEnabled(bool)));
	connect(ui.flipCheckBox, SIGNAL(toggled(bool)), this, SIGNAL(detectorFlipImageEnabled(bool)));

	connect(m_applyButton, SIGNAL(clicked()), this, SLOT(apply()));
}

SettingsDialog::~SettingsDialog()
{
}

void SettingsDialog::enableApply(bool enable)
{
	
	m_applyButton->setEnabled(enable);
}

void SettingsDialog::applySettings()
{
	bool bbChecked = m_settings.value(BOUNDING_BOX_ENABLED, BOUNDING_BOX_ENABLED_DEF).toBool();
	ui.InitialBoundingBoxEnabledCheckBox->setChecked(bbChecked);
	bool bbAutoStart = m_settings.value(BOUNDING_BOX_AUTO_START, BOUNDING_BOX_AUTO_START_DEF).toBool();
	ui.InitialBoundingBoxAutoStartCheckBox->setEnabled(bbChecked);
	ui.initialBoundingBoxXLabel->setEnabled(bbChecked);
	ui.initialBoundingBoxYLabel->setEnabled(bbChecked);
	ui.initialBoundingBoxWidthLabel->setEnabled(bbChecked);
	ui.initialBoundingBoxHeightLabel->setEnabled(bbChecked);
	ui.initialBoundingBoxXSpinBox->setEnabled(bbChecked);
	ui.initialBoundingBoxYSpinBox->setEnabled(bbChecked);
	ui.initialBoundingBoxWidthSpinBox->setEnabled(bbChecked);
	ui.initialBoundingBoxHeightSpinBox->setEnabled(bbChecked);
	ui.InitialBoundingBoxAutoStartCheckBox->setChecked(bbAutoStart);
	ui.initialBoundingBoxXSpinBox->setValue(m_settings.value(BOUNDING_BOX_X, BOUNDING_BOX_X_DEF).toDouble());
	ui.initialBoundingBoxYSpinBox->setValue(m_settings.value(BOUNDING_BOX_Y, BOUNDING_BOX_Y_DEF).toDouble());
	ui.initialBoundingBoxWidthSpinBox->setValue(m_settings.value(BOUNDING_BOX_WIDTH, BOUNDING_BOX_WIDTH_DEF).toDouble());
	ui.initialBoundingBoxHeightSpinBox->setValue(m_settings.value(BOUNDING_BOX_HEIGHT, BOUNDING_BOX_HEIGHT_DEF).toDouble());

	bool fpsLimitEnabledChecked = m_settings.value(FPS_LIMIT_ENABLED, FPS_LIMIT_ENABLED_DEF).toBool();
	ui.fpsLimitEnablecheckBox->setChecked(fpsLimitEnabledChecked);
	ui.fpsLimitSpinBox->setEnabled(fpsLimitEnabledChecked);
	ui.fpsLimitSpinBox->setValue(m_settings.value(FPS_LIMIT, FPS_LIMIT_DEF).toInt());

	bool geometryRememberPosition = m_settings.value(GEOMETRY_REMEMBER_POSITION_ENABLED, GEOMETRY_REMEMBER_POSITION_ENABLED_DEF).toBool();
	ui.geometryRememberPositionCheckBox->setChecked(geometryRememberPosition);
	ui.geometryXLabel->setEnabled(geometryRememberPosition);
	ui.geometryXSpinBox->setEnabled(geometryRememberPosition);
	ui.geometryYLabel->setEnabled(geometryRememberPosition);
	ui.geometryYSpinBox->setEnabled(geometryRememberPosition);
	ui.geometryWidthLabel->setEnabled(geometryRememberPosition);
	ui.geometryWidthSpinBox->setEnabled(geometryRememberPosition);
	ui.geometryHeightLabel->setEnabled(geometryRememberPosition);
	ui.geometryHeightSpinBox->setEnabled(geometryRememberPosition);

	ui.displayBoundingBoxCheckBox->setChecked(m_settings.value(DETECTOR_BOUNDING_BOX_ENABLED, DETECTOR_BOUNDING_BOX_ENABLED_DEF).toBool());
	ui.displayPointsCheckBox->setChecked(m_settings.value(DETECTOR_POINTS_ENABLED, DETECTOR_POINTS_ENABLED_DEF).toBool());
	ui.displayConfidenceCheckBox->setChecked(m_settings.value(DETECTOR_CONFIDENCE_ENABLED, DETECTOR_CONFIDENCE_ENABLED_DEF).toBool());
	ui.displayDetectionsCheckBox->setChecked(m_settings.value(DETECTOR_DETECTIONS_ENABLED, DETECTOR_DETECTIONS_ENABLED_DEF).toBool());
	ui.displayPositiveCheckBox->setChecked(m_settings.value(DETECTOR_POSITIVE_ENABLED, DETECTOR_POSITIVE_ENABLED_DEF).toBool());
	ui.displayNegativeCheckBox->setChecked(m_settings.value(DETECTOR_NEGATIVE_ENABLED, DETECTOR_NEGATIVE_ENABLED_DEF).toBool());
	ui.flipCheckBox->setChecked(m_settings.value(DETECTOR_FLIP_IMAGE_ENABLED, DETECTOR_FLIP_IMAGE_ENABLED_DEF).toBool());
}

void SettingsDialog::accept()
{
	apply();
	QDialog::accept();
}

void SettingsDialog::apply()
{
	bool e = ui.InitialBoundingBoxEnabledCheckBox->isChecked();
	bool a = ui.InitialBoundingBoxAutoStartCheckBox->isChecked();
	double x = ui.initialBoundingBoxXSpinBox->value();
	double y = ui.initialBoundingBoxYSpinBox->value();
	double w = ui.initialBoundingBoxWidthSpinBox->value();
	double h = ui.initialBoundingBoxHeightSpinBox->value();
	m_settings.setValue(BOUNDING_BOX_ENABLED, e);
	m_settings.setValue(BOUNDING_BOX_AUTO_START, a);
	m_settings.setValue(BOUNDING_BOX_X, x);
	m_settings.setValue(BOUNDING_BOX_Y, y);
	m_settings.setValue(BOUNDING_BOX_WIDTH, w);
	m_settings.setValue(BOUNDING_BOX_HEIGHT, h);
	if (e) emit boundingRect(QRectF(x,y,w,h), false);
	emit autoStart(e && a);

	bool fpsLimitEnabled = ui.fpsLimitEnablecheckBox->isChecked();
	int fl = ui.fpsLimitSpinBox->value();
	m_settings.setValue(FPS_LIMIT_ENABLED, fpsLimitEnabled);
	m_settings.setValue(FPS_LIMIT, fl);
	if (fpsLimitEnabled) emit fpsLimit(fl);

	bool grpe = ui.geometryRememberPositionCheckBox->isChecked();
	m_settings.setValue(GEOMETRY_REMEMBER_POSITION_ENABLED, grpe);
	if (grpe && m_settings.contains(GEOMETRY))
	{
		QByteArray g = m_settings.value("geometry").toByteArray();
		emit geometry(g);
	}

	m_settings.setValue(DETECTOR_BOUNDING_BOX_ENABLED, ui.displayBoundingBoxCheckBox->isChecked());
	emit detectorBoundingBoxEnabled(ui.displayBoundingBoxCheckBox->isChecked());
	m_settings.setValue(DETECTOR_POINTS_ENABLED, ui.displayPointsCheckBox->isChecked());
	emit detectorPointsEnabled(ui.displayPointsCheckBox->isChecked());
	m_settings.setValue(DETECTOR_CONFIDENCE_ENABLED, ui.displayConfidenceCheckBox->isChecked());
	emit detectorConfidenceEnabled(ui.displayConfidenceCheckBox->isChecked());
	m_settings.setValue(DETECTOR_DETECTIONS_ENABLED, ui.displayDetectionsCheckBox->isChecked());
	emit detectorDetectionsEnabled(ui.displayDetectionsCheckBox->isChecked());
	m_settings.setValue(DETECTOR_POSITIVE_ENABLED, ui.displayPositiveCheckBox->isChecked());
	emit detectorPositiveEnabled(ui.displayPositiveCheckBox->isChecked());
	m_settings.setValue(DETECTOR_NEGATIVE_ENABLED, ui.displayNegativeCheckBox->isChecked());
	emit detectorNegativeEnabled(ui.displayNegativeCheckBox->isChecked());
	m_settings.setValue(DETECTOR_FLIP_IMAGE_ENABLED, ui.flipCheckBox->isChecked());
	emit detectorFlipImageEnabled(ui.flipCheckBox->isChecked());
	enableApply(false);
}

void SettingsDialog::reject()
{
	applySettings();
	enableApply(false);
	QDialog::reject();
}