#include "theme.h"

#include <QApplication>
#include <QPalette>
#include <QStyleFactory>

void
apply_theme(QApplication &app)
{
	app.setStyle(QStyleFactory::create("Fusion"));
	QPalette p;
	p.setColor(QPalette::Window, pal::panel);
	p.setColor(QPalette::WindowText, pal::text);
	p.setColor(QPalette::Base, pal::bg);
	p.setColor(QPalette::AlternateBase, QColor(24, 29, 38));
	p.setColor(QPalette::Text, pal::text);
	p.setColor(QPalette::Button, pal::card);
	p.setColor(QPalette::ButtonText, pal::text);
	p.setColor(QPalette::Highlight, pal::accent);
	p.setColor(QPalette::HighlightedText, Qt::white);
	p.setColor(QPalette::ToolTipBase, pal::card);
	p.setColor(QPalette::ToolTipText, pal::text);
	p.setColor(QPalette::PlaceholderText, pal::dim);
	app.setPalette(p);

	app.setStyleSheet(R"(
		QMainWindow, QWidget#side { background: #1a1f29; }
		QFrame#topbar { background: #12161e; border-bottom: 1px solid #343c4c; }
		QLabel#title { font-size: 15pt; font-weight: 700; color: #ffffff; padding-right: 4px; }
		QLabel#pill { background: #212733; border-radius: 11px; padding: 4px 12px; }
		QLabel#section { color: #848ea0; font-size: 7.5pt; font-weight: 700; letter-spacing: 1px; }
		QLabel#hint { color: #848ea0; }
		QToolButton#mode { background: #212733; color: #dee2e9; border: 1px solid #343c4c; border-radius: 7px; padding: 6px 12px; }
		QToolButton#mode:hover { border-color: #409cff; }
		QToolButton#mode:checked { background: #409cff; border-color: #409cff; color: #ffffff; font-weight: 600; }
		QToolButton#view { background: #212733; color: #848ea0; border: 1px solid #343c4c; padding: 6px 12px; font-weight: 700; }
		QToolButton#view:checked { background: #2c3a52; color: #ffffff; border-color: #409cff; }
		QPushButton { border-radius: 7px; padding: 6px 14px; font-weight: 600; border: 1px solid #343c4c; background: #212733; color: #dee2e9; }
		QPushButton:hover { border-color: #409cff; }
		QPushButton#go { background: #2ecc71; border-color: #2ecc71; color: #08210f; min-width: 64px; }
		QPushButton#go:hover { background: #4cdb88; }
		QPushButton#stop { background: #f05252; border-color: #f05252; color: #ffffff; min-width: 64px; }
		QPushButton#stop:hover { background: #f46f6f; }
		QPushButton#route { background: #24324a; border-color: #409cff; color: #9cc9ff; }
		QComboBox { background: #212733; border: 1px solid #343c4c; border-radius: 7px; padding: 5px 10px; color: #dee2e9; }
		QComboBox QAbstractItemView { background: #212733; color: #dee2e9; selection-background-color: #409cff; }
		QFrame#layers { background: rgba(18,22,30,225); border: 1px solid #343c4c; border-radius: 8px; }
		QCheckBox { color: #dee2e9; spacing: 6px; }
		QCheckBox::indicator { width: 12px; height: 12px; border-radius: 3px; border: 1px solid #4a5468; background: #12161e; }
		QCheckBox::indicator:checked { background: #409cff; border-color: #409cff; }
		QToolButton#overlaybtn { background: rgba(18,22,30,215); color: #dee2e9; border: 1px solid #343c4c; border-radius: 7px; padding: 5px 12px; font-weight: 600; }
		QToolButton#overlaybtn:checked { border-color: #409cff; color: #9cc9ff; }
		QToolButton#overlaybtn:hover { border-color: #409cff; }
		QToolButton#eyebtn { background: rgba(18,22,30,215); border: 1px solid #343c4c; border-radius: 22px; padding: 0px; }
		QToolButton#eyebtn:hover { border-color: #409cff; }
		QToolButton#eyebtn:checked { background: rgba(64,156,255,90); border-color: #409cff; }
		QComboBox#overlaycombo { background: rgba(18,22,30,215); }
		QTabWidget::pane { border: none; }
		QTabBar::tab { background: #12161e; color: #848ea0; padding: 8px 22px; border: none; border-bottom: 2px solid transparent; }
		QTabBar::tab:selected { color: #ffffff; border-bottom: 2px solid #409cff; }
		QTreeWidget { background: #161b24; alternate-background-color: #1a2029; border: 1px solid #343c4c; border-radius: 8px; color: #dee2e9; }
		QHeaderView::section { background: #212733; color: #848ea0; border: none; padding: 6px; font-weight: 600; }
		QPlainTextEdit#console { background: #0d1016; color: #c9d1d9; border: 1px solid #343c4c; border-radius: 8px; }
		QStatusBar { background: #12161e; color: #848ea0; border-top: 1px solid #343c4c; }
		QScrollArea { background: #1a1f29; }
		QSplitter::handle { background: #343c4c; }
	)");
}
