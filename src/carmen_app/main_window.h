/* Janela principal: barra de comandos, vista 3D, painel, aba de modulos. */
#ifndef CARMEN_APP_MAIN_WINDOW_H
#define CARMEN_APP_MAIN_WINDOW_H

#include "scene.h"

#include <QJsonArray>
#include <QMainWindow>
#include <QMap>
#include <functional>

class Connection;
class Dashboard;
class MapView3D;
class QButtonGroup;
class QCheckBox;
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;
class QWidget;

class MainWindow : public QMainWindow
{
public:
	explicit MainWindow(Connection *conn);

	QTabWidget *tabs = nullptr;

	/* modo de teste (main.cpp --screenshot) */
	void test_select_place(int i);
	void test_full_map();
	void test_top_view();
	void test_close();
	void test_options();
	void test_map(const QString &source, bool overlay);
	void test_pick();
	void test_cameras();

private:
	enum Mode { NAVIGATE = 0, INITIAL_POSE, FINAL_GOAL, ROUTE_TO };
	struct Place { QString name; double x, y, theta; };

	/* interface */
	void build_ui();
	QWidget *build_top_bar();
	QWidget *build_map_page();
	QWidget *build_overlay(QWidget *parent);
	QWidget *build_modules_page();
	QToolButton *mode_button(const QString &label, int id);
	QPushButton *action_button(const QString &label, const char *name, std::function<void()> fn);
	MapView *view() const;

	/* comunicacao */
	bool send(const QJsonObject &payload, QJsonObject *reply = nullptr, QByteArray *blob = nullptr);
	void simple_cmd(const char *cmd, const QString &ok_message);
	void request_snapshot();
	void load_car_model();
	void load_missions();
	void request_map(bool refresh, bool verbose);
	void toggle_full_map(bool on);
	void tick();
	void update_status();
	void handle_topic(const std::string &t, const QJsonObject &d, const QByteArray &blob);

	/* dados */
	void on_pose(const QJsonObject &d);
	void on_robot(const QJsonObject &d);
	void on_behavior(const QJsonObject &d);
	void on_car(const QJsonObject &d);
	void on_proc(const QJsonObject &d);
	void on_proc_item_changed(QTreeWidgetItem *item, int column);
	void on_proc_selected(QTreeWidgetItem *current);
	void merge_annotations();
	void update_places();
	void on_place_selected(int index);
	void place_as_goal();
	void reset_click();
	void on_pick(double x, double y, double theta);
	void send_layer_requests();	// pede (e renova) o mapa escolhido e o lidar ao servidor
	void set_map(const QJsonObject &meta, const QByteArray &blob);
	void set_lidar(const QJsonObject &meta, const QByteArray &blob);
	void refresh_views();

	Connection *conn;
	Scene scene;
	MapView3D *view3d = nullptr;
	Dashboard *dash = nullptr;
	QButtonGroup *mode_group = nullptr;
	QCheckBox *follow_cb = nullptr;
	int lidar_id = 5, lidar_quality = 1;
	std::vector<int> cameras_on;	// cameras pedidas ao servidor
	QComboBox *missions_combo = nullptr, *places_combo = nullptr, *task_combo = nullptr, *algorithm_combo = nullptr;
	bool updating_behavior = false;
	void send_behavior();
	QToolButton *full_btn = nullptr, *options_btn = nullptr;
	QLabel *lbl_conn = nullptr, *lbl_output = nullptr;
	QTreeWidget *proc_tree = nullptr;
	QPlainTextEdit *output_text = nullptr;
	QMap<QString, QTreeWidgetItem *> proc_items, group_items;
	bool updating_tree = false;
	QString selected_module;
	double last_hello = 0.0, last_map_request = 0.0;
	bool got_snapshot = false, car_model_requested = false;
	QJsonArray file_annotations, live_annotations;
	std::vector<Place> places;
	QStringList place_names;
};

#endif
