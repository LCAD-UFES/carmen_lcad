#include "main_window.h"
#include "connection.h"
#include "dashboard.h"
#include "map_image.h"
#include "map_view_3d.h"
#include "theme.h"

#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QFontDatabase>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSet>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <zlib.h>
#include <cstring>

static const char *
route_state_name(int s)
{
	static const char *names[] = {"IDLE", "PLANNING_FROM_POSE_TO_LANE", "EXECUTING_OFFROAD_PLAN", "PUBLISHING_ROUTE",
			"ROUTE_FINISHED", "WAITING_FOR_PLAN", "PLANNING_FROM_LANE_TO_POSE"};
	if (s >= 0 && s < (int) (sizeof(names) / sizeof(names[0])))
		return names[s];
	return "?";
}

static QString
short_annotation_name(const QString &text)
{
	if (text.startsWith("RDDF_ANNOTATION_TYPE_"))
		return text.mid(21);
	if (text.startsWith("RDDF_"))
		return text.mid(5);
	return text;
}

MainWindow::MainWindow(Connection *conn) : conn(conn)
{
	setWindowTitle(QString("carmen app  -  %1").arg(QString::fromStdString(conn->host)));
	resize(1500, 920);
	build_ui();

	QTimer *timer = new QTimer(this);
	QObject::connect(timer, &QTimer::timeout, [this]() { tick(); });
	timer->start(30);

	QTimer *slow = new QTimer(this);
	QObject::connect(slow, &QTimer::timeout, [this]() { update_status(); dash->update(); });
	slow->start(250);

	/* o servidor desliga o mapa extra e o lidar se o app nao renovar o pedido em 30 s (app fechado) */
	/* "estou vivo" para o watchdog do servidor: sem pedido do app por 5 s ele da' STOP no robo */
	QTimer *heartbeat = new QTimer(this);
	QObject::connect(heartbeat, &QTimer::timeout, [this]() {
		if (!this->conn->link_alive())
			return;
		QJsonObject ping, reply;
		ping["cmd"] = "ping";
		this->conn->request(ping, &reply, nullptr, 1500);
	});
	heartbeat->start(1000);

	QTimer *keepalive = new QTimer(this);
	QObject::connect(keepalive, &QTimer::timeout, [this]() { send_layer_requests(); });
	keepalive->start(10000);

	QTimer::singleShot(200, [this]() { request_snapshot(); });
}

/* ============================================================================================ */
/* Interface                                                                                    */
/* ============================================================================================ */

QToolButton *
MainWindow::mode_button(const QString &label, int id)
{
	QToolButton *b = new QToolButton;
	b->setText(label);
	b->setCheckable(true);
	b->setObjectName("mode");
	mode_group->addButton(b, id);
	return b;
}

QPushButton *
MainWindow::action_button(const QString &label, const char *name, std::function<void()> fn)
{
	QPushButton *b = new QPushButton(label);
	b->setObjectName(name);
	b->setCursor(Qt::PointingHandCursor);
	QObject::connect(b, &QPushButton::clicked, fn);
	return b;
}

void
MainWindow::build_ui()
{
	QWidget *central = new QWidget;
	QVBoxLayout *root = new QVBoxLayout(central);
	root->setContentsMargins(0, 0, 0, 0);
	root->setSpacing(0);
	setCentralWidget(central);
	root->addWidget(build_top_bar());

	tabs = new QTabWidget;
	tabs->setDocumentMode(true);
	tabs->addTab(build_map_page(), "Mapa");
	tabs->addTab(build_modules_page(), "Modulos");
	root->addWidget(tabs, 1);

	statusBar()->showMessage("Conectando...");
	update_status();
}

QWidget *
MainWindow::build_top_bar()
{
	QFrame *top = new QFrame;
	top->setObjectName("topbar");
	QHBoxLayout *tb = new QHBoxLayout(top);
	tb->setContentsMargins(14, 8, 14, 8);
	tb->setSpacing(8);
	QLabel *title = new QLabel("carmen");
	title->setObjectName("title");
	tb->addWidget(title);
	lbl_conn = new QLabel;
	lbl_conn->setObjectName("pill");
	tb->addWidget(lbl_conn);
	tb->addSpacing(10);

	mode_group = new QButtonGroup(this);
	mode_group->setExclusive(true);
	tb->addWidget(mode_button("Navegar", NAVIGATE));
	tb->addWidget(mode_button("Pose inicial", INITIAL_POSE));
	tb->addWidget(mode_button("Destino", FINAL_GOAL));
	mode_group->button(NAVIGATE)->setChecked(true);
	for (QAbstractButton *b : mode_group->buttons())
		QObject::connect(b, &QAbstractButton::clicked, [this]() { reset_click(); });
	tb->addSpacing(10);
	tb->addWidget(action_button("Calcular rota", "route", [this]() { simple_cmd("compute_route", "Rota pedida ao route_planner"); }));
	tb->addWidget(action_button("GO", "go", [this]() { simple_cmd("go", "GO enviado"); }));
	tb->addWidget(action_button("STOP", "stop", [this]() { simple_cmd("stop", "STOP enviado"); }));
	tb->addSpacing(10);

	/* os combos do navigator_gui2: tarefa e planejador (A* = pre-rota pelo offroad_planner) */
	task_combo = new QComboBox;
	task_combo->setToolTip("Tarefa do behavior_selector");
	for (const char *t : {"Follow Route", "Park", "Park Semi-trailer", "Park Truck and Semi-trailer", "Move to Engage Pose"})
		task_combo->addItem(t);
	algorithm_combo = new QComboBox;
	algorithm_combo->setToolTip("Planejador: Frenet, A* (offroad), RRT, RDDF ou Gradient");
	for (const char *a : {"Frenet", "A* (offroad)", "RRT", "RDDF", "Gradient"})
		algorithm_combo->addItem(a);
	tb->addWidget(task_combo);
	tb->addWidget(algorithm_combo);
	QObject::connect(task_combo, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated), [this](int) { send_behavior(); });
	QObject::connect(algorithm_combo, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated), [this](int) { send_behavior(); });
	tb->addStretch(1);

	missions_combo = new QComboBox;
	missions_combo->setMinimumWidth(200);
	tb->addWidget(missions_combo);
	tb->addWidget(action_button("Executar", "secondary", [this]() {
		if (missions_combo->currentText().isEmpty())
			return;
		QJsonObject req;
		req["cmd"] = "execute_mission";
		req["name"] = missions_combo->currentText();
		if (send(req))
			statusBar()->showMessage("Missao enviada: " + missions_combo->currentText(), 5000);
	}));
	tb->addWidget(action_button("Abortar", "secondary", [this]() { simple_cmd("abort_mission", "Missao abortada"); }));
	return top;
}

QWidget *
MainWindow::build_map_page()
{
	QSplitter *split = new QSplitter;
	split->setHandleWidth(1);

	/* vista 3D com o quadro flutuante por cima */
	QWidget *views = new QWidget;
	QVBoxLayout *vl = new QVBoxLayout(views);
	vl->setContentsMargins(0, 0, 0, 0);
	view3d = new MapView3D(&scene);
	view3d->on_pick = [this](double x, double y, double theta) { on_pick(x, y, theta); };
	vl->addWidget(view3d);
	QWidget *overlay = build_overlay(views);
	overlay->raise();
	split->addWidget(views);

	QWidget *side = new QWidget;
	side->setObjectName("side");
	QVBoxLayout *sl = new QVBoxLayout(side);
	sl->setContentsMargins(0, 0, 0, 0);
	sl->setSpacing(0);
	dash = new Dashboard(&scene.robot);
	QScrollArea *scroll = new QScrollArea;
	scroll->setWidget(dash);
	scroll->setWidgetResizable(true);
	scroll->setFrameShape(QFrame::NoFrame);
	sl->addWidget(scroll, 1);

	QFrame *follow_box = new QFrame;
	follow_box->setObjectName("layers");
	QHBoxLayout *fl = new QHBoxLayout(follow_box);
	fl->setContentsMargins(14, 8, 14, 8);
	follow_cb = new QCheckBox("Seguir o robo");
	follow_cb->setChecked(true);
	fl->addWidget(follow_cb);
	fl->addStretch(1);
	fl->addWidget(action_button("Centralizar", "secondary", [this]() { follow_cb->setChecked(true); view()->zoom_on_robot(); }));
	sl->addWidget(follow_box);

	split->addWidget(side);
	split->setStretchFactor(0, 1);
	split->setStretchFactor(1, 0);
	split->setSizes(QList<int>() << 1100 << 360);
	return split;
}

QWidget *
MainWindow::build_overlay(QWidget *parent)
{
	QWidget *overlay = new QWidget(parent);
	overlay->move(12, 12);
	QVBoxLayout *ov = new QVBoxLayout(overlay);
	ov->setContentsMargins(0, 0, 0, 0);
	ov->setSpacing(6);

	auto overlay_button = [](const QString &label, bool checkable) {
		QToolButton *b = new QToolButton;
		b->setObjectName("overlaybtn");
		b->setText(label);
		b->setCheckable(checkable);
		return b;
	};

	QHBoxLayout *row = new QHBoxLayout;
	row->setSpacing(6);
	QToolButton *layers_btn = options_btn = overlay_button("Opcoes", true);
	row->addWidget(layers_btn);
	places_combo = new QComboBox;
	places_combo->setObjectName("overlaycombo");
	places_combo->setMinimumWidth(210);
	places_combo->addItem("Ir para um lugar...");
	row->addWidget(places_combo);
	QToolButton *dest_btn = overlay_button("Destino aqui", false);
	QToolButton *back_btn = overlay_button("Voltar ao robo", false);
	QToolButton *reload_btn = overlay_button("Recarregar", false);
	full_btn = overlay_button("Mapa completo", true);
	for (QToolButton *b : {dest_btn, back_btn, reload_btn, full_btn})
		row->addWidget(b);
	row->addStretch(1);
	ov->addLayout(row);

	QObject::connect(places_combo, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated), [this](int i) { on_place_selected(i); });
	QObject::connect(dest_btn, &QToolButton::clicked, [this]() { place_as_goal(); });
	QObject::connect(back_btn, &QToolButton::clicked, [this]() {
		follow_cb->setChecked(true);
		scene.have_highlight = false;
		view()->zoom_on_robot();
		places_combo->setCurrentIndex(0);
	});
	QObject::connect(reload_btn, &QToolButton::clicked, [this]() { request_map(true, true); });
	QObject::connect(full_btn, &QToolButton::toggled, [this](bool on) { toggle_full_map(on); });

	/* opcoes de exibicao: as do menu Options do viewer_3D + o menu Maps do navigator_gui2 */
	QFrame *box = new QFrame;
	box->setObjectName("layers");
	QGridLayout *lg = new QGridLayout(box);
	lg->setContentsMargins(12, 10, 12, 10);
	lg->setHorizontalSpacing(18);
	lg->setVerticalSpacing(5);
	struct Item { const char *label; bool *flag; const char *color; std::function<void()> after; };
	struct Group { const char *title; std::vector<Item> items; };
	Layers &L = scene.layers;
	auto layers_changed = [this]() { send_layer_requests(); };
	const Group groups[] = {
		{"MAPA", {{"Mapa", &L.map, "#8a96ab", layers_changed}, {"Sobrepor ao offline", &L.map_overlay, "#8a96ab", nullptr},
				{"Grade", &L.grid, "#5a6a88", nullptr}}},
		{"PLANEJAMENTO", {{"Rota", &L.route, "#409cff", nullptr}, {"Grafo (lanes)", &L.lanes, "#6e7d96", nullptr}, {"RDDF", &L.rddf, "#ffffff", nullptr},
				{"Show Path Plans (frenets)", &L.frenets, "#26c6da", nullptr}, {"Path Plan (caminho)", &L.path, "#2ecc71", nullptr},
				{"Motion Plan (MPP)", &L.plan, "#f5a623", nullptr}, {"Obst. Av. Plan", &L.avoider, "#ff69b4", nullptr},
				{"Pre-rota (offroad)", &L.offroad, "#b266ff", nullptr}, {"Waypoints (goals)", &L.goals, "#facc15", nullptr},
				{"Robot Waypoints", &L.waypoints, "#2ecc71", nullptr}}},
		{"SENSORES E LOCALIZACAO", {{"Lidar", &L.lidar, "#ff5050", layers_changed}, {"Intensidade do lidar", &L.lidar_intensity, "#cccccc", nullptr},
				{"GPS", &L.gps, "#26c6da", nullptr}, {"GPS Axis", &L.gps_axis, "#26c6da", nullptr},
				{"Fused Odometry", &L.fused, "#409cff", nullptr}, {"Global Pos (rastro)", &L.globalpos_trail, "#ffd84d", nullptr},
				{"XSENS Axis (IMU)", &L.imu_axis, "#ff4040", nullptr}, {"Particulas", &L.particles, "#b266ff", nullptr},
				{"Moving Objects", &L.objects, "#f5a623", nullptr}}},
		{"ROBO E ANOTACOES", {{"Car (modelo 3D)", &L.car_model, "#e8ecf2", nullptr}, {"Collision Model", &L.collision, "#ff5050", nullptr},
				{"Annotation", &L.annotations, "#facc15", nullptr}, {"Nomes das anotacoes", &L.labels, "#facc15", nullptr}}},
	};
	int col = 0;
	for (const Group &g : groups)
	{
		QVBoxLayout *gl = new QVBoxLayout;
		gl->setSpacing(5);
		QLabel *t = new QLabel(g.title);
		t->setObjectName("section");
		gl->addWidget(t);
		if (col == 0)
		{
			/* qual mapa: o mesmo menu Maps do navigator_gui2 (o servidor so' assina o escolhido) */
			QComboBox *maps = new QComboBox;
			const char *names[][2] = {{"Offline Map", "offline"}, {"Map (mapper)", "mapper"}, {"Map Level1", "mapper_level1"},
					{"Costs Map", "cost"}, {"Likelihood", "likelihood"}, {"Global Likelihood", "global_likelihood"},
					{"Remission Map", "remission"}, {"Moving Objects Map", "moving_objects"}, {"Road Map", "road"}};
			for (auto &n : names)
				maps->addItem(n[0], QString(n[1]));
			maps->setCurrentIndex(std::max(0, maps->findData(scene.map_choice)));
			QObject::connect(maps, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated), [this, maps](int i) {
				scene.map_choice = maps->itemData(i).toString();
				scene.layer_image = QImage();
				scene.layer_version++;
				send_layer_requests();
				refresh_views();
			});
			gl->addWidget(maps);
		}
		for (const Item &it : g.items)
		{
			QCheckBox *cb = new QCheckBox(it.label);
			cb->setChecked(*it.flag);
			cb->setStyleSheet(QString("QCheckBox::indicator:checked { background: %1; border-color: %1; }").arg(it.color));
			bool *flag = it.flag;
			std::function<void()> after = it.after;
			QObject::connect(cb, &QCheckBox::toggled, [this, flag, after](bool on) {
				*flag = on;
				if (after)
					after();
				refresh_views();
			});
			gl->addWidget(cb);
		}
		if (col == 2)
		{
			/* o lidar e' o que mais pesa na rede: id e quanto mandar */
			QHBoxLayout *lr = new QHBoxLayout;
			QLabel *lid = new QLabel("lidar id");
			lid->setObjectName("hint");
			QComboBox *ids = new QComboBox;
			for (int i = 0; i <= 15; i++)
				ids->addItem(QString::number(i));
			ids->setCurrentIndex(lidar_id);
			QComboBox *quality = new QComboBox;
			quality->addItem("leve (5k pts, 1 Hz)");
			quality->addItem("media (15k pts, 2 Hz)");
			quality->addItem("alta (40k pts, 5 Hz)");
			quality->setCurrentIndex(lidar_quality);
			QObject::connect(ids, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated), [this](int i) { lidar_id = i; send_layer_requests(); });
			QObject::connect(quality, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated), [this](int i) { lidar_quality = i; send_layer_requests(); });
			lr->addWidget(lid);
			lr->addWidget(ids);
			gl->addLayout(lr);
			gl->addWidget(quality);
		}
		if (col == 3)
		{
			QLabel *ct = new QLabel("CAMERAS");
			ct->setObjectName("section");
			gl->addSpacing(6);
			gl->addWidget(ct);
			struct CamItem { const char *label; bool *flag; };
			for (const CamItem &ci : {CamItem{"Camera no canto", &L.camera_corner}, CamItem{"Cameras no mapa", &L.camera_map}})
			{
				QCheckBox *cb = new QCheckBox(ci.label);
				cb->setChecked(*ci.flag);
				bool *flag = ci.flag;
				QObject::connect(cb, &QCheckBox::toggled, [this, flag](bool on) { *flag = on; send_layer_requests(); refresh_views(); });
				gl->addWidget(cb);
			}
			for (int side = 0; side < 2; side++)
			{
				QHBoxLayout *cr = new QHBoxLayout;
				QLabel *cl = new QLabel(side == 0 ? "esquerda/canto" : "direita");
				cl->setObjectName("hint");
				QComboBox *ids = new QComboBox;
				ids->addItem("nenhuma", -1);
				for (int i = 1; i <= 9; i++)
					ids->addItem(QString("camera %1").arg(i), i);
				int current = side == 0 ? scene.camera_left : scene.camera_right;
				ids->setCurrentIndex(std::max(0, ids->findData(current)));
				QObject::connect(ids, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated), [this, ids, side](int i) {
					(side == 0 ? scene.camera_left : scene.camera_right) = ids->itemData(i).toInt();
					send_layer_requests();
					refresh_views();
				});
				cr->addWidget(cl);
				cr->addWidget(ids);
				gl->addLayout(cr);
			}
			gl->addSpacing(6);
			QCheckBox *follow = new QCheckBox("Follow Car");
			follow->setChecked(true);
			QObject::connect(follow, &QCheckBox::toggled, [this](bool on) { follow_cb->setChecked(on); });
			gl->addWidget(follow);
		}
		gl->addStretch(1);
		lg->addLayout(gl, 0, col++);
	}
	box->setVisible(false);
	ov->addWidget(box);
	QObject::connect(layers_btn, &QToolButton::toggled, [box, overlay](bool on) { box->setVisible(on); overlay->adjustSize(); });
	overlay->adjustSize();
	return overlay;
}

QWidget *
MainWindow::build_modules_page()
{
	QWidget *page = new QWidget;
	QHBoxLayout *hp = new QHBoxLayout(page);
	hp->setContentsMargins(12, 12, 12, 12);
	hp->setSpacing(12);
	QVBoxLayout *left = new QVBoxLayout;
	proc_tree = new QTreeWidget;
	proc_tree->setHeaderLabels(QStringList() << "Modulo" << "Estado" << "PID");
	proc_tree->setColumnWidth(0, 270);
	proc_tree->setColumnWidth(1, 110);
	proc_tree->setAlternatingRowColors(true);
	QObject::connect(proc_tree, &QTreeWidget::itemChanged, [this](QTreeWidgetItem *it, int col) { on_proc_item_changed(it, col); });
	QObject::connect(proc_tree, &QTreeWidget::currentItemChanged, [this](QTreeWidgetItem *cur, QTreeWidgetItem *) { on_proc_selected(cur); });
	left->addWidget(proc_tree);
	QLabel *note = new QLabel("Marque/desmarque para ligar/desligar o modulo ou o grupo inteiro.");
	note->setObjectName("hint");
	left->addWidget(note);
	hp->addLayout(left, 2);
	QVBoxLayout *right = new QVBoxLayout;
	lbl_output = new QLabel("SAIDA  (selecione um modulo)");
	lbl_output->setObjectName("section");
	right->addWidget(lbl_output);
	output_text = new QPlainTextEdit;
	output_text->setObjectName("console");
	output_text->setReadOnly(true);
	output_text->setMaximumBlockCount(3000);
	output_text->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
	right->addWidget(output_text);
	hp->addLayout(right, 3);
	return page;
}

MapView *
MainWindow::view() const
{
	return view3d;
}

void
MainWindow::refresh_views()
{
	view()->refresh();
}

/* ============================================================================================ */
/* Comunicacao                                                                                  */
/* ============================================================================================ */

bool
MainWindow::send(const QJsonObject &payload, QJsonObject *reply, QByteArray *blob)
{
	QJsonObject r;
	if (!conn->request(payload, &r, blob))
	{
		statusBar()->showMessage("o carmen_server nao respondeu", 5000);
		return false;
	}
	if (!r["ok"].toBool())
		statusBar()->showMessage("erro: " + r["error"].toString(), 6000);
	if (reply)
		*reply = r;
	return r["ok"].toBool();
}

void
MainWindow::simple_cmd(const char *cmd, const QString &ok_message)
{
	QJsonObject req;
	req["cmd"] = cmd;
	if (send(req))
		statusBar()->showMessage(ok_message, 4000);
}

void
MainWindow::load_missions()
{
	QJsonObject req, reply;
	req["cmd"] = "missions";
	if (!send(req, &reply))
		return;
	QString current = missions_combo->currentText();
	missions_combo->clear();
	for (const QJsonValue &m : reply["missions"].toArray())
		missions_combo->addItem(m.toString());
	int idx = missions_combo->findText(current);
	missions_combo->setCurrentIndex(idx >= 0 ? idx : (missions_combo->count() > 0 ? 0 : -1));
	missions_combo->setEnabled(missions_combo->count() > 0);
}

void
MainWindow::set_map(const QJsonObject &meta, const QByteArray &blob)
{
	if (decode_map(meta, blob, &scene.map_image, &scene.map_rect))
	{
		scene.map_version++;
		refresh_views();
	}
}

void
MainWindow::request_snapshot()
{
	QJsonObject reply, req;
	QByteArray blob;
	req["cmd"] = "snapshot";
	if (!send(req, &reply, &blob))
	{
		QTimer::singleShot(2000, [this]() { request_snapshot(); });
		return;
	}
	got_snapshot = true;
	if (reply["robot"].isObject())
		on_robot(reply["robot"].toObject());
	if (reply["map"].isObject() && !blob.isEmpty())
		set_map(reply["map"].toObject(), blob);
	else
		QTimer::singleShot(1500, [this]() { request_map(true, false); });

	/* camadas guardadas no servidor: rota, caminho, goals, frenets... aparecem na hora */
	QJsonObject layers = reply["layers"].toObject();
	for (auto it = layers.begin(); it != layers.end(); ++it)
		handle_topic(it.key().toStdString(), it.value().toObject(), QByteArray());

	const char *simple[] = {"pose", "proc", "goal", "behavior", "car", "nav", "mission"};
	for (const char *t : simple)
		if (reply[t].isObject())
			handle_topic(t, reply[t].toObject(), QByteArray());
	if (reply["annotation_file"].isArray())
	{
		file_annotations = reply["annotation_file"].toArray();
		merge_annotations();
	}
	if (follow_cb->isChecked())
		view()->zoom_on_robot();
	load_missions();
	statusBar()->showMessage("Conectado a " + reply["host"].toString(), 4000);
	if (!car_model_requested)
		QTimer::singleShot(300, [this]() { load_car_model(); });
	QTimer::singleShot(500, [this]() { send_layer_requests(); });	// reconectou: pede de novo o que estava ligado
}

/* O .obj do carro (o mesmo do viewer_3D) vem do servidor uma vez, comprimido. Sem ele, fica a caixa. */
void
MainWindow::load_car_model()
{
	car_model_requested = true;
	QJsonObject req, reply;
	QByteArray blob;
	req["cmd"] = "car_model";
	QJsonObject r;
	if (!conn->request(req, &r, &blob) || !r["ok"].toBool() || blob.isEmpty())
	{
		statusBar()->showMessage("modelo 3D do carro indisponivel: " + r["error"].toString(), 5000);
		return;
	}
	QJsonObject m = r["model"].toObject();
	QJsonArray sz = m["size"].toArray();
	double size[3] = {sz.at(0).toDouble(1.0), sz.at(1).toDouble(1.0), sz.at(2).toDouble(1.0)};
	/* zlib puro; o tamanho original vem em "bytes" */
	QByteArray obj((int) m["bytes"].toDouble(), Qt::Uninitialized);
	uLongf obj_size = (uLongf) obj.size();
	if (obj.isEmpty() || uncompress((Bytef *) obj.data(), &obj_size, (const Bytef *) blob.constData(), blob.size()) != Z_OK)
	{
		statusBar()->showMessage("modelo 3D do carro corrompido", 5000);
		return;
	}
	obj.resize((int) obj_size);
	view3d->set_car_model(obj, size, m["yaw"].toDouble());
	if (view3d->has_car_model())
		statusBar()->showMessage("Modelo 3D do carro: " + m["file"].toString(), 4000);
}

void
MainWindow::request_map(bool refresh, bool verbose)
{
	last_map_request = now_seconds();
	QJsonObject req, reply;
	QByteArray blob;
	req["cmd"] = "get_map";
	req["refresh"] = refresh ? 1 : 0;
	if (send(req, &reply, &blob) && reply["map"].isObject() && !blob.isEmpty())
	{
		set_map(reply["map"].toObject(), blob);
		if (verbose)
			statusBar()->showMessage("Mapa recarregado", 3000);
	}
}

void
MainWindow::toggle_full_map(bool on)
{
	if (!on)
	{
		scene.show_full_map = false;
		refresh_views();
		return;
	}
	follow_cb->setChecked(false);	// para a vista ficar no mapa inteiro
	if (!scene.full_image.isNull())
	{
		scene.show_full_map = true;
		view()->fit_rect(scene.full_rect);
		return;
	}
	statusBar()->showMessage("Montando o mapa completo no robo...");
	QApplication::setOverrideCursor(Qt::WaitCursor);
	QJsonObject req, reply;
	QByteArray blob;
	req["cmd"] = "full_map";
	req["max_side"] = 3000;
	bool ok = send(req, &reply, &blob) && reply["map"].isObject() && !blob.isEmpty();
	QApplication::restoreOverrideCursor();
	QJsonObject meta = reply["map"].toObject();
	if (!ok || !decode_map(meta, blob, &scene.full_image, &scene.full_rect))
	{
		full_btn->setChecked(false);
		return;
	}
	scene.full_version++;
	scene.show_full_map = true;
	view()->fit_rect(scene.full_rect);
	statusBar()->showMessage(QString("Mapa completo: %1 blocos, %2 x %3 celulas de %4 m")
			.arg(meta["blocks"].toInt()).arg(meta["x_size"].toInt()).arg(meta["y_size"].toInt())
			.arg(meta["resolution"].toDouble(), 0, 'f', 2), 6000);
}

void
MainWindow::tick()
{
	bool changed = false;
	for (const Frames &f : conn->poll())
	{
		handle_topic(f.topic, f.json, f.blob);
		changed = true;
	}
	if (changed)
		refresh_views();
}

void
MainWindow::handle_topic(const std::string &t, const QJsonObject &d, const QByteArray &blob)
{
	RobotState &r = scene.robot;
	if (t == "hello")
	{
		last_hello = now_seconds();
		if (!got_snapshot)
			request_snapshot();
	}
	else if (t == "pose") on_pose(d);
	else if (t == "map" && !blob.isEmpty()) set_map(d, blob);
	else if (t == "map2" && !blob.isEmpty())
	{
		QString src = d["source"].toString();
		if (src == scene.map_choice)
		{
			/* mapas de ocupacao sobrepostos: so' os obstaculos, em azul suave, por cima do offline cinza */
			bool occupancy = (src == "mapper" || src == "mapper_level1" || src == "moving_objects" || src == "cost");
			bool colored = scene.layers.map_overlay && occupancy;
			bool ok = colored ? decode_map_overlay(d, blob, QColor(90, 175, 255), &scene.layer_image, &scene.layer_rect)
					: decode_map(d, blob, &scene.layer_image, &scene.layer_rect);
			if (ok)
			{
				scene.layer_colored = colored;
				scene.layer_version++;
			}
		}
	}
	else if (t == "lidar" && scene.layers.lidar) set_lidar(d, blob);
	else if (t == "watchdog")
	{
		if (d["state"].toString() == "stopped")
			statusBar()->showMessage(QString("WATCHDOG: o robo ficou %1 s sem ouvir o app e recebeu STOP").arg(d["silence"].toDouble(), 0, 'f', 1), 15000);
		else
			statusBar()->showMessage("Conexao com o robo de volta (watchdog rearmado). De GO de novo se quiser seguir.", 10000);
	}
	else if (t == "camera" && !blob.isEmpty())
	{
		Scene::Camera &cam = scene.cameras[d["id"].toInt()];
		if (cam.image.loadFromData(blob, "JPG"))
		{
			cam.version++;
			cam.t = now_seconds();
		}
	}
	else if (t == "path") { scene.path = json_to_polygon(d["path"].toArray()); scene.goals = json_to_polygon(d["goals"].toArray()); }
	else if (t == "plan") scene.plan = json_to_polygon(d["path"].toArray());
	else if (t == "avoider") scene.avoider = json_to_polygon(d["path"].toArray());
	else if (t == "route") { scene.route = json_to_polygon(d["path"].toArray()); r.t_route = now_seconds(); }
	else if (t == "offroad") scene.offroad = json_to_polygon(d["path"].toArray());
	else if (t == "lanes")
	{
		scene.lanes.clear();
		for (const QJsonValue &v : d["lanes"].toArray())
			scene.lanes.push_back(json_to_polygon(v.toArray()));
	}
	else if (t == "frenets")
	{
		scene.frenets.clear();
		for (const QJsonValue &v : d["paths"].toArray())
			scene.frenets.push_back(json_to_polygon(v.toArray()));
		scene.frenet_selected = d["selected"].toInt(-1);
		scene.frenet_collision.clear();
		for (const QJsonValue &v : d["collision"].toArray())
			scene.frenet_collision.push_back(v.toDouble(-1));
		scene.rddf = json_to_polygon(d["rddf"].toArray());
		r.frenet_paths = (int) scene.frenets.size();
		r.frenet_selected = scene.frenet_selected;
		r.t_frenet = now_seconds();
	}
	else if (t == "annotations") { live_annotations = d["items"].toArray(); merge_annotations(); }
	else if (t == "goal")
	{
		scene.final_goal = QPointF(d["x"].toDouble(), d["y"].toDouble());
		scene.final_goal_theta = d["theta"].toDouble();
		scene.have_final_goal = true;
	}
	else if (t == "objects")
	{
		scene.objects.clear();
		for (const QJsonValue &v : d["items"].toArray())
		{
			QJsonObject o = v.toObject();
			scene.objects.push_back({o["x"].toDouble(), o["y"].toDouble(), o["theta"].toDouble(), o["l"].toDouble(),
					o["w"].toDouble(), o["h"].toDouble(), o["v"].toDouble(), o["id"].toInt()});
		}
	}
	else if (t == "particles")
	{
		scene.particles.clear();
		for (const QJsonValue &v : d["items"].toArray())
		{
			QJsonArray a = v.toArray();
			scene.particles.push_back(QPointF(a[0].toDouble(), a[1].toDouble()));
		}
	}
	else if (t == "gps")
	{
		scene.gps = QPointF(d["x"].toDouble(), d["y"].toDouble());
		scene.have_gps = true;
		r.gps_quality = d["quality"].toInt();
		r.gps_sats = d["sats"].toInt();
		r.gps_hdop = d["hdop"].toDouble();
		r.t_gps = now_seconds();
	}
	else if (t == "fused")
	{
		scene.fused = QPointF(d["x"].toDouble(), d["y"].toDouble());
		scene.fused_theta = d["theta"].toDouble();
		scene.have_fused = true;
	}
	else if (t == "imu")
	{
		r.roll = d["roll"].toDouble();
		r.pitch = d["pitch"].toDouble();
		r.t_imu = now_seconds();
	}
	else if (t == "behavior") on_behavior(d);
	else if (t == "proc") on_proc(d);
	else if (t == "robot") on_robot(d);
	else if (t == "car") on_car(d);
	else if (t == "nav") { r.autonomous = d["autonomous"].toInt(); r.goal_distance = d["goal_distance"].toDouble(-1); }
	else if (t == "mission") { r.mission_state = d["state"].toString("-"); r.mission_id = d["id"].toString(); }
	else if (t == "odom") r.phi = d["phi"].toDouble();
	else if (t == "cmd") { r.v_cmd = d["v"].toDouble(); r.phi_cmd = d["phi"].toDouble(); r.t_cmd = now_seconds(); }
	else if (t == "out" && d["module"].toString() == selected_module)
	{
		QScrollBar *sb = output_text->verticalScrollBar();
		bool at_end = sb->value() >= sb->maximum() - 2;
		output_text->moveCursor(QTextCursor::End);
		output_text->insertPlainText(d["text"].toString());
		if (at_end)
			sb->setValue(sb->maximum());
	}
}

void
MainWindow::update_status()
{
	QString host = QString::fromStdString(conn->host);
	bool ok = last_hello > 0 && now_seconds() - last_hello < 3.0;
	lbl_conn->setText(QString("<span style='color:%1'>&#9679;</span>&nbsp; %2&nbsp;&nbsp;<span style='color:#848ea0'>%3</span>")
			.arg(ok ? "#2ecc71" : "#f05252").arg(ok ? "conectado" : "sem resposta").arg(host));
	if (!ok)
		got_snapshot = false;
	/* sem CAN (simulador): marcha deduzida do movimento, como no viewer_3D_dash */
	RobotState &r = scene.robot;
	double ref = fabs(r.v) > 0.05 ? r.v : r.v_cmd;
	if (ref > 0.05) { r.gear_estimated = "D"; r.last_moving = now_seconds(); }
	else if (ref < -0.05) { r.gear_estimated = "R"; r.last_moving = now_seconds(); }
	else if (now_seconds() - r.last_moving > 1.5) r.gear_estimated = "N";
}

/* ============================================================================================ */
/* Dados                                                                                        */
/* ============================================================================================ */

void
MainWindow::on_pose(const QJsonObject &d)
{
	RobotState &r = scene.robot;
	double v = d["v"].toDouble(), t = d["t"].toDouble();
	if (fabs(v) > 100.0)	// mensagem corrompida (mesmo filtro do viewer_3D_dash)
		return;
	if (r.last_v_t > 0 && t - r.last_v_t > 0.001)
		r.a_long = 0.7 * r.a_long + 0.3 * (v - r.last_v) / (t - r.last_v_t);
	r.last_v = v;
	r.last_v_t = t;
	r.x = d["x"].toDouble();
	r.y = d["y"].toDouble();
	r.theta = d["theta"].toDouble();
	r.v = v;
	r.phi = d["phi"].toDouble();
	r.converged = d["converged"].toInt() != 0;
	r.have_pose = true;
	r.t_pose = now_seconds();
	if (r.axle > 0.01)
		r.a_lat = v * v * tan(r.phi) / r.axle;
	if (scene.trail.empty() || hypot(scene.trail.back().x() - r.x, scene.trail.back().y() - r.y) > 0.1)
	{
		scene.trail.push_back(QPointF(r.x, r.y));
		while (scene.trail.size() > 3000)
			scene.trail.pop_front();
	}
	r.speed_history.push_back((float) v);
	while (r.speed_history.size() > 240)
		r.speed_history.pop_front();
	if (follow_cb->isChecked())
		view()->follow_robot();
	if (got_snapshot && !scene.contains_map(r.x, r.y, 5.0) && now_seconds() - last_map_request > 5.0)
		request_map(true, false);
}

void
MainWindow::on_robot(const QJsonObject &d)
{
	RobotState &r = scene.robot;
	r.length = d["length"].toDouble(r.length);
	r.width = d["width"].toDouble(r.width);
	r.axle = d["axle"].toDouble(r.axle);
	r.rear_overhang = d["rear_overhang"].toDouble(r.rear_overhang);
	r.max_v = d["max_v"].toDouble(r.max_v);
	r.max_phi = d["max_phi"].toDouble(r.max_phi);
	r.collision.clear();
	for (const QJsonValue &v : d["collision"].toArray())
	{
		QJsonArray a = v.toArray();
		r.collision.push_back(QVector3D((float) a[0].toDouble(), (float) a[1].toDouble(), (float) a[2].toDouble()));
	}
}

void
MainWindow::send_behavior()
{
	QJsonObject req;
	req["cmd"] = "behavior";
	req["task"] = task_combo->currentIndex();
	req["algorithm"] = algorithm_combo->currentIndex();	// a ordem dos itens e' a do enum do behavior_selector
	if (send(req))
		statusBar()->showMessage("Tarefa: " + task_combo->currentText() + "   planejador: " + algorithm_combo->currentText(), 5000);
}

void
MainWindow::on_behavior(const QJsonObject &d)
{
	/* acompanha o que o behavior_selector esta' usando (mudou por outra interface, missao...) */
	int task = d["task_code"].toInt(-1), algorithm = d["algorithm"].toInt(-1);
	if (task >= 0 && task < task_combo->count() && task != task_combo->currentIndex())
		task_combo->setCurrentIndex(task);
	if (algorithm >= 0 && algorithm < algorithm_combo->count() && algorithm != algorithm_combo->currentIndex())
		algorithm_combo->setCurrentIndex(algorithm);
	RobotState &r = scene.robot;
	r.behavior = d["state"].toString("-");
	r.task = d["task"].toString("-");
	r.route_state = route_state_name(d["route_planner_state"].toInt(-1));
	r.t_behavior = now_seconds();
}

void
MainWindow::on_car(const QJsonObject &d)
{
	RobotState &r = scene.robot;
	r.throttle = d["throttle"].toDouble();
	r.brake = d["brake"].toDouble();
	r.steering_pct = d["steering_pct"].toDouble();
	r.gear = d["gear"].toString();
	r.engine = d["engine"].toInt();
	r.parking_brake = d["parking_brake"].toInt();
	r.turn_signal = d["turn_signal"].toInt();
	r.headlights = d["headlights"].toInt();
	r.horn = d["horn"].toInt();
	r.t_car = now_seconds();
}

void
MainWindow::on_proc(const QJsonObject &d)
{
	updating_tree = true;
	for (const QJsonValue &v : d["processes"].toArray())
	{
		QJsonObject p = v.toObject();
		QString group = p["group"].toString(), module = p["module"].toString();
		QTreeWidgetItem *g = group_items.value(group, nullptr);
		if (!g)
		{
			g = new QTreeWidgetItem(proc_tree, QStringList() << group << "" << "");
			g->setFlags(g->flags() | Qt::ItemIsUserCheckable);
			g->setData(0, Qt::UserRole, "group");
			QFont f = g->font(0);
			f.setBold(true);
			g->setFont(0, f);
			g->setExpanded(true);
			group_items[group] = g;
		}
		QTreeWidgetItem *it = proc_items.value(module, nullptr);
		if (!it)
		{
			it = new QTreeWidgetItem(g, QStringList() << module << "" << "");
			it->setFlags(it->flags() | Qt::ItemIsUserCheckable);
			it->setData(0, Qt::UserRole, "module");
			proc_items[module] = it;
		}
		bool requested = p["requested"].toInt() != 0;
		bool running = p["active"].toInt() != 0 && p["pid"].toInt() > 0;
		it->setCheckState(0, requested ? Qt::Checked : Qt::Unchecked);
		it->setText(1, QString::fromUtf8(running ? "\xe2\x97\x8f rodando" : (requested ? "\xe2\x97\x8f iniciando" : "\xe2\x97\x8b parado")));
		it->setForeground(1, QBrush(running ? pal::green : (requested ? pal::orange : pal::dim)));
		it->setText(2, p["pid"].toInt() > 0 ? QString::number(p["pid"].toInt()) : "");
	}
	for (QTreeWidgetItem *g : group_items)
	{
		int on = 0, n = g->childCount();
		for (int i = 0; i < n; i++)
			on += (g->child(i)->checkState(0) == Qt::Checked);
		g->setCheckState(0, on == 0 ? Qt::Unchecked : (on == n ? Qt::Checked : Qt::PartiallyChecked));
		g->setText(1, QString("%1/%2").arg(on).arg(n));
		g->setForeground(1, QBrush(pal::dim));
	}
	updating_tree = false;
}

void
MainWindow::on_proc_item_changed(QTreeWidgetItem *item, int column)
{
	if (updating_tree || column != 0 || item->checkState(0) == Qt::PartiallyChecked)
		return;
	QJsonObject req;
	req["cmd"] = item->data(0, Qt::UserRole).toString();
	req["name"] = item->text(0);
	req["state"] = item->checkState(0) == Qt::Checked ? 1 : 0;
	send(req);
}

void
MainWindow::on_proc_selected(QTreeWidgetItem *current)
{
	if (!current || current->data(0, Qt::UserRole).toString() != "module")
		return;
	selected_module = current->text(0);
	lbl_output->setText("SAIDA  " + selected_module);
	output_text->clear();
	QJsonObject req, reply;
	req["cmd"] = "get_output";
	req["name"] = selected_module;
	if (send(req, &reply))
	{
		QStringList lines;
		for (const QJsonValue &l : reply["lines"].toArray())
			lines << l.toString();
		output_text->setPlainText(lines.join("\n") + "\n");
		output_text->moveCursor(QTextCursor::End);
	}
}

/* --- anotacoes: arquivo (todas) + ao vivo, sem repetir --------------------------------------- */

void
MainWindow::merge_annotations()
{
	std::vector<Annotation> all;
	for (const QJsonArray *src : {&file_annotations, &live_annotations})
		for (const QJsonValue &v : *src)
		{
			QJsonObject o = v.toObject();
			Annotation a{o["text"].toString(), o["x"].toDouble(), o["y"].toDouble(), o["theta"].toDouble(), o["type"].toInt()};
			if (a.text.isEmpty())	// a mensagem do RDDF as vezes vem com um item vazio em (0, 0)
				continue;
			/* o arquivo e a mensagem trazem a mesma anotacao com coordenadas um pouco diferentes
			   (1029,98 x 1029,63): lugar e' unico pelo nome; o resto, mesmo nome a menos de 3 m */
			bool place = a.text.startsWith("RDDF_PLACE");
			bool dup = false;
			for (const Annotation &b : all)
				if (b.text == a.text && (place || hypot(b.x - a.x, b.y - a.y) < 3.0))
				{
					dup = true;
					break;
				}
			if (!dup)
				all.push_back(a);
		}
	scene.annotations = all;
	update_places();
	refresh_views();
}

void
MainWindow::update_places()
{
	std::vector<Place> list;
	for (const Annotation &a : scene.annotations)
		list.push_back({short_annotation_name(a.text), a.x, a.y, a.theta});
	/* lugares (PLACE_*) primeiro, depois o resto, cada grupo em ordem alfabetica */
	std::sort(list.begin(), list.end(), [](const Place &a, const Place &b) {
		bool pa = a.name.startsWith("PLACE"), pb = b.name.startsWith("PLACE");
		return (pa != pb) ? pa : a.name < b.name;
	});
	QStringList names;
	for (const Place &p : list)
		names << p.name;
	if (names == place_names)	// so' reconstroi quando muda (nao perde a selecao)
		return;
	place_names = names;
	places = list;
	QString current = places_combo->currentText();
	places_combo->blockSignals(true);
	places_combo->clear();
	places_combo->addItem("Ir para um lugar...");
	for (const Place &p : places)
		places_combo->addItem(p.name);
	int idx = places_combo->findText(current);
	places_combo->setCurrentIndex(idx > 0 ? idx : 0);
	places_combo->blockSignals(false);
}

void
MainWindow::on_place_selected(int index)
{
	if (index <= 0 || index - 1 >= (int) places.size())
		return;
	const Place &p = places[index - 1];
	follow_cb->setChecked(false);	// senao a vista volta pro robo na proxima pose
	scene.highlight = QPointF(p.x, p.y);
	scene.have_highlight = true;
	view()->look_at(p.x, p.y);
	statusBar()->showMessage(QString("%1  (x %2  y %3)  -  \"Destino aqui\" marca o destino neste lugar")
			.arg(p.name).arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2), 8000);
}

void
MainWindow::place_as_goal()
{
	int index = places_combo->currentIndex();
	if (index <= 0 || index - 1 >= (int) places.size())
	{
		statusBar()->showMessage("Escolha um lugar na lista primeiro", 4000);
		return;
	}
	const Place &p = places[index - 1];
	QJsonObject req;
	req["cmd"] = "final_goal";
	req["x"] = p.x;
	req["y"] = p.y;
	req["theta"] = p.theta;
	if (send(req))
	{
		scene.final_goal = QPointF(p.x, p.y);
		scene.final_goal_theta = p.theta;
		scene.have_final_goal = true;
		refresh_views();
		statusBar()->showMessage("Destino: " + p.name + "  -  \"Calcular rota\" e depois GO (\"Voltar ao robo\" para acompanhar)", 8000);
	}
}

/* --- clique no mapa (2D ou 3D) ------------------------------------------------------------- */

void
MainWindow::reset_click()
{
	scene.preview.clear();
	int mode = mode_group->checkedId();
	view3d->set_pick_mode(mode == NAVIGATE ? 0 : mode);
	refresh_views();
	if (mode != NAVIGATE)
		statusBar()->showMessage("Clique onde o robo fica e ARRASTE para onde ele deve apontar (solte para enviar)");
	else
		statusBar()->clearMessage();
}

void
MainWindow::on_pick(double x, double y, double theta)
{
	int mode = mode_group->checkedId();
	if (mode == NAVIGATE)
		return;
	static const char *cmds[] = {"", "initial_pose", "final_goal", "route_to"};
	QJsonObject req;
	req["cmd"] = cmds[mode];
	req["x"] = x;
	req["y"] = y;
	req["theta"] = theta;
	if (send(req))
	{
		statusBar()->showMessage(QString("%1 enviado: x=%2 y=%3 th=%4%5").arg(cmds[mode])
				.arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(theta * 180.0 / M_PI, 0, 'f', 1).arg(QChar(0x00B0)), 6000);
		if (mode == FINAL_GOAL)
		{
			scene.final_goal = QPointF(x, y);
			scene.final_goal_theta = theta;
			scene.have_final_goal = true;
		}
		if (mode == INITIAL_POSE)
		{
			scene.trail.clear();
			QTimer::singleShot(2000, [this]() { request_map(true, false); });
		}
	}
	mode_group->button(NAVIGATE)->setChecked(true);	// volta ao modo navegar depois de um envio
	view3d->set_pick_mode(0);
	refresh_views();
}

void
MainWindow::send_layer_requests()
{
	if (last_hello <= 0)
		return;
	QJsonObject m;
	m["cmd"] = "map_layer";
	m["source"] = (scene.layers.map && scene.map_choice != "offline") ? scene.map_choice : QString("none");
	m["period"] = 1.0;
	m["max_side"] = 600;
	QJsonObject reply;
	conn->request(m, &reply, nullptr);

	static const int points[] = {5000, 15000, 40000};
	static const double hz[] = {1.0, 2.0, 5.0};
	QJsonObject l;
	l["cmd"] = "lidar";
	l["id"] = scene.layers.lidar ? lidar_id : -1;
	l["hz"] = hz[lidar_quality];
	l["max_points"] = points[lidar_quality];
	QJsonObject lr;
	if (conn->request(l, &lr, nullptr) && !lr["ok"].toBool() && scene.layers.lidar)
		statusBar()->showMessage("lidar: " + lr["error"].toString(), 6000);

	/* cameras: a da esquerda (canto e mapa) e a da direita (mapa); o resto desliga */
	std::vector<int> want;
	if ((scene.layers.camera_corner || scene.layers.camera_map) && scene.camera_left >= 0)
		want.push_back(scene.camera_left);
	if (scene.layers.camera_map && scene.camera_right >= 0 && scene.camera_right != scene.camera_left)
		want.push_back(scene.camera_right);
	for (int id : cameras_on)
		if (std::find(want.begin(), want.end(), id) == want.end())
		{
			QJsonObject off, r;
			off["cmd"] = "camera";
			off["id"] = id;
			off["on"] = 0;
			conn->request(off, &r, nullptr);
		}
	for (int id : want)
	{
		QJsonObject on, r;
		on["cmd"] = "camera";
		on["id"] = id;
		on["on"] = 1;
		on["hz"] = 5.0;
		on["width"] = 480;
		on["quality"] = 60;
		conn->request(on, &r, nullptr);
	}
	cameras_on = want;
}

void
MainWindow::set_lidar(const QJsonObject &meta, const QByteArray &blob)
{
	int n = meta["n"].toInt();
	if (n <= 0)
	{
		scene.lidar_xyz.clear();
		return;
	}
	std::vector<unsigned char> raw((size_t) n * 7);
	uLongf size = (uLongf) raw.size();
	if (uncompress(raw.data(), &size, (const Bytef *) blob.constData(), blob.size()) != Z_OK || size != raw.size())
		return;
	scene.lidar_xyz.resize((size_t) n * 3);
	scene.lidar_intensity.resize(n);
	for (int i = 0; i < n; i++)
	{
		const unsigned char *p = raw.data() + (size_t) i * 7;
		short v[3];
		memcpy(v, p, 6);
		scene.lidar_xyz[i * 3] = v[0] * 0.01f;
		scene.lidar_xyz[i * 3 + 1] = v[1] * 0.01f;
		scene.lidar_xyz[i * 3 + 2] = v[2] * 0.01f;
		scene.lidar_intensity[i] = p[6];
	}
	scene.lidar_x = meta["x"].toDouble();
	scene.lidar_y = meta["y"].toDouble();
	scene.lidar_theta = meta["theta"].toDouble();
	scene.have_lidar = true;
}

/* --- modo de teste ------------------------------------------------------------------------- */

void
MainWindow::test_select_place(int i)
{
	if (i < places_combo->count())
	{
		places_combo->setCurrentIndex(i);
		on_place_selected(i);
	}
}

void
MainWindow::test_full_map()
{
	full_btn->setChecked(true);
}

void
MainWindow::test_top_view()
{
	view3d->top_view();
}

void
MainWindow::test_close()
{
	follow_cb->setChecked(true);
	view3d->set_orbit(3.0, 0.25);
	view3d->follow_robot();
}

void
MainWindow::test_options()
{
	options_btn->setChecked(true);
}

void
MainWindow::test_map(const QString &source, bool overlay)
{
	scene.map_choice = source;
	scene.layers.map_overlay = overlay;
	scene.layers.collision = true;
	scene.layers.imu_axis = true;
	scene.layers.globalpos_trail = true;
	send_layer_requests();
}

void
MainWindow::test_pick()
{
	mode_group->button(FINAL_GOAL)->setChecked(true);
	reset_click();
}

void
MainWindow::test_cameras()
{
	scene.layers.camera_map = true;
	send_layer_requests();
}
