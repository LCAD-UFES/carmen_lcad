import sys
import os
import glob
import subprocess
import shutil
import re
import math
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QFileDialog, QLabel, 
                             QGraphicsView, QGraphicsScene, QGraphicsPixmapItem,
                             QGraphicsPathItem, QGraphicsEllipseItem, QSlider, 
                             QStatusBar, QMessageBox, QProgressDialog, QDoubleSpinBox,
                             QComboBox, QSpinBox, QListWidget, QInputDialog, 
                             QGraphicsLineItem, QGraphicsTextItem, QGraphicsItem, 
                             QGraphicsPolygonItem, QStyle, QPlainTextEdit,
                             QStackedWidget)
from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen, QPainterPath, QBrush, QPolygonF, QFont

# --- CONFIGURAÇÃO ---
EXE_READ = os.path.expanduser("./carmen_read")
EXE_SAVE = os.path.expanduser("./carmen_save")
EXE_ROAD_NETWORK = os.path.expanduser("~/carmen_lcad/bin/road_network_generator") 

# =============================================================================
# ALGORITMO DE FILTRO ESPACIAL
# =============================================================================
def dynamic_filter(points, spacing_m):
    if len(points) < 3: return points
    kept = [points[0]]
    for i in range(1, len(points) - 1):
        p = points[i]
        last = kept[-1]
        dist = math.hypot(p[0] - last[0], p[1] - last[1])
        if dist < 0.05: continue
        if spacing_m <= 0.05:
            kept.append(p)
            continue
        next_p = points[i+1]
        ang1 = math.atan2(p[1] - last[1], p[0] - last[0])
        ang2 = math.atan2(next_p[1] - p[1], next_p[0] - p[0])
        ang_diff = abs(math.atan2(math.sin(ang2-ang1), math.cos(ang2-ang1)))
        if dist >= spacing_m or ang_diff > math.radians(5):
            kept.append(p)
    if kept[-1] != points[-1]: kept.append(points[-1])
    return kept

# =============================================================================
# DADOS DO MAPA
# =============================================================================
class CarmenMapData:
    def __init__(self, filepath):
        self.filepath = filepath
        self.filename = os.path.basename(filepath)
        self.grid = None
        self.rows = 0
        self.cols = 0
        self.res = 0.02 
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.text_header = []
        self.is_binary_c = False
        self.is_modified = False
        self.load()

    def parse_filename_coords(self):
        try:
            name_clean = os.path.splitext(self.filename)[0]
            numbers = re.findall(r'-?\d+', name_clean)
            if len(numbers) >= 2:
                raw_x = float(numbers[-2])
                raw_y = float(numbers[-1])
                if self.offset_x == 0.0 and self.offset_y == 0.0:
                    self.offset_x = raw_x
                    self.offset_y = raw_y
        except Exception as e:
            print(f"Erro ao ler nome {self.filename}: {e}")

    def load(self):
        if not self.try_load_text():
            self.load_via_c_bridge()
        if self.offset_x == 0.0 and self.offset_y == 0.0:
            self.parse_filename_coords()

    def try_load_text(self):
        try:
            with open(self.filepath, 'rb') as f:
                if b'\x00' in f.read(1024): return False 
            with open(self.filepath, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
            raw_vals = []
            header_done = False
            for line in lines:
                clean = line.strip()
                if not header_done:
                    self.text_header.append(line)
                    parts = clean.split()
                    if len(parts) >= 2:
                        k, v = parts[0], parts[1]
                        if 'res' in k: self.res = float(v)
                        elif 'offset_x' in k: self.offset_x = float(v)
                        elif 'offset_y' in k: self.offset_y = float(v)
                        elif 'size_x' in k: self.cols = int(v)
                        elif 'size_y' in k: self.rows = int(v)
                    if clean in ['data', 'map', 'grid']: header_done = True
                else:
                    try: raw_vals.extend([float(x) for x in clean.split()])
                    except: pass
            if len(raw_vals) > 0 and self.rows > 0:
                self.grid = np.array(raw_vals[:self.rows*self.cols], dtype=np.float32).reshape((self.cols, self.rows), order='C')
                return True
        except: pass
        return False

    def load_via_c_bridge(self):
        if not os.path.exists(EXE_READ): return
        try:
            result = subprocess.run([EXE_READ, self.filepath], capture_output=True, text=True)
            lines = result.stdout.splitlines()
            data_started = False
            raw_vals = []
            for line in lines:
                if line == "BRIDGE_DATA_START":
                    data_started = True
                    continue
                if not data_started:
                    parts = line.split()
                    if len(parts) >= 2:
                        if parts[0] == 'cols': self.cols = int(parts[1])
                        elif parts[0] == 'rows': self.rows = int(parts[1])
                        elif parts[0] == 'res': self.res = float(parts[1])
                else:
                    raw_vals.extend([float(x) for x in line.split()])
            if len(raw_vals) > 0:
                self.grid = np.array(raw_vals, dtype=np.float32).reshape((self.cols, self.rows), order='C')
                self.is_binary_c = True
        except Exception as e: print(f"Erro na ponte: {e}")

    def save(self, new_path=None):
        if self.grid is None: return False
        target_path = new_path if new_path else self.filepath
        try:
            if self.is_binary_c:
                if not os.path.exists(EXE_SAVE): return False
                import tempfile
                with tempfile.NamedTemporaryFile(mode='w+', delete=False) as tmp:
                    tmp_name = tmp.name
                    np.savetxt(tmp, self.grid.flatten(order='C'), fmt='%.4g', delimiter=' ')
                cmd = [EXE_SAVE, target_path, str(self.cols), str(self.rows), str(self.res)]
                with open(tmp_name, 'r') as f_in:
                    process = subprocess.run(cmd, stdin=f_in, capture_output=True, text=True)
                os.remove(tmp_name)
                return process.returncode == 0
            else:
                with open(target_path, 'w', encoding='utf-8') as f:
                    for line in self.text_header: f.write(line)
                    np.savetxt(f, self.grid.flatten(order='C'), fmt='%.4g', delimiter=' ')
                return True
        except Exception as e:
            if 'tmp_name' in locals() and os.path.exists(tmp_name): os.remove(tmp_name)
            return False

# =============================================================================
# ITENS GRÁFICOS E NÓS
# =============================================================================
def _redistribute_points(rows, p_start, p_end):
    """Redistribui os pontos internos de um grupo uniformemente entre p_start e p_end.

    Nao cria nem deleta pontos — apenas reposiciona os existentes em linha reta
    entre os dois extremos, com espacamento proporcional as distancias originais.
    """
    n = len(rows)
    if n <= 1:
        if n == 1:
            rows[0][0] = p_start[0]; rows[0][1] = p_start[1]
        return
    if n == 2:
        rows[0][0] = p_start[0]; rows[0][1] = p_start[1]
        rows[-1][0] = p_end[0];  rows[-1][1] = p_end[1]
        return

    # Distancias acumuladas originais (para manter proporcao relativa)
    dists = [0.0]
    for i in range(1, n):
        dx = rows[i][0] - rows[i-1][0]
        dy = rows[i][1] - rows[i-1][1]
        dists.append(dists[-1] + math.hypot(dx, dy))
    total = dists[-1]

    for i, row in enumerate(rows):
        t = (dists[i] / total) if total > 1e-9 else (i / (n - 1))
        row[0] = p_start[0] + t * (p_end[0] - p_start[0])
        row[1] = p_start[1] + t * (p_end[1] - p_start[1])


def _hermite_blend(rows_a, rows_b, n_blend):
    """Faz uma interpolacao Hermite (curva-S) entre o fim do grupo A e o inicio do grupo B.

    Pega os ultimos n_blend pontos de rows_a e os primeiros n_blend de rows_b
    e os redistribui ao longo de uma curva cubica de Hermite, eliminando o L
    na juncao sem criar pontos novos.
    n_blend e limitado ao menor dos dois grupos.
    """
    n_blend = min(n_blend, len(rows_a), len(rows_b))
    if n_blend < 2:
        return

    # Pontos de controle: posicoes dos extremos e tangentes estimadas
    p0 = [rows_a[-n_blend][0], rows_a[-n_blend][1]]  # inicio da zona de blend em A
    p1 = [rows_b[n_blend - 1][0], rows_b[n_blend - 1][1]]  # fim da zona de blend em B

    # Tangentes: direcao do segmento imediatamente antes/depois da zona
    if len(rows_a) > n_blend:
        ta = [rows_a[-n_blend][0] - rows_a[-n_blend - 1][0],
              rows_a[-n_blend][1] - rows_a[-n_blend - 1][1]]
    else:
        ta = [p1[0] - p0[0], p1[1] - p0[1]]

    if len(rows_b) > n_blend:
        tb = [rows_b[n_blend][0] - rows_b[n_blend - 1][0],
              rows_b[n_blend][1] - rows_b[n_blend - 1][1]]
    else:
        tb = [p1[0] - p0[0], p1[1] - p0[1]]

    # Normaliza e escala as tangentes pelo comprimento total da zona
    total_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    for tang in (ta, tb):
        l = math.hypot(tang[0], tang[1])
        if l > 1e-9:
            tang[0] = tang[0] / l * total_len
            tang[1] = tang[1] / l * total_len

    # Todos os pontos da zona: ultimos de A + primeiros de B
    zone = list(rows_a[-n_blend:]) + list(rows_b[:n_blend])
    total_pts = len(zone)

    for k, row in enumerate(zone):
        t = k / (total_pts - 1) if total_pts > 1 else 0.0
        # Polinomios de Hermite
        h00 =  2*t**3 - 3*t**2 + 1
        h10 =    t**3 - 2*t**2 + t
        h01 = -2*t**3 + 3*t**2
        h11 =    t**3 -   t**2
        row[0] = h00*p0[0] + h10*ta[0] + h01*p1[0] + h11*tb[0]
        row[1] = h00*p0[1] + h10*ta[1] + h01*p1[1] + h11*tb[1]


class RDDFNode(QGraphicsEllipseItem):
    """Ponto interativo do RDDF.

    Ao arrastar:
    1. Move todos os pontos do grupo pelo offset do drag.
    2. Redistribui os pontos internos uniformemente entre inicio e fim do grupo
       (evita empilhamento ao mover reta para tras/frente).
    3. Aplica interpolacao Hermite nas juncoes com grupos vizinhos
       (elimina o angulo reto / efeito L).
    """
    def __init__(self, owned_rows, scene_x, scene_y, editor):
        super().__init__(-3, -3, 6, 6)
        self.owned_rows    = owned_rows
        self.rddf_data_row = owned_rows[0]
        self.editor        = editor
        self._last_scene_x = scene_x
        self._last_scene_y = scene_y
        self.prev_node     = None
        self.next_node     = None

        self.setPos(scene_x, scene_y)
        self.setFlag(QGraphicsEllipseItem.ItemIsMovable, False)
        self.setFlag(QGraphicsEllipseItem.ItemSendsGeometryChanges, True)
        self.setBrush(QBrush(QColor(255, 50, 50)))
        self.setPen(QPen(Qt.black))
        self.setZValue(15.0)
        self.setVisible(False)

    def itemChange(self, change, value):
        if change == QGraphicsEllipseItem.ItemPositionChange:
            if hasattr(self, 'editor') and self.editor and self.editor.current_mode == 1:
                new_x = value.x()
                new_y = value.y()
                dx_scene = new_x - self._last_scene_x
                dy_scene = new_y - self._last_scene_y
                res = self.editor.global_res
                dx_world =  dx_scene * res
                dy_world = -dy_scene * res

                # 1. Move todos os pontos do grupo
                for row in self.owned_rows:
                    row[0] += dx_world
                    row[1] += dy_world

                # 2. Redistribui pontos internos uniformemente entre primeiro e ultimo
                #    (evita empilhamento quando a reta e movida lateralmente)
                if len(self.owned_rows) > 2:
                    p_start = [self.owned_rows[0][0],  self.owned_rows[0][1]]
                    p_end   = [self.owned_rows[-1][0], self.owned_rows[-1][1]]
                    _redistribute_points(self.owned_rows, p_start, p_end)

                # 3. Blending Hermite nas juncoes com vizinhos
                #    Usa 30% dos pontos de cada lado como zona de transicao
                n_blend = max(2, min(8,
                    len(self.owned_rows) // 3,
                    len(self.prev_node.owned_rows) // 3 if self.prev_node else 99,
                ))
                if self.prev_node is not None:
                    _hermite_blend(self.prev_node.owned_rows, self.owned_rows, n_blend)
                if self.next_node is not None:
                    n_blend_next = max(2, min(8,
                        len(self.owned_rows) // 3,
                        len(self.next_node.owned_rows) // 3,
                    ))
                    _hermite_blend(self.owned_rows, self.next_node.owned_rows, n_blend_next)

                self._last_scene_x = new_x
                self._last_scene_y = new_y
                self.editor.schedule_rddf_update()
        return super().itemChange(change, value)

class PlaceNode(QGraphicsEllipseItem):
    """Ponto Interativo das Anotações (Locais do Robô) com controle de Ângulo"""
    def __init__(self, name, gx, gy, theta, global_res, editor):
        rad = 6 
        super().__init__(-rad, -rad, rad*2, rad*2)
        self.name = name
        self.editor = editor
        self.global_res = global_res
        self.theta = theta
        self.rotating = False
        
        sx = gx / global_res
        sy = -gy / global_res - 1
        self.setPos(sx, sy)
        
        self.setFlag(QGraphicsItem.ItemIsMovable, False)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        
        self.setBrush(QBrush(QColor(50, 255, 50, 220))) # Verde para Places
        self.setPen(QPen(Qt.black, 2))
        self.setZValue(20.0) 
        
        # Seta preta indicando o Theta (Para onde o robô aponta)
        self.arrow = QGraphicsLineItem(0, 0, 20, 0, self)
        self.arrow.setPen(QPen(QColor(0, 0, 0), 3))
        
        # Texto com o nome do local (MUDANÇA PARA COR MAGENTA E NEGRITO)
        label_text = self.name.replace("RDDF_PLACE_", "")
        self.label = QGraphicsTextItem(label_text, self)
        self.label.setPos(rad, rad)
        self.label.setDefaultTextColor(QColor(255, 0, 255)) # Magenta bem chamativo
        
        font = self.label.font()
        font.setBold(True)
        self.label.setFont(font)
        
        self.update_arrow()

    def set_theta(self, new_theta):
        self.theta = new_theta
        self.update_arrow()
        
    def update_arrow(self):
        lx = 20 * math.cos(self.theta)
        ly = -20 * math.sin(self.theta)
        self.arrow.setLine(0, 0, lx, ly)

    def mousePressEvent(self, event):
        if self.editor.current_mode != 2: 
            event.ignore()
            return
        if event.button() == Qt.RightButton:
            self.rotating = True
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.editor.current_mode != 2: return
        if getattr(self, 'rotating', False):
            dx = event.scenePos().x() - self.scenePos().x()
            dy = self.scenePos().y() - event.scenePos().y()
            self.set_theta(math.atan2(dy, dx))
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.editor.current_mode != 2: return
        if getattr(self, 'rotating', False):
            self.rotating = False
            event.accept()
        else:
            super().mouseReleaseEvent(event)

class MapGraphicsItem(QGraphicsPixmapItem):
    def __init__(self, map_data, brush_manager):
        super().__init__()
        self.map_data = map_data
        self.brush_manager = brush_manager
        self.history = []
        self.update_image()
        
        if map_data.res != 0:
            px_x = map_data.offset_x / map_data.res
            px_y = -(map_data.offset_y / map_data.res) - map_data.rows
        else: px_x, px_y = 0, 0
        self.setPos(px_x, px_y)
        self.setAcceptHoverEvents(True)

    def save_history(self):
        if len(self.history) >= 10: self.history.pop(0)
        self.history.append(self.map_data.grid.copy())

    def undo(self):
        if len(self.history) > 0:
            self.map_data.grid = self.history.pop()
            self.map_data.is_modified = True
            self.update_image()

    def hoverMoveEvent(self, event):
        pos = event.pos()
        pixel_x = int(pos.x())
        pixel_y = self.map_data.rows - 1 - int(pos.y())
        global_x = self.map_data.offset_x + (pixel_x * self.map_data.res)
        global_y = self.map_data.offset_y + (pixel_y * self.map_data.res)
        mod_status = "[MODIFICADO]" if self.map_data.is_modified else ""
        msg = (f"{mod_status} Arquivo: {self.map_data.filename} | Grid[{pixel_x}, {pixel_y}] | Global({global_x:.2f}, {global_y:.2f})")
        if hasattr(self.brush_manager, 'status'): self.brush_manager.status.showMessage(msg)
        super().hoverMoveEvent(event)

    def update_image(self):
        if self.map_data.grid is None: return
        grid_data = np.nan_to_num(self.map_data.grid, nan=-1.0)
        grid_T = grid_data.T
        h, w = grid_T.shape
        rgb_data = np.full((h, w, 3), 255, dtype=np.uint8)
        rgb_data[grid_T >= 0.5] = [0, 0, 0]
        rgb_data[(grid_T < 0) & (grid_T > -2)] = [0, 0, 255]
        rgb_data[grid_T <= -2] = [255, 0, 0]
        final_data = np.ascontiguousarray(rgb_data)
        qimg = QImage(final_data.data, w, h, w*3, QImage.Format_RGB888)
        self.setPixmap(QPixmap.fromImage(qimg.mirrored(False, True)))

    def mousePressEvent(self, event):
        bm = self.brush_manager
        if bm.current_mode == 0 and bm.tool_active:
            self.brush_manager.active_stroke_chunks = set()
            self.save_history()
            self.brush_manager.active_stroke_chunks.add(self)
            self.brush_manager.last_edited_item = self
            self.map_data.is_modified = True
            self.paint_on_grid(event.pos())
            event.accept()
            
        elif bm.current_mode == 1 and bm.drawing_rddf and event.button() == Qt.LeftButton:
            # Desenhando um RDDF do zero: cada clique acrescenta um vertice
            sp = event.scenePos()
            gx = sp.x() * bm.global_res
            gy = -(sp.y() + 1) * bm.global_res
            bm.add_draw_rddf_point(gx, gy)
            event.accept()

        elif bm.current_mode == 3 and bm.pending_mission_goal and event.button() == Qt.LeftButton:
            # Escolhendo o destino de uma task da missao: clica e arrasta para o theta
            sp = event.scenePos()
            gx = sp.x() * bm.global_res
            gy = -(sp.y() + 1) * bm.global_res
            node = PlaceNode("ALVO", gx, gy, 0.0, bm.global_res, bm)
            bm.scene.addItem(node)
            node.rotating = True
            bm.mission_goal_node = node
            event.accept()

        elif bm.current_mode == 2 and bm.pending_place_name and event.button() == Qt.LeftButton:
            # Novo Place disparado com clique apenas SE O BOTÃO FOI ATIVADO (+)
            scene_pos = event.scenePos()
            gx = scene_pos.x() * bm.global_res
            gy = -(scene_pos.y() + 1) * bm.global_res
            
            node = PlaceNode(bm.pending_place_name, gx, gy, 0.0, bm.global_res, bm)
            node.setFlag(QGraphicsItem.ItemIsMovable, True)
            bm.scene.addItem(node)
            bm.place_nodes.append(node)
            bm.update_place_list()
            
            node.rotating = True
            bm.active_new_place_node = node
            bm.pending_place_name = None 
            
            # VOLTA O CURSOR AO NORMAL APÓS CLICAR
            bm.view.viewport().setCursor(Qt.ArrowCursor)
            bm.view.setDragMode(QGraphicsView.ScrollHandDrag)
            event.accept()
        else: 
            super().mousePressEvent(event) 

    def mouseMoveEvent(self, event):
        bm = self.brush_manager
        if bm.current_mode == 0 and bm.tool_active:
            scene_pos = event.scenePos()
            for item in self.scene().items(scene_pos):
                if isinstance(item, MapGraphicsItem):
                    if hasattr(self.brush_manager, 'active_stroke_chunks') and item not in self.brush_manager.active_stroke_chunks:
                        item.save_history()
                        self.brush_manager.active_stroke_chunks.add(item)
                        self.brush_manager.last_edited_item = item
                    item.map_data.is_modified = True
                    item.paint_on_grid(item.mapFromScene(scene_pos))
                    break 
        elif bm.current_mode == 2 and getattr(bm, 'active_new_place_node', None):
            node = bm.active_new_place_node
            dx = event.scenePos().x() - node.scenePos().x()
            dy = node.scenePos().y() - event.scenePos().y()
            node.set_theta(math.atan2(dy, dx))
        elif bm.current_mode == 3 and getattr(bm, 'mission_goal_node', None):
            node = bm.mission_goal_node
            dx = event.scenePos().x() - node.scenePos().x()
            dy = node.scenePos().y() - event.scenePos().y()
            node.set_theta(math.atan2(dy, dx))
        else: super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        bm = self.brush_manager
        if bm.current_mode == 2 and getattr(bm, 'active_new_place_node', None):
            bm.active_new_place_node.rotating = False
            bm.active_new_place_node = None
        elif bm.current_mode == 3 and getattr(bm, 'mission_goal_node', None):
            node = bm.mission_goal_node
            gx = node.scenePos().x() * bm.global_res
            gy = -(node.scenePos().y() + 1) * bm.global_res
            theta = node.theta
            bm.scene.removeItem(node)
            bm.mission_goal_node = None
            bm.pending_mission_goal = False
            bm.view.viewport().setCursor(Qt.ArrowCursor)
            bm.view.setDragMode(QGraphicsView.ScrollHandDrag)
            bm.mission_insert_goal(gx, gy, theta)
        else:
            super().mouseReleaseEvent(event)

    def paint_on_grid(self, pos):
        cx, cy = int(pos.x()), self.map_data.rows - 1 - int(pos.y())
        radius = self.brush_manager.brush_size // 2 
        min_x, max_x = max(0, cx - radius), max(0, min(self.map_data.cols, cx + radius + 1))
        min_y, max_y = max(0, cy - radius), max(0, min(self.map_data.rows, cy + radius + 1))
        if min_x >= max_x or min_y >= max_y: return
        self.map_data.grid[min_x:max_x, min_y:max_y] = self.brush_manager.current_value
        self.update_image()

class CanvasView(QGraphicsView):
    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHint(QPainter.Antialiasing, False)
        self.setRenderHint(QPainter.SmoothPixmapTransform, False)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setBackgroundBrush(QColor(40, 40, 40))

    def wheelEvent(self, event):
        if event.modifiers() & Qt.ControlModifier:
            zoom_in = 1.15
            if event.angleDelta().y() > 0: self.scale(zoom_in, zoom_in)
            else: self.scale(1/zoom_in, 1/zoom_in)
        else: super().wheelEvent(event)

# =============================================================================
# JANELA PRINCIPAL
# =============================================================================
class MapEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MAP EDITOR 2")
        self.resize(1400, 850)
        self.current_value = 1.0 
        self.tool_active = False
        self.brush_size = 1

        self.last_edited_item = None
        self.global_res = 0.02
        
        self.original_rddf_data = [] 
        self.rddf_nodes = []         
        self.rddf_path_items = []    
        self.current_rddf_filepath = None 
        
        self.place_nodes = []
        self.original_annotations = [] 
        self.current_annotations_filepath = None
        self.pending_place_name = None 

        # 0 = MAPA | 1 = RDDF | 2 = PLACES (ANOTAÇÕES) | 3 = MISSÃO
        self.current_mode = 0

        # --- desenho de RDDF do zero ---
        self.drawing_rddf = False
        self.draw_rddf_points = []   # [(gx, gy)] em metros
        self.draw_rddf_items = []    # itens graficos do preview

        # --- editor de missoes ---
        self.mission_dir = None
        self.current_mission_path = None
        self.mission_dirty = False
        self.pending_mission_goal = False   # esperando clique no mapa
        self.mission_goal_node = None       # no temporario para girar o theta

        # Timers de debounce
        self._res_timer = QTimer(self)
        self._res_timer.setSingleShot(True)
        self._res_timer.setInterval(150)
        self._res_timer.timeout.connect(self._apply_res_change)

        self._density_timer = QTimer(self)
        self._density_timer.setSingleShot(True)
        self._density_timer.setInterval(120)
        self._density_timer.timeout.connect(self.apply_rddf_filter)

        main = QWidget()
        layout = QVBoxLayout()
        
        # --- TOOLBAR 1: ARQUIVOS GLOBAIS ---
        toolbar1 = QHBoxLayout()
        toolbar1.addWidget(QLabel("<b>AÇÕES GLOBAIS:</b>"))

        btn_load = QPushButton("Abrir Pasta (Mapas/RDDF/Anotações)")
        btn_load.setIcon(self.style().standardIcon(QStyle.SP_DirOpenIcon))
        btn_load.clicked.connect(self.load_folder)
        toolbar1.addWidget(btn_load)

        btn_save = QPushButton("Salvar Mapas")
        btn_save.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        btn_save.clicked.connect(self.save_all)
        toolbar1.addWidget(btn_save)

        btn_generate_graph = QPushButton("Gerar Grafo (.gr)")
        btn_generate_graph.setIcon(
            self.style().standardIcon(QStyle.SP_ComputerIcon)
        )
        btn_generate_graph.setStyleSheet(
            "color: #008000; font-weight: bold;"
        )
        btn_generate_graph.clicked.connect(self.generate_graph_action)
        toolbar1.addWidget(btn_generate_graph)

        toolbar1.addWidget(QLabel(" | Modo de Ação:"))

        self.combo_mode = QComboBox()
        self.combo_mode.addItems([
            "Pintar Mapa",
            "Editar Caminho (RDDF)",
            "Editar Locais (Anotações)",
            "Criar/Editar Missão"
        ])

        self.combo_mode.setStyleSheet(
            "font-weight: bold; background-color: #DDDDDD;"
        )

        self.combo_mode.currentIndexChanged.connect(
            self.change_editor_mode
        )

        toolbar1.addWidget(self.combo_mode)
        
        toolbar1.addStretch()

        # --- TOOLBAR 2: PINTURA DE MAPA ---
        self.toolbar_map = QWidget()
        t2_layout = QHBoxLayout(self.toolbar_map)
        t2_layout.setContentsMargins(0,0,0,0)
        t2_layout.addWidget(QLabel("<b>PINTURA:</b>"))
        
        btn_move = QPushButton("Mover")
        btn_move.setIcon(self.style().standardIcon(QStyle.SP_ArrowBack))
        btn_move.clicked.connect(lambda: self.set_tool(0.0, False))

        btn_pen = QPushButton("Obstáculo")
        btn_pen.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        btn_pen.clicked.connect(lambda: self.set_tool(1.0, True))

        btn_eraser = QPushButton("Borracha")
        btn_eraser.setIcon(self.style().standardIcon(QStyle.SP_DialogResetButton))
        btn_eraser.clicked.connect(lambda: self.set_tool(0.0, True))

        btn_unknown = QPushButton("Desconhecido")
        btn_unknown.setIcon(self.style().standardIcon(QStyle.SP_MessageBoxQuestion))
        btn_unknown.clicked.connect(lambda: self.set_tool(-1.0, True))
        btn_undo = QPushButton("Desfazer")
        btn_undo.setIcon(self.style().standardIcon(QStyle.SP_ArrowBack))
        btn_undo.setShortcut("Ctrl+Z")
        btn_undo.clicked.connect(self.undo_last_action)
        
        t2_layout.addWidget(btn_move)
        t2_layout.addWidget(btn_pen)
        t2_layout.addWidget(btn_eraser)
        t2_layout.addWidget(btn_unknown)
        t2_layout.addWidget(btn_undo)
        
        t2_layout.addWidget(QLabel(" | Tam Pincel:"))
        self.size_slider = QSlider(Qt.Horizontal)
        self.size_slider.setRange(1, 10) 
        self.size_slider.setValue(1)
        self.size_slider.setFixedWidth(80)
        self.size_slider.valueChanged.connect(self.change_brush_size)
        t2_layout.addWidget(self.size_slider)
        
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(1, 300)
        self.zoom_slider.setValue(20)
        self.zoom_slider.setFixedWidth(100)
        self.zoom_slider.valueChanged.connect(self.manual_zoom)
        t2_layout.addWidget(QLabel(" | Zoom Geral:"))
        t2_layout.addWidget(self.zoom_slider)
        t2_layout.addStretch()

        # --- TOOLBAR 3: EDIÇÃO DE RDDF ---
        self.toolbar_rddf = QWidget()
        t3_layout = QHBoxLayout(self.toolbar_rddf)
        t3_layout.setContentsMargins(0,0,0,0)
        t3_layout.addWidget(QLabel("<b>RDDF:</b> Arquivo:"))

        self.combo_rddf = QComboBox()
        self.combo_rddf.setMinimumWidth(180)
        self.combo_rddf.addItem("-- Selecione o Caminho --")
        self.combo_rddf.currentIndexChanged.connect(self.on_combo_rddf_changed)
        t3_layout.addWidget(self.combo_rddf)

        btn_load_rddf_manual = QPushButton("Abrir Manual")
        btn_load_rddf_manual.clicked.connect(self.load_rddf_manual)
        t3_layout.addWidget(btn_load_rddf_manual)

        btn_save_rddf = QPushButton("Salvar RDDF")
        btn_save_rddf.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        btn_save_rddf.setStyleSheet("color: #0000FF; font-weight: bold;")
        btn_save_rddf.clicked.connect(self.save_rddf)
        t3_layout.addWidget(btn_save_rddf)

        # --- desenhar um RDDF do zero, clicando no mapa ---
        self.btn_draw_rddf = QPushButton("Desenhar Novo")
        self.btn_draw_rddf.setIcon(self.style().standardIcon(QStyle.SP_FileDialogNewFolder))
        self.btn_draw_rddf.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
        self.btn_draw_rddf.setToolTip("Cria um caminho do zero: clique ponto a ponto no mapa.")
        self.btn_draw_rddf.clicked.connect(self.toggle_draw_rddf)
        t3_layout.addWidget(self.btn_draw_rddf)

        self.btn_finish_draw = QPushButton("Finalizar Desenho")
        self.btn_finish_draw.setEnabled(False)
        self.btn_finish_draw.clicked.connect(self.finish_draw_rddf)
        t3_layout.addWidget(self.btn_finish_draw)

        self.btn_undo_draw = QPushButton("Apagar Último Ponto")
        self.btn_undo_draw.setEnabled(False)
        self.btn_undo_draw.clicked.connect(self.undo_draw_point)
        t3_layout.addWidget(self.btn_undo_draw)

        t3_layout.addWidget(QLabel(" Espaç.(m):"))
        self.spin_draw_spacing = QDoubleSpinBox()
        self.spin_draw_spacing.setRange(0.1, 10.0)
        self.spin_draw_spacing.setValue(0.5)
        self.spin_draw_spacing.setSingleStep(0.1)
        self.spin_draw_spacing.setFixedWidth(60)
        self.spin_draw_spacing.setToolTip("Distancia entre pontos gerados ao longo do traco.")
        t3_layout.addWidget(self.spin_draw_spacing)

        t3_layout.addWidget(QLabel(" Vel.(m/s):"))
        self.spin_draw_v = QDoubleSpinBox()
        self.spin_draw_v.setRange(0.1, 30.0)
        self.spin_draw_v.setValue(2.777778)
        self.spin_draw_v.setDecimals(3)
        self.spin_draw_v.setFixedWidth(70)
        self.spin_draw_v.setToolTip("Velocidade gravada em cada ponto do RDDF.")
        t3_layout.addWidget(self.spin_draw_v)
        
        t3_layout.addWidget(QLabel(" | Detalhe Curva (\xb0):"))
        self.slider_density = QSlider(Qt.Horizontal)
        self.slider_density.setRange(1, 45)
        self.slider_density.setValue(8)
        self.slider_density.setFixedWidth(80)
        self.slider_density.valueChanged.connect(lambda: self._density_timer.start())
        t3_layout.addWidget(self.slider_density)

        t3_layout.addWidget(QLabel(" Força Suavização:"))
        self.spin_smooth = QSpinBox()
        self.spin_smooth.setRange(1, 100) 
        self.spin_smooth.setValue(10)
        self.spin_smooth.setFixedWidth(40)
        t3_layout.addWidget(self.spin_smooth)

        btn_smooth = QPushButton("Suavizar")
        btn_smooth.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        btn_smooth.clicked.connect(self.smooth_rddf_action)
        t3_layout.addWidget(btn_smooth)

        t3_layout.addWidget(QLabel(" | Raio(m):"))
        self.spin_radius = QDoubleSpinBox()
        self.spin_radius.setRange(0.01, 5.0)
        self.spin_radius.setValue(0.10)
        self.spin_radius.setSingleStep(0.05)
        self.spin_radius.setFixedWidth(60)
        self.spin_radius.valueChanged.connect(self.schedule_rddf_update)
        t3_layout.addWidget(self.spin_radius)
        
        t3_layout.addWidget(QLabel(" Res:"))
        self.spin_res = QDoubleSpinBox()
        self.spin_res.setRange(0.001, 1.0)
        self.spin_res.setDecimals(3)
        self.spin_res.setValue(self.global_res)
        self.spin_res.setSingleStep(0.01)
        self.spin_res.setFixedWidth(60)
        self.spin_res.valueChanged.connect(self.on_res_changed)
        t3_layout.addWidget(self.spin_res)
        t3_layout.addStretch()

        layout.addLayout(toolbar1)
        layout.addWidget(self.toolbar_map)
        layout.addWidget(self.toolbar_rddf)

        # PAINEL INFERIOR: MAPA (Esquerda) + ANOTAÇÕES (Direita)
        h_split = QHBoxLayout()
        
        self.scene = QGraphicsScene()
        self.view = CanvasView(self.scene)
        h_split.addWidget(self.view, 1) 
        
        # --- PAINEL LATERAL DE ANOTAÇÕES (PLACES) ---
        self.panel_ann = QWidget()
        self.panel_ann.setFixedWidth(280)
        ann_layout = QVBoxLayout(self.panel_ann)

        ann_layout.addWidget(QLabel("<h3 style='margin:0;'>Locais e Anotações</h3>"))

        self.combo_ann = QComboBox()
        self.combo_ann.addItem("-- Criar Novo / Nenhum --")
        self.combo_ann.currentIndexChanged.connect(self.on_combo_ann_changed)
        ann_layout.addWidget(self.combo_ann)

        btn_load_ann = QPushButton("Abrir Arquivo Manual")
        btn_load_ann.setIcon(self.style().standardIcon(QStyle.SP_DirOpenIcon))
        btn_load_ann.clicked.connect(self.load_ann_manual)
        ann_layout.addWidget(btn_load_ann)

        btn_save_ann = QPushButton("Salvar Anotações")
        btn_save_ann.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        btn_save_ann.setStyleSheet(
            "background-color: #4CAF50; color: white; font-weight: bold;"
        )
        btn_save_ann.clicked.connect(self.save_annotations)
        ann_layout.addWidget(btn_save_ann)

        # --- BOTÃO DE ADICIONAR ---
        self.btn_add_place = QPushButton("Adicionar Novo Local")
        self.btn_add_place.setIcon(
            self.style().standardIcon(QStyle.SP_FileDialogNewFolder)
        )
        self.btn_add_place.setStyleSheet(
            "background-color: #2196F3; color: white; font-weight: bold;"
        )
        self.btn_add_place.clicked.connect(self.start_adding_place)
        ann_layout.addWidget(self.btn_add_place)

        ann_layout.addWidget(QLabel("<b>Lugares Iniciais (Places):</b>"))

        self.list_places = QListWidget()
        self.list_places.currentRowChanged.connect(self.on_place_selected)
        ann_layout.addWidget(self.list_places)

        btn_del_place = QPushButton("Excluir Selecionado")
        btn_del_place.setIcon(self.style().standardIcon(QStyle.SP_TrashIcon))
        btn_del_place.clicked.connect(self.delete_selected_place)
        ann_layout.addWidget(btn_del_place)

        lbl_info = QLabel(
            "<b>Dica:</b><br>"
            "- Clique no mapa para adicionar.<br>"
            "- Arraste com botão Esq. para mover.<br>"
            "- Arraste com botão Dir. para girar."
        )
        lbl_info.setStyleSheet("color: #666;")
        lbl_info.setWordWrap(True)
        ann_layout.addWidget(lbl_info)

        h_split.addWidget(self.panel_ann, 0) 

        # --- PAINEL LATERAL DE MISSOES ---
        self.panel_mission = QWidget()
        self.panel_mission.setFixedWidth(420)
        mis_layout = QVBoxLayout(self.panel_mission)

        mis_layout.addWidget(QLabel("<h3 style='margin:0;'>Missão</h3>"))

        row_dir = QHBoxLayout()
        self.lbl_mission_dir = QLabel("<i>nenhuma pasta</i>")
        self.lbl_mission_dir.setStyleSheet("color:#666;")
        self.lbl_mission_dir.setWordWrap(True)
        btn_mission_dir = QPushButton("Pasta...")
        btn_mission_dir.setIcon(self.style().standardIcon(QStyle.SP_DirOpenIcon))
        btn_mission_dir.setToolTip("Pasta 'missoes/' que o navigator_gui2 lista no combo Mission.")
        btn_mission_dir.clicked.connect(self.mission_choose_dir)
        row_dir.addWidget(self.lbl_mission_dir, 1)
        row_dir.addWidget(btn_mission_dir, 0)
        mis_layout.addLayout(row_dir)

        self.combo_mission = QComboBox()
        self.combo_mission.addItem("-- Nova missão --", None)
        self.combo_mission.currentIndexChanged.connect(self.on_combo_mission_changed)
        mis_layout.addWidget(self.combo_mission)

        row_new = QHBoxLayout()
        btn_mission_new = QPushButton("Nova")
        btn_mission_new.setIcon(self.style().standardIcon(QStyle.SP_FileIcon))
        btn_mission_new.clicked.connect(self.mission_new)
        row_new.addWidget(btn_mission_new)
        btn_mission_tpl = QPushButton("Modelo")
        btn_mission_tpl.setToolTip("Insere o esqueleto de missão que funciona.")
        btn_mission_tpl.clicked.connect(self.mission_insert_template)
        row_new.addWidget(btn_mission_tpl)
        btn_mission_del = QPushButton("Excluir")
        btn_mission_del.setIcon(self.style().standardIcon(QStyle.SP_TrashIcon))
        btn_mission_del.clicked.connect(self.mission_delete)
        row_new.addWidget(btn_mission_del)
        mis_layout.addLayout(row_new)

        self.edit_mission = QPlainTextEdit()
        self.edit_mission.setStyleSheet("font-family: monospace; font-size: 12px;")
        self.edit_mission.setPlaceholderText(
            "Uma task por linha.\n\n"
            "Toda task na coluna 0 -- linha comecando com espaco e' IGNORADA.\n"
            "Use o par 'set course to' + 'go'. Nunca 'go after set course to'.\n"
            "So' ponha 'stop' se a missao navegar de verdade."
        )
        self.edit_mission.textChanged.connect(self._on_mission_text_changed)
        mis_layout.addWidget(self.edit_mission, 1)

        mis_layout.addWidget(QLabel("<b>Inserir na posição do cursor:</b>"))

        row_ins = QHBoxLayout()
        self.btn_pick_goal = QPushButton("Destino pelo mapa")
        self.btn_pick_goal.setIcon(self.style().standardIcon(QStyle.SP_ArrowForward))
        self.btn_pick_goal.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        self.btn_pick_goal.setToolTip("Clique no mapa e arraste para definir o theta; insere 'set course to x y theta' + 'go'.")
        self.btn_pick_goal.clicked.connect(self.mission_start_pick_goal)
        row_ins.addWidget(self.btn_pick_goal)
        mis_layout.addLayout(row_ins)

        row_place = QHBoxLayout()
        self.combo_mission_place = QComboBox()
        self.combo_mission_place.setToolTip("Locais (places tipo 13) carregados das anotações.")
        row_place.addWidget(self.combo_mission_place, 1)
        btn_ins_place = QPushButton("Inserir place")
        btn_ins_place.clicked.connect(self.mission_insert_place)
        row_place.addWidget(btn_ins_place, 0)
        mis_layout.addLayout(row_place)

        row_save = QHBoxLayout()
        self.btn_mission_check = QPushButton("Verificar")
        self.btn_mission_check.setToolTip("Procura as armadilhas que quebram a missão em silêncio.")
        self.btn_mission_check.clicked.connect(self.mission_validate)
        row_save.addWidget(self.btn_mission_check)
        btn_mission_save = QPushButton("Salvar")
        btn_mission_save.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        btn_mission_save.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        btn_mission_save.clicked.connect(self.mission_save)
        row_save.addWidget(btn_mission_save)
        btn_mission_saveas = QPushButton("Salvar como...")
        btn_mission_saveas.clicked.connect(self.mission_save_as)
        row_save.addWidget(btn_mission_saveas)
        mis_layout.addLayout(row_save)

        lbl_mis_info = QLabel(
            "<b>Lembre:</b><br>"
            "- A última linha precisa terminar com quebra de linha.<br>"
            "- Task escrita errada não para a missão, só é ignorada.<br>"
            "- Vírgula só entre números.<br>"
            "- Para aparecer no combo do navigator_gui2, salve em <code>missoes/</code> "
            "e reinicie o navigator_gui2."
        )
        lbl_mis_info.setStyleSheet("color: #666;")
        lbl_mis_info.setWordWrap(True)
        mis_layout.addWidget(lbl_mis_info)

        h_split.addWidget(self.panel_mission, 0)
        layout.addLayout(h_split)
        
        main.setLayout(layout)
        self.setCentralWidget(main)
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        
        self.maps = []
        self.change_editor_mode(0) 

    # =========================================================================
    # CONTROLE DE MODOS (MAPA, RDDF, PLACES)
    # =========================================================================
    def change_editor_mode(self, index):
        self.current_mode = index
        self.set_tool(0.0, False) 
        self.pending_place_name = None 
        self.view.viewport().setCursor(Qt.ArrowCursor) 
        
        if index == 0: 
            self.status.showMessage("MODO MAPA: Desenhe obstáculos no grid.")
            for n in self.rddf_nodes: 
                n.setFlag(QGraphicsItem.ItemIsMovable, False)
                n.setVisible(False) # Oculta RDDF
            for p in self.place_nodes: 
                p.setFlag(QGraphicsItem.ItemIsMovable, False)
            
        elif index == 1: 
            self.status.showMessage("MODO RDDF: Arraste as bolinhas vermelhas para modificar a trajetória.")
            for n in self.rddf_nodes: 
                n.setFlag(QGraphicsItem.ItemIsMovable, True)
                n.setVisible(True) # Exibe RDDF
            for p in self.place_nodes: 
                p.setFlag(QGraphicsItem.ItemIsMovable, False)
            
        elif index == 2: 
            self.status.showMessage("MODO PLACES: Selecione Locais. Arraste para mover ou Clique Direito para rotacionar.")
            for n in self.rddf_nodes: 
                n.setFlag(QGraphicsItem.ItemIsMovable, False)
                n.setVisible(False) # Oculta RDDF
            for p in self.place_nodes: 
                p.setFlag(QGraphicsItem.ItemIsMovable, True)

        elif index == 3:
            self.status.showMessage("MODO MISSÃO: escreva a missão. Use 'Destino pelo mapa' para inserir coordenadas.")
            for n in self.rddf_nodes:
                n.setFlag(QGraphicsItem.ItemIsMovable, False)
                n.setVisible(False)
            for p in self.place_nodes:
                p.setFlag(QGraphicsItem.ItemIsMovable, False)   # visiveis, para servirem de referencia
            self.mission_refresh_places()

        # Sai do desenho de RDDF se trocar de modo
        if index != 1 and self.drawing_rddf:
            self.cancel_draw_rddf()
        # Cancela a espera de clique da missao se trocar de modo
        if index != 3:
            self.pending_mission_goal = False

        # Toolbars e paineis conforme o modo
        self.toolbar_map.setVisible(index == 0)
        self.toolbar_rddf.setVisible(index == 1)
        self.panel_ann.setVisible(index == 2)
        self.panel_mission.setVisible(index == 3)
        if index != 3:
            self.view.viewport().setCursor(Qt.ArrowCursor)


    # =========================================================================
    # FUNÇÕES DE ANOTAÇÕES (LOCAIS)
    # =========================================================================
    def start_adding_place(self):
        """Prepara o sistema para colocar um novo ponto no clique do mapa com Cursor em Cruz (+)"""
        if self.current_mode != 2:
            self.combo_mode.setCurrentIndex(2) 
            
        name, ok = QInputDialog.getText(self, "Novo Local", "Nome do Local (ex: ELEVADOR):")
        if ok and name.strip():
            final_name = name.strip()
            if not final_name.startswith("RDDF_PLACE_"):
                final_name = "RDDF_PLACE_" + final_name
            self.pending_place_name = final_name
            self.status.showMessage(f"Aguardando... CLIQUE EM UM PONTO VAZIO DO MAPA para posicionar '{final_name}'")
            
            self.view.setDragMode(QGraphicsView.NoDrag)
            self.view.viewport().setCursor(Qt.CrossCursor)

    def load_ann_manual(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "Abrir Anotações", "", "Text Files (*.txt);;All Files (*)")
        if filepath: self.load_annotations_from_path(filepath)

    def on_combo_ann_changed(self, index):
        filepath = self.combo_ann.itemData(index)
        if filepath: self.load_annotations_from_path(filepath)

    def load_annotations_from_path(self, filepath):
        self.clear_places()
        self.original_annotations.clear()
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line_str = line.strip()
                    if line_str.startswith("RDDF_PLACE_"):
                        parts = line_str.split()
                        if len(parts) >= 6:
                            name = parts[0]
                            theta = float(parts[3])
                            gx = float(parts[4])
                            gy = float(parts[5])
                            
                            node = PlaceNode(name, gx, gy, theta, self.global_res, self)
                            node.setFlag(QGraphicsItem.ItemIsMovable, self.current_mode == 2)
                            self.scene.addItem(node)
                            self.place_nodes.append(node)
                    else:
                        self.original_annotations.append(line) 
            
            self.current_annotations_filepath = filepath
            self.update_place_list()
            self.status.showMessage(f"Anotações carregadas: {len(self.place_nodes)} locais encontrados.")
            
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao carregar anotações:\n{str(e)}")

    def update_place_list(self):
        self.list_places.blockSignals(True)
        self.list_places.clear()
        for node in self.place_nodes:
            self.list_places.addItem(node.name.replace("RDDF_PLACE_", ""))
        self.list_places.blockSignals(False)

    def on_place_selected(self, index):
        """Dá o Zoom 3x3 no bloco selecionado na lista"""
        if 0 <= index < len(self.place_nodes):
            node = self.place_nodes[index]
            
            # Pega o tamanho do "chunk" visual (3 metros)
            chunk_px = 3.0 / self.global_res
            half_chunk = chunk_px / 2.0
            
            cx = node.scenePos().x()
            cy = node.scenePos().y()
            
            # Cria a janela de visão (Viewport rect)
            rect = QRectF(cx - half_chunk, cy - half_chunk, chunk_px, chunk_px)
            self.view.fitInView(rect, Qt.KeepAspectRatio)
            
            # Ajusta o Slider lá em cima automaticamente para refletir o nível de aproximação
            self.zoom_slider.blockSignals(True)
            scale = self.view.transform().m11()
            self.zoom_slider.setValue(int(scale * 10))
            self.zoom_slider.blockSignals(False)

    def delete_selected_place(self):
        row = self.list_places.currentRow()
        if 0 <= row < len(self.place_nodes):
            node = self.place_nodes.pop(row)
            self.scene.removeItem(node)
            self.update_place_list()
            self.status.showMessage(f"Local removido: {node.name}")

    def clear_places(self):
        for node in self.place_nodes:
            self.scene.removeItem(node)
        self.place_nodes.clear()
        self.original_annotations.clear()
        self.update_place_list()

    def save_annotations(self):
        if not self.place_nodes and not self.original_annotations:
            QMessageBox.warning(self, "Aviso", "Não há anotações para salvar.")
            return
            
        filepath = self.current_annotations_filepath
        
        if not filepath:
            if not self.maps:
                QMessageBox.warning(self, "Aviso", "Carregue a pasta do ambiente primeiro para definir o diretório raiz.")
                return
            
            base_dir = os.path.dirname(self.maps[0].filepath)
            ann_dir = os.path.join(base_dir, "annotations")
            os.makedirs(ann_dir, exist_ok=True)
            
            filepath, _ = QFileDialog.getSaveFileName(self, "Salvar Anotações", os.path.join(ann_dir, "annotations.txt"), "Text Files (*.txt)")
            if not filepath: return

        try:
            with open(filepath, 'w') as f:
                for node in self.place_nodes:
                    gx = node.scenePos().x() * self.global_res
                    gy = -(node.scenePos().y() + 1) * self.global_res
                    f.write(f"{node.name}\t13\t0\t{node.theta:.6f}\t{gx:.6f}\t{gy:.6f}\t0.0\n")
                
                for raw_line in self.original_annotations:
                    f.write(raw_line)
                    if not raw_line.endswith('\n'): f.write('\n')
                    
            self.current_annotations_filepath = filepath
            QMessageBox.information(self, "Sucesso", f"Anotações salvas com sucesso em:\n{filepath}")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao salvar:\n{str(e)}")


    # =========================================================================
    # FUNÇÕES DE RDDF
    # =========================================================================
    def on_combo_rddf_changed(self, index):
        filepath = self.combo_rddf.itemData(index)
        if filepath: self.load_rddf_from_path(filepath)

    def load_rddf_manual(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "Abrir RDDF", "", "Arquivos de Texto (*.txt);;Todos Arquivos (*)")
        if filepath: self.load_rddf_from_path(filepath)

    def load_rddf_from_path(self, filepath):
        self.original_rddf_data.clear()
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        try:
                            row_data = [float(val) for val in parts]
                            self.original_rddf_data.append(row_data)
                        except ValueError: pass
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao ler RDDF:\n{str(e)}")
            return

        if not self.original_rddf_data: return
        self.current_rddf_filepath = filepath
        self.apply_rddf_filter()
        self.status.showMessage(f"RDDF Carregado: {os.path.basename(filepath)} com {len(self.original_rddf_data)} pontos.")

    def apply_rddf_filter(self):
        """Agrupa pontos de retas em 1 handle; curvas ficam individualmente visiveis.

        O slider 'Detalhe Curva (graus)' e o limiar de angulo acumulado:
        - Valor baixo (1-5): quase todos os pontos viram handles individuais.
        - Valor alto (20-45): retas longas ficam como 1 handle so.
        Apos criar os nos, liga prev_node/next_node para que o itemChange
        costure as fronteiras com Hermite ao arrastar.
        """
        if not self.original_rddf_data:
            return

        for node in self.rddf_nodes:
            self.scene.removeItem(node)
        self.rddf_nodes.clear()

        data       = self.original_rddf_data
        total      = len(data)
        thresh_rad = math.radians(max(1, self.slider_density.value()))

        groups        = []
        current_group = [data[0]]
        accum_angle   = 0.0

        for i in range(1, total):
            if i >= 2:
                p0 = data[i - 2]; p1 = data[i - 1]; p2 = data[i]
                dx1 = p1[0] - p0[0]; dy1 = p1[1] - p0[1]
                dx2 = p2[0] - p1[0]; dy2 = p2[1] - p1[1]
                l1  = math.hypot(dx1, dy1)
                l2  = math.hypot(dx2, dy2)
                if l1 > 1e-9 and l2 > 1e-9:
                    cos_a = (dx1*dx2 + dy1*dy2) / (l1 * l2)
                    delta = math.acos(max(-1.0, min(1.0, cos_a)))
                else:
                    delta = 0.0
            else:
                delta = 0.0

            accum_angle += delta
            if accum_angle >= thresh_rad:
                groups.append(current_group)
                current_group = [data[i]]
                accum_angle   = 0.0
            else:
                current_group.append(data[i])

        if current_group:
            groups.append(current_group)

        for group in groups:
            anchor = group[0]
            sx = anchor[0] / self.global_res
            sy = -anchor[1] / self.global_res - 1
            node = RDDFNode(group, sx, sy, self)
            node.setVisible(self.current_mode == 1)
            node.setFlag(QGraphicsItem.ItemIsMovable, self.current_mode == 1)
            self.scene.addItem(node)
            self.rddf_nodes.append(node)

        # Liga vizinhos — necessario para Hermite funcionar no drag
        for k, node in enumerate(self.rddf_nodes):
            node.prev_node = self.rddf_nodes[k - 1] if k > 0                        else None
            node.next_node = self.rddf_nodes[k + 1] if k < len(self.rddf_nodes) - 1 else None

        self.schedule_rddf_update()
        self.status.showMessage(
            f"Detalhe {self.slider_density.value()}\xb0 | "
            f"Handles: {len(self.rddf_nodes)} / Pontos: {total}"
        )


    def apply_laplacian_smoothing(points, iterations, alpha=0.5):
        """Puxa os pontos para o alinhamento ideal, reduzindo zigue-zague sem deletar pontos."""
        if not points or len(points) < 3: 
            return points
        
        # Repete a matemática "N" vezes (Força da Suavização)
        for _ in range(iterations):
            # Cria uma cópia temporária para não distorcer o cálculo em cadeia
            temp_data = [list(row) for row in points]
            
            # Ignora o primeiro e o último ponto (eles ficam ancorados)
            for i in range(1, len(points) - 1):
                prev_p = points[i-1]
                curr_p = points[i]
                next_p = points[i+1]
                
                # Puxa o ponto X e Y atual (curr_p) em direção ao ponto médio entre o anterior e o próximo
                temp_data[i][0] = curr_p[0] * (1 - alpha) + ((prev_p[0] + next_p[0]) / 2.0) * alpha
                temp_data[i][1] = curr_p[1] * (1 - alpha) + ((prev_p[1] + next_p[1]) / 2.0) * alpha
                
            points = [list(row) for row in temp_data]
            
        return points

    def resample_path(points, spacing_m):
        """Interpola a curva e cria novos pontos a uma distância exata, mantendo a geometria."""
        if len(points) < 2: 
            return points
        
        # 1. Calcula as distâncias acumuladas ponto a ponto do trajeto original
        cum_dist = [0.0]
        for i in range(1, len(points)):
            dx = points[i][0] - points[i-1][0]
            dy = points[i][1] - points[i-1][1]
            cum_dist.append(cum_dist[-1] + math.hypot(dx, dy))
            
        total_len = cum_dist[-1]
        
        # Se o caminho inteiro for menor que o espaçamento, não faz nada
        if total_len <= spacing_m: 
            return points
        
        new_points = []
        
        # 2. Cria os "alvos" de distância exatos (ex: 0.0, 0.5, 1.0, 1.5...)
        dists = np.arange(0, total_len, spacing_m)
        
        for d in dists:
            # Encontra em qual segmento original essa distância cai
            idx = np.searchsorted(cum_dist, d)
            
            if idx == 0:
                new_points.append(list(points[0]))
                continue
            if idx >= len(cum_dist):
                new_points.append(list(points[-1]))
                continue
                
            # 3. Interpolação Matemática: Pega o ponto Exato na reta entre idx-1 e idx
            d0 = cum_dist[idx-1]
            d1 = cum_dist[idx]
            p0 = np.array(points[idx-1])
            p1 = np.array(points[idx])
            
            t = (d - d0) / (d1 - d0) if d1 > d0 else 0
            pt = p0 + t * (p1 - p0)
            new_points.append(pt.tolist())
            
        # 4. Garante que o ÚLTIMO ponto original faça parte da lista (para o robô não parar antes)
        last_orig = points[-1]
        last_new = new_points[-1]
        if math.hypot(last_new[0] - last_orig[0], last_new[1] - last_orig[1]) > 0.05:
            new_points.append(list(last_orig))
            
        return new_points

    def smooth_rddf_action(self):
        if not self.original_rddf_data: return
        iterations = self.spin_smooth.value()
        alpha = 0.5 
        for _ in range(iterations):
            temp_data = [list(row) for row in self.original_rddf_data]
            for i in range(1, len(self.original_rddf_data) - 1):
                prev_p = self.original_rddf_data[i-1]
                curr_p = self.original_rddf_data[i]
                next_p = self.original_rddf_data[i+1]
                temp_data[i][0] = curr_p[0] * (1 - alpha) + ((prev_p[0] + next_p[0]) / 2.0) * alpha
                temp_data[i][1] = curr_p[1] * (1 - alpha) + ((prev_p[1] + next_p[1]) / 2.0) * alpha
            self.original_rddf_data = [list(row) for row in temp_data]

        # Reagrupa os nos de controle a partir dos dados suavizados
        # (apply_rddf_filter agora NUNCA deleta pontos, apenas reagrupa)
        self.apply_rddf_filter()
        self.status.showMessage(f"Trajeto suavizado com Forca {iterations}. Pontos mantidos: {len(self.original_rddf_data)}.")

    def schedule_rddf_update(self):
        for item in self.rddf_path_items: self.scene.removeItem(item)
        self.rddf_path_items.clear()
        if not self.rddf_nodes: return

        path = QPainterPath()
        first_node = self.rddf_nodes[0]
        path.moveTo(first_node.scenePos().x(), first_node.scenePos().y())
        for node in self.rddf_nodes[1:]:
            path.lineTo(node.scenePos().x(), node.scenePos().y())

        rddf_area = QGraphicsPathItem(path)
        pen_area = QPen(QColor(0, 255, 255, 100))
        pen_area.setWidthF((self.spin_radius.value() * 2) / self.global_res)
        pen_area.setCapStyle(Qt.RoundCap); pen_area.setJoinStyle(Qt.RoundJoin); pen_area.setCosmetic(False)
        rddf_area.setPen(pen_area); rddf_area.setZValue(9.0) 
        
        rddf_line = QGraphicsPathItem(path)
        pen_line = QPen(QColor(255, 200, 0, 255), 2); pen_line.setCosmetic(True) 
        rddf_line.setPen(pen_line); rddf_line.setZValue(10.0)
        
        self.scene.addItem(rddf_area)
        self.scene.addItem(rddf_line)
        self.rddf_path_items.extend([rddf_area, rddf_line])

        # --- NOVOS MARCADORES DE INÍCIO E FIM (Seta) ---
        if len(self.rddf_nodes) >= 2:
            p0 = self.rddf_nodes[0].scenePos()
            p1 = self.rddf_nodes[1].scenePos()
            ang_start = math.atan2(p1.y() - p0.y(), p1.x() - p0.x())
            bar_px = 30
            px1 = p0.x() + bar_px * math.cos(ang_start + math.pi/2)
            py1 = p0.y() + bar_px * math.sin(ang_start + math.pi/2)
            px2 = p0.x() + bar_px * math.cos(ang_start - math.pi/2)
            py2 = p0.y() + bar_px * math.sin(ang_start - math.pi/2)
            
            start_bar = QGraphicsLineItem(px1, py1, px2, py2)
            start_bar.setPen(QPen(QColor(0, 255, 0), 4)) # Verde fluorescente
            start_bar.setZValue(11.0)
            self.scene.addItem(start_bar)
            self.rddf_path_items.append(start_bar)

            pn = self.rddf_nodes[-1].scenePos()
            pn_prev = self.rddf_nodes[-2].scenePos()
            ang_end = math.atan2(pn.y() - pn_prev.y(), pn.x() - pn_prev.x())
            
            arr_sz = 20
            p_top = QPointF(pn.x() + arr_sz * math.cos(ang_end), pn.y() + arr_sz * math.sin(ang_end))
            p_bl = QPointF(pn.x() + arr_sz * math.cos(ang_end + 2.5), pn.y() + arr_sz * math.sin(ang_end + 2.5))
            p_br = QPointF(pn.x() + arr_sz * math.cos(ang_end - 2.5), pn.y() + arr_sz * math.sin(ang_end - 2.5))
            
            triangle = QGraphicsPolygonItem(QPolygonF([p_top, p_bl, p_br]))
            triangle.setBrush(QBrush(QColor(255, 0, 0))) # Vermelho sólido
            triangle.setPen(QPen(Qt.black, 1))
            triangle.setZValue(11.0)
            self.scene.addItem(triangle)
            self.rddf_path_items.append(triangle)

    def save_rddf(self):
        if not self.original_rddf_data: return
        filepath, _ = QFileDialog.getSaveFileName(self, "Salvar RDDF", "", "Arquivos de Texto (*.txt)")
        if not filepath: return

        try:
            # Salva TODOS os pontos originais (incluindo os dos grupos ocultos),
            # recalculando Theta de cada ponto em relacao ao proximo vizinho.
            all_rows = self.original_rddf_data
            n = len(all_rows)
            with open(filepath, 'w') as f:
                for i in range(n):
                    row = all_rows[i]
                    gx = row[0]
                    gy = row[1]
                    if i < n - 1:
                        nx_ = all_rows[i + 1][0]
                        ny_ = all_rows[i + 1][1]
                        theta = math.atan2(ny_ - gy, nx_ - gx)
                    else:
                        if i > 0:
                            theta = math.atan2(gy - all_rows[i-1][1], gx - all_rows[i-1][0])
                        else:
                            theta = 0.0
                    row_out = list(row)
                    if len(row_out) >= 3:
                        row_out[2] = theta
                    f.write("\t".join([f"{v:.6f}" for v in row_out]) + "\n")

            self.current_rddf_filepath = filepath
            QMessageBox.information(self, "Sucesso", f"RDDF salvo com {n} pontos e Theta recalculado.")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao salvar: {str(e)}")

    def generate_graph_action(self):
        if not self.current_rddf_filepath:
            QMessageBox.warning(self, "Aviso", "Nenhum RDDF salvo ativo.")
            return
        if QMessageBox.question(self, "Construir Grafo?", "Gerar malha de navegação (.gr)?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.generate_graph(self.current_rddf_filepath)

    def generate_graph(self, rddf_filepath):
        graphs_dir = os.path.join(os.path.dirname(os.path.dirname(rddf_filepath)), "graphs")
        try: os.makedirs(graphs_dir, exist_ok=True)
        except Exception as e: return QMessageBox.critical(self, "Erro", f"Erro:\n{e}")

        base_name = os.path.basename(rddf_filepath).replace(".txt", ".gr")
        if "rddf-" in base_name and not base_name.startswith("graph-"): base_name = base_name.replace("rddf-", "graph-rddf-")
        elif not base_name.startswith("graph-"): base_name = "graph-" + base_name
        graph_filepath = os.path.join(graphs_dir, base_name)

        if not os.path.exists(EXE_ROAD_NETWORK): return QMessageBox.critical(self, "Erro", f"Executável não encontrado:\n{EXE_ROAD_NETWORK}")

        cmd = [EXE_ROAD_NETWORK, "--rddf", rddf_filepath, graph_filepath, "250.0"]
        self.status.showMessage("Gerando grafo... Aguardando Central do Carmen.")
        QApplication.processEvents()
        try:
            process = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            if process.returncode != 0: QMessageBox.critical(self, "Erro na Geração", f"A Central está rodando?\n{process.stderr}")
            else: QMessageBox.information(self, "Sucesso!", f"Grafo gerado:\n{graph_filepath}")
        except subprocess.TimeoutExpired:
            QMessageBox.critical(self, "Timeout", "A Central não está respondendo.")

    def on_res_changed(self, value):
        self.global_res = value
        self._res_timer.start()

    def _apply_res_change(self):
        for node in self.rddf_nodes:
            sx = node.rddf_data_row[0] / self.global_res
            sy = -node.rddf_data_row[1] / self.global_res - 1
            node.setPos(sx, sy)
            node._last_scene_x = sx
            node._last_scene_y = sy
        self.schedule_rddf_update()

    # =========================================================================
    # DESENHAR UM RDDF DO ZERO
    # =========================================================================
    def toggle_draw_rddf(self):
        if self.drawing_rddf:
            self.cancel_draw_rddf()
        else:
            self.start_draw_rddf()

    def start_draw_rddf(self):
        if not self.maps:
            QMessageBox.warning(self, "Aviso", "Carregue a pasta do ambiente primeiro.")
            return
        if self.original_rddf_data:
            r = QMessageBox.question(self, "Descartar RDDF atual?",
                                     "Ja' existe um caminho carregado. Descartar e desenhar do zero?",
                                     QMessageBox.Yes | QMessageBox.No)
            if r != QMessageBox.Yes:
                return
            self.clear_rddf()

        if self.current_mode != 1:
            self.combo_mode.setCurrentIndex(1)

        self.drawing_rddf = True
        self.draw_rddf_points = []
        self._clear_draw_preview()
        self.btn_draw_rddf.setText("Cancelar Desenho")
        self.btn_finish_draw.setEnabled(True)
        self.btn_undo_draw.setEnabled(True)
        self.view.setDragMode(QGraphicsView.NoDrag)
        self.view.viewport().setCursor(Qt.CrossCursor)
        self.status.showMessage("DESENHANDO: clique no mapa para marcar os vértices do caminho. "
                                "Depois use 'Finalizar Desenho'.")

    def cancel_draw_rddf(self):
        self.drawing_rddf = False
        self.draw_rddf_points = []
        self._clear_draw_preview()
        self.btn_draw_rddf.setText("Desenhar Novo")
        self.btn_finish_draw.setEnabled(False)
        self.btn_undo_draw.setEnabled(False)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.view.viewport().setCursor(Qt.ArrowCursor)
        self.status.showMessage("Desenho cancelado.")

    def _clear_draw_preview(self):
        for it in self.draw_rddf_items:
            try: self.scene.removeItem(it)
            except Exception: pass
        self.draw_rddf_items = []

    def _redraw_draw_preview(self):
        self._clear_draw_preview()
        pen_line = QPen(QColor(255, 140, 0), 0)
        pen_line.setCosmetic(True)
        pen_line.setWidth(2)
        prev = None
        for i, (gx, gy) in enumerate(self.draw_rddf_points):
            sx = gx / self.global_res
            sy = -gy / self.global_res - 1
            r = 3.0
            dot = QGraphicsEllipseItem(sx - r, sy - r, 2 * r, 2 * r)
            dot.setBrush(QBrush(QColor(255, 140, 0)))
            dot.setPen(QPen(Qt.black, 0))
            dot.setZValue(60)
            self.scene.addItem(dot)
            self.draw_rddf_items.append(dot)
            if prev is not None:
                line = QGraphicsLineItem(prev[0], prev[1], sx, sy)
                line.setPen(pen_line)
                line.setZValue(59)
                self.scene.addItem(line)
                self.draw_rddf_items.append(line)
            prev = (sx, sy)

    def add_draw_rddf_point(self, gx, gy):
        self.draw_rddf_points.append((gx, gy))
        self._redraw_draw_preview()
        self.status.showMessage(f"DESENHANDO: {len(self.draw_rddf_points)} vértice(s). "
                                f"Último: {gx:.2f}, {gy:.2f}")

    def undo_draw_point(self):
        if not self.draw_rddf_points:
            return
        self.draw_rddf_points.pop()
        self._redraw_draw_preview()
        self.status.showMessage(f"DESENHANDO: {len(self.draw_rddf_points)} vértice(s).")

    def finish_draw_rddf(self):
        """Interpola os vertices clicados a cada N metros, calcula theta e vira o RDDF atual."""
        if len(self.draw_rddf_points) < 2:
            QMessageBox.warning(self, "Aviso", "Marque pelo menos 2 vértices.")
            return

        spacing = self.spin_draw_spacing.value()
        v = self.spin_draw_v.value()

        # Reamostra o poligono clicado em passos de 'spacing' metros
        pts = []
        for i in range(len(self.draw_rddf_points) - 1):
            x0, y0 = self.draw_rddf_points[i]
            x1, y1 = self.draw_rddf_points[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(round(seg / spacing)))
            for k in range(n):
                t = k / float(n)
                pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        pts.append(self.draw_rddf_points[-1])

        # Monta as linhas no formato do rddf: x y theta v phi timestamp
        rows = []
        n = len(pts)
        for i, (gx, gy) in enumerate(pts):
            if i < n - 1:
                theta = math.atan2(pts[i + 1][1] - gy, pts[i + 1][0] - gx)
            else:
                theta = math.atan2(gy - pts[i - 1][1], gx - pts[i - 1][0])
            rows.append([gx, gy, theta, v, 0.0, 0.0])

        self.drawing_rddf = False
        self._clear_draw_preview()
        self.draw_rddf_points = []
        self.btn_draw_rddf.setText("Desenhar Novo")
        self.btn_finish_draw.setEnabled(False)
        self.btn_undo_draw.setEnabled(False)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.view.viewport().setCursor(Qt.ArrowCursor)

        self.original_rddf_data = rows
        self.current_rddf_filepath = None   # ainda nao foi salvo
        self.apply_rddf_filter()
        self.status.showMessage(f"RDDF criado com {n} pontos (espaçamento {spacing:.2f} m). "
                                "Ajuste arrastando as bolinhas e use 'Salvar RDDF'.")
        QMessageBox.information(self, "RDDF criado",
                                f"{n} pontos gerados.\n\n"
                                "Agora dá para arrastar as bolinhas, suavizar, e então "
                                "'Salvar RDDF'. Depois 'Gerar Grafo (.gr)' se quiser navegar por ele.")

    # =========================================================================
    # EDITOR DE MISSÕES
    # =========================================================================
    MISSION_TEMPLATE = (
        "# Uma frase dizendo o que a missao faz -- vira a descricao no combo do navigator_gui2.\n"
        "\n"
        "publish mission state STARTING_MISSION minha_missao\n"
        "\n"
        "set local double v equal to 2.0\n"
        "set maximum speed to v m/s\n"
        "\n"
        "# use sempre o par 'set course to' + 'go'\n"
        "\n"
        "stop\n"
        "\n"
        "publish mission state MISSION_COMPLETED minha_missao\n"
        "mission completed\n"
    )

    def _on_mission_text_changed(self):
        self.mission_dirty = True

    def mission_choose_dir(self):
        d = QFileDialog.getExistingDirectory(self, "Pasta das missões (missoes/)")
        if not d:
            return
        self.mission_set_dir(d)

    def mission_set_dir(self, d):
        self.mission_dir = d
        self.lbl_mission_dir.setText(d)
        self.mission_scan_dir()

    def mission_scan_dir(self):
        self.combo_mission.blockSignals(True)
        self.combo_mission.clear()
        self.combo_mission.addItem("-- Nova missão --", None)
        if self.mission_dir and os.path.isdir(self.mission_dir):
            for f in sorted(glob.glob(os.path.join(self.mission_dir, "*.txt"))):
                self.combo_mission.addItem(os.path.basename(f), f)
        self.combo_mission.blockSignals(False)

    def _mission_confirm_discard(self):
        if not self.mission_dirty or not self.edit_mission.toPlainText().strip():
            return True
        r = QMessageBox.question(self, "Descartar alterações?",
                                 "A missão aberta tem alterações não salvas. Descartar?",
                                 QMessageBox.Yes | QMessageBox.No)
        return r == QMessageBox.Yes

    def on_combo_mission_changed(self, index):
        path = self.combo_mission.itemData(index)
        if path is None:
            return
        if not self._mission_confirm_discard():
            return
        try:
            with open(path, 'r') as f:
                self.edit_mission.setPlainText(f.read())
            self.current_mission_path = path
            self.mission_dirty = False
            self.status.showMessage(f"Missão aberta: {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Não consegui abrir:\n{e}")

    def mission_new(self):
        if not self._mission_confirm_discard():
            return
        self.edit_mission.setPlainText(self.MISSION_TEMPLATE)
        self.current_mission_path = None
        self.mission_dirty = True
        self.combo_mission.blockSignals(True)
        self.combo_mission.setCurrentIndex(0)
        self.combo_mission.blockSignals(False)
        self.status.showMessage("Missão nova. Use 'Salvar como...' para gravar em missoes/.")

    def mission_insert_template(self):
        self.edit_mission.insertPlainText(self.MISSION_TEMPLATE)

    def mission_refresh_places(self):
        """Preenche o combo de places com o que estiver carregado nas anotações."""
        self.combo_mission_place.clear()
        nomes = []
        for node in self.place_nodes:
            nomes.append(node.name)
        for raw in self.original_annotations:
            parts = raw.split()
            if len(parts) >= 7:
                try:
                    if int(parts[1]) == 13:
                        nomes.append(parts[0])
                except ValueError:
                    pass
        for n in sorted(set(nomes)):
            self.combo_mission_place.addItem(n)
        if not nomes:
            self.combo_mission_place.addItem("(nenhum place carregado)")

    def mission_insert_place(self):
        nome = self.combo_mission_place.currentText()
        if not nome or nome.startswith("("):
            QMessageBox.warning(self, "Aviso",
                                "Nenhum local carregado. Abra o arquivo de anotações no modo "
                                "'Editar Locais' primeiro.")
            return
        # O task_manager casa o nome exatamente como esta' na primeira coluna do annotations.txt
        self.edit_mission.insertPlainText(f"set course to place {nome}\ngo\n")
        self.mission_dirty = True

    def mission_start_pick_goal(self):
        if self.current_mode != 3:
            self.combo_mode.setCurrentIndex(3)
        if not self.maps:
            QMessageBox.warning(self, "Aviso", "Carregue a pasta do ambiente primeiro.")
            return
        self.pending_mission_goal = True
        self.view.setDragMode(QGraphicsView.NoDrag)
        self.view.viewport().setCursor(Qt.CrossCursor)
        self.status.showMessage("Clique no mapa onde o robô deve chegar e ARRASTE para definir a orientação.")

    def mission_insert_goal(self, gx, gy, theta):
        self.edit_mission.insertPlainText(
            f"set course to {gx:.3f} {gy:.3f} {theta:.6f}\ngo\n")
        self.mission_dirty = True
        self.status.showMessage(f"Destino inserido: {gx:.3f} {gy:.3f} {theta:.6f}")

    def mission_validate(self):
        """Procura as armadilhas que fazem a missao falhar em silencio."""
        txt = self.edit_mission.toPlainText()
        if not txt.strip():
            QMessageBox.information(self, "Verificação", "A missão está vazia.")
            return

        probs = []
        avisos = []
        linhas = txt.split("\n")
        tem_navegacao = False
        tem_stop = False

        for i, ln in enumerate(linhas, start=1):
            if not ln.strip():
                continue
            if ln[0] in (" ", "\t"):
                probs.append(f"linha {i}: começa com espaço/tab — o interpretador IGNORA essa linha.")
                continue
            corpo = ln.split("#", 1)[0].rstrip()
            if not corpo:
                continue
            if corpo.startswith("go after set course to"):
                probs.append(f"linha {i}: 'go after set course to' é uma task quebrada — "
                             "não publica o destino. Use 'set course to' + 'go'.")
            if corpo.startswith("set course to") or corpo.startswith("park at"):
                tem_navegacao = True
            if corpo == "stop":
                tem_stop = True
            # virgula dentro do nome da task (vira um espaco a mais e o strncmp nao bate)
            m = re.match(r"^([A-Za-z][A-Za-z_ ]*?),", corpo)
            if m and not re.match(r"^set (pose|local pose)", corpo):
                probs.append(f"linha {i}: vírgula dentro do nome da task — "
                             "ela vira um espaço a mais e a task deixa de ser reconhecida.")
            if corpo.startswith("execute task at label ") and "#" in ln:
                avisos.append(f"linha {i}: comentário na mesma linha de "
                              "'execute task at label' (já corrigido no task_manager, "
                              "mas ainda quebra no astro).")

        if not txt.endswith("\n"):
            probs.append("o arquivo não termina com quebra de linha — "
                         "o último caractere é comido pelo interpretador.")

        if tem_stop and not tem_navegacao:
            probs.append("tem 'stop' mas nenhuma navegação: o 'stop' espera o "
                         "model_predictive_planner confirmar, e ele só publica depois de ter "
                         "um goal. A missão TRAVA para sempre.")

        # labels usados x definidos
        definidos = set()
        for ln in linhas:
            t = ln.strip()
            if t.endswith(":") and " " not in t[:-1] and "\t" not in t[:-1] and t[:-1]:
                definidos.add(t[:-1])
        for i, ln in enumerate(linhas, start=1):
            m = re.search(r"execute task at label\s+(\S+)", ln.split("#", 1)[0])
            if m and m.group(1) not in definidos:
                probs.append(f"linha {i}: label '{m.group(1)}' não existe — "
                             "label não encontrado mata a missão com exit(1).")

        if probs:
            QMessageBox.warning(self, "Problemas encontrados",
                                "\n\n".join("• " + p for p in probs) +
                                (("\n\nAvisos:\n" + "\n".join("• " + a for a in avisos)) if avisos else ""))
        elif avisos:
            QMessageBox.information(self, "Avisos", "\n".join("• " + a for a in avisos))
        else:
            QMessageBox.information(self, "Verificação", "Nenhuma armadilha encontrada.")

    def _mission_normalize(self, txt):
        """Garante a quebra de linha final, que o interpretador exige."""
        if not txt.endswith("\n"):
            txt += "\n"
        return txt

    def mission_save(self):
        if not self.current_mission_path:
            return self.mission_save_as()
        try:
            with open(self.current_mission_path, 'w') as f:
                f.write(self._mission_normalize(self.edit_mission.toPlainText()))
            self.mission_dirty = False
            self.status.showMessage(f"Missão salva: {self.current_mission_path}")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao salvar:\n{e}")

    def mission_save_as(self):
        base = self.mission_dir or ""
        path, _ = QFileDialog.getSaveFileName(self, "Salvar missão",
                                              os.path.join(base, "minha_missao.txt"),
                                              "Text Files (*.txt)")
        if not path:
            return
        if not path.endswith(".txt"):
            path += ".txt"
        try:
            with open(path, 'w') as f:
                f.write(self._mission_normalize(self.edit_mission.toPlainText()))
            self.current_mission_path = path
            self.mission_dirty = False
            if not self.mission_dir:
                self.mission_set_dir(os.path.dirname(path))
            else:
                self.mission_scan_dir()
            idx = self.combo_mission.findData(path)
            if idx >= 0:
                self.combo_mission.blockSignals(True)
                self.combo_mission.setCurrentIndex(idx)
                self.combo_mission.blockSignals(False)
            self.status.showMessage(f"Missão salva: {path}")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao salvar:\n{e}")

    def mission_delete(self):
        path = self.current_mission_path
        if not path or not os.path.exists(path):
            QMessageBox.warning(self, "Aviso", "Nenhuma missão salva selecionada.")
            return
        r = QMessageBox.question(self, "Excluir missão?",
                                 f"Apagar definitivamente\n{path}?",
                                 QMessageBox.Yes | QMessageBox.No)
        if r != QMessageBox.Yes:
            return
        try:
            os.remove(path)
            self.current_mission_path = None
            self.edit_mission.clear()
            self.mission_dirty = False
            self.mission_scan_dir()
            self.status.showMessage("Missão excluída.")
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao excluir:\n{e}")

    def clear_rddf(self):
        self.original_rddf_data.clear()
        for node in self.rddf_nodes: self.scene.removeItem(node)
        for item in self.rddf_path_items: self.scene.removeItem(item)
        self.rddf_nodes.clear(); self.rddf_path_items.clear()
        self.combo_rddf.setCurrentIndex(0)
        self.current_rddf_filepath = None
        self.status.showMessage("RDDF Removido.")

    # =========================================================================
    # FUNÇÕES DE MAPA E ABERTURA DE PASTA GERAL
    # =========================================================================
    def set_tool(self, val, active):
        if self.current_mode != 0 and active:
            self.status.showMessage("Mude para MODO: MAPA se quiser pintar.")
            return
        self.current_value = val
        self.tool_active = active
        self.view.setDragMode(QGraphicsView.NoDrag if active else QGraphicsView.ScrollHandDrag)

    def change_brush_size(self, value):
        self.brush_size = value

    def undo_last_action(self):
        if self.last_edited_item: self.last_edited_item.undo()

    def closeEvent(self, event):
        unsaved = sum(1 for m in self.maps if m.is_modified)
        if unsaved > 0:
            if QMessageBox.question(self, "Sair?", "Mapas modificados. Salvar?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.save_all()
        if self.mission_dirty and self.edit_mission.toPlainText().strip():
            if QMessageBox.question(self, "Sair?", "A missão tem alterações não salvas. Salvar?",
                                    QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.mission_save()
        event.accept()

    def load_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Pasta do Ambiente")
        if not folder: return
        
        files = [f for f in glob.glob(os.path.join(folder, "*.map")) if os.path.basename(f).lower().startswith('m')]
        self.scene.clear()
        self.rddf_nodes.clear(); self.rddf_path_items.clear()
        self.clear_places()
        self.maps.clear()
        self.current_rddf_filepath = None
        self.current_annotations_filepath = None
        
        prog = QProgressDialog("Montando Mapas...", "Cancelar", 0, len(files), self)
        prog.setWindowModality(Qt.WindowModal); prog.show()
        
        min_x = float('inf')
        for i, f in enumerate(files):
            if prog.wasCanceled(): break
            prog.setValue(i)
            m_data = CarmenMapData(f)
            if m_data.grid is not None:
                self.maps.append(m_data)
                self.global_res = m_data.res 
                item = MapGraphicsItem(m_data, self)
                self.scene.addItem(item)
                if abs(item.pos().x()) < 1e9: min_x = min(min_x, item.pos().x())
        prog.setValue(len(files))
        
        if min_x != float('inf'):
            self.spin_res.blockSignals(True)
            self.spin_res.setValue(self.global_res)
            self.spin_res.blockSignals(False)
            
            self.combo_rddf.blockSignals(True); self.combo_rddf.clear()
            self.combo_rddf.addItem("-- Selecione um RDDF --", None)
            rddf_dir = next((os.path.join(folder, d) for d in os.listdir(folder) if d.lower() == 'rddf' and os.path.isdir(os.path.join(folder, d))), None)
            if rddf_dir:
                for rf in sorted(glob.glob(os.path.join(rddf_dir, "*.txt"))): self.combo_rddf.addItem(os.path.basename(rf), rf)
            self.combo_rddf.blockSignals(False)

            self.combo_ann.blockSignals(True); self.combo_ann.clear()
            self.combo_ann.addItem("-- Criar Novo / Vazio --", None)
            
            ann_dir = next((os.path.join(folder, d) for d in os.listdir(folder) if d.lower() == 'annotations' and os.path.isdir(os.path.join(folder, d))), None)
            ann_files = []
            if ann_dir:
                ann_files.extend(glob.glob(os.path.join(ann_dir, "*.txt")))
            ann_files.extend(glob.glob(os.path.join(folder, "*annotation*.txt")))
            ann_files = sorted(list(set(ann_files)))
            
            for af in ann_files: 
                self.combo_ann.addItem(os.path.basename(af), af)
            self.combo_ann.blockSignals(False)

            # Missoes: procura uma pasta 'missoes'/'missions' aqui ou um nivel acima
            mis_dir = None
            for base in (folder, os.path.dirname(folder), os.path.dirname(os.path.dirname(folder))):
                if not base or not os.path.isdir(base):
                    continue
                for d in os.listdir(base):
                    if d.lower() in ("missoes", "missions", "missao", "missions_folder"):
                        cand = os.path.join(base, d)
                        if os.path.isdir(cand):
                            mis_dir = cand
                            break
                if mis_dir:
                    break
            if mis_dir:
                self.mission_set_dir(mis_dir)
            else:
                self.mission_scan_dir()
            
            self.status.showMessage(
                f"Ambiente carregado. Mapas: {len(self.maps)} | RDDFs: {self.combo_rddf.count()-1} "
                f"| Anotações: {self.combo_ann.count()-1} | Missões: {self.combo_mission.count()-1}")
            
        else: QMessageBox.warning(self, "Aviso", "Nenhum mapa válido.")

    def manual_zoom(self, value):
        transform = self.view.transform()
        transform.reset()
        scale = value / 10.0
        transform.scale(scale, scale)
        self.view.setTransform(transform)

    def save_all(self):
        folder = QFileDialog.getExistingDirectory(self, "Salvar Novos Mapas")
        if not folder: return False
        c_copied = c_saved = 0
        for m in self.maps:
            new_fp = os.path.join(folder, m.filename)
            if m.is_modified:
                if m.save(new_fp): c_saved += 1
            else:
                try: shutil.copy2(m.filepath, new_fp); c_copied += 1
                except: pass
        QMessageBox.information(self, "Sucesso", f"Mapas: {c_saved} recriados, {c_copied} copiados originais.")
        return True

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MapEditor()
    win.show()
    sys.exit(app.exec_())