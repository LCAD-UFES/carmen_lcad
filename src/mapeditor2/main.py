import sys
import os
import glob
import subprocess
import shutil
import re
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QFileDialog, QLabel, 
                             QGraphicsView, QGraphicsScene, QGraphicsPixmapItem,
                             QSlider, QStatusBar, QMessageBox, QProgressDialog)
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen

# --- CONFIGURAÇÃO ---
# BRIDGE_EXE = "./carmen_bridge" 
EXE_READ = "./carmen_read"
EXE_SAVE = "./carmen_save"

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
        self.res = 0.2
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.text_header = []
        self.is_binary_c = False
        self.is_modified = False
        
        self.load()

    def parse_filename_coords(self):
        """Extrai coordenadas do nome do arquivo (ex: m7756700_-363930.map)"""
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
        # 1. Tenta carregar
        if not self.try_load_text():
            self.load_via_c_bridge()
            
        # 2. Ajusta coordenadas se vieram zeradas do C
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
                        k = parts[0]
                        v = parts[1]
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
                # --- CORREÇÃO 1: Mudei para order='C' (Row-Major) ---
                # Isso garante que o numpy leia linha por linha, como o C escreve.
                self.grid = np.array(raw_vals[:self.rows*self.cols], dtype=np.float32).reshape((self.cols, self.rows), order='C')
                return True
        except:
            pass
        return False

    # def load_via_c_bridge(self):
    #     if not os.path.exists(BRIDGE_EXE):
    #         return

    #     try:
    #         result = subprocess.run([BRIDGE_EXE, self.filepath], capture_output=True, text=True)
    #         lines = result.stdout.splitlines()
    #         data_started = False
    #         raw_vals = []

    #         for line in lines:
    #             if line == "BRIDGE_DATA_START":
    #                 data_started = True
    #                 continue
                
    #             if not data_started:
    #                 if line == "BRIDGE_HEADER_START": continue
    #                 parts = line.split()
    #                 if len(parts) >= 2:
    #                     if parts[0] == 'cols': self.cols = int(parts[1])
    #                     elif parts[0] == 'rows': self.rows = int(parts[1])
    #                     elif parts[0] == 'res': self.res = float(parts[1])
    #                     elif parts[0] == 'offset_x': self.offset_x = float(parts[1])
    #                     elif parts[0] == 'offset_y': self.offset_y = float(parts[1])
    #             else:
    #                 raw_vals.extend([float(x) for x in line.split()])

    #         if len(raw_vals) > 0:
    #             # --- CORREÇÃO 1 (Repetida): Mudei para order='C' aqui também ---
    #             self.grid = np.array(raw_vals, dtype=np.float32).reshape((self.cols, self.rows), order='C')
    #             self.is_binary_c = True

    #     except Exception as e:
    #         print(f"Erro na ponte: {e}")

    def load_via_c_bridge(self):
        if not os.path.exists(EXE_READ):
            print(f"Erro: {EXE_READ} não encontrado.")
            return

        try:
            # Chama o executável de leitura direto com o arquivo
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
                # O leitor C lê linha por linha, então usamos order='C'
                self.grid = np.array(raw_vals, dtype=np.float32).reshape((self.cols, self.rows), order='C')
                self.is_binary_c = True

        except Exception as e:
            print(f"Erro na ponte de leitura: {e}")

    # def save(self):
    #     if self.grid is None: return False
    #     try:
    #         if os.path.exists(self.filepath) and not os.path.exists(self.filepath + ".bak"):
    #             shutil.copy(self.filepath, self.filepath + ".bak")

    #         with open(self.filepath, 'w', encoding='utf-8') as f:
    #             if self.is_binary_c:
    #                 f.write(f"grid_map\n")
    #                 f.write(f"num_x {self.cols}\n")
    #                 f.write(f"num_y {self.rows}\n")
    #                 f.write(f"resolution {self.res}\n")
    #                 f.write(f"offset_x {self.offset_x}\n")
    #                 f.write(f"offset_y {self.offset_y}\n")
    #                 f.write(f"data\n")
    #             else:
    #                 for line in self.text_header:
    #                     f.write(line)
                
    #             # Salva usando order='C' para manter consistência com a leitura
    #             data_to_save = self.grid.flatten(order='C')
    #             np.savetxt(f, data_to_save, fmt='%.4g', delimiter=' ')
    #         return True
    #     except Exception as e:
    #         print(f"Erro ao salvar: {e}")
    #         return False

    # def save(self, new_path=None):
    #     if self.grid is None: return False
        
    #     target_path = new_path if new_path else self.filepath
        
    #     try:
    #         # --- MODO BINÁRIO (Usa carmen_save) ---
    #         if self.is_binary_c:
    #             if not os.path.exists(EXE_SAVE):
    #                 print("Executável de salvamento não encontrado.")
    #                 return False

    #             # SOLUÇÃO DO TRAVAMENTO: Usar arquivo temporário em vez de PIPE
    #             import tempfile
                
    #             # 1. Salva os dados num arquivo temporário
    #             # Isso evita que o buffer do sistema encha e trave o Python
    #             with tempfile.NamedTemporaryFile(mode='w+', delete=False) as tmp:
    #                 tmp_name = tmp.name
    #                 np.savetxt(tmp, self.grid.flatten(order='C'), fmt='%.4g', delimiter=' ')
                
    #             # 2. Prepara o comando C
    #             cmd = [
    #                 EXE_SAVE, target_path,
    #                 str(self.cols), str(self.rows), str(self.res)
    #             ]
                
    #             # 3. Executa o C mandando ele ler do arquivo temporário
    #             with open(tmp_name, 'r') as f_in:
    #                 process = subprocess.run(cmd, stdin=f_in, capture_output=True, text=True)
                
    #             # 4. Limpa o arquivo temporário
    #             os.remove(tmp_name)
                
    #             if process.returncode != 0:
    #                 print(f"Erro ao salvar binário: {process.stderr}")
    #                 return False
    #             return True

    #         # --- MODO TEXTO (Python Puro - mantido como fallback) ---
    #         else:
    #             with open(target_path, 'w', encoding='utf-8') as f:
    #                 for line in self.text_header:
    #                     f.write(line)
    #                 data_to_save = self.grid.flatten(order='C')
    #                 np.savetxt(f, data_to_save, fmt='%.4g', delimiter=' ')
    #             return True
                
    #     except Exception as e:
    #         print(f"Erro ao salvar: {e}")
    #         if 'tmp_name' in locals() and os.path.exists(tmp_name):
    #             os.remove(tmp_name)
    #         return False

    def save(self, new_path=None):
        if self.grid is None: return False
        
        target_path = new_path if new_path else self.filepath
        
        try:
            # --- MODO BINÁRIO (Usa carmen_save em C) ---
            if self.is_binary_c:
                if not os.path.exists(EXE_SAVE):
                    print("Executável de salvamento não encontrado.")
                    return False

                # 1. Cria arquivo temporário com os dados
                # Isso evita o "crash" se o mapa for muito grande
                import tempfile
                with tempfile.NamedTemporaryFile(mode='w+', delete=False) as tmp:
                    tmp_name = tmp.name
                    np.savetxt(tmp, self.grid.flatten(order='C'), fmt='%.4g', delimiter=' ')
                
                # 2. Chama o seu programa C
                cmd = [
                    EXE_SAVE, target_path,
                    str(self.cols), str(self.rows), str(self.res)
                ]
                
                # O C lê do arquivo temporário em vez do teclado
                with open(tmp_name, 'r') as f_in:
                    process = subprocess.run(cmd, stdin=f_in, capture_output=True, text=True)
                
                # 3. Limpa a sujeira
                os.remove(tmp_name)
                
                if process.returncode != 0:
                    print(f"Erro no C: {process.stderr}")
                    return False
                return True

            # --- MODO TEXTO (Fallback) ---
            else:
                with open(target_path, 'w', encoding='utf-8') as f:
                    for line in self.text_header:
                        f.write(line)
                    data_to_save = self.grid.flatten(order='C')
                    np.savetxt(f, data_to_save, fmt='%.4g', delimiter=' ')
                return True
                
        except Exception as e:
            print(f"Erro ao salvar: {e}")
            if 'tmp_name' in locals() and os.path.exists(tmp_name):
                os.remove(tmp_name)
            return False

# =============================================================================
# ITEM GRÁFICO (CORREÇÃO DE CHUVISCO E LOGS)
# =============================================================================
# class MapGraphicsItem(QGraphicsPixmapItem):
#     def __init__(self, map_data, brush_manager):
#         super().__init__()
#         self.map_data = map_data
#         self.brush_manager = brush_manager

#         # --- NOVIDADE: A Pilha de Histórico ---
#         self.history = []
        
#         self.update_image()
        
#         # Posicionamento
#         if map_data.res != 0:
#             px_x = map_data.offset_x / map_data.res
#             px_y = -(map_data.offset_y / map_data.res) - map_data.rows
#         else:
#             px_x, px_y = 0, 0
        
#         self.setPos(px_x, px_y)
#         self.setAcceptHoverEvents(True)

#     # --- NOVIDADE: Métodos de Undo ---
#     def save_history(self):
#         # Limite de segurança (ex: 10 passos)
#         if len(self.history) >= 10:
#             self.history.pop(0) # Remove o mais antigo (base da pilha)
        
#         # Salva uma CÓPIA do grid atual na pilha
#         self.history.append(self.map_data.grid.copy())

#     def undo(self):
#         if len(self.history) > 0:
#             # Pega o último estado salvo e restaura
#             previous_grid = self.history.pop()
#             self.map_data.grid = previous_grid
#             self.update_image()
#             print("Desfeito! Histórico restante:", len(self.history))
#         else:
#             print("Nada para desfazer neste mapa.")

#     # --- NOVA MECÂNICA: Rastreamento do Mouse ---
#     def hoverMoveEvent(self, event):
#         # 1. Pega a posição do mouse em pixels DENTRO desta imagem específica
#         pos = event.pos()
#         pixel_x = int(pos.x())
#         # Lembra que o Y na tela é invertido em relação ao grid do Carmen?
#         pixel_y = self.map_data.rows - 1 - int(pos.y())
        
#         # 2. Calcula a coordenada Global (Mundo Real)
#         # Fórmula: Origem + (Pixel * Resolução)
#         global_x = self.map_data.offset_x + (pixel_x * self.map_data.res)
#         global_y = self.map_data.offset_y + (pixel_y * self.map_data.res)
        
#         # 3. Formata a mensagem
#         msg = (f"Arquivo: {self.map_data.filename} | "
#                f"Grid[{pixel_x}, {pixel_y}] | "
#                f"Global({global_x:.2f}, {global_y:.2f})")
        
#         # 4. Envia para a barra de status
#         # CORREÇÃO AQUI: Usar self.brush_manager em vez de self.main_window
#         if hasattr(self.brush_manager, 'status'):
#             self.brush_manager.status.showMessage(msg)
        
#         # Chama o evento original
#         super().hoverMoveEvent(event)

#     def update_image(self):
#         if self.map_data.grid is None: return
        
#         # 1. Tratamento de dados
#         # Mantemos os valores originais para identificar -1 (desconhecido) e -2 (offlimits)
#         # Substituímos NaN por -1 por segurança
#         grid_data = np.nan_to_num(self.map_data.grid, nan=-1.0)
        
#         # Transpõe para bater com a visualização (Y, X)
#         grid_T = grid_data.T
        
#         h, w = grid_T.shape
        
#         # 2. Criação da Imagem RGB (Altura, Largura, 3 canais de cor)
#         # Inicializamos tudo branco (255)
#         rgb_data = np.full((h, w, 3), 255, dtype=np.uint8)
        
#         # --- APLICANDO AS CORES DO CARMEN ---
        
#         # Obstáculos (Probabilidade >= 0.5) -> Preto [0, 0, 0]
#         mask_obstacle = (grid_T >= 0.5)
#         rgb_data[mask_obstacle] = [0, 0, 0]
        
#         # Desconhecido (Valor entre 0 e -2) -> Azul [0, 0, 255]
#         mask_unknown = (grid_T < 0) & (grid_T > -2)
#         rgb_data[mask_unknown] = [0, 0, 255]
        
#         # Offlimits/Proibido (Valor <= -2) -> Vermelho [255, 0, 0]
#         mask_offlimits = (grid_T <= -2)
#         rgb_data[mask_offlimits] = [255, 0, 0]
        
#         # 3. Alinhamento de Memória (Crucial para QImage)
#         final_data = np.ascontiguousarray(rgb_data)
        
#         # Bytes por linha: largura * 3 canais (R, G, B)
#         bytes_per_line = w * 3
        
#         # --- SISTEMA DE LOGS (MANTIDO) ---
#         try:
#             with open("render_log.txt", "a") as f:
#                 f.write("--- Nova Renderização (RGB) ---\n")
#                 f.write(f"Arquivo: {self.map_data.filename}\n")
#                 f.write(f"Grid Transposto Shape: {grid_T.shape}\n")
#                 f.write(f"Final Data Shape (h, w, c): {final_data.shape}\n")
#                 f.write(f"Final Data Strides: {final_data.strides}\n")
#                 f.write(f"QImage espera: w={w}, h={h}, bytesPerLine={bytes_per_line}\n")
#                 f.write("-" * 30 + "\n")
#         except Exception as e:
#             print(f"Erro ao logar: {e}")

#         # Cria a QImage com formato RGB888
#         qimg = QImage(final_data.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
#         # Espelha a imagem (QImage) antes de converter para Pixmap
#         self.setPixmap(QPixmap.fromImage(qimg.mirrored(False, True)))

#     def mousePressEvent(self, event):
#         if self.brush_manager.tool_active:
#             self.save_history()
#             self.brush_manager.last_edited_item = self
#             self.paint_on_grid(event.pos())
#         else:
#             super().mousePressEvent(event)

#     def mouseMoveEvent(self, event):
#         if self.brush_manager.tool_active:
#             self.paint_on_grid(event.pos())
#         else:
#             super().mouseMoveEvent(event)

#     # def paint_on_grid(self, pos):
#     #     x = int(pos.x())
#     #     y = self.map_data.rows - 1 - int(pos.y())
        
#     #     if 0 <= x < self.map_data.cols and 0 <= y < self.map_data.rows:
#     #         self.map_data.grid[x, y] = self.brush_manager.current_value
#     #         self.update_image()

#     def paint_on_grid(self, pos):
#         # Converte posição do mouse (pixels da imagem) para índices do grid
#         cx = int(pos.x())
#         cy = self.map_data.rows - 1 - int(pos.y())
        
#         # Pega o tamanho do slider da janela principal
#         size = self.brush_manager.brush_size
#         radius = size // 2 
        
#         # Calcula os limites do quadrado (sem sair para fora do mapa)
#         min_x = max(0, cx - radius)
#         max_x = min(self.map_data.cols, cx + radius + 1)
        
#         min_y = max(0, cy - radius)
#         max_y = min(self.map_data.rows, cy + radius + 1)
        
#         val = self.brush_manager.current_value
        
#         # Pinta a região inteira de uma vez (slice do numpy)
#         self.map_data.grid[min_x:max_x, min_y:max_y] = val
        
#         self.update_image()

# =============================================================================
# ITEM GRÁFICO (CORREÇÃO: MARCAR COMO MODIFICADO)
# =============================================================================
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
        else:
            px_x, px_y = 0, 0
        
        self.setPos(px_x, px_y)
        self.setAcceptHoverEvents(True)

    def save_history(self):
        if len(self.history) >= 10:
            self.history.pop(0)
        self.history.append(self.map_data.grid.copy())

    def undo(self):
        if len(self.history) > 0:
            previous_grid = self.history.pop()
            self.map_data.grid = previous_grid
            
            # --- CORREÇÃO 1: Undo também conta como modificação ---
            self.map_data.is_modified = True
            
            self.update_image()
            print("Desfeito! Histórico restante:", len(self.history))
        else:
            print("Nada para desfazer neste mapa.")

    def hoverMoveEvent(self, event):
        pos = event.pos()
        pixel_x = int(pos.x())
        pixel_y = self.map_data.rows - 1 - int(pos.y())
        global_x = self.map_data.offset_x + (pixel_x * self.map_data.res)
        global_y = self.map_data.offset_y + (pixel_y * self.map_data.res)
        
        # Dica visual: Mostra se está modificado na barra de status
        mod_status = "[MODIFICADO]" if self.map_data.is_modified else ""
        
        msg = (f"{mod_status} Arquivo: {self.map_data.filename} | "
               f"Grid[{pixel_x}, {pixel_y}] | "
               f"Global({global_x:.2f}, {global_y:.2f})")
        
        if hasattr(self.brush_manager, 'status'):
            self.brush_manager.status.showMessage(msg)
        super().hoverMoveEvent(event)

    def update_image(self):
        if self.map_data.grid is None: return
        grid_data = np.nan_to_num(self.map_data.grid, nan=-1.0)
        grid_T = grid_data.T
        h, w = grid_T.shape
        rgb_data = np.full((h, w, 3), 255, dtype=np.uint8)
        mask_obstacle = (grid_T >= 0.5)
        rgb_data[mask_obstacle] = [0, 0, 0]
        mask_unknown = (grid_T < 0) & (grid_T > -2)
        rgb_data[mask_unknown] = [0, 0, 255]
        mask_offlimits = (grid_T <= -2)
        rgb_data[mask_offlimits] = [255, 0, 0]
        final_data = np.ascontiguousarray(rgb_data)
        bytes_per_line = w * 3
        qimg = QImage(final_data.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.setPixmap(QPixmap.fromImage(qimg.mirrored(False, True)))

    # def mousePressEvent(self, event):
    #     if self.brush_manager.tool_active:
    #         self.save_history()
    #         self.brush_manager.last_edited_item = self
            
    #         # --- CORREÇÃO 2: AVISSAR QUE FOI MODIFICADO ---
    #         # Sem isso, o save_all acha que o mapa é original e não salva as mudanças
    #         self.map_data.is_modified = True
            
    #         self.paint_on_grid(event.pos())
    #     else:
    #         super().mousePressEvent(event)

    # def mouseMoveEvent(self, event):
    #     if self.brush_manager.tool_active:
    #         self.paint_on_grid(event.pos())
    #     else:
    #         super().mouseMoveEvent(event)

    def mousePressEvent(self, event):
        if self.brush_manager.tool_active:
            # --- INÍCIO DA PINCELADA ---
            # Criamos um conjunto (set) na janela principal para lembrar 
            # quais mapas já salvaram histórico nesta "arrastada" atual.
            self.brush_manager.active_stroke_chunks = set()
            
            # Salva histórico deste mapa atual e adiciona na lista
            self.save_history()
            self.brush_manager.active_stroke_chunks.add(self)
            
            self.brush_manager.last_edited_item = self
            self.map_data.is_modified = True
            
            self.paint_on_grid(event.pos())
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.brush_manager.tool_active:
            # --- PINTURA DINÂMICA ENTRE CHUNKS ---
            
            # 1. Pega a posição global do mouse na Cena (não apenas neste item)
            scene_pos = event.scenePos()
            
            # 2. Pergunta à cena: "O que tem debaixo do mouse agora?"
            items = self.scene().items(scene_pos)
            
            for item in items:
                # Verifica se o item encontrado é um pedaço de mapa
                if isinstance(item, MapGraphicsItem):
                    
                    # 3. Gerencia o Histórico (Só salva 1x por mapa nesta pincelada)
                    # Verifica se esse conjunto existe (segurança) e se o item já está lá
                    if hasattr(self.brush_manager, 'active_stroke_chunks'):
                        if item not in self.brush_manager.active_stroke_chunks:
                            item.save_history() # Salva backup deste novo mapa
                            self.brush_manager.active_stroke_chunks.add(item)
                            
                            # Atualiza o "último editado" para o Undo funcionar nele
                            self.brush_manager.last_edited_item = item
                    
                    # 4. Converte a coordenada global para a coordenada local DO VIZINHO
                    local_pos = item.mapFromScene(scene_pos)
                    
                    # Marca como modificado
                    item.map_data.is_modified = True
                    
                    # Pinta no vizinho!
                    item.paint_on_grid(local_pos)
                    
                    # Encontrou o mapa do topo, pode parar de procurar
                    break 
        else:
            super().mouseMoveEvent(event)

    # def paint_on_grid(self, pos):
    #     cx = int(pos.x())
    #     cy = self.map_data.rows - 1 - int(pos.y())
        
    #     size = self.brush_manager.brush_size
    #     radius = size // 2 
        
    #     min_x = max(0, cx - radius)
    #     max_x = min(self.map_data.cols, cx + radius + 1)
    #     min_y = max(0, cy - radius)
    #     max_y = min(self.map_data.rows, cy + radius + 1)
        
    #     val = self.brush_manager.current_value
        
    #     self.map_data.grid[min_x:max_x, min_y:max_y] = val
    #     self.update_image()

    def paint_on_grid(self, pos):
        cx = int(pos.x())
        cy = self.map_data.rows - 1 - int(pos.y())
        
        size = self.brush_manager.brush_size
        radius = size // 2 
        
        # --- CORREÇÃO DO BUG DE PREENCHIMENTO ---
        # Usamos max(0, ...) para garantir que nunca seja negativo.
        # Se for negativo, o Python entende como "índice reverso" e pinta tudo errado.
        min_x = max(0, cx - radius)
        max_x = max(0, min(self.map_data.cols, cx + radius + 1))
        
        min_y = max(0, cy - radius)
        max_y = max(0, min(self.map_data.rows, cy + radius + 1))
        
        # Se estiver fora da área válida, não faz nada
        if min_x >= max_x or min_y >= max_y:
            return
        
        val = self.brush_manager.current_value
        self.map_data.grid[min_x:max_x, min_y:max_y] = val
        self.update_image()

# =============================================================================
# VISUALIZADOR PERSONALIZADO (NOVA CLASSE PARA ZOOM)
# =============================================================================
class CanvasView(QGraphicsView):
    def __init__(self, scene):
        super().__init__(scene)
        # Configurações de renderização para qualidade
        self.setRenderHint(QPainter.Antialiasing, False)
        self.setRenderHint(QPainter.SmoothPixmapTransform, False)
        
        # O pulo do gato: Zoom onde o mouse está
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)
        
        # Configuração padrão de arraste
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setBackgroundBrush(QColor(40, 40, 40))

    def wheelEvent(self, event):
        # Captura o scroll do mouse para dar Zoom
        zoom_in_factor = 1.15
        zoom_out_factor = 1 / zoom_in_factor

        # Se girar para frente (>0), zoom in. Senão, zoom out.
        if event.angleDelta().y() > 0:
            self.scale(zoom_in_factor, zoom_in_factor)
        else:
            self.scale(zoom_out_factor, zoom_out_factor)

# =============================================================================
# JANELA PRINCIPAL
# =============================================================================
class MapEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MAP EDITOR 2")
        self.resize(1200, 800)
        self.current_value = 1.0 
        self.tool_active = False

        self.last_edited_item = None

        main = QWidget()
        layout = QVBoxLayout()
        
        # Toolbar
        tools = QHBoxLayout()
        btn_load = QPushButton("📂 Abrir Pasta")
        btn_load.clicked.connect(self.load_folder)
        
        btn_save = QPushButton("💾 Salvar")
        btn_save.setStyleSheet("background-color: #388E3C; color: white; font-weight: bold;")
        btn_save.clicked.connect(self.save_all)
        
        tools.addWidget(QLabel("Ferramentas:"))
        btn_move = QPushButton("✋ Mover")
        btn_move.clicked.connect(lambda: self.set_tool(0.0, False))
        
        btn_pen = QPushButton("✏️ Obstáculo")
        btn_pen.clicked.connect(lambda: self.set_tool(1.0, True))
        
        btn_eraser = QPushButton("⬜ Borracha")
        btn_eraser.clicked.connect(lambda: self.set_tool(0.0, True))

        # --- NOVIDADE: Botão Desconhecido (Azul) ---
        btn_unknown = QPushButton("🔵 Desconhecido")
        # Passamos o valor -1.0, que seu visualizador já sabe que deve pintar de azul
        btn_unknown.clicked.connect(lambda: self.set_tool(-1.0, True))

        # --- NOVIDADE: Botão Desfazer ---
        btn_undo = QPushButton("↩️ Desfazer")
        btn_undo.setShortcut("Ctrl+Z") # Atalho de teclado
        btn_undo.clicked.connect(self.undo_last_action)
        
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(1, 300)
        self.zoom_slider.setValue(20)
        self.zoom_slider.valueChanged.connect(self.manual_zoom)
        
        tools.addWidget(btn_load)
        tools.addWidget(btn_move)
        tools.addWidget(btn_pen)
        tools.addWidget(btn_eraser)
        tools.addWidget(btn_unknown)
        tools.addWidget(btn_undo)

        self.brush_size = 1
        
        lbl_size = QLabel("Tamanho:")
        self.size_slider = QSlider(Qt.Horizontal)
        self.size_slider.setRange(1, 10) # Tamanho de 1 a 10
        self.size_slider.setValue(1)
        self.size_slider.setFixedWidth(100)
        self.size_slider.valueChanged.connect(self.change_brush_size)
        
        tools.addWidget(lbl_size)
        tools.addWidget(self.size_slider)

        tools.addWidget(QLabel("Zoom:"))
        tools.addWidget(self.zoom_slider)
        tools.addStretch()
        tools.addWidget(btn_save)
        layout.addLayout(tools)


        # Viewer
        self.scene = QGraphicsScene()
        self.view = CanvasView(self.scene)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.view.setBackgroundBrush(QColor(40, 40, 40))
        
        self.view.setRenderHint(QPainter.Antialiasing, False)
        self.view.setRenderHint(QPainter.SmoothPixmapTransform, False)
        # --- NOVA MECÂNICA: Configuração do Zoom ---
        # Isso faz o zoom acontecer onde o mouse está apontando
        self.view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.view.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.view.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)
        
        layout.addWidget(self.view)
        
        main.setLayout(layout)
        self.setCentralWidget(main)
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        
        self.maps = []

    def set_tool(self, val, active):
        self.current_value = val
        self.tool_active = active
        if active:
            tool = "Obstáculo" if val > 0.5 else "Borracha"
            self.status.showMessage(f"Modo: {tool}")
            self.view.setDragMode(QGraphicsView.NoDrag)
        else:
            self.status.showMessage("Modo: Navegação")
            self.view.setDragMode(QGraphicsView.ScrollHandDrag)

    def closeEvent(self, event):
        # 1. Verifica quantos mapas foram modificados
        unsaved_count = sum(1 for m in self.maps if m.is_modified)

        # 2. Se houver alterações não salvas...
        if unsaved_count > 0:
            msg = f"Existem {unsaved_count} mapas modificados não salvos.\nDeseja salvar antes de sair?"
            
            reply = QMessageBox.question(
                self, 
                "Salvar Alterações?", 
                msg,
                QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
                QMessageBox.Save
            )

            if reply == QMessageBox.Save:
                # Tenta salvar. Se o save_all retornar True (sucesso), aceita fechar.
                # Se retornar False (usuário cancelou a pasta), ignora o fechamento.
                if self.save_all():
                    event.accept()
                else:
                    event.ignore()
            
            elif reply == QMessageBox.Discard:
                # Descarta alterações e fecha
                event.accept()
            
            else:
                # Cancelou (não fecha a janela)
                event.ignore()
        else:
            # Se não tem nada modificado, fecha direto
            event.accept()

    def undo_last_action(self):
        if self.last_edited_item:
            self.last_edited_item.undo()
            self.status.showMessage("Última ação desfeita.")
        else:
            self.status.showMessage("Nada para desfazer ainda.")

    def change_brush_size(self, value):
        self.brush_size = value
        self.status.showMessage(f"Tamanho do Pincel: {value}")

    # def load_folder(self):
    #     # Aviso se não existir o bridge, mas não impede de rodar se for só texto
    #     if not os.path.exists(BRIDGE_EXE):
    #         print("Aviso: carmen_bridge não encontrado. Tentando modo texto apenas.")

    #     folder = QFileDialog.getExistingDirectory(self, "Pasta com mapas")
    #     if not folder: return
        
    #     files = glob.glob(os.path.join(folder, "*.map"))
    #     self.scene.clear()
    #     self.maps.clear()
        
    #     prog = QProgressDialog("Montando...", "Cancelar", 0, len(files), self)
    #     prog.setWindowModality(Qt.WindowModal)
    #     prog.show()
        
    #     loaded = 0
    #     min_x, min_y = float('inf'), float('inf')
    #     max_x, max_y = float('-inf'), float('-inf')

    #     for i, f in enumerate(files):
    #         if prog.wasCanceled(): break
    #         prog.setValue(i)
            
    #         m_data = CarmenMapData(f)
    #         if m_data.grid is not None:
    #             self.maps.append(m_data)
    #             item = MapGraphicsItem(m_data, self)
    #             self.scene.addItem(item)
                
    #             pos = item.pos()
    #             # Evita problemas com coordenadas infinitas
    #             if abs(pos.x()) < 1e9 and abs(pos.y()) < 1e9: 
    #                 min_x = min(min_x, pos.x())
    #                 min_y = min(min_y, pos.y())
    #                 max_x = max(max_x, pos.x() + item.pixmap().width())
    #                 max_y = max(max_y, pos.y() + item.pixmap().height())
                
    #             loaded += 1
        
    #     prog.setValue(len(files))

    def load_folder(self):
        # --- CORREÇÃO: Verificamos se o leitor (EXE_READ) existe ---
        if not os.path.exists(EXE_READ):
            print(f"Aviso: {EXE_READ} não encontrado. Tentando modo texto apenas.")

        folder = QFileDialog.getExistingDirectory(self, "Pasta com mapas")
        if not folder: return
        
        # Busca todos os mapas, mas filtra apenas os que começam com a letra 'M' ou 'm'
        todos_arquivos = glob.glob(os.path.join(folder, "*.map"))
        files = [f for f in todos_arquivos if os.path.basename(f).lower().startswith('m')]
        self.scene.clear()
        self.maps.clear()
        
        prog = QProgressDialog("Montando...", "Cancelar", 0, len(files), self)
        prog.setWindowModality(Qt.WindowModal)
        prog.show()
        
        loaded = 0
        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = float('-inf'), float('-inf')

        for i, f in enumerate(files):
            if prog.wasCanceled(): break
            prog.setValue(i)
            
            m_data = CarmenMapData(f)
            if m_data.grid is not None:
                self.maps.append(m_data)
                item = MapGraphicsItem(m_data, self)
                self.scene.addItem(item)
                
                pos = item.pos()
                if abs(pos.x()) < 1e9 and abs(pos.y()) < 1e9: 
                    min_x = min(min_x, pos.x())
                    min_y = min(min_y, pos.y())
                    max_x = max(max_x, pos.x() + item.pixmap().width())
                    max_y = max(max_y, pos.y() + item.pixmap().height())
                
                loaded += 1
        
        prog.setValue(len(files))
        
        if loaded > 0 and min_x != float('inf'):
            self.status.showMessage(f"{loaded} mapas carregados.")
            rect = QRectF(min_x, min_y, max_x - min_x, max_y - min_y)
            self.view.setSceneRect(rect)
            self.view.fitInView(rect, Qt.KeepAspectRatio)
        else:
            QMessageBox.warning(self, "Aviso", "Nenhum mapa válido encontrado.")
        
        if loaded > 0 and min_x != float('inf'):
            self.status.showMessage(f"{loaded} mapas carregados.")
            rect = QRectF(min_x, min_y, max_x - min_x, max_y - min_y)
            self.view.setSceneRect(rect)
            self.view.fitInView(rect, Qt.KeepAspectRatio)
        else:
            QMessageBox.warning(self, "Aviso", "Nenhum mapa válido encontrado.")

    def manual_zoom(self, value):
        scale = value / 10.0
        transform = self.view.transform()
        transform.reset()
        transform.scale(scale, scale)
        self.view.setTransform(transform)

    # def save_all(self):
    #     if QMessageBox.question(self, "Salvar", "Confirmar?") == QMessageBox.Yes:
    #         count = 0
    #         prog = QProgressDialog("Salvando...", None, 0, len(self.maps), self)
    #         prog.show()
    #         for i, m in enumerate(self.maps):
    #             prog.setValue(i)
    #             if m.save(): count += 1
    #         QMessageBox.information(self, "Fim", f"{count} salvos.")

    # def save_all(self):
    #     # Abre diálogo para selecionar pasta
    #     folder = QFileDialog.getExistingDirectory(self, "Escolha pasta para salvar novos mapas")
    #     if not folder: return

    #     count = 0
    #     prog = QProgressDialog("Salvando em nova pasta...", None, 0, len(self.maps), self)
    #     prog.show()
        
    #     for i, m in enumerate(self.maps):
    #         prog.setValue(i)
            
    #         # Cria o caminho completo: Pasta Nova / Nome do Arquivo Original
    #         new_filepath = os.path.join(folder, m.filename)
            
    #         # Chama o método save passando o novo caminho
    #         if m.save(new_filepath): 
    #             count += 1
                
    #     QMessageBox.information(self, "Fim", f"{count} mapas salvos em:\n{folder}")

    def save_all(self):
        # Seleciona pasta de destino
        folder = QFileDialog.getExistingDirectory(self, "Escolha pasta para salvar novos mapas")
        if not folder: return False

        # --- CORREÇÃO DO TRAVAMENTO VISUAL ---
        # Força o fechamento da janela de diálogo antes de começar a salvar
        QApplication.processEvents()
        # -------------------------------------

        count_copied = 0
        count_saved = 0
        
        prog = QProgressDialog("Processando...", None, 0, len(self.maps), self)
        prog.setWindowModality(Qt.WindowModal)
        prog.show()
        
        for i, m in enumerate(self.maps):
            prog.setValue(i)
            # Mantém a barra de progresso viva
            QApplication.processEvents()
            
            new_filepath = os.path.join(folder, m.filename)
            
            # Se foi modificado, chama o save (que vai chamar o C)
            if m.is_modified:
                if m.save(new_filepath):
                    count_saved += 1
            else:
                # Se não, apenas copia o arquivo original (muito rápido)
                try:
                    shutil.copy2(m.filepath, new_filepath)
                    count_copied += 1
                except Exception as e:
                    print(f"Erro ao copiar {m.filename}: {e}")

        prog.setValue(len(self.maps))

        msg = (f"Concluído!\n\n"
               f"Salvos (Recriados pelo C): {count_saved}\n"
               f"Copiados (Originais): {count_copied}\n"
               f"Total: {count_saved + count_copied}")
        
        QMessageBox.information(self, "Relatório", msg)

        return True

    def wheelEvent(self, event):
        # Se segurar Control, dá zoom (padrão de interfaces)
        # Se quiser zoom direto sem Control, remova o "if" e deixe só o código de dentro
        if event.modifiers() & Qt.ControlModifier:
            zoom_in_factor = 1.15
            zoom_out_factor = 1 / zoom_in_factor

            # Se o scroll foi para frente (>0), zoom in. Senão, zoom out.
            if event.angleDelta().y() > 0:
                self.view.scale(zoom_in_factor, zoom_in_factor)
            else:
                self.view.scale(zoom_out_factor, zoom_out_factor)
        else:
            # Comportamento padrão (scroll da barra lateral) se não segurar Ctrl
            super().wheelEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MapEditor()
    win.show()
    sys.exit(app.exec_())