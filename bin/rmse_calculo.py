import sys
import math
import argparse
import matplotlib.pyplot as plt
from time import time

def get_args():
    parser = argparse.ArgumentParser(description="Calcula RMSE Unificado.")
    parser.add_argument("graph_file", help="Arquivo do Grafo (.gr)")
    parser.add_argument("traj_file", help="Arquivo da Trajetória (.txt)")
    # Filtros
    parser.add_argument("--max_y", type=float, default=float('inf'))
    parser.add_argument("--min_y", type=float, default=float('-inf'))
    parser.add_argument("--max_x", type=float, default=float('inf'))
    parser.add_argument("--min_x", type=float, default=float('-inf'))
    parser.add_argument("--distance_threshold", type=float, default=float(2.0))
    parser.add_argument("--no_plot", action="store_true", help="Não mostrar gráfico")
    return parser.parse_args()

def check_filters(x, y, args):
    if x > args.max_x or x < args.min_x: return False
    if y > args.max_y or y < args.min_y: return False
    return True

def ler_dados(args):
    nodes = {}
    edges = []
    traj = []
    
    try:
        with open(args.graph_file, 'r') as f:
            for line in f:
                parts = line.split()
                if not parts: continue
                if parts[0] == 'NODE':
                    try:
                        nid = int(parts[1])
                        x, y = float(parts[4]), float(parts[5])
                        if check_filters(x, y, args):
                            nodes[nid] = (x, y)
                    except: continue
                elif parts[0].startswith('EDGE') and len(parts) >= 3:
                    try:
                        u, v = int(parts[1]), int(parts[2])
                        edges.append((u, v))
                    except: continue
    except FileNotFoundError: sys.exit("Grafo não encontrado")

    # Arestas válidas (somente se ambos os nós passaram no filtro)
    valid_edges = [e for e in edges if e[0] in nodes and e[1] in nodes]

    # --- LER TRAJETÓRIA ---
    pts_total = 0
    try:
        with open(args.traj_file, 'r') as f:
            for line in f:
                parts = line.split()
                if len(parts) < 2: continue
                try:
                    x, y = float(parts[0]), float(parts[1])
                    pts_total += 1
                    if check_filters(x, y, args):
                        traj.append((x, y))
                except: continue
    except FileNotFoundError: sys.exit("Trajetória não encontrada")
    
    return nodes, valid_edges, traj, pts_total

def projetar(px, py, x1, y1, x2, y2):
    vx, vy = x2 - x1, y2 - y1
    l2 = vx*vx + vy*vy
    if l2 == 0.0: return (px - x1)**2 + (py - y1)**2, x1, y1
    t = ((px - x1) * vx + (py - y1) * vy) / l2
    if t < 0.0: t = 0.0
    elif t > 1.0: t = 1.0
    proj_x = x1 + t * vx
    proj_y = y1 + t * vy
    return (px - proj_x)**2 + (py - proj_y)**2, proj_x, proj_y

def main():
    start_time = time()
    args = get_args()
    nodes, edges, traj, total_lidos = ler_dados(args)
    
    print("="*60)
    print(f"RELATÓRIO DE DADOS")
    print(f"Filtro Y:      {args.min_y} a {args.max_y}")
    print(f"Pontos Totais: {total_lidos}")
    print(f"Pontos Usados: {len(traj)} (Removidos: {total_lidos - len(traj)})")
    print(f"Arestas Grafo: {len(edges)}")
    print("="*60)

    if not traj or not nodes:
        print("ERRO: Sobraram 0 pontos após o filtro.")
        return

    soma_erro = 0
    max_erro = 0
    max_erro_coord = (0,0)
    
    plot_x, plot_y, proj_x_list, proj_y_list, colors = [], [], [], [], []

    for px, py in traj:
        min_dist = float('inf')
        best_p = (px, py)
        
        for u, v in edges:
            x1, y1 = nodes[u]
            x2, y2 = nodes[v]
            # Ignora ínicio das retas muito distantes
            if (px-x1)**2 + (py-y1)**2 > args.distance_threshold**2:
                continue
            d2, prj_x, prj_y = projetar(px, py, x1, y1, x2, y2)
            if d2 < min_dist:
                min_dist = d2
                best_p = (prj_x, prj_y)
        
        dist = math.sqrt(min_dist)
        soma_erro += min_dist
        if dist > max_erro: 
            max_erro = dist
            max_erro_coord = (px, py)
            
        plot_x.append(px)
        plot_y.append(py)
        proj_x_list.append(best_p[0])
        proj_y_list.append(best_p[1])
        colors.append('r--' if dist > 0.1 else 'g--') # Vermelho se erro > 10cm

    rmse = math.sqrt(soma_erro / len(traj))
    
    print(f"RESULTADOS:")
    print(f"RMSE:          {rmse:.6f} m")
    print(f"Erro Máximo:   {max_erro:.6f} m")
    print(f"Local Pior Pt: {max_erro_coord}")
    print(f"Tempo de Execução: {time()-start_time:.3f}s")
    print("="*60)

    if not args.no_plot:
        plt.figure(figsize=(10, 8))
        # Grafo
        for u, v in edges:
            x1, y1 = nodes[u]
            x2, y2 = nodes[v]
            plt.plot([x1, x2], [y1, y2], 'b-', alpha=0.3)
        
        # Erros
        for i in range(len(plot_x)):
            plt.plot([plot_x[i], proj_x_list[i]], [plot_y[i], proj_y_list[i]], colors[i], linewidth=0.5)
            
        plt.scatter(plot_x, plot_y, c='red', s=5, label='Robô', zorder=5)
        
        # Highlight no pior erro
        plt.scatter([max_erro_coord[0]], [max_erro_coord[1]], c='black', s=50, marker='x', label='Pior Erro')
        
        plt.title(f"RMSE: {rmse:.4f}m | Max: {max_erro:.4f}m\n({len(traj)} pontos)")
        plt.legend()
        plt.axis('equal')
        plt.show()

if __name__ == "__main__":
    main()