//Criado no dia 28/03/2014 às 9:11 am por Rafael Nascimento.

Procedimento para limpar mapas no carmen.

1) execute a central:
./central

2)em outro terminal execute param_daemon com carmen-ford-escape.ini como parametro:
./param_daemon ../src/carmen-ford-escape.ini

3) Construa o mapa completo com o comando:
./build_complete_map -map_path PATH_DO_MAPA -map_resolution RESOLUCAO_DO_MAPA

4) Abra a interface de edicaoo de mapas com o comando:
./map_editor PATH_DO_MAPA/complete_map.map

4.1) Use o zoom com cuidado. O botao da direita reduz e o da esquerda aumenta. Nao pode usar com muito zoom senao fica muito lento de editar.

5) Na interface coloque a "Ink(Probability) para zero - localizado no canto inferior esquerdo - e use o botão "brush" - localizado na barra "Tools" no canto superior esquerdo - para remover 
objetos indesejados do mapa. Para aumentar a quantidade de pixels pintados de uma só vez use o botão "larger" na opção "Brush size" - localizado na barra inferior no centro. A cada alteração use 
o atalho "Ctrl + S" para salvar, para voltar use o atalho "Ctrl + Z".
OBS: Faça todo o procedimento com cuidado e com calma, pois dependendo do tamanho do mapa e do computador utilizado pode ser que ocorra pequenos travamentos.

6) Depois de terminar de editar o mapa, Salve-o e em sequida faça o seguinte comando para dividir o mapa em blocks menores:
./complete_map_to_block_map -map_path PATH_DO_MAPA -map_resolution RESOLUCAO_DO_MAPA -block_size_in_meters TAMANHO_DO_BLOCO_EM_METROS


Exemplo de comandos:

./build_complete_map -map_path ../data/map_voltadaufes-20140326/ -map_resolution 0.2
./map_editor ../data/map_voltadaufes-20140326/complete_map.map 
./complete_map_to_block_map -map_path ../data/map_voltadaufes-20140326 -map_resolution 0.2 -block_size_in_meters 210.0
