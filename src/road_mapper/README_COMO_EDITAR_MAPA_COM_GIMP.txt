
rm -r /dados/tmp/img/ && rm -r /dados/tmp/map

1) Transforme todos os arquivos de mapas (.map) numa unica imagem (.png)

   mkdir -p /dados/tmp/img/ ; $CARMEN_HOME/bin/save_map_images  -input_dir $CARMEN_HOME/data/map_volta_da_ufes-20190915  -out_dir /dados/tmp/img/

2) Abra o arquivo de imagem usando o gimp

   gimp  $(ls /dados/tmp/img/complete_*.png)

3) Menu gimp: Tools > Paint Tools > Pencil

4) Menu gimp: Windows > Dockable Dialogs > Tool Options

   Ajuste: Mode=Normal , Opacity=100 , Brush=1 , Size=1 (pode ser maior), Aspect Ratio=0 , Angle=0

5) Menu gimp: Windows > Dockable Dialogs > Colors

   Ajuste: Foreground vermelho   ff0000 para pintar celulas com valor -2.0
   Ajuste: Foreground azul claro 1e90ff para pintar celulas com valor -1.0
   Ajuste: Foreground branco     ffffff para pintar celulas com valor  0.0
   Ajuste: Foreground preto      000000 para pintar celulas com valor  1.0

6) Menu gimp: File > Overwrite <arquivo.png>

7) Menu gimp: File > Quit > Discard Changes

8) Atualize os mapas usando o arquivo de imagem editado

   mkdir -p /dados/tmp/map/ ; $CARMEN_HOME/bin/update_map_from_image  $(ls /dados/tmp/img/complete_*.png)  -input_dir $CARMEN_HOME/data/map_volta_da_ufes-20190915  -out_dir /dados/tmp/map/

# Os mapas alterados ficam em /dados/tmp/map/

