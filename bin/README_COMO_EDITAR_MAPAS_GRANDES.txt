ls -t map_ate_a_ponte-20161205/m* > caco.txt
split -l 50 caco.txt split_map_
chmod +x copy_maps.bat
chmod +x post_process_maps.bat
./copy_maps.bat
## realizar as edicoes nos mapas
./post_process_maps.bat
## IMPORTANTE: APAGUE OS DIRETORIOS E ARQUIVOS TEMPORARIOS E NUNCA (NUNCA!!!!!!!) SUBA PARA O GIT.
rm -rf split_map*
rm -rf dir_split_map*
