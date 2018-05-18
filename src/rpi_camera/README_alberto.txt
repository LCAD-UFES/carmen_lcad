Para criar este modulo no carmen_lcad soi usado o seguinte procedimento:
- Foi criado o diretorio src/rpi_camera e incluindo um arquivo texto README apenas para ter um arquivo no diretorio
- Foi feito o commit e push deste diretorio no github
- Foi baixado o diretorio rpi_camera no Raspberry Pi do github do LCAD usando o procedimento https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo , que basicamente consistiu de:
 sudo apt-get install subversion
 cd
 svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/src/rpi_camera
- A partir deste ponto podemos usar o svn para manter o modulo rpi_camera do Raspberry Pi atualizado no github do LCAD

