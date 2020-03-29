# User Preferences (libuser_preferences.a)

Esta biblioteca disponibiliza funcionalidades que auxiliam o salvamento e a recuperacao das preferencias do usuario, especialmente as preferencias relacionadas as dimensoes e a posicao inicial das janelas de um programa.

Um exemplo de uso pode ser encontrado no programa: * proccontrol_gui.cpp *

## Instrucoes de uso

1) Inclua no Makefile a referencia a biblioteca. Nota: se o seu programa utiliza * Qt * e * qmake *, entao a linha abaixo deve ser inserida no arquivo de projeto qmake (.pro) e nao diretamente no Makefile.
```
 LIBS += -luser_preferences
```

2) Inclua no seu programa o header da biblioteca.
```
 #include <carmen/user_preferences.h>
```

3) Crie as variaveis globais que serao passadas como parametros nas chamadas das funcoes da biblioteca.
```
 const char *user_pref_module;
 user_param_t *user_pref_param_list;
 int user_pref_num_items;
```

4) Crie as variaveis globais em que as preferencias do usuario serao salvas e recuperadas. Defina valores default para essas variaveis.
```
 int user_pref_window_width = 600;
 int user_pref_window_height = 400;
 int user_pref_window_x = -1;
 int user_pref_window_y = -1;
```

5) Nas funcoes de construcao das janelas do programa, utilize as variaveis globais.
```
 resize(user_pref_window_width, user_pref_window_height);
 if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
     move(user_pref_window_x, user_pref_window_y);
```

6) Crie uma funcao * read_preferences * para recuperar as preferencias do usuario que estao salvas em arquivo ou na linha de comando. A funcao * user_preferences_read * faz a leitura de um arquivo com nome * ./user_preferences.ini *. Caso queira usar um arquivo com nome ou caminho diferente, chame a funcao * user_preferences_read_from_file *.
```
 void
 read_preferences(int argc, char** argv)
 {
 	static user_param_t param_list[] =
 	{
 		{"window_width",  USER_PARAM_TYPE_INT, &user_pref_window_width},
 		{"window_height", USER_PARAM_TYPE_INT, &user_pref_window_height},
 		{"window_x",      USER_PARAM_TYPE_INT, &user_pref_window_x},
 		{"window_y",      USER_PARAM_TYPE_INT, &user_pref_window_y},
 	};
 	user_pref_module = basename(argv[0]);
 	user_pref_param_list = param_list;
 	user_pref_num_items = sizeof(param_list) / sizeof(param_list[0]);
 	user_preferences_read(user_pref_module, user_pref_param_list, user_pref_num_items);
 	user_preferences_read_commandline(argc, argv, user_pref_param_list, user_pref_num_items);
 }
```

Exemplo de arquivo de preferencias do usuario: 
```
 proccontrol_gui	window_width	1041
 proccontrol_gui	window_height	755
 proccontrol_gui	window_x		879
 proccontrol_gui	window_y		297
```

Exemplos de linha de comando:
```
 ./proccontrol_gui  -window_x  879  -window_y  297
 ./proccontrol_gui  -window_x  879  -window_y  297  -window_width  1041  -window_height  755
```

7) A chamada a funcao * read_preferences * deve ocorrer antes da construcao das janelas do programa. Essa funcao nao depende do Carmen IPC.
```
 int
 main(int argc, char** argv)
 {
 	read_preferences(argc, argv);
 	QApplication app(argc, argv);
 	QDisplay gui;
```

8) Crie uma funcao * save_preferences * para salvar as preferencias do usuario em arquivo. A funcao * user_preferences_save * grava um arquivo com nome * ./user_preferences.ini *. Caso queira gravar um arquivo com nome ou caminho diferente, chame a funcao * user_preferences_save_to_file *. Caso o arquivo contenha dados de outros programas, eles serao preservados. Os dados do programa corrente serao atualizados. Caso o arquivo nao exista, ele sera criado.
```
 void
 save_preferences()
 {
 	user_pref_window_width = qdisplay->width();
 	user_pref_window_height = qdisplay->height();
 	user_pref_window_x = qdisplay->x() + 10;
 	user_pref_window_y = qdisplay->y() + 10;
 	user_preferences_save(user_pref_module, user_pref_param_list, user_pref_num_items);
 }
```

9) A chamada a funcao * save_preferences * deve ocorrer antes do shutdown do programa.
```
 void
 shutdown(int sig)
 {
 	save_preferences();
 	exit(sig);
 }
 
 int
 main(int argc, char** argv)
 {
 	...
 	signal(SIGINT, shutdown);
 	...
 }
```
