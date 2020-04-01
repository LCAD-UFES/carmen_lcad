# User Preferences (libuser_preferences.a)

Esta biblioteca disponibiliza as seguintes funcoes que auxiliam a recuperacao e o salvamento das preferencias do usuario, especialmente aquelas relacionadas as dimensoes e a posicao das janelas de um programa:

```
 void user_preferences_read(const char *module, user_param_t *param_list, int num_items);
 void user_preferences_read_from_file(const char *module, user_param_t *param_list, int num_items, const char *filename);
 void user_preferences_read_commandline(int argc, char **argv, user_param_t *param_list, int num_items);
 void user_preferences_save(const char *module, user_param_t *param_list, int num_items);
 void user_preferences_save_to_file(const char *module, user_param_t *param_list, int num_items, const char *filename);
```

Exemplos de uso podem ser encontrados nos seguintes programas: 
```
 proccontrol_gui.cpp  (usando Qt)
 navigator_gui2_main.cpp  (usando GTK)
 bumblebee_basic_view.cpp  (usando OpenCV)
```

## Instrucoes de uso

__1)__ Se o seu programa utiliza __make__, entao inclua no Makefile a referencia a biblioteca.
```
 LFLAGS += -luser_preferences
```

Se o seu programa utiliza __qmake__, entao inclua no arquivo de projeto qmake (.pro) a referencia a biblioteca.
```
 LIBS += -luser_preferences
```

__2)__ Inclua no seu programa o header da biblioteca.
```
 #include <carmen/user_preferences.h>
```

__3)__ Crie as variaveis globais que serao passadas como parametros nas chamadas das funcoes da biblioteca.
```
 const char *user_pref_module;
 user_param_t *user_pref_param_list;
 int user_pref_num_items;
```

__4)__ Crie as variaveis globais em que as preferencias do usuario serao salvas e recuperadas. Defina valores default para essas variaveis.
```
 int user_pref_window_width = 600;
 int user_pref_window_height = 400;
 int user_pref_window_x = -1;
 int user_pref_window_y = -1;
 // ... e outras variaveis que deseje recuperar e salvar
```

__5)__ Crie uma funcao __read_preferences__ para recuperar as preferencias do usuario que estao salvas em arquivo ou na linha de comando. A funcao __user_preferences_read__ faz a leitura de um arquivo com nome __./user_preferences.ini__. Caso queira usar um arquivo com nome ou caminho diferente, chame a funcao __user_preferences_read_from_file__. Caso o arquivo nao exista, a funcao encerra normalmente sem alterar os valores default das variaveis globais.
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
 		// { ... e outras variaveis que deseje recuperar},
 	};
 	user_pref_module = basename(argv[0]);
 	user_pref_param_list = param_list;
 	user_pref_num_items = sizeof(param_list) / sizeof(param_list[0]);
 	user_preferences_read(user_pref_module, user_pref_param_list, user_pref_num_items);
 	user_preferences_read_commandline(argc, argv, user_pref_param_list, user_pref_num_items);
 	
  	// ... coloque aqui os processamentos que possam ser efetuados de imediato com as variaveis recuperadas 
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

__6)__ Insira a manipulacao das janelas do programa utilizando as variaveis globais. Isto pode ser colocado dentro de __read_preferences__, ou numa funcao separada __set_window_preferences__ caso precise ser executado num momento posterior, apos a criacao da janela.

Exemplo com objeto __QWidget__ (deve ser inserido antes da primeira chamada a __(QWidget*)->show()__):
```
 if (user_pref_window_width >= 0 && user_pref_window_height >= 0)
     qdisplay->resize(user_pref_window_width, user_pref_window_height);
 if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
     qdisplay->move(user_pref_window_x, user_pref_window_y);
```

Exemplo com objeto __GtkWidget__ (deve ser inserido __apos__ a primeira chamada a __gtk_widget_show_all(GtkWidget*)__):
```
 void
 set_window_preferences()
 {
     if (user_pref_window_width >= 0 && user_pref_window_height >= 0)
         gtk_window_resize(GTK_WINDOW(gui->controls_.main_window), user_pref_window_width, user_pref_window_height);
     if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
         gtk_window_move(GTK_WINDOW(gui->controls_.main_window), user_pref_window_x, user_pref_window_y);
 }
```

Exemplo com __OpenCV__ (pode ser inserido antes da primeira chamada a __cv::imshow()__):
```
 if (user_pref_window_width >= 0 && user_pref_window_height >= 0)
     cv::resizeWindow(window_name, user_pref_window_width, user_pref_window_height);
 if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
     cv::moveWindow(window_name, user_pref_window_x, user_pref_window_y);
```

__7)__ A chamada a funcao __read_preferences__ nao depende do Carmen IPC, contudo convem ser colocada apos a chamada de __carmen_param_install_params__, para que sobreponha os parametros obtidos de __param_daemon__.

Exemplo:
```
 int
 main(int argc, char** argv)
 {
    ...
	read_parameters(argc, argv);
	read_preferences(argc, argv);
    ...
	static View::GtkGui _gui(argc, argv);
	gui = &_gui;
    ...
	init_navigator_gui_variables(argc, argv);
 	set_window_preferences();
    ...
```

__8)__ Crie uma funcao __save_preferences__ para salvar as preferencias do usuario em arquivo. A funcao __user_preferences_save__ utiliza um arquivo com nome __./user_preferences.ini__. Caso queira utilizar um arquivo com nome ou caminho diferente, chame a funcao __user_preferences_save_to_file__. Caso o arquivo contenha dados de outros programas, eles serao preservados; somente os dados do programa corrente sao atualizados. A versao anterior do arquivo sera renomeada com extensao __.bak__.

Exemplo com objeto __QWidget__: 
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

Exemplo com objeto __GtkWidget__:
```
 void
 save_preferences()
 {
    gtk_window_get_size(GTK_WINDOW(gui->controls_.main_window), &user_pref_window_width, &user_pref_window_height);
    gtk_window_get_position(GTK_WINDOW(gui->controls_.main_window), &user_pref_window_x, &user_pref_window_y);
    user_preferences_save(user_pref_module, user_pref_param_list, user_pref_num_items);
 }
```

Exemplo com __OpenCV__ (funciona apenas com OpenCV versao 3.4.1 ou superior):
```
 void
 save_preferences()
 {
 #if (CV_VERSION_MAJOR * 10000 + CV_VERSION_MINOR * 100 + CV_VERSION_REVISION) >= 30401
 	Rect display = getWindowImageRect(window_name);
 	user_pref_window_width  = display.width;
 	user_pref_window_height = display.height;
 	user_pref_window_x = display.x;
 	user_pref_window_y = display.y - 56;
 	user_preferences_save(user_pref_module, user_pref_param_list, user_pref_num_items);
 #endif
 }
```

__9)__ A chamada a funcao __save_preferences__ deve ser acionada antes do encerramento do programa.
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

__10)__ Edite manualmente o arquivo __user_preferences_default.ini__ da pasta __/bin__. Adicione as novas linhas correspondente ao programa e as variaveis que foram implementadas. Coloque valores default para essas variaveis. Coloque __#__ no inicio das linhas que devem ser desabilitadas por default para todos os usuarios. Este arquivo eh controlado pelo __git__.

__11)__ Faca uma copia do arquivo __user_preferences_default.ini__, renomeando-o para __user_preferences.ini__. Este arquivo eh de uso pessoal de cada usuario e nao eh controlado pelo __git__. Em seguida, edite manualmente este arquivo, colocando __#__ no inicio das linhas cujas variaveis voce deseja desabilitar para seu uso pessoal.
