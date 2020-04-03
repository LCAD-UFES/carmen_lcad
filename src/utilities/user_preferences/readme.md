# User Preferences (libuser_preferences.a)

Esta biblioteca disponibiliza as seguintes funcoes que auxiliam a recuperacao e o salvamento das preferencias pessoais de cada usuario, especialmente as preferencias relacionadas as dimensoes e a posicao das janelas de um programa:

```
 void user_preferences_read(const char *filename, const char *module, user_param_t *param_list, int num_items);
 void user_preferences_read_commandline(int argc, char **argv, user_param_t *param_list, int num_items);
 void user_preferences_save(const char *filename, const char *module, user_param_t *param_list, int num_items);
```

Exemplos de uso podem ser encontrados nos seguintes programas: 
```
 Usando Qt: proccontrol_gui.cpp
 Usando GTK: navigator_gui2_main.cpp
 Usando OpenCV:  bumblebee_basic_view.cpp
 Usando OpenGL (X11):  viewer_3D.cpp
```


## Instrucoes para usuarios dos programas

Para habilitar o uso das funcionalidades, faca uma copia do arquivo __$CARMEN_HOME/bin/default_user_preferences.ini__ com nome __user_preferences.ini__. Em seguida, edite manualmente o arquivo __user_preferences.ini__, colocando ou retirando o caractere __#__ do inicio de cada linha, conforme deseje desabilitar ou habilitar a recuperacao e o salvamento de cada variavel de programa.

Para desativar totalmente o uso das funcionalidades em todos os programas, basta remover o arquivo __$CARMEN_HOME/bin/user_preferences.ini__.


## Instrucoes para implementadores da funcionalidade em novos programas

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
 char *user_pref_filename = NULL;
 const char *user_pref_module;
 user_param_t *user_pref_param_list;
 int user_pref_num_items;
```

__4)__ Crie as variaveis globais em que as preferencias do usuario serao salvas e recuperadas. Defina valores default para essas variaveis.
```
 int user_pref_window_width  = 600;
 int user_pref_window_height = 400;
 int user_pref_window_x = -1;
 int user_pref_window_y = -1;
 // ... e outras variaveis que deseje recuperar e salvar
```

__5)__ Crie uma funcao __read_user_preferences__ para recuperar as preferencias do usuario que estao salvas em arquivo ou na linha de comando. O nome default do arquivo eh __./user_preferences.ini__. Caso queira usar um arquivo com nome ou caminho diferente, use a funcao __user_preferences_read_from_file__. Caso o arquivo nao exista, a funcao encerra normalmente sem alterar os valores default das variaveis globais.
```
 void
 read_user_preferences(int argc, char** argv)
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
 	user_preferences_read(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
 	user_preferences_read_commandline(argc, argv, user_pref_param_list, user_pref_num_items);
 }
```

Exemplo de conteudo de arquivo de preferencias do usuario: 
```
 proccontrol_gui	window_width	1041
 proccontrol_gui	window_height	755
 proccontrol_gui	window_x		879
 proccontrol_gui	window_y		297
```

Exemplos de linha de comando (que sobrepuja os dados recuperados de arquivo):
```
 ./proccontrol_gui  -window_x  879  -window_y  297
 ./proccontrol_gui  -window_x  879  -window_y  297  -window_width  1041  -window_height  755
```

__6)__ Crie uma funcao __set_user_preferences__ para configurar as janelas do programa utilizando as variaveis globais.

Exemplo com objeto __QWidget__ (deve ser inserido __antes__ da primeira chamada a __(QWidget*)->show()__):
```
 void
 set_user_preferences()
 {
     if (user_pref_window_width > 0 && user_pref_window_height > 0)
         qdisplay->resize(user_pref_window_width, user_pref_window_height);
     if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
         qdisplay->move(user_pref_window_x, user_pref_window_y);
 }
```

Exemplo com objeto __GtkWidget__ (deve ser inserido __apos__ a primeira chamada a __gtk_widget_show_all(GtkWidget*)__):
```
 void
 set_user_preferences()
 {
     if (user_pref_window_width > 0 && user_pref_window_height > 0)
         gtk_window_resize(GTK_WINDOW(gui->controls_.main_window), user_pref_window_width, user_pref_window_height);
     if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
         gtk_window_move(GTK_WINDOW(gui->controls_.main_window), user_pref_window_x, user_pref_window_y);
 }
```

Exemplo com __OpenCV__ (pode ser inserido __antes__ da primeira chamada a __cv::imshow()__):
```
 void
 set_user_preferences()
 {
     if (user_pref_window_width > 0 && user_pref_window_height > 0)
         cv::resizeWindow(window_name, user_pref_window_width, user_pref_window_height);
     if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
         cv::moveWindow(window_name, user_pref_window_x, user_pref_window_y);
 }
```

Exemplo com __OpenGL (X11)__ (deve ser inserido __apos__ a chamada a __XCreateWindow()__):
```
 void
 set_user_preferences()
 {
     if (user_pref_window_width > 0 && user_pref_window_height > 0)
         XResizeWindow(w->g_pDisplay, w->g_window, user_pref_window_width, user_pref_window_height);
     if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
         XMoveWindow(w->g_pDisplay, w->g_window, user_pref_window_x, user_pref_window_y);
 }
```

__7)__ Crie uma funcao __save_user_preferences__ para salvar as preferencias do usuario em arquivo. O nome default do arquivo eh __./user_preferences.ini__. Caso queira utilizar um arquivo com nome ou caminho diferente, use a funcao __user_preferences_save_to_file__. Caso o arquivo contenha dados de outros programas, eles serao preservados; somente os dados do programa corrente sao atualizados. A versao anterior do arquivo sera renomeada com extensao __.bak__.

Exemplo com objeto __QWidget__: 
```
 void
 save_user_preferences()
 {
    user_pref_window_width  = qdisplay->width();
    user_pref_window_height = qdisplay->height();
    user_pref_window_x = qdisplay->x() + 10;
    user_pref_window_y = qdisplay->y() + 10;
    // ... e outras variaveis que deseje salvar
    user_preferences_save(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
 }
```

Exemplo com objeto __GtkWidget__:
```
 void
 save_user_preferences()
 {
    gtk_window_get_size(GTK_WINDOW(gui->controls_.main_window), &user_pref_window_width, &user_pref_window_height);
    gtk_window_get_position(GTK_WINDOW(gui->controls_.main_window), &user_pref_window_x, &user_pref_window_y);
    // ... e outras variaveis que deseje salvar
    user_preferences_save(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
 }
```

Exemplo com __OpenCV__ (funciona apenas com OpenCV versao 3.4.1 ou superior):
```
 void
 save_user_preferences()
 {
 #if (CV_VERSION_MAJOR * 10000 + CV_VERSION_MINOR * 100 + CV_VERSION_REVISION) >= 30401
 	Rect display = getWindowImageRect(window_name);
 	user_pref_window_width  = display.width;
 	user_pref_window_height = display.height;
 	user_pref_window_x = display.x;
 	user_pref_window_y = display.y - 56;
    // ... e outras variaveis que deseje salvar
 	user_preferences_save(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
 #endif
 }
```

Exemplo com __OpenGL (X11)__:
```
void
save_user_preferences()
{
 	XWindowAttributes attr;
 	Window child_window;

 	XGetWindowAttributes(w->g_pDisplay, w->g_window, &attr);
 	XTranslateCoordinates(w->g_pDisplay, w->g_window, RootWindow(w->g_pDisplay, DefaultScreen(w->g_pDisplay)),
        attr.x, attr.y, &user_pref_window_x, &user_pref_window_y, &child_window);
 
 	user_pref_window_width  = attr.width;
 	user_pref_window_height = attr.height;
 	user_pref_window_y -= 28;
    // ... e outras variaveis que deseje salvar
 	user_preferences_save(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
}
```

__8)__ A chamada a funcao __read_user_preferences__ nao depende do Carmen IPC, contudo convem ser colocada apos a execucao de __carmen_param_install_params__, para que sobrepuje os parametros que sao obtidos de __param_daemon__. A chamada a funcao  __set_user_preferences__ deve ser colocada antes ou apos a exibicao da janela do programa, conforme exigido pela biblioteca grafica.

Exemplo:
```
 int
 main(int argc, char** argv)
 {
    ...
	read_parameters(argc, argv);  // executa carmen_param_install_params()
	read_user_preferences(argc, argv);
    ...
	static View::GtkGui _gui(argc, argv);
	gui = &_gui;
    ...
	init_navigator_gui_variables(argc, argv);  // exibe a janela GTK
 	set_user_preferences();
    ...
```

__9)__ A chamada a funcao __save_user_preferences__ deve ser acionada antes do encerramento do programa, seja por encerramento normal ou por interrupcao. Se o objeto correspondente a janela do programa estiver destruido, as suas informacoes de posicao e dimensoes devem ser previamente salvas. 
```
 void
 shutdown(int sig)
 {
 	save_user_preferences();
 	exit(sig);
 }
 
 int
 main(int argc, char** argv)
 {
 	...
 	signal(SIGINT, shutdown);
 	...
 	return 0;
 }
```

__10)__ Edite manualmente o arquivo __$CARMEN_HOME/bin/default_user_preferences.ini__. Adicione as novas linhas correspondente ao programa e as variaveis que foram implementadas. Coloque valores default para essas variaveis. Coloque __#__ no inicio das linhas que devem ser desabilitadas por default para todos os usuarios. Este arquivo eh controlado pelo __git__.

__11)__ Faca uma copia do arquivo __default_user_preferences.ini__ com nome __user_preferences.ini__. Este arquivo eh de uso pessoal de cada usuario e nao eh controlado pelo __git__. Em seguida, edite manualmente este arquivo, colocando __#__ no inicio das linhas cujas variaveis voce deseja desabilitar para seu uso pessoal.
