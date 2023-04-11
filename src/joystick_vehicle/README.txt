Este modulo é o driver de joysticks e permite controlar o simulador ou robôs reais com joysticks.

Para executar o driver, execute:
 ./joystick_vehicle <-show_state on> <-direct_v_and_phi_mode on>

- A flag opcional show_state permite ver o estado bruto dos botões do joystick. v é controlado pelo axix 4 do joystick, equanto que phi, pelo axis 0.
- A flag opcional direct_v_and_phi_mode permite ativar o direct_v_and_phi_mode, que torna possível enviar v e phi diretamente para o robô.
  No modo default, o joystick adiciona incrementos a v e phi (variáveis globais mantêm o estado de v e phi).

Exemplo de uso (antes de rodar o exemplo abaixo, em um terminal rode o central e, em outro, o proccontrol process-navigate-volta-da-ufes-pid.ini,
no proccontrol_gui, pare o behavior_selector)
 ./joystick_vehicle -direct_v_and_phi_mode on

Para ativar o joystick, aperte o botão grande central (XBox), ou SELECT (PC), ou SHARE (Playstation). Após ser ativado, o driver envia mensagens
carmen_robot_and_trailer_motion_command_t e carmen_behavior_selector_state_message.

- No modo default, você tem que apertar (com o dedo indicador) os botões frontais do joystick para ativar o movimento para frente e para trás.
- No modo direct_v_and_phi_mode, basta usar os axis 4 e 0 para ajustar v e phi diretamente.
