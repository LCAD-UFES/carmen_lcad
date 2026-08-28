Ligar a interface CAN:
 sudo ip link set can0 up type can bitrate 500000

process para testar
 ./proccontrol process-ecotech4.ini

Linha de comando para enviar estimulo de calibracao do PID
 ./ford_escape_hybrid_tune_pid -max_v 1.5 -max_phi 20.0 -timer_period 1.0 -t1 1.565 -frequency 2.0

Salvar a configuracao:
    To save the configuration to a file on the PC, execute odrivetool and:
	odrv0.save_configuration()
	odrv0.reboot()
	quit
    Then, execute in a terminal	
	odrivetool backup-config my_config.json
    (i) To restore the configuration form such a file, run 
	odrivetool restore-config odrive_config_top.json {Esta que salvei que esta boa.}
    Tem que desligar e ligar depois de restaurar. Além disso, entre no odrivetool e:
In [8]: odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
In [9]: odrv0.axis0.controller.current_setpoint=5 {para testar. O motor deve rodar}
In [10]: odrv0.axis0.controller.current_setpoint=0
In [11]: odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
In [12]: odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
In [15]: odrv0.save_configuration()
In [16]: odrv0.reboot()
    Verifique se a velocidade do can foi ajustada corretamente:
     odrv0.can
    A saída deve ser:
     baud_rate = 500000 (int)
    Se nao foi, repita todo o processo de restaurar a configuracao (i)
ODrive e CAN bus:
https://discourse.odriverobotics.com/t/can-interface-available-for-testing/1448
https://discourse.odriverobotics.com/t/save-restore-configuration-question/3217

Comandos para o merge (ver https://stackoverflow.com/questions/54033842/how-to-pull-a-pull-request-from-upstream-in-github):
git clone https://github.com/madcowswe/ODrive.git {baixei o devel original}
cd ODrive
git remote add upstream https://github.com/Wetmelon/ODrive {liguei o remote com o pull request ao repositorio}
git pull upstream feature/CAN:odrive_with_can {merge com o pull request}
cd Firmware {ver detalhes em https://docs.odriverobotics.com/developer-guide#configuring-the-build}
make clean {se já existir codigo compilado; dee dar um monte de warnings}
make
odrivetool dfu build/ODriveFirmware.hex
{desligue e ligue o ODrive antes de testar}


Para instalar o odrivetall
 sudo pip3 install odrive

Para compilar novamente:
 cd /home/alberto/ODrive/Firmware
 make 
 odrivetool dfu build/ODriveFirmware.hex
Ou, atualize com o ultimo na Internet
 odrivetool dfu
Ou, ainda, pegue o oficial daqui https://github.com/madcowswe/ODrive/releases e, depois de baixar para o diretorio corrente:
 odrivetool dfu ODriveFirmware_v3.6-24V.hex

Limitador de velocidade:
 https://github.com/Wetmelon/ODrive/blob/1494cb333fa4e087057f1b2cb9320c28062bd51d/Firmware/MotorControl/controller.cpp#L255

Configuracoes:
odrv:
odrv0.config.brake_resistance = 0.5

encoder:
odrv0.axis0.encoder.config.mode=2
odrv0.axis0.encoder.config.cpr=6283*3
odrv0.axis0.encoder.config.calib_range=100
odrv0.axis0.encoder.config.bandwidth=500

controller:
odrv0.axis0.controller.config.control_mode=CTRL_MODE_CURRENT_CONTROL {CTRL_MODE_VELOCITY_CONTROL, CTRL_MODE_POSITION_CONTROL}
odrv0.axis0.controller.config.vel_limit=200000
odrv0.axis0.controller.config.vel_gain=0.0001
odrv0.axis0.controller.config.vel_integrator_gain=0.001
odrv0.axis0.controller.config.vel_limit_tolerance=100.0
odrv0.axis0.controller.config.pos_gain=20

motor:
odrv0.axis0.motor.config.current_lim = 10
odrv0.axis0.motor.config.pole_pairs=3
odrv0.axis0.motor.config.motor_type=MOTOR_TYPE_HIGH_CURRENT


Comandos uteis:
odrv0.vbus_voltage

odrv0.save_configuration()
odrv0.reboot()

dump_errors(odrv0)

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.error=0 
odrv0.axis0.motor.error=0
odrv0.axis0.controller.error=0
odrv0.axis0.encoder.error=0

odrv0.axis0.encoder.shadow_count=0 

start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])


++++++++++++++++++++++++++++++++++++
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.controller.config.control_mode=CTRL_MODE_CURRENT_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.current_setpoint=-5

- If all looks good then you can tell the ODrive that saving this calibration to presistent memory is OK:

odrv0.axis0.encoder.config.pre_calibrated = True

- Automatic startup
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.config.startup_motor_calibration=True
odrv0.axis0.config.startup_closed_loop_control = True



++++++++++++++++++++++++++++++++++++
odrv0.axis0.requested_state = AXIS_STATE_SENSORLESS_CONTROL
odrv0.axis0.controller.vel_setpoint = 0



+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Configuracoes:
In [198]: odrv0                                                                                                                                                                                                     
Out[198]: 
serial_number = 20563594524B (int)
hw_version_major = 3 (int)
enter_dfu_mode()
test_function(delta: int)
system_stats:
  uptime = 2173029 (int)
  min_stack_space_startup = 604 (int)
  i2c: ...
  min_stack_space_usb_irq = 1796 (int)
  min_stack_space_usb = 3292 (int)
  min_heap_space = 4216 (int)
  min_stack_space_axis0 = 7764 (int)
  usb: ...
  min_stack_space_comms = 9668 (int)
  min_stack_space_axis1 = 7868 (int)
  min_stack_space_uart = 3932 (int)
hw_version_minor = 6 (int)
reboot()
get_adc_voltage(gpio: int)
fw_version_minor = 4 (int)
get_oscilloscope_val(index: int)
save_configuration()
hw_version_variant = 24 (int)
can:
  unexpected_errors = 0 (int)
  unhandled_messages = 0 (int)
  received_msg_cnt = 0 (int)
  received_ack = 0 (int)
  TxMailboxAbortCallbackCnt = 0 (int)
  node_id = 0 (int)
  TxMailboxCompleteCallbackCnt = 0 (int)
axis0:
  watchdog_feed()
  current_state = 1 (int)
  sensorless_estimator: ...
  error = 0x0020 (int)
  motor: ...
  requested_state = 0 (int)
  step_dir_active = False (bool)
  trap_traj: ...
  lockin_state = 0 (int)
  controller: ...
  encoder: ...
  loop_counter = 17371410 (int)
  config: ...
vbus_voltage = 14.00244140625 (float)
fw_version_major = 0 (int)
brake_resistor_armed = True (bool)
fw_version_unreleased = 0 (int)
fw_version_revision = 11 (int)
test_property = 0 (int)
erase_configuration()
axis1:
  watchdog_feed()
  current_state = 1 (int)
  sensorless_estimator: ...
  error = 0x0000 (int)
  motor: ...
  requested_state = 0 (int)
  step_dir_active = False (bool)
  trap_traj: ...
  lockin_state = 0 (int)
  controller: ...
  encoder: ...
  loop_counter = 17371431 (int)
  config: ...
config:
  brake_resistance = 0.5 (float)
  enable_uart = True (bool)
  gpio4_pwm_mapping: ...
  enable_ascii_protocol_on_usb = True (bool)
  enable_i2c_instead_of_can = False (bool)
  dc_bus_undervoltage_trip_level = 8.0 (float)
  gpio4_analog_mapping: ...
  gpio1_pwm_mapping: ...
  gpio3_analog_mapping: ...
  gpio3_pwm_mapping: ...
  dc_bus_overvoltage_trip_level = 25.68000030517578 (float)
  gpio2_pwm_mapping: ...
user_config_loaded = True (bool)

---------------------------------------------------------
In [199]: odrv0.config                                                                                                                                                                                              
Out[199]: 
brake_resistance = 0.5 (float)
enable_uart = True (bool)
gpio4_pwm_mapping:
  endpoint = (0, 0) (RemoteProperty)
  max = 0.0 (float)
  min = 0.0 (float)
enable_ascii_protocol_on_usb = True (bool)
enable_i2c_instead_of_can = False (bool)
dc_bus_undervoltage_trip_level = 8.0 (float)
gpio4_analog_mapping:
  endpoint = (0, 0) (RemoteProperty)
  max = 0.0 (float)
  min = 0.0 (float)
gpio1_pwm_mapping:
  endpoint = (0, 0) (RemoteProperty)
  max = 0.0 (float)
  min = 0.0 (float)
gpio3_analog_mapping:
  endpoint = (0, 0) (RemoteProperty)
  max = 0.0 (float)
  min = 0.0 (float)
gpio3_pwm_mapping:
  endpoint = (0, 0) (RemoteProperty)
  max = 0.0 (float)
  min = 0.0 (float)
dc_bus_overvoltage_trip_level = 25.68000030517578 (float)
gpio2_pwm_mapping:
  endpoint = (0, 0) (RemoteProperty)
  max = 0.0 (float)
  min = 0.0 (float)

------------------------------------------------------------

In [203]: odrv0.axis0                                                                                                                                                                                               
Out[203]: 
watchdog_feed()
current_state = 1 (int)
sensorless_estimator:
  phase = -0.28476035594940186 (float)
  config: ...
  error = 0x0000 (int)
  vel_estimate = 0.05063961073756218 (float)
  pll_pos = -0.2861593961715698 (float)
error = 0x0020 (int)
motor:
  DC_calib_phB = -0.06240083649754524 (float)
  phase_current_rev_gain = 0.02500000037252903 (float)
  error = 0x0010 (int)
  thermal_current_lim = 44.42158126831055 (float)
  current_meas_phC = 0.3404313325881958 (float)
  current_control: ...
  current_meas_phB = -0.42133766412734985 (float)
  DC_calib_phC = -1.7099910974502563 (float)
  is_calibrated = True (bool)
  gate_driver: ...
  timing_log: ...
  armed_state = 0 (int)
  config: ...
  get_inverter_temp()
requested_state = 0 (int)
step_dir_active = False (bool)
trap_traj:
  config: ...
lockin_state = 0 (int)
controller:
  set_pos_setpoint(pos_setpoint: float, vel_feed_forward: float, current_feed_forward: float)
  vel_ramp_enable = False (bool)
  error = 0x0000 (int)
  vel_integrator_current = 0.0 (float)
  set_current_setpoint(current_setpoint: float)
  move_to_pos(pos_setpoint: float)
  start_anticogging_calibration()
  pos_setpoint = 3137.790771484375 (float)
  vel_ramp_target = 0.0 (float)
  current_setpoint = 0.0 (float)
  move_incremental(displacement: float, from_goal_point: bool)
  vel_setpoint = 0.0 (float)
  config: ...
  set_vel_setpoint(vel_setpoint: float, current_feed_forward: float)
encoder:
  index_found = False (bool)
  pos_estimate = 963133.75 (float)
  vel_estimate = 776.25 (float)
  interpolation = 0.0 (float)
  phase = -0.2879304885864258 (float)
  calib_scan_response = 49845.0 (float)
  count_in_cpr = 1801 (int)
  shadow_count = 963067 (int)
  config: ...
  is_ready = True (bool)
  set_linear_count(count: int)
  pos_cpr = 1805.476318359375 (float)
  hall_state = 7 (int)
  error = 0x0000 (int)
loop_counter = 19219344 (int)
config:
  startup_encoder_index_search = False (bool)
  calibration_lockin: ...
  sensorless_ramp: ...
  watchdog_timeout = 0.0 (float)
  counts_per_step = 2.0 (float)
  step_gpio_pin = 1 (int)
  enable_step_dir = False (bool)
  startup_motor_calibration = False (bool)
  startup_encoder_offset_calibration = False (bool)
  dir_gpio_pin = 2 (int)
  startup_closed_loop_control = False (bool)
  general_lockin: ...
  startup_sensorless_control = False (bool)

-------------------------------------------------------------------------
In [202]: odrv0.axis0.config                                                                                                                                                                                        
Out[202]: 
startup_encoder_index_search = False (bool)
calibration_lockin:
  ramp_time = 0.4000000059604645 (float)
  vel = 40.0 (float)
  accel = 20.0 (float)
  current = 10.0 (float)
  ramp_distance = 3.1415927410125732 (float)
sensorless_ramp:
  ramp_time = 0.4000000059604645 (float)
  finish_on_distance = False (bool)
  finish_distance = 100.0 (float)
  ramp_distance = 3.1415927410125732 (float)
  finish_on_enc_idx = False (bool)
  accel = 200.0 (float)
  current = 10.0 (float)
  vel = 400.0 (float)
  finish_on_vel = True (bool)
watchdog_timeout = 0.0 (float)
counts_per_step = 2.0 (float)
step_gpio_pin = 1 (int)
enable_step_dir = False (bool)
startup_motor_calibration = False (bool)
startup_encoder_offset_calibration = False (bool)
dir_gpio_pin = 2 (int)
startup_closed_loop_control = False (bool)
general_lockin:
  ramp_time = 0.4000000059604645 (float)
  finish_on_distance = False (bool)
  finish_distance = 100.0 (float)
  ramp_distance = 3.1415927410125732 (float)
  finish_on_enc_idx = False (bool)
  accel = 20.0 (float)
  current = 10.0 (float)
  vel = 40.0 (float)
  finish_on_vel = False (bool)
startup_sensorless_control = False (bool)

------------------------------------------------------------------------
In [205]: odrv0.axis0.motor.config                                                                                                                                                                                  
Out[205]: 
calibration_current = 10.0 (float)
phase_inductance = 0.00012413713557180017 (float)
current_lim_tolerance = 50.0 (float)
pole_pairs = 3 (int)
direction = 1 (int)
current_control_bandwidth = 100.0 (float)
inverter_temp_limit_lower = 100.0 (float)
motor_type = 0 (int)
resistance_calib_max_voltage = 2.0 (float)
current_lim = 10.0 (float)
pre_calibrated = False (bool)
requested_current_range = 60.0 (float)
phase_resistance = 0.03535434603691101 (float)
inverter_temp_limit_upper = 120.0 (float)

----------------------------------------------------------------------
In [206]: odrv0.axis0.controller.config                                                                                                                                                                             
Out[206]: 
vel_ramp_rate = 10000.0 (float)
vel_gain = 9.999999747378752e-05 (float)
control_mode = 1 (int)
vel_limit = 50000.0 (float)
pos_gain = 1.0 (float)
vel_limit_tolerance = 100.0 (float)
setpoints_in_cpr = False (bool)
vel_integrator_gain = 0.0010000000474974513 (float)

----------------------------------------------------------------------
In [207]: odrv0.axis0.encoder.config                                                                                                                                                                                
Out[207]: 
calib_scan_distance = 50.26548385620117 (float)
use_index = False (bool)
idx_search_unidirectional = False (bool)
find_idx_on_lockin_only = False (bool)
enable_phase_interpolation = True (bool)
ignore_illegal_hall_state = False (bool)
zero_count_on_find_idx = True (bool)
bandwidth = 100.0 (float)
calib_scan_omega = 12.566370964050293 (float)
cpr = 18849 (int)
mode = 2 (int)
pre_calibrated = False (bool)
offset = 27195 (int)
calib_range = 100.0 (float)
offset_float = 0.9246718883514404 (float)

