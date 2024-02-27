# CAN (can.c)
O ESP embarcado no carro troca informações com o computador por meio de uma comunicação CAN.

O ESP envia mensagens de odometria para o computador, sendo elas:
- Velocidade -> ID = ODOM_VELOCITY_CAN_ID
- Ângulo de volante -> ID = ODOM_STEERING_CAN_ID

As ID's são definidas no arquivo system.h. É importante que os valores escolhidos coincidam com os valores esperados pelo Carmen. O valor esperado pelo Carmen pode ser encontrado em $CARMEN_HOME/sharedlib/OpenJAUS/ojWheeltec/src/main.c -> update_Car_state(). As duas mensagens contêm um campo de dados com 2 bytes.

O computador, por sua vez, envia para o ESP os comandos de velocidade e ângulo de volante em uma única mensagem CAN.
- Comando -> ID = COMMAND_CAN_ID

A ID é definida em system.h e deve corresponder ao que está em $CARMEN_HOME/sharedlib/OpenJAUS/ojWheeltec/src/pd.c -> send_efforts(). O campo de dados da mensagem é formada por 4 bytes, sendo os dois primeiros para a velocidade e os dois últimos para ângulo de volante.

Para a comunicação utilizam-se as variáveis globais odom_velocity, odom_steering. command_velocity e command_steering. A task can_reading_task recebe as mensagens enviadas pelo computador e extrai delas dois inteiros que representam o comando de velocidade e de ângulo de volante que são salvas nas variáveis command_velocity e command_steering. Esses valores vão de -CAN_COMMAND_MAX até +CAN_COMMAND_MAX (definidas em system.h). O envio de dados de odometria é feito pela task can_writing_task que pega os valores odom_velocity e odom_steering, que representam velocidade e ângulo de volante no sistema internacional (m/s e rad), multiplicam o valor por uma escala para que ocupem ao máximo os 16 bits que são enviados via CAN (reduzindo o imprecisão de representação numérica). Posteriormente, esses valores são obtidos pelo computador e é feita a operação inversa (divisão pela constante que foi multiplicada), sendo importante, portanto, que VELOCITY_CONVERSION_CONSTANT e STEERING_CONVERSION_CONSTANT sejam iguais aos valores presentes em $CARMEN_HOME/sharedlib/OpenJAUS/ojWheeltec/src/main.c

# Controle do motor (control.c)
O sistema do Carmen calcula o esforço de aceleração do veículo como um valor double de 0 a 100, o esforço de volante como um valor double de -100 a 100 e fornece uma variável de sentido de movimento (marcha ré ou não). Por meio da variável de sentido, a aceleração do veículo é convertida em um valor double de -100 a 100 que carrega consigo o sentido que o carro deve andar. Para que o valor seja enviado para o ESP via CAN dentro dos 4 bytes de dados, acelerador e volante são convertidos em valores inteiros. Como temos 16 bits, os valores inteiros que podemos enviar são de -32768 a 32767. Portanto, multiplica-se o valor das variáveis double (que vão de -100 a 100) pela constante EFFORT_TO_INT ($CARMEN_HOME/sharedlib/OpenJAUS/ojWheeltec/src/pd.c) que deve ser, no máximo, 327,67. Para facilitar a implementação, foi criada uma variável em system.h chamada CAN_COMMAND_MAX que deve ser igual a 100 * EFFORT_TO_INT. No caso de EFFORT_TO_INT=327,67 teríamos CAN_COMMAND_MAX=32767 que significaria que o valor máximo que será recebido pelo CAN é igual ao valor máximo possível nos 16 bits.

No ESP, o controle de velocidade do motor é feito por PWM. A biblioteca driver/ledc permite controlar o PWM com uma precisão de até 14 bits, chamada de duty_resolution. Para isso, devemos chamar a função ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, command_velocity) sendo command_velocity um número no intervalo [0, (2^duty_resolution)]. Em system.h foi definida a variável auxiliar DUTY_RESOLUTION.

O Hercules possui um motor para cada roda traseira. Assim, é preciso calcular a velocidade de cada uma com base no comando de velocidade e de ângulo. Por meio das equações de movimento do Ackermann (vide $CARMEN_HOME/doc/kinematic_model_of_ackerman_steering_vehicles.pdf), segue que:

$v_r = v (1 + \frac{l\cdot \tan \phi}{2L})$

$v_l = v (1 - \frac{l\cdot \tan \phi}{2L})$

Como queremos que o veículo responda corretamente, a velocidade máxima ($v$) que podemos pedir é tal que nem $v_r$ nem $v_l$ ultrapasse o máximo da escala. Portanto, segue que:

$2^{DUTY\_RESOLUTION} = v_{max} (1 + \frac{l\cdot \tan\phi _{max}}{2L})$

$v_{max} = \frac{2^{DUTY\_RESOLUTION}}{1 + \frac{l\cdot \tan\phi _{max}}{2L}}$

Dessa forma, deve ser efetuada a seguinte operação sobre o valor de velocidade recebido via CAN:

$v_{PWM} = v_{CAN} \cdot \frac{2^{DUTY\_RESOLUTION}}{CAN\_COMMAND\_MAX\cdot (1 + \frac{l\cdot \tan\phi _{max}}{2L})}$

Para a realização dos cálculos acima, não foi necessário que a velocidade estivesse em m/s, de fato, as operações são realizadas com os valores inteiros que representam o valor que será aplicado no PWM. Já o ângulo é necessário que esteja em alguma unidade padronizada para o cálculo de $tan \phi$. Foi escolhido trabalhar com radianos. Assim, o ângulo em radiano pode ser calculado por:

$\phi _{rad} = \phi _{CAN} \cdot \frac{\phi _{max}}{CAN\_COMMAND\_MAX}$

Finalmente, o valor a ser aplicado no PWM de cada motor pode ser calculado da seguinte forma.

Setup:

$angle\_can\_to\_rad = \frac{\phi _{max}}{CAN\_COMMAND\_MAX}$

$velocity\_can\_to\_pwm = \frac{2^{DUTY\_RESOLUTION}}{CAN\_COMMAND\_MAX\cdot (1 + \frac{l\cdot \tan\phi _{max}}{2L})}$

$left\_to\_right\_difference\_constant = \frac{l}{2L}$

A cada iteração:

$v = v_{CAN} \cdot velocity\_can\_to\_pwm$

$\phi _{rad} = \phi _{CAN} \cdot angle\_can\_to\_rad$

$left\_to\_right\_difference = \tan (\phi _{rad}) \cdot left\_to\_right\_difference\_constant$

$v_r = v \cdot (1 + left\_to\_right\_difference)$

$v_l = v \cdot (1 - left\_to\_right\_difference)$

# Controle do servo (control.c)

# Medição de velocidade pelo encoder (odom.c)

# Medição de ângulo pelo potenciômetro (odom.c)
