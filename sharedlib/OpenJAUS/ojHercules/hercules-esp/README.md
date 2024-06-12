# CAN (can.c)
O ESP embarcado no carro troca informações com o computador por meio de uma comunicação CAN.

O ESP envia mensagens de odometria para o computador, sendo elas:
- Velocidade -> ID = ODOM_VELOCITY_CAN_ID
- Ângulo de volante -> ID = ODOM_STEERING_CAN_ID

As ID's são definidas no arquivo system.h. É importante que os valores escolhidos coincidam com os valores esperados pelo Carmen. O valor esperado pelo Carmen pode ser encontrado em $CARMEN_HOME/sharedlib/OpenJAUS/ojHercules/src/main.c -> update_Car_state(). As duas mensagens contêm um campo de dados com 2 bytes.

O computador, por sua vez, envia para o ESP os comandos de velocidade e ângulo de volante em uma única mensagem CAN.
- Comando -> ID = COMMAND_CAN_ID

A ID é definida em system.h e deve corresponder ao que está em $CARMEN_HOME/sharedlib/OpenJAUS/ojHercules/src/pd.c -> send_efforts(). O campo de dados da mensagem é formada por 4 bytes, sendo os dois primeiros para a velocidade e os dois últimos para ângulo de volante.

Para a comunicação utilizam-se as variáveis globais odom_left_velocity, odom_right_velocity, odom_steering. command_velocity e command_steering_effort.

A task can_reading_task recebe as mensagens enviadas pelo computador e extrai delas dois inteiros que representam o comando de velocidade e de ângulo de volante que são salvas nas variáveis command_velocity e command_steering. Um pouco sobre como isso funciona ...

O Carmen trata comandos de velocidade e ângulo da seguinte forma: o comando que nós usamos como comando de velocidade é o throttle_effort (em automóveis, esse valor corresponde à quanto o acelerador deve ser pressionado). Esse valor é um double que varia de 0 a 100. No sistema do Hercules, tratamos esse comando como comando de velocidade e multiplicamos ele pelo sentido de movimento, obtendo um valor de -100 a 100. De forma semelhante, o comando de ângulo de volante é o steering_effort (nos automóveis, corresponde ao esforço que deve ser feito para virar o volante). Esse valor varia de -100 a 100. Para enviar esses valores temos direito a usar 16 bits para o throttle_effort e 16 bits para o steering_effort. Para maximizar a precisão do comando enviado, devemos converter o valor em inteiros que ocupem ao máximo o espaço disponível. Para isso, multiplicamos esses valores por CAN_CONVERSION_CONSTANT, presente em $CARMEN_HOME/sharedlib/OpenJAUS/ojHercules/src/pd.c e usado na função send_efforts().

Assim, os valores de comando recebidos pelo ESP via CAN são -100 * CAN_CONVERSION_CONSTANT até 100 * CAN_CONVERSION_CONSTANT. Esse valor máximo recebido via CAN é definido em system.h como CAN_COMMAND_MAX. Dessa forma, é importante que CAN_COMMAND_MAX seja definido como 100 * CAN_CONVERSION_CONSTANT, sendo CAN_CONVERSION_CONSTANT o valor presente em $CARMEN_HOME/sharedlib/OpenJAUS/ojHercules/src/pd.c.

Já no sentido contrário (envio de dados de odometria do ESP para o Carmen) ...

O envio de dados de odometria é feito pela task can_writing_task que pega os valores odom_right_velocity, odom_left_velocity e odom_steering, que representam velocidade das rodas e ângulo de volante.

odom_right_velocity e odom_left_velocity representam a velocidade das rodas no sistema internacional (m/s). A velocidade linear do carro é calculada como a média das duas velocidades. Para enviar esse vallor via CAN, ele é multiplicado por um valor de escala para que ocupe ao máximo os 16 bits. A constante usada é VELOCITY_CONVERSION_CONSTANT, presente em system.h. É importante que esse valor seja o inverso do valor de VELOCITY_CONVERSION_CONSTANT presente em $CARMEN_HOME/sharedlib/OpenJAUS/ojHercules/src/main.c e usado na função update_car_speed().

Já odom_steering é um número de 0 a (2**ADC_BITWIDTH_POTENTIOMETER - 1), onde 0 representa o ângulo máximo em um sentido e (2\*\*ADC_BITWIDTH_POTENTIOMETER - 1) o ângulo máximo no outro sentido. O valor pode ser enviado via CAN da forma como está. O importante será fazer a conversão disso para rad no código do Carmen em $CARMEN_HOME/sharedlib/OpenJAUS/ojHercules/src/main.c na função update_steering_angle(). Esse cálculo pode ser feito da seguinte forma:

$\phi = -MAX\_ANGLE + \frac{\phi _{CAN} \cdot 200}{2^{ADC\_BITWIDTH\_POTENTIOMETER} - 1}$

Em \$CARMEN_HOME/sharedlib/OpenJAUS/ojHercules/src/main.c MAX_ANGLE é definido também como MAX_ANGLE e o fator que multiplica $\phi _{CAN}$ é definido como ANGLE_CONVERSION_CONSTANT.

# Controle do motor (control.c)
O sistema do Carmen calcula o esforço de aceleração do veículo como um valor double de 0 a 100, o esforço de volante como um valor double de -100 a 100 e fornece uma variável de sentido de movimento (marcha ré ou não). Por meio da variável de sentido, a aceleração do veículo é convertida em um valor double de -100 a 100 que carrega consigo o sentido que o carro deve andar. Para que o valor seja enviado para o ESP via CAN dentro dos 4 bytes de dados, acelerador e volante são convertidos em valores inteiros. Como temos 16 bits, os valores inteiros que podemos enviar são de -32768 a 32767. Portanto, multiplica-se o valor das variáveis double (que vão de -100 a 100) pela constante EFFORT_TO_INT ($CARMEN_HOME/sharedlib/OpenJAUS/ojHercules/src/pd.c) que deve ser, no máximo, 327,67. Para facilitar a implementação, foi criada uma variável em system.h chamada CAN_COMMAND_MAX que deve ser igual a 100 * EFFORT_TO_INT. No caso de EFFORT_TO_INT=327,67 teríamos CAN_COMMAND_MAX=32767 que significaria que o valor máximo que será recebido pelo CAN é igual ao valor máximo possível nos 16 bits.

No ESP, o controle de velocidade do motor é feito por PWM. A biblioteca driver/ledc permite controlar o PWM com uma precisão de até 14 bits, chamada de duty_resolution. Para isso, devemos chamar a função ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, command_velocity) sendo command_velocity um número no intervalo [0, (2^duty_resolution)]. Em system.h foi definida a variável auxiliar MOTOR_DUTY_RESOLUTION.

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
O sistema do carmen envia o steering_effort, que significa o quanto o volante (ângulo) do carro deve virar. Esse valor varia de -25600 a 25600, que representa os valores mínimos e máximos de variação do ângulo. Para ser mais exato, uma valor positivo significa que o carro deve virar à esquerda, enquanto um valor negativo deve ser virar para a direita. Note que a servo_task tem dois principais valores: command_steering e command_steering_effort, enquanto a primeira é o ângulo que o esp32 (e por extensão o servo) busca impor no carro, o segundo é o pedido do carmen, que indica o quanto esse ângulo deve mudar, sendo somado uma vez a cada execução do loop da task.

A task inicia convertendo os esforço da medida can (25600~0 e 65536~65536-25600) para a unidade de ângulo (25600~-25600). Em seguida, uma fração do valor do esforço é adicionado ao valor atual de ângulo alvo, e obtemos então o próximo valor de ângulo desejado:

$command_steering = command_steering + \frac{command_steering_effort}{128}$

Obs: como em outras partes do código, os valores de command_steering e command_steering_effort são salvos em variáveis locais para liberar o uso para outras tasks.

O servo é o agente que controla o ângulo das rodas com base no tempo em nivel alto do PWM enviado pelo esp32. Quando queremos que o carro vire o máximo para a esquerda, colocamos o tempo em nivel alto, referido no código como target_T_HIGH, no valor mínimo MIN_T_HIGH, e quando queremos que vire o máximo para a direita, colocamos no valor máximo MAX_T_HIGH, assumindo uma aproximação linear, segundo a fórmula abaixo:

(WIP)

em que:

(WIP) $angle_can_to_T_HIGH_coefficient = \frac{MIN_T_HIGH - MAX_T_HIGH}{2*CAN_COMMAND_MAX}$

# Medição de velocidade pelo encoder (odom.c)

# Medição de ângulo pelo potenciômetro (odom.c)
