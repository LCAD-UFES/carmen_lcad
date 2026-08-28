imu_teste -- painel de bancada para conferir se uma IMU esta' funcionando
=========================================================================

Abre uma janela (GTK3 + cairo) com os tres vetores do sensor, cada um com a sua
unidade, mais a orientacao do filtro de Kalman e a taxa de mensagens. A ideia e'
nao ter que interpretar numero solto: cada painel compara a norma medida com o
que uma IMU parada e sadia deve dar, e mostra OK / ATENCAO / FORA.

    acelerometro   m/s2    parado -> |v| = 9.81 (a gravidade)
    giroscopio     rad/s   parado -> |v| = 0    (so' ruido de bias)
    magnetometro   a.u.    |v| = 1.0            (o MTi normaliza pelo campo terrestre)

Teclas: q ou Esc sai, espaco congela a tela, z zera os picos das escalas.
As barras tem o zero no centro, entao da' para ver sinal e magnitude de um golpe
de olho; a escala acompanha o pico recente e esta' escrita embaixo das barras.

DE ONDE VEM O DADO
------------------
Assina as tres mensagens do modulo xsens -- carmen_xsens_global_{quat,euler,matrix}
-- e desenha a que chegar, entao funciona com xsens_mti_settings 1, 2 ou 3.

Nao e' so' para o xsens: o modulo pi_imu publica a mesma
carmen_xsens_global_quat_message (ver src/pi_imu/pi_imu_server_logger.cpp), de
modo que este painel serve para conferir a IMU do Raspberry do mesmo jeito.

COMO RODAR
----------
Pelo process pronto:

    cd $CARMEN_HOME/bin
    ./central &
    ./proccontrol ../data/iara/process/process-iara-imu_teste.ini

Ou na mao:

    ./central &
    ./param_daemon ../data/iara/parameters/carmen.ini &
    ./xsens -xsens_mti_dev /dev/ttyUSB0 -xsens_mti_baudrate 115200 -xsens_mti_mode 6 &
    ./imu_teste

O -xsens_mti_mode 6 importa: e' o unico modo que traz acc/gyr/mag junto com a
orientacao. Nos modos sem CALIB (o 5, que e' o do ini da IARA) os tres vetores
saem zerados na mensagem e o painel mostra tres barras mortas -- o sensor esta'
bom, quem nao esta' mandando os dados e' a configuracao.

QUANDO O ./xsens NAO ABRE
-------------------------
Ele morre no EXIT_ON_ERROR do doHardwareScan() sem dizer qual foi o problema.
Sao quase sempre duas coisas:

1. Permissao. /dev/ttyUSB* e' root:dialout 660.
       sudo usermod -aG dialout $USER     (exige relogar)
       sudo chmod 666 /dev/ttyUSB0        (destrava so' esta sessao)

2. Baudrate errado. O MTi sai de fabrica em 115200, a IARA usa 460800. Para
   descobrir o do seu sensor, com o ./xsens parado:

       python3 descobre_xsens.py /dev/ttyUSB0

   O script fala o protocolo MT nativo (GoToConfig + ReqDID + ReqProductCode)
   varrendo os baudrates, e responde de uma vez se e' mesmo um Xsens, qual o
   modelo e em que velocidade ele esta'. Precisa do pyserial.

TAXA E PERDAS
-------------
O painel mostra "mensagens publicadas" (o que o ./xsens manda pelo IPC) e
"amostras do sensor" (deduzido dos saltos do contador de amostra). A diferenca
entre as duas e' a perda.

Um MTi a 200 Hz costuma aparecer aqui com ~50% de perda: o laco principal do
src/xsens/xsens.cpp tem um usleep(10000) depois de cada leitura, o que trava o
modulo em 100 Hz. O cmt3.waitForDataMessage() ja' bloqueia ate' chegar pacote,
entao esse usleep so' serve para ficar para tras. Nao foi mexido porque muda o
comportamento da IARA.
