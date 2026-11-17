# pi_nit_server — detecção de pessoas no Raspberry Pi 5 + Hailo-8L

Esta pasta é **tudo o que sobe para o Raspberry**. O PC do CARMEN não precisa
de nada daqui, e o Raspberry não precisa de nada do CARMEN: os dois só
conversam pelas portas ZMQ 5560 (frames) e 5561 (detecções).

O serviço é deliberadamente burro: recebe uma imagem 640×640, roda a rede no
Hailo, devolve as caixas. Não abre câmera, não grava nada em disco, não
guarda estado entre frames. Todo o resto (recorte, letterbox, tracking,
publicação IPC) é feito no PC pelo `pi_nit_client_driver`.

```
PC do CARMEN                              Raspberry Pi 5 + Hailo-8L
─────────────                            ─────────────────────────
camera_drivers                    PUSH
   ↓                            ────────►  :5560  pi_nit_server.py
pi_nit_client_driver                          ↓
   ↑                            ◄────────  :5561  Hailo-8L (13 TOPS)
neural_detector_message           PUSH
```

### Ferramentas em `tools/`

| Arquivo | Onde roda | Para que |
|---|---|---|
| `pi_nit_check.sh` | no Raspberry | diagnóstico do PCIe, do HAT, do driver e do serviço |
| `pi_nit_viewer.py` | qualquer PC da rede | mosaico das câmeras com as caixas, lido do PUB 5562 |
| `test_client.py` | qualquer PC da rede | manda uma imagem e imprime as detecções, sem CARMEN |
| `pi_nit_dummy.sh` | no Raspberry | sobe o servidor em modo `--dummy` (sem Hailo) |
| `acha_objetos_no_log.py` | no PC do CARMEN | varre um log e diz em que trechos há pessoa/carro |
| `webcam_ao_vivo.sh` | no seu notebook | webcam → Raspberry → janela com as caixas, sem CARMEN nenhum |

---

## 1. Hardware

| Item | Observação |
|---|---|
| Raspberry Pi 5 (2 GB) | 2 GB dá conta: o serviço usa ~400 MB |
| Hailo AI HAT+ 13 TOPS (Hailo-8L) | conecta no PCIe do Pi 5 |
| Fonte 27 W USB-C oficial | o HAT+ puxa corrente; fonte fraca causa reset |
| Dissipador ativo | sem ele o Pi 5 faz throttling e o fps cai |
| Rede cabeada gigabit | Wi-Fi funciona, mas a latência oscila muito |

> A 15 fps com JPEG qualidade 80, o tráfego fica em torno de **10–15 Mbit/s**.
> Com `-jpeg_quality 0` (BGR cru) sobe para **~150 Mbit/s** — só use em
> gigabit dedicado.

---

## 2. Sistema operacional

Há dois caminhos. **O plano do time usa o A** (imagem
`2026-06-18-raspios-bookworm-arm64`).

### A. Raspberry Pi OS Bookworm 64-bit — recomendado

Resolve driver, HailoRT, bindings Python, Tappas e modelos com **um comando**:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y hailo-all
sudo reboot
```

Os modelos já vêm em `/usr/share/hailo-models/` — não precisa rodar o
`download_model.sh`, só apontar o `PI_NIT_HEF` para lá:

```bash
ls /usr/share/hailo-models/         # yolov8s_h8l.hef, yolov6n.hef, ...
```

Ainda é preciso habilitar o PCIe (§ 3) e verificar com
`hailortcli fw-control identify` (§ 5). Depois pule direto para o § 6.

Nesse ambiente o backend `gstreamer` também fica disponível de graça, porque
o Tappas vem junto.

### B. Ubuntu 24.04 Server — só se houver uma razão para isso

Nada vem pronto: driver, HailoRT e bindings precisam ser compilados do fonte
(§ 4 e § 5), e o Tappas não está disponível — o que obriga o backend
`hailort`. São umas 2 h de trabalho contra 5 min do caminho A. As instruções
completas estão aqui porque foram pedidas, mas **se não houver um motivo
específico, use o Raspberry Pi OS.**

### Instalação do Ubuntu 24.04 Server no Pi 5

Grave a imagem com o **Raspberry Pi Imager**: `Other general-purpose OS →
Ubuntu → Ubuntu Server 24.04.x LTS (64-bit)`. Use o botão de engrenagem para
já deixar hostname, usuário e SSH configurados. (No caminho A, a imagem é a
`raspios_oldstable_arm64` bookworm — o resto desta seção vale igual.)

Depois do primeiro boot:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y build-essential cmake git curl \
    python3-dev python3-venv python3-pip \
    linux-headers-$(uname -r)
sudo reboot
```

Confirme que é mesmo um Pi 5 com kernel `raspi`:

```bash
uname -a                       # deve conter 'raspi'
cat /proc/device-tree/model    # Raspberry Pi 5 Model B
```

### IP fixo (netplan)

O PC do CARMEN precisa saber onde achar o Raspberry. Edite
`/etc/netplan/50-cloud-init.yaml` (ou crie `/etc/netplan/99-pi-nit.yaml`):

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      addresses: [192.168.1.20/24]
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8]
```

```bash
sudo chmod 600 /etc/netplan/*.yaml
sudo netplan apply
```

---

## 3. Habilitar o PCIe (obrigatório para o HAT+)

No Ubuntu o `config.txt` do firmware fica em `/boot/firmware/config.txt`.
Acrescente ao final:

```ini
# Hailo AI HAT+
dtparam=pciex1
dtparam=pciex1_gen=3
```

```bash
sudo reboot
```

Depois do boot, o dispositivo PCIe tem que aparecer:

```bash
lspci | grep -i hailo
# 0000:01:00.0 Co-processor: Hailo Technologies Ltd. Hailo-8 AI Processor (rev 01)
```

Se não aparecer nada, o problema é elétrico/firmware — não adianta seguir
para a instalação do driver.

> Gen 3 é oficialmente "não suportado" pela Raspberry Pi, mas é o que o
> próprio kit da Hailo recomenda e é onde o 13 TOPS entrega o throughput
> esperado. Se o link ficar instável (erros de PCIe no `dmesg`), volte para
> `dtparam=pciex1_gen=2`.

---

## 4. Driver PCIe do Hailo

O Ubuntu não traz o `hailo_pci`. Compile do fonte oficial. **Use a mesma tag
nos dois repositórios** (driver e runtime) — versões diferentes não
conversam.

```bash
export HAILORT_VERSION=v4.24.0

cd ~
git clone https://github.com/hailo-ai/hailort-drivers.git
cd hailort-drivers
git checkout ${HAILORT_VERSION}

# módulo do kernel
cd linux/pcie
make all
sudo make install
sudo modprobe hailo_pci

# firmware do acelerador
cd ~/hailort-drivers
./download_firmware.sh
sudo mkdir -p /lib/firmware/hailo
sudo cp hailo8_fw.*.bin /lib/firmware/hailo/hailo8_fw.bin

# regras de udev (dão acesso ao /dev/hailo0 sem root)
sudo cp linux/pcie/51-hailo-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Carregar no boot:

```bash
echo hailo_pci | sudo tee /etc/modules-load.d/hailo_pci.conf
```

Verifique:

```bash
ls -l /dev/hailo0
dmesg | grep -i hailo
```

> **Atualização de kernel:** o módulo é compilado contra o kernel atual. Se o
> `apt` subir a versão do kernel, refaça `make all && sudo make install` e
> reinicie. Para evitar surpresas em campo:
> `sudo apt-mark hold linux-image-raspi linux-headers-raspi`.

---

## 5. HailoRT + bindings Python

```bash
cd ~
git clone https://github.com/hailo-ai/hailort.git
cd hailort
git checkout ${HAILORT_VERSION}

cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release -DHAILO_BUILD_PYBIND=1
sudo cmake --build build --config release --target install
sudo ldconfig
```

A compilação demora bastante em um Pi 5 (30–60 min). Se faltar memória, crie
swap antes:

```bash
sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

Instale o wheel Python gerado (o nome traz a versão do Python — no Ubuntu
24.04 é `cp312`):

```bash
find ~/hailort/build -name "hailort-*.whl"
sudo pip3 install --break-system-packages ~/hailort/build/hailort/libhailort/bindings/python/src/dist/hailort-*.whl
```

### Verificação

```bash
hailortcli fw-control identify
```

Tem que sair algo como:

```
Identifying board
Control Protocol Version: 2
Firmware Version: 4.24.0
Device Architecture: HAILO8L
Serial Number: HLD...
```

> O número exato varia por placa/instalação — o `pi-nit` já em campo, por
> exemplo, está em `4.20.0`. Não é erro nem precisa igualar a este exemplo;
> o que importa é `Device Architecture: HAILO8L` aparecer e o comando não
> falhar. Ao escolher a `ZOO_VERSION` de um `.hef` novo (veja "Qual versão
> do YOLO usar" no README principal), confira sempre a versão *desta*
> instalação com o comando acima antes de assumir compatibilidade.

E o Python precisa enxergar:

```bash
python3 -c "import hailo_platform; print(hailo_platform.__version__)"
```

**Se esses dois comandos não funcionarem, pare aqui.** Todo o resto depende
deles.

---

## 6. Instalar o serviço

Copie esta pasta para o Raspberry (do PC do CARMEN):

```bash
rsync -av $CARMEN_HOME/src/pi_nit/pi_nit_server/ pi@192.168.1.20:~/pi_nit_server/
```

No Raspberry:

```bash
cd ~/pi_nit_server
sudo ./install.sh
sudo ./download_model.sh          # baixa yolov8s_h8l.hef
```

O `install.sh`:

- instala os pacotes de sistema necessários;
- cria o usuário de serviço `pi_nit` (sem login, sem home);
- instala a aplicação em `/opt/pi_nit/app` e o venv em `/opt/pi_nit/venv`
  (com `--system-site-packages`, para enxergar o `hailo_platform`);
- instala a configuração em `/etc/pi_nit/pi_nit_server.conf` — **e nunca
  sobrescreve** a existente (a nova vai como `.conf.new`);
- instala e habilita a unit `pi_nit_server.service`.

Rodar de novo o `install.sh` atualiza só o código.

### Configuração

Edite `/etc/pi_nit/pi_nit_server.conf` (formato `CHAVE=valor`):

| Variável | Padrão | Para que serve |
|---|---|---|
| `PI_NIT_HEF` | `/opt/pi_nit/models/yolov8s_h8l.hef` | modelo |
| `PI_NIT_BACKEND` | `hailort` | `hailort` ou `gstreamer` (§ 9) |
| `PI_NIT_FRAME_PORT` | `5560` | porta de entrada de frames |
| `PI_NIT_RESULT_PORT` | `5561` | porta de saída de detecções |
| `PI_NIT_BIND` | `0.0.0.0` | troque pelo IP do Pi para restringir |
| `PI_NIT_CONFIDENCE` | `0.35` | confiança mínima devolvida |
| `PI_NIT_CLASSES` | `0,2,3,5,7` | classes COCO devolvidas (veja abaixo) |
| `PI_NIT_BATCH_SIZE` | `1` | **igual ao número de câmeras** (veja abaixo) |
| `PI_NIT_BATCH_WINDOW_MS` | `20` | espera pelas outras câmeras antes de inferir |
| `PI_NIT_VIEWER_PORT` | `0` | `5562` liga o PUB que o `pi_nit_viewer.py` lê |
| `PI_NIT_DUMMY` | `0` | `1` testa o enlace ZMQ sem usar o Hailo |

```bash
sudo systemctl restart pi_nit_server
```

**`PI_NIT_CLASSES`** — o padrão `0,2,3,5,7` é exatamente o conjunto que o MOT
sabe nomear, porque o `obj_id` publicado é o **id COCO menos 1**
(`multiple_object_tracker.cpp:1012`):

| COCO devolvido aqui | `obj_id` publicado | nome no MOT |
|---|---|---|
| 0 pessoa | `-1` | `pedestrian` |
| 2 carro | `1` | `Car` |
| 3 moto | `2` | `motorcycle` |
| 5 ônibus | `4` | `bus` |
| 7 caminhão | `6` | `truck` |

`all` devolve tudo — inclusive cachorro, semáforo e mochila, que caem em
`unknown` no MOT com dimensões de veículo. Use `0` para voltar a só pedestre.

**`PI_NIT_BATCH_SIZE`** — tem que ser igual ao número de câmeras publicando.
Com batch 3 e uma câmera só, o `hailo_person_detector.py:145` completa o lote
**repetindo a última imagem** e joga fora os resultados extras: dois terços do
acelerador trabalham à toa. O serviço detecta e avisa:

```
batch de 3 configurado mas chegam 1.0 imagem(ns) por vez:
o acelerador processa 3 copias e descarta 2. Ajuste PI_NIT_BATCH_SIZE para 1.
```

---

## 7. Deixar rodando sozinho (systemd)

O `install.sh` já roda o `systemctl enable`, então **o serviço sobe sozinho a
cada boot**. Comandos do dia a dia:

```bash
sudo systemctl start pi_nit_server      # iniciar agora
sudo systemctl status pi_nit_server     # estado
journalctl -u pi_nit_server -f          # log ao vivo
journalctl -u pi_nit_server -b          # log desde o último boot
sudo systemctl restart pi_nit_server    # aplicar mudança de configuração
sudo systemctl disable pi_nit_server    # não subir mais no boot
```

O que a unit garante (`pi_nit_server.service`):

- `Restart=always` + `RestartSec=3` e `StartLimitIntervalSec=0` — reinicia
  para sempre, sem desistir depois de N falhas. Se o Hailo travar, o processo
  morre e volta em 3 s;
- `ConditionPathExists=/dev/hailo0` — não fica em loop de falha quando o
  acelerador não está presente;
- `After=network-online.target` — só sobe com a rede de pé;
- `Nice=-5` — a inferência não é preterida por tarefa de fundo;
- roda como usuário `pi_nit`, sem privilégios, com `ProtectSystem=full` e
  `ProtectHome=yes`.

Confirme que o boot automático está mesmo armado:

```bash
systemctl is-enabled pi_nit_server     # -> enabled
sudo reboot
# depois do boot:
systemctl is-active pi_nit_server      # -> active
```

---

## 8. Testando

**No próprio Raspberry** (sem o CARMEN, sem rede):

```bash
/opt/pi_nit/venv/bin/python3 ~/pi_nit_server/tools/test_client.py --host 127.0.0.1
```

**Do PC do CARMEN**, com uma foto de verdade:

```bash
python3 tools/test_client.py --host 192.168.1.20 --image /tmp/pessoas.jpg --show
# ou, sem Python, com o binário do módulo:
$CARMEN_HOME/bin/pi_nit_link_test 192.168.1.20 -image /tmp/pessoas.jpg -show
```

Saída esperada:

```
frame    12 | rtt   41.3 ms | hailo  18.7 ms | fila   3.1 ms | 2 deteccao(oes)
        classe 0 conf 0.91 [412,180 -> 528,640]
        classe 0 conf 0.78 [ 96,210 -> 190,612]
```

Se `rtt` for alto mas `hailo` for baixo, o gargalo é rede. Se `hailo` for
alto, é o modelo (troque para `yolov8n`).

### Ver as 3 câmeras na tela do Raspberry

Ligue a porta de visualização e reinicie:

```bash
sudo sed -i 's/^PI_NIT_VIEWER_PORT=.*/PI_NIT_VIEWER_PORT=5562/' /etc/pi_nit/pi_nit_server.conf
sudo systemctl restart pi_nit_server
```

Na tela do Raspberry (ou de qualquer PC da rede):

```bash
/opt/pi_nit/venv/bin/python3 /opt/pi_nit/app/tools/pi_nit_viewer.py --host 127.0.0.1
/opt/pi_nit/venv/bin/python3 /opt/pi_nit/app/tools/pi_nit_viewer.py --host 192.168.1.20 --cameras 3,4,5
```

Mostra um mosaico com as câmeras lado a lado, cada uma com as caixas de
pessoa, o **timestamp da imagem original**, o número de pessoas, o tempo de
inferência e o fps daquela câmera. `q` ou `ESC` fecha; `--save saida.mp4`
grava.

A visualização sai por um socket PUB separado e **não interfere na
detecção**: se o viewer não acompanhar, o PUB descarta. Com a porta em `0`
(padrão) nada é publicado.

> O viewer mostra a imagem **como o Raspberry a recebeu**: 640×640 já com o
> letterbox (as faixas cinza em cima e embaixo, no caso de uma câmera 640×480).
> É de propósito — é exatamente o que a rede neural enxerga.

As caixas saem com o nome da classe e uma cor por classe (pessoa, carro, moto,
ônibus, caminhão), conforme o `PI_NIT_CLASSES`.

### Ver o acelerador trabalhando (o "nvtop" do Hailo)

```bash
hailortcli monitor
```

Só funciona se **o processo que faz a inferência** tiver `HAILO_MONITOR=1` no
ambiente — por isso o `pi_nit_server.service` já traz
`Environment=HAILO_MONITOR=1`. Sem isso o monitor abre vazio, reclamando que
não há aplicação rodando.

O `hailortcli measure-power` **não funciona no HAT+ 13 TOPS**: a placa responde
`CONTROL_PROTOCOL_STATUS_UNSUPPORTED_DEVICE`. Para temperatura, use
`hailortcli fw-control identify` e o `vcgencmd measure_temp` do próprio Pi.

---

## 9. Backends de inferência

| | `hailort` (padrão) | `gstreamer` | `cpu` |
|---|---|---|---|
| Onde roda | Raspberry | Raspberry | **PC, para teste** |
| Dependências | HailoRT + bindings Python | HailoRT **+ hailo-tappas-core** | ultralytics + torch |
| Raspberry Pi OS (`hailo-all`) | funciona | funciona | — |
| Ubuntu 24.04 Server | funciona | exige compilar o Tappas | — |
| Pós-processamento | NMS embarcado no HEF | `hailofilter` do Tappas | ultralytics |
| Correspondência frame→detecção | exata | exata (um frame por vez) | exata |

O backend `cpu` **não roda no Raspberry** (o Pi 5 não aguenta YOLO em CPU a
15 fps). Existe para validar toda a cadeia no PC antes de o Hailo estar
pronto — veja a seção *Testando tudo no PC* do [README do módulo](../README.md).

O backend `gstreamer` (`gstreamer_person_detector.py`) existe para reaproveitar
o pipeline `hailonet → hailofilter → identity` que já está validado em
`reference/camera_pipeline.py` — a única diferença é que a fonte é um
`appsrc` alimentado pelo ZMQ, no lugar do `rtspsrc`. Use se você já tem o
ambiente Tappas de pé e quer o mesmo caminho de labels/pós-processamento dos
outros projetos.

`reference/camera_pipeline.py` e `reference/pipeline_manager.py` são os dois
arquivos que já estavam nesta pasta. Ficam aqui como **referência apenas**:
eles leem de câmeras RTSP e importam `pignurse.firmware.*`, que não existe
aqui. Nada em produção os importa.

---

## 10. Problemas comuns

| Sintoma | Causa provável | O que fazer |
|---|---|---|
| `lspci` não mostra o Hailo | PCIe desabilitado ou HAT mal encaixado | conferir `/boot/firmware/config.txt`, reencaixar, fonte de 27 W |
| `brcm-pcie 1000110000.pcie: link down` | o controlador subiu, mas nada respondeu | veja a seção abaixo |
| `/dev/hailo0` não existe | módulo não carregado | `sudo modprobe hailo_pci`, ver `dmesg \| grep hailo` |
| `hailortcli` responde, Python não | wheel não instalado ou Python errado | reinstalar o wheel `cp312` |
| serviço reinicia em loop | modelo ausente ou errado | `journalctl -u pi_nit_server -n 50`; conferir `PI_NIT_HEF` |
| `o HEF nao tem NMS embarcado` | modelo cru | usar o `.hef` do Model Zoo (`hailo8l`), não um compilado sem NMS |
| roda mas não detecta nada | HEF de `hailo8` no lugar de `hailo8l` | rebaixar a confiança para 0.1 e testar; se continuar, trocar o modelo |
| fps abaixo de 15 | throttling térmico | `vcgencmd measure_temp`, `vcgencmd get_throttled` (0x0 = ok) |
| nada chega no CARMEN | firewall | `sudo ufw allow from <ip_do_pc> to any port 5560,5561 proto tcp` |
| `queue_ms` alto | decodificação do JPEG pesada | baixar `-jpeg_quality` no cliente |

Diagnóstico rápido do acelerador:

```bash
hailortcli fw-control identify
hailortcli monitor                                  # o "nvtop" do Hailo (§ 8)
hailortcli run /opt/pi_nit/models/yolov8s_h8l.hef   # benchmark puro do modelo
```

> `hailortcli measure-power` **não funciona neste HAT+ de 13 TOPS**: a placa
> responde `CONTROL_PROTOCOL_STATUS_UNSUPPORTED_DEVICE`. Não é defeito.

---

### PCIe `link down` — o que já foi eliminado

`link down` **não** significa PCIe desabilitado. Se estivesse desabilitado, o
controlador `1000110000.pcie` nem apareceria no `dmesg`. Ele aparece,
inicializa, cria o bus `0001:00` e então reporta que ninguém respondeu.

Diagnóstico feito em campo (ago/2026), com tudo isto **descartado**:

| Verificado | Resultado |
|---|---|
| `dtparam=pciex1` na seção `[all]` | presente, e o controlador inicializa |
| Gen 3 → Gen 2 → **Gen 1** | `link down` nas três velocidades |
| `dtoverlay=pciex1-compat-pi5,no-l0s,no-mip` | sem efeito |
| `pcie_aspm=off` no `cmdline.txt` | sem efeito |
| Bootloader (`rpi-eeprom-update`) | atualizado para o mais recente |
| `echo 1 > /sys/bus/pci/rescan` | nada aparece |
| Driver `hailo_pci` | compilado e registrado no kernel certo |
| Fonte | 5 A negociados, `throttled=0x0`, `EXT5V_V` 5,13 V |

Se tudo acima estiver ok e o link continuar down, a sequência é esta — e a
**ordem importa**:

1. **TROQUE O CABO FLAT DO PCIe.** Foi a causa em 05/08/2026, depois de umas
   seis horas perdidas em driver, EEPROM, `dtparam`, fonte e SDK. O cabo tinha
   continuidade nas primeiras e nas últimas vias e falha nas do meio, então
   medir só as pontas com o multímetro "aprovava" um cabo quebrado. Sem o cabo
   passando, o HAT não recebe o 3V3 do PCIe: o LED fica apagado e a placa fria,
   o que **parece** defeito de alimentação da placa mesmo com os 5 V chegando
   no conector. As duas vias mudas ao mesmo tempo (PCIe e a EEPROM I2C dos 40
   pinos) sugerem "placa morta", mas o cabo explica as duas.
2. **O LED de alimentação do HAT acende?** Se não, meça 5 V entre os pinos
   2/4 e o pino 6 (terra) **no conector do HAT**, em modo tensão DC.
3. **Não chega 5 V no HAT mas chega no header do Pi** → contato. Reencaixe
   pressionando os quatro cantos, atenção à traseira.
4. **Chega 5 V, cabo novo, e a placa continua apagada e fria** → aí sim é o
   circuito de alimentação do HAT. Nenhuma configuração resolve.

O relatório completo daquele diagnóstico, com o que foi eliminado e por que a
conclusão inicial estava errada, está em
[`diagnostico_hailo.txt`](../diagnostico_hailo.txt).

> Medindo o LED com o multímetro em **modo diodo**, ~2,0 V é a queda direta
> normal de um LED amarelo — o LED está bom. Já ~0,5 V em **modo tensão**
> sobre o LED significa que ele não está sendo alimentado: um LED amarelo
> precisa de ~2 V para acender.

---

## 11. Plano B: Raspberry Pi OS

Se a instalação do HailoRT no Ubuntu travar, o Raspberry Pi OS Bookworm
64-bit resolve tudo com um comando:

```bash
sudo apt install hailo-all      # driver + HailoRT + Python + Tappas + modelos
sudo reboot
```

Os modelos ficam em `/usr/share/hailo-models/`. O `install.sh` e o serviço
funcionam igual — só ajuste `PI_NIT_HEF` no `.conf`. Nesse ambiente o backend
`gstreamer` também fica disponível de graça.
