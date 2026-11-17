# Imagem pronta do Raspberry — gravar e ligar

Esta pasta gera uma imagem do Raspberry Pi OS **já customizada**: usuário,
senha, SSH, IP fixo, PCIe do Hailo e o `pi_nit_server` instalado e habilitado
no boot. Você grava no cartão, liga, e está funcionando — sem `raspi-config`,
sem editar arquivo nenhum.

## O que vem configurado

| | |
|---|---|
| Sistema | Raspberry Pi OS Bookworm **Lite** arm64 (`2026-06-18`, o do plano) |
| Usuário | `pi` |
| Senha | `1q2w3e4r` |
| Hostname | `pi-nit` |
| IP fixo (eth0) | `192.168.1.20/24`, gateway `192.168.1.1` |
| SSH | ligado |
| PCIe | `dtparam=pciex1` + `dtparam=pciex1_gen=3` |
| Serviço | `pi_nit_server` habilitado — sobe sozinho a cada boot |
| Aplicação | `/opt/pi_nit/` |
| Modelo | `yolov8s_h8l.hef` (o do `hailo-all`, ou baixado no build) |
| Classes | `0,2,3,5,7` — pessoa, carro, moto, ônibus, caminhão |
| Batch | `1` — **igual ao número de câmeras** |

Tudo isso é ajustável na hora de gerar (`--ip`, `--hostname`, `--password`…).

### Detecção: dois valores que mudam o resultado

O `.conf` da imagem vem de `pi_nit_server/pi_nit_server.conf`, copiado inteiro
para dentro dela (`build_pi_image.sh:304`) — então qualquer ajuste feito no
repositório antes do build já entra na imagem.

- **`PI_NIT_CLASSES=0,2,3,5,7`** é o conjunto que o `multiple_object_tracker`
  sabe nomear. O `obj_id` publicado é o **id COCO menos 1**, que é a convenção
  dele (`multiple_object_tracker.cpp:1012`): pessoa `-1`, carro `1`, moto `2`,
  ônibus `4`, caminhão `6`. Para voltar a só pedestre, use `0`.
- **`PI_NIT_BATCH_SIZE=1`** porque hoje é uma câmera. Com batch maior que o
  número de câmeras, o Hailo **repete a última imagem** para completar o lote e
  descarta os resultados extras — com batch 3 e uma câmera, dois terços do
  acelerador trabalham à toa. O serviço avisa no `journalctl`.

Depois de gravado, dá para mudar sem regerar a imagem:

```bash
ssh pi@192.168.1.20 "sudo sed -i 's/^PI_NIT_BATCH_SIZE=.*/PI_NIT_BATCH_SIZE=3/' \
    /etc/pi_nit/pi_nit_server.conf && sudo systemctl restart pi_nit_server"
```

---

## 1. Gerar a imagem (no PC, uma vez)

```bash
cd $CARMEN_HOME/src/pi_nit/pi_nit_server/image
sudo ./build_pi_image.sh
```

Precisa de `sudo` (o script monta a imagem em loop device) e de espaço livre:
**~3,5 GB** no modo padrão, **~7 GB** com `--offline`. O resultado sai em
`/var/tmp/pi_nit_image/pi_nit-pi-nit-AAAAMMDD.img`.

Variações:

```bash
sudo ./build_pi_image.sh --ip 192.168.1.30 --hostname pi-nit-2   # segundo Pi
sudo ./build_pi_image.sh --offline                               # sem depender de internet no Pi
sudo ./build_pi_image.sh --compress                              # gera .img.xz para guardar/enviar
sudo ./build_pi_image.sh --work-dir /dados/tmp                   # se faltar espaço em /var/tmp
```

### Os dois modos — escolha com cuidado

**`firstboot` (padrão)** — a imagem sai em ~5 min. Na **primeira vez** que o
Raspberry ligar, ele instala o `hailo-all` e a aplicação sozinho (~5 min) e
reinicia. **Exige que o Pi tenha internet nesse primeiro boot.** Acompanhe:

```bash
ssh pi@192.168.1.20 'tail -f /var/log/pi_nit_firstboot.log'
```

**`--offline`** — instala tudo dentro da imagem, por emulação arm64. O
Raspberry **nunca precisa de internet**. Demora ~30 min para gerar e exige:

```bash
sudo apt install qemu-user-static binfmt-support
```

Se a rede do veículo não tiver saída para a internet, é este o modo a usar.

### Sobre o `e2fsprogs` (só afeta o `--offline`)

A imagem do Raspberry Pi OS 2026 usa a feature ext4 **`orphan_file`** — que
aparece como `FEATURE_C12` nas ferramentas antigas. O `e2fsprogs` do Ubuntu
20.04 é o 1.45.5, que não a conhece:

```
/dev/loopXp2 has unsupported feature(s): FEATURE_C12
e2fsck: Get a newer version of e2fsck!
```

Isso **não afeta o modo padrão**: `orphan_file` é uma feature *compat*, ou
seja, quem não a entende simplesmente ignora e monta o filesystem
normalmente. O `e2fsck` é que é estrito, porque precisa entender tudo para
checar — e ele só entra em cena quando há **redimensionamento**, que só
acontece no `--offline`.

O script resolve sozinho: detecta a versão antiga **antes** do download de
443 MB e compila o `e2fsprogs 1.47.0` no diretório de trabalho (uns 2 min, só
na primeira vez). Não instala nada no sistema e não mexe no `e2fsprogs` do
Ubuntu.

No modo padrão nada disso acontece — o filesystem nem é tocado, porque o
próprio Raspberry Pi OS expande a raiz para o tamanho do cartão no primeiro
boot (`init=/usr/lib/raspberrypi-sys-mods/firstboot` no `cmdline.txt`).

---

## 1b. Ajustar a imagem sem refazer o build

Trocar modelo, IP, senha ou qualquer chave da configuração **não exige gerar a
imagem de novo** (40 min) nem ligar o Raspberry. O `patch_image.sh` monta a
imagem aqui no PC e edita direto:

```bash
sudo ./patch_image.sh --list        # o que tem lá dentro e como está configurado
sudo ./patch_image.sh               # conserta o PI_NIT_HEF sozinho
sudo ./patch_image.sh --hef /usr/share/hailo-models/yolov6n_h8l.hef
sudo ./patch_image.sh --ip 192.168.1.30 --hostname pi-nit-2
sudo ./patch_image.sh --set PI_NIT_VIEWER_PORT=5562
sudo ./patch_image.sh --password outrasenha
```

Sem `--image`, ele pega a imagem mais recente de `/var/tmp/pi_nit_image`.

O `--list` mostra os modelos que existem na imagem e avisa quando o
`PI_NIT_HEF` aponta para um arquivo que não está lá:

```
--- modelos disponiveis na imagem ---
/usr/share/hailo-models/yolov6n_h8l.hef
/usr/share/hailo-models/yolov8s_h8l.hef
...
ATENCAO: PI_NIT_HEF aponta para '/opt/pi_nit/models/yolov8s_h8l.hef', que NAO existe na imagem.
```

Rodando sem argumento nenhum, ele escolhe um detector válido e conserta.

> Só funciona em imagem gerada com `--offline`. No modo firstboot a
> configuração ainda não existe — ela só é criada no primeiro boot do Pi.

**Escolha do modelo:** tem que ser `_h8l` (Hailo-8L de 13 TOPS — um `_h8` é do
Hailo-8 de 26 TOPS e não roda) e tem que ser detector, não `_pose_` nem
`_seg_`, cujo formato de saída é outro. O script já respeita isso.

---

## 2. Gravar no cartão

Com o **Raspberry Pi Imager**:

1. `Choose OS` → **`Use custom`** → selecione o `.img` gerado
2. `Choose Storage` → o cartão SD (ou o SSD NVMe)
3. **NÃO** use a engrenagem de configuração — já está tudo dentro da imagem;
   o que você puser lá vai sobrescrever o usuário e a rede
4. `Write`

Ou direto pelo terminal (**confira o `of=` duas vezes** — errar aqui apaga o
disco errado):

```bash
lsblk                                   # descubra qual é o cartão
sudo dd if=pi_nit-pi-nit-20260805.img of=/dev/sdX bs=4M status=progress conv=fsync
sync
```

---

## 3. Ligar e conferir

Encaixe o AI HAT+, ligue o cabo de rede e a fonte de 27 W.

```bash
ssh pi@192.168.1.20          # senha 1q2w3e4r

lspci | grep -i hailo        # o acelerador apareceu?
hailortcli fw-control identify   # deve dizer HAILO8L
systemctl status pi_nit_server   # deve estar 'active (running)'
journalctl -u pi_nit_server -f   # log ao vivo
```

No modo `firstboot`, dê ~7 min desde ligar: ele instala e reinicia sozinho.

---

## 4. Se algo não subir

| Sintoma | O que olhar |
|---|---|
| Não responde no IP | cabo/switch; o IP fixo é só na **eth0** (Wi-Fi não é configurado) |
| SSH recusa a senha | gravou com a engrenagem do Imager? ela sobrescreve o `userconf.txt` |
| `lspci` sem Hailo | HAT mal encaixado ou fonte fraca; conferir `/boot/firmware/config.txt` |
| Serviço parado | `journalctl -u pi_nit_server -n 50`; no firstboot ver `/var/log/pi_nit_firstboot.log` |
| Firstboot travado | o Pi não tinha internet — regere a imagem com `--offline` |
| `FEATURE_C12` / `Get a newer version of e2fsck` | `e2fsprogs` antigo; o script compila o 1.47 sozinho (veja acima). Só ocorre com `--offline` |

Trocar o IP depois de gravado, sem regerar a imagem:

```bash
sudo nmcli connection modify pi-nit-eth0 ipv4.addresses 192.168.1.30/24
sudo nmcli connection up pi-nit-eth0
```
