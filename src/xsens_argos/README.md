
---

# xsens_argos

Este pacote atua como uma ponte (bridge) entre o **Unitree SDK 2** e o ecossistema **CARMEN**. Ele subscreve aos dados de baixo nível (`LowState`) do robô (Go2/G1) via DDS e os republica no formato de mensagens `XSENS_QUAT` compatíveis com o framework CARMEN.

## 📋 Descrição

O `xsens_argos` é essencial para integrar a IMU nativa dos robôs da Unitree com módulos do CARMEN que dependem de orientação e aceleração (como filtros de Kalman, odometria ou SLAM). Ele extrai o Quatérnion, Acelerômetro, Giroscópio e Temperatura, convertendo-os em tempo real para as mensagens `CARMEN_XSENS_GLOBAL_QUAT_NAME`.

## 🛠️ Requisitos

Antes de compilar, certifique-se de ter instalado:

* **CARMEN**: O framework de navegação deve estar configurado e a variável `CARMEN_HOME` apontada corretamente no Makefile.
* **Unitree SDK 2**: Instalado em `/usr/local` (incluindo as dependências de DDS).
* **Dependências de Linkagem**:
    * Unitree SDK 2 (`unitree_sdk2`, `ddscxx`, `ddsc`)
    * CARMEN IPC e bibliotecas de interface (`global`, `ipc`, `param_interface`, etc.)

## 🚀 Compilação

Para compilar o módulo, utilize o comando `make` no diretório raiz do pacote:

```bash
make
```

O comando irá:
1.  Gerar o objeto `xsens_argos_main.o`.
2.  Linkar as bibliotecas necessárias.
3.  Gerar o executável `xsens_argos`.
4.  Copiar automaticamente o executável para o diretório `$CARMEN_HOME/bin/`.

## 📂 Uso

Para executar o módulo, utilize a seguinte sintaxe:

```bash
./xsens_argos <interface_de_rede> <domain_id> <display_hz>
```

### Parâmetros:
* **interface_de_rede**: Interface onde o robô está conectado (ex: `eth0`, `wlan0` ou `lo`). Padrão: `eth0`.
* **domain_id**: ID de domínio do DDS (geralmente `0`). Padrão: `0`.
* **display_hz**: Frequência de atualização do log no terminal. Padrão: `10.0`.

**Exemplo:**
```bash
./xsens_argos enp3s0 0 20.0
```

## 📊 Mensagens Publicadas

O módulo publica e registra no log o padrão `XSENS_QUAT` com a seguinte estrutura de dados:

| Campo | Descrição |
| :--- | :--- |
| **Acc (X, Y, Z)** | Aceleração linear bruta do robô. |
| **Gyr (X, Y, Z)** | Velocidade angular bruta. |
| **Mag (X, Y, Z)** | Magnetismo (zerado por padrão no Go2). |
| **Quat (W, X, Y, Z)** | Orientação em formato de Quatérnion. |
| **Temp** | Temperatura interna da IMU. |
| **Timestamp** | Tempo do sistema CARMEN. |

## ⚙️ Parâmetros do CARMEN

O módulo lê as seguintes configurações do arquivo `carmen.ini`:

* `xsens_type`: Define o tipo de simulação/sensor (Padrão: `1`).

---

### Notas de Implementação
* **Filtro de Log**: O código possui um limitador de taxa para o `printf`, evitando o flooding do terminal enquanto mantém a publicação no IPC na frequência máxima permitida pelo SDK da Unitree.
* **Sinais**: Implementa o tratamento de `SIGINT` (Ctrl+C) para desconectar o IPC do CARMEN de forma segura.

---

Essa documentação cobre o que você construiu no código e no Makefile. Parece que o sistema de teleoperação do G1 está ficando bem robusto com essa integração!

Alguma parte específica da lógica de conversão da IMU que você queira detalhar mais no Markdown?