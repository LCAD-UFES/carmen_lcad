# Prometheus Drivers

Para fazer os drivers foi utilizado o SDK disponibilizado plea Unitree, basta fazer o download do repositório [aqui](https://github.com/unitreerobotics/unitree_sdk2).

Para instalar o SDK no seu computador, basta ler o guia da [Unitree](https://github.com/unitreerobotics/unitree_sdk2/blob/main/README.md), e seguir os passos de instalação.

# Prometheus Odom to CAN

O código aceita a flag "--log_sensor_data", para salvar os dados de odometria direto em um arquivo '.txt' que está hardcoded no fonte como "log_prometheus_sensor_data.txt".

Exemplo de uso:
```
./prometheus_odom2can --log_sensor_data
```