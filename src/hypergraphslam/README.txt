1 - Comando para ler o log e gravar os dados

./parser <log.txt> <output>

ex:
./parser /dados/log_ponte-20161208.txt sync.txt


2 - Rodar o hypergraphslam para otimizar as poses

./hypergraphslam sync.txt poses.txt
