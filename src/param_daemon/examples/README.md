# param_daemon/examples

## Exemplos de múltiplos arquivos de parâmetros estruturados hierarquicamente 

Arquivos de parâmetros de exemplo neste diretório:

O arquivo `carmen-ford-escape-sensorbox.ini` contém 2003 parâmetros organizados de forma linear, sem utilização da diretiva `$include`.

O arquivo `carmen-ford-escape-sensorbox-includes.ini` está estruturado de forma hierárquica, utilizando diretivas `$path` e `$include`, que referenciam os arquivos do diretório `./subfiles`. Utilizando-se este arquivo, o resultado final é equivalente, com 2003 parâmetros carregados.
