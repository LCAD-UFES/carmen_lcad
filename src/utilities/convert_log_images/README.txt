como usar o script para converter imagens dos logs NOVOS para png: 

1. extraia as mensagens da bumblebee do log (substitua o log.txt pelo nome do seu log de interesse): 
grep BUMB log.txt > log_filtrado.txt

2. use o programa para converter imagens (substitua o nome pasta_saida pelo nome da sua pasta de interesse):
./to_png_new_log log_filtrado.txt pasta_saida

O programa vai mostrar as imagens na tela a medida que o log for convertido. 
Se elas estiverem esquisitas é sinal de que alguma coisa esta errada.

 

