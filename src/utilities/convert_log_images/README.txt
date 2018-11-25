------------------------------------------------------------------
Como usar o script para converter imagens dos logs NOVOS para png: 
------------------------------------------------------------------

1. Extraia as mensagens da bumblebee do log (substitua o log.txt pelo nome do seu log de interesse): 
grep BUMB log.txt > log_filtrado.txt

2. Use o programa para converter imagens (substitua o nome pasta_saida pelo nome da sua pasta de interesse):
./to_png_new_log log_filtrado.txt pasta_saida

* Para exibir as imagens que estão sendo convertidas acrescente a flag -show
./to_png_new_log log_filtrado.txt pasta_saida -show

* Caso queira converter apenas um dos pares de imagens acrescente a flag -side "lado"  (0 left ou 1 right)
./to_png_new_log log_filtrado.txt pasta_saida -side 0

** Ambas as flasgs podem ser usadas ao mesmo tempo:
./to_png_new_log log_filtrado.txt pasta_saida -show -side 0

** Se a pasta de saida não existir, ela sera criada.