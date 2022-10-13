Latency
Separar mensagens:
grep ford->oj_effort results_latency-ford.txt > results_latecy_ford_escape-ford-effort.txt
grep oj-from-ford_effort results_latency-oj.txt > results_latecy_jaus-oj-effort.txt
grep oj-from-ford->car_effort results_latency-oj.txt > results_latecy_jaus-car-effort.txt
grep car->oj_feedback results_latency-oj.txt > results_latecy_jaus-car-feedback.txt
grep oj->ford_feedback results_latency-oj.txt > results_latecy_jaus-oj-feedback.txt
grep ford_form_oj_feedback results_latency-ford.txt > results_latecy_ford_escape-ford-feedback.txt

k=1628183577.563692
plot "./results_latecy_ford_escape-ford-effort.txt" using ($7-k):($5/1000) with lines title "f->oj-effort", \
"./results_latecy_jaus-oj-effort.txt" using ($7-k):($5/1000) with lines title "oj<-ford-effort", \
"./results_latecy_jaus-car-effort.txt" using ($7-k):((($5/60)/1000)*-1.0) with lines title "oj->car", \
"./results_latecy_jaus-car-feedback.txt" using ($7-k):6 with lines title "car->oj_feedback", \
"./results_latecy_jaus-oj-feedback.txt" using ($7-k):6 with lines title "oj->ford_feedback", \
"./results_latecy_ford_escape-ford-feedback.txt" using ($7-k):6 with lines title "ford_form_oj_feedback"


Mapeado
Sentido carmen->carro->carmen:
Comando enviado pro oj: ford->oj_effort ford_escape_hybrid.c
Comando recebido pelo oj: oj-from-ford_effort pd.c
Comando enviado do oj para o carro: oj-from-ford->car_effort pd.c
Feedback do carro para o oj: car->oj_feedback main.c
Feedback enviado do oj para o ford: oj->ford_feedback mpd.c
Feedback recebido do oj lido no ford: ford_form_oj_feedback ford_escape_hybrid.c
