plot "./results_pid-velocity.txt" using 12 with lines title "cv", "./results_pid-velocity.txt" using 13 with lines title "dv", "./results_pid-velocity.txt" using 14 with lines title "error", "./results_pid-velocity.txt" using ($15/10) with lines title "throt", "./results_pid-velocity.txt" using ($16/100) with lines title "breaks", "./results_pid-velocity.txt" using ($17/1) with lines title "integ", "./results_pid-velocity.txt" using ($18/10) with lines title "deriv"
k=1628183577.563692
plot "./results_pid-steering.txt" using ($14-k):8 with lines title "cc", "./results_pid-steering.txt" using ($14-k):9 with lines title "dc", "./results_pid-steering.txt" using ($14-k):10 with lines title "e", "./results_pid-steering.txt" using ($14-k):11 with lines title "i", "./results_pid-steering.txt" using ($14-k):($12/10) with lines title "d", "./results_pid-steering.txt" using ($14-k):($13/1000) with lines title "s"


Plot com linhas mais grossas
plot "./results_pid-velocity.txt" using 12 with lines lw 3 title "cv", "./results_pid-velocity.txt" using 13 with lines lw 3 title "dv", "./results_pid-velocity.txt" using 14 with lines lw 3 title "error", "./results_pid-velocity.txt" using ($15/10) with lines lw 3 title "throt", "./results_pid-velocity.txt" using ($16/100) with lines lw 3 title "breaks", "./results_pid-velocity.txt" using ($17/1) with lines lw 3 title "integ", "./results_pid-velocity.txt" using ($18/10) with lines lw 3 title "deriv"
k=1651845875.448186
plot "./results_pid-steering.txt" using ($14-k):8 with lines lw 3 title "cc", "./results_pid-steering.txt" using ($14-k):9 with lines lw 3 title "dc", "./results_pid-steering.txt" using ($14-k):10 with lines lw 3 title "e", "./results_pid-steering.txt" using ($14-k):11 with lines lw 3 title "i", "./results_pid-steering.txt" using ($14-k):($12/10) with lines lw 3 title "d", "./results_pid-steering.txt" using ($14-k):($13/1000) with lines lw 3 title "s"

plot "./results_pid-velocity.txt" using 19:12 with lines title "cv", \
"./results_pid-velocity.txt" using 19:13 with lines title "dv", \
"./results_pid-velocity.txt" using 19:($15/10) with lines title "throt", \
"./results_pid-velocity.txt" using 19:($16/100) with lines title "breaks", \
"../../bin/bs.txt" using 2:4 with lines title "goal-v", \
"../../bin/bs.txt" using 2:($6/10) with lines title "who set v", \
"../../bin/bs.txt" using 2:($8/10) with lines title "bs s", \
"../../bin/bs.txt" using 2:($10/10) with lines title "rp s", \
"./mpp.txt" using 2:6 with lines title "tt", \
"./mpp.txt" using 2:14 with lines title "s", \
"./mpp.txt" using 2:8 with lines title "a"
