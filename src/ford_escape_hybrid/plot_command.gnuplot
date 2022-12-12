plot "./results_pid-velocity.txt" using 13 with lines title "cv", "./results_pid-velocity.txt" using 14 with lines title "dv", "./results_pid-velocity.txt" using 15 with lines title "error", "./results_pid-velocity.txt" using ($16/10) with lines title "throt", "./results_pid-velocity.txt" using ($17/100) with lines title "breaks", "./results_pid-velocity.txt" using ($18/1) with lines title "integ", "./results_pid-velocity.txt" using ($19/10) with lines title "deriv"

k = "`head -1 ./results_pid-velocity.txt | awk '{print $20}'`"
plot "./results_pid-velocity.txt" using ($20-k):13 with lines title "cv", "./results_pid-velocity.txt" using ($20-k):14 with lines title "dv", "./results_pid-velocity.txt" using ($20-k):15 with lines title "error", "./results_pid-velocity.txt" using ($20-k):($16/10) with lines title "throt", "./results_pid-velocity.txt" using ($20-k):($17/100) with lines title "breaks", "./results_pid-velocity.txt" using ($20-k):($18/1) with lines title "integ", "./results_pid-velocity.txt" using ($20-k):($19/10) with lines title "deriv", "./results_pid-velocity.txt" using ($20-k):21 with lines title "a"
k = "`head -1 ./results_pid-steering.txt | awk '{print $14}'`"
plot "./results_pid-steering.txt" using ($14-k):8 with lines title "cc", "./results_pid-steering.txt" using ($14-k):9 with lines title "dc", "./results_pid-steering.txt" using ($14-k):10 with lines title "e", "./results_pid-steering.txt" using ($14-k):11 with lines title "i", "./results_pid-steering.txt" using ($14-k):($12/10) with lines title "d", "./results_pid-steering.txt" using ($14-k):($13/1000) with lines title "s"


Plot com linhas mais grossas
plot "./results_pid-velocity.txt" using 13 with lines lw 3 title "cv", "./results_pid-velocity.txt" using 14 with lines lw 3 title "dv", "./results_pid-velocity.txt" using 15 with lines lw 3 title "error", "./results_pid-velocity.txt" using ($16/10) with lines lw 3 title "throt", "./results_pid-velocity.txt" using ($17/100) with lines lw 3 title "breaks", "./results_pid-velocity.txt" using ($18/1) with lines lw 3 title "integ", "./results_pid-velocity.txt" using ($19/10) with lines lw 3 title "deriv"
k = "`head -1 ./results_pid-velocity.txt | awk '{print $14}'`"
plot "./results_pid-steering.txt" using ($14-k):8 with lines lw 3 title "cc", "./results_pid-steering.txt" using ($14-k):9 with lines lw 3 title "dc", "./results_pid-steering.txt" using ($14-k):10 with lines lw 3 title "e", "./results_pid-steering.txt" using ($14-k):11 with lines lw 3 title "i", "./results_pid-steering.txt" using ($14-k):($12/10) with lines lw 3 title "d", "./results_pid-steering.txt" using ($14-k):($13/1000) with lines lw 3 title "s"

plot "./results_pid-velocity.txt" using 19:12 with lines title "cv", \
"./results_pid-velocity.txt" using 19:13 with lines title "dv", \
"./results_pid-velocity.txt" using 19:($15/10) with lines title "throt", \
"./results_pid-velocity.txt" using 19:($16/100) with lines title "breaks", \
"./bs.txt" using 2:4 with lines title "goal-v", \
"./bs.txt" using 2:($6/10) with lines title "who set v", \
"./bs.txt" using 2:($8/10) with lines title "bs s", \
"./bs.txt" using 2:($10/10) with lines title "rp s", \
"./mpp.txt" using 2:6 with lines title "tt", \
"./mpp.txt" using 2:14 with lines title "s", \
"./mpp.txt" using 2:8 with lines title "a"
