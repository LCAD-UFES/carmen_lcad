
info:
	@echo "\
	- A RNA gerada ja se encontra no arquivo steering_ann.net. Porem, se quiser faze-la novamente, execute: make train \
	\n- Para ele compilar todos os fontes, gerar as entradas, fazer o teste e exibir os graficos: make graphs \
	"

build:
	@for i in *.c;do                \
		echo "Building $$i...";     \
		gcc $$i -o $${i%c}e -lfann; \
	done

clean:
	@rm -vrf *.e *.saida entradas

graphs: test
	@for ARQ in 20140324-6.saida 20140324-15-2.saida;do \
	   	echo "plot '$$ARQ' using 2 title 'Resultado' with lines, '$$ARQ' using 4 title 'Desejado' with lines, '$$ARQ' using 6 title 'Erro' with lines" | gnuplot -persist; \
	done

train: build_inputs build
	./treino.e entradas/results_pid-steering-20140324-6.train

test: build_inputs build
	./teste.e entradas/results_pid-steering-20140324-6.train | grep '^Resultado' > 20140324-6.saida
	./teste.e entradas/results_pid-steering-20140324-15-2.train | grep '^Resultado' > 20140324-15-2.saida

build_inputs: build
	mkdir -p entradas
	for i in DadosPID/*;do         \
		trainFile=$${i#DadosPID/}; \
		echo "./gerarEntrada.e $$i 40 > entradas/$${trainFile%txt}train" ; \
		./gerarEntrada.e $$i 40 > entradas/$${trainFile%txt}train; \
	done

