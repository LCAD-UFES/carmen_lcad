Instrucoes:

1. Ajuste a variavel de ambiente CAFFE_HOME para apontar para o diretorio caffe-segnet
2. Feche o terminal e abra de novo para atualizar as variaveis de ambiente (em tese, dar um bash no terminal deveria atualizar as variaveis, mas isso nao funcionou algumas vezes...).
3. Compile a caffe-segnet usando os comandos abaixo:

	cd caffe-segnet
	make all -j 8

4. Compile o modulo semantic_segmentation usando os comando abaixo (no diretorio raiz do modulo)

	make 

5. Baixe o caffemodel de http://mi.eng.cam.ac.uk/~agk34/resources/SegNet/segnet_iter_30000_timo.caffemodel
6. Para rodar o programa de exemplo, use: 

	./semantic_segmentation_test

O codigo le uma imagem chamada exemplo.png, faz a segmentacao, e mostra a imagem segmentada como saida. As cores de cada classe sao lidas do arquivo camvid12.png (voce pode abrir o arquivo com "eog camvid12.png" para visualizar a paleta de cores) . Processar uma imagem usando a CPU demora cerca de 5 segundos. A opcao de usar a cpu ou gpu esta hardcoded no arquivo semantic_segmentation.cpp . 
