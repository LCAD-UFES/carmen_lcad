#!/bin/bash

# Script para rodar testes do Estereo VGRAM que esta no carmen
# Neste Script nos fizemos os testes com as imagens do middlebury stereo evaluation
# O executavel stereo_test foi implementado para fazer teste com imagens stereo estaticas.

# O executavel stereo_test recebe como parametro:
# 	Imagem Esquerda
#	Imagem direita
#	Maximo de disparidade
#	Nome do Algoritmo de estereo
#	Raio Gaussiana inicial
#	Numero de sinapses
#	Numero de itereções do algoritmo que otimiza de disparidades comparando com vizinhança  
#	Lambda: parametro de entrada para funcao que combina duas funcoes de custo
#	tal1, tal2: 	limiar de cor para algoritmo que otimiza as disparidades em regioes sem textura (tal1 > tal2)
#	L1, L2:		tamanho maximo da regiao com falta de textura (L1 > L2) (Obs: se um deste dois paramtros for igual a zero este otimizaçao nao sera aplicada)
#	Arquivo onde sao gravado alguns parametros que foram usados para produzir a imagem de saida
#	Nome da imagem de saida (Obs: nao e necessario informar a extensao de  saida, sera grava como ".pgm")
#	Arquivo onde sao gravados os nomes da imagens que foram testadas (este arquivo e gravado no formato que o avaliador de desempenho do middlebury espera)
#	Numero de redes utilizadas para calcular as disparidades
#	Fator de multiplicacao do raio da gaussiana a cada nova rede




#Este parametros foram estimados empiricamente (eles deram melhor resultado do que o paper anterior)
#O raio da Gaussiana pode ser escalado para cima e para baixo, vericamos que o raio sendo escalado para baixo o resultado fica melhor.

  
./stereo_test ~/Downloads/stereo-pairs/tsukuba/imL.png ~/Downloads/stereo-pairs/tsukuba/imR.png \
16 VGRAM 3.0 64 0 30 20 6 34 17 16 tsukuba_param.txt tsukuba tsukuba_middlebury_test.txt 1 0.95

./stereo_test ~/Downloads/stereo-pairs/venus/imL.png ~/Downloads/stereo-pairs/venus/imR.png \
20 VGRAM 3.0 64 0 30 20 6 34 17 8 venus_param.txt venus venus_middlebury_test.txt 1 0.95

./stereo_test ~/Downloads/stereo-pairs/teddy/imL.png ~/Downloads/stereo-pairs/teddy/imR.png \
60 VGRAM 3.0 64 0 30 20 6 34 17 4 teddy_param.txt teddy teddy_middlebury_test.txt 1 0.95

./stereo_test ~/Downloads/stereo-pairs/cones/imL.png ~/Downloads/stereo-pairs/cones/imR.png \
60 VGRAM 3.0 64 0 30 20 6 34 17 4 cones_param.txt cones cones_middlebury_test.txt 1 0.95
