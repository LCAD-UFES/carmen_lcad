NEURAL_GLOBAL_LOCALIZER
Posicionamento global por meio de aprendizagem de imagens.

Base de dados:

Traning - Imagens da Volta da UFES no ano de 2012, capturadas de 10 em 10 metros.
Test - Imagens da Volta da UFES no ano de 2013, capturadas de 50 em 50 metros.

#Como rodar.
./neural_global_localizer neural_global_localizer.cml

#Parametros CML:
NUMBER_OF_TRAINING_FRAMES - Número de quadros a serem treinados.
NUMBER_OF_TEST_FRAMES - Número de quadros a serem reconhecidos.
TRAINING - 1 ou 0 - Informa se será realizar o treinamento ou se será lido do arquivo de treinamento .mem

#Parametros CON:
INPUT_TRAINING_PATH - Aponta para o caminho da sua base de treino;
Ex: INPUT_TRAINING_PATH = "/home/alberto/robotics/code/carmen/data/neural_localizer/test/";

INPUT_TEST_PATH - Aponta para o caminho da sua base de teste;
Ex: INPUT_TEST_PATH = "/home/lauro/robotics/code/carmen/data/neural_localizer/test/";

DATA_SET_FILE - Configura o nome do arquivo com os nomes das imagens e suas correspondencias entre a base de teste e a base de treino.



