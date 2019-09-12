O script de treino da rede com pytorch está no git em src/neural_mapper/pytorch_neural_mapper .

Há também um script de testes que abre um modelo treinado e escolhe um indice de entrada pra retornar uma saída (test_model.py).

Utilização da rede:
- Instalar python 3.5 e o pytorch para ele (python padrão do ubuntu 16.04)
- Colocar a pasta 60mts/, em anexo,  em /dados/neural_mapper/
- criar a pasta saved_models/ em (...) /pytorch_neural_mapper/
- Para treinar:
    python3 train.py --batch-size 5 --log-interval 10 --lr 0.001 --epochs 1000
- Para testar:
    python3 test_model.py --model-name <nome_do_modelo> //(Ex. 1000.model)

Usando Script de Treino para dataset em formato binário

- Instalar python 3.5 e o pytorch para ele (python padrão do ubuntu 16.04)

- Modifique o arquivo de configuração passando os caminhos e config necessárias:
    weights_initialization
    model_weights
    bach_size
    interations
    learning_rate
    step_size
    shuffle_data
    train_data
    test_data
    
    

- Para treinar:
    python3 train_with_binary_dataset.py
- Para testar:
    python3 test_model.py --model-name <nome_do_modelo> //(Ex. 1000.model)

