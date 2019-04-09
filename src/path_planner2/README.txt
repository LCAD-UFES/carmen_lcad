Instale as dependências no python3 
    pip3 install -r FrenetOptimalTrajectory/requirements.txt
Adicione a pasta do FrenetOptimalTrajectory no PYTHONPATH
    export PYTHONPATH=PYTHONPATH:$CARMEN_HOME/src/path_planner2/FrenetOptimalTrajectory

Metodo, provisorio para rodar:
    Rode um process...ini de sua preferencia (normalmente o navigate), mate o RDDF_play, rode o modulo path_planner2 com os parametros do RDDF_play
    para inserir um pedestre rode o neural_object_detector_simulator que fica em src/neural_object_detector3 e um pedestre parado devera aparecer perto da Cantina do CT
    ou rode um detector de pedestre em um log que possua pedestres.