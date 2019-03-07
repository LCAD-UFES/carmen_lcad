
#include <iostream>
#include <cmath>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


Matrix<double, 3, 3>
euler2rotation(double roll, double pitch, double yaw)
{
    double rx, ry, rz;
    Matrix<double, 3, 3> Rx, Ry, Rz, R;

    rx = roll;
    ry = pitch;
    rz = yaw;

    Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
    Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
    Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
    R  = Rz * Ry * Rx;

    return R;
}


Matrix<double, 4, 4>
pose2transform(double x, double y, double z, double roll, double pitch, double yaw)
{
    Matrix<double, 4, 4> T;
    Matrix<double, 3, 3> R = euler2rotation(roll, pitch, yaw);

    T << R(0, 0), R(0, 1), R(0, 2), x, 
        R(1, 0), R(1, 1), R(1, 2), y, 
        R(2, 0), R(2, 1), R(2, 2), z, 
        0, 0, 0, 1;

    return T;
}


void
ambiguity_example()
{
    Matrix<double, 3, 3> R;
    double roll, pitch, yaw;

    // the inverse of this rotation leads to an ambiguity.    
    roll = 0.050175; 
    pitch = 0.003312; 
    yaw = -0.9314506732051; 

    R = euler2rotation(roll, pitch, yaw);
    R = R.inverse() * R;    
    Matrix<double, 3, 1> ypr = R.eulerAngles(2, 1, 0);

	printf("rx: %lf ry: %lf rz: %lf\n", roll, pitch, yaw);
	printf("roll: %lf pitch: %lf yaw: %lf\n", ypr(2, 0), ypr(1, 0), ypr(0, 0));
}


void 
euler2rotation_example()
{
    Matrix<double, 3, 3> R, R1, R2;
    Matrix<double, 3, 1> axis;

    // ********************************************************************************************
    // A matriz de rotacao, eh construida fazendo em sequencia uma rotacao ao redor do eixo x (roll), 
    // depois uma rotacao ao redor do eixo y (pitch), depois uma rotacao ao redor 
    // do eixo z (yaw). IMPORTANTE: essas rotacoes sao definidas com relacao ao sistema 
    // de referencia original, antes de qualquer rotacao (pelo menos do jeito que a Eigen representa elas).
    // Ex.: Considere o sistema x: frente, y: esquerda, z: cima. A rotacao abaixo fara' primeiro roll
    // para esquerda, fazendo o sistema virar x: frente, y: baixo, z: esquerda. Depois fara' um yaw para
    // direita (o yaw eh feito em relacao ao sistema original em z: cima), fazendo o sistema virar
    // x: direita, y: baixo, z: frente.
    // ********************************************************************************************
    cout << "--------------------------------------------------------" << endl;
    cout << "Analise da rotacao direta" << endl;
    R = euler2rotation(-M_PI / 2., 0, -M_PI / 2);

    // saida esperada: 0, -1, 0
    axis << 1, 0, 0;
    cout << axis << " -> " << R * axis << endl << endl;

    // saida esperada: 0, 0, -1
    axis << 0, 1, 0;
    cout << axis << " -> " << R * axis << endl << endl;

    // saida esperada: 1, 0, 0
    axis << 0, 0, 1;
    cout << axis << " -> " << R * axis << endl << endl << endl;

    // ******************************************
    // Quando aplicamos a inversa, transformamos x: direita, y: baixo, z: frente no sistema original. 
    // Note, contudo que escrevemos o sistema rotacionado como se estivéssemos observando ele do sistema
    // original. Isto eh, ao inves de lermos x: direita, y: baixo, z: frente virou x: frente, y: esquerda, z: cima,
    // dizemos que x: frente, y: esquerda, z: cima virou x: cima, y: tras, z: direita. 
    // A transformada inversa resulta no mesmo que aplicar ao sistema original, a sequencia de rotacoes
    // +pi/2 yaw(z), +pi/2 roll(x), como esperado.
    // ******************************************
    cout << "--------------------------------------------------------" << endl;
    cout << "Analise da rotacao inversa:" << endl << endl;

    // saida esperada: 0, 0, 1
    axis << 1, 0, 0;
    cout << axis << " -> " << R.inverse() * axis << endl << endl;

    // saida esperada: -1, 0, 0
    axis << 0, 1, 0;
    cout << axis << " -> " << R.inverse() * axis << endl << endl;

    // saida esperada: 0, -1, 0
    axis << 0, 0, 1;
    cout << axis << " -> " << R.inverse() * axis << endl << endl << endl;


    // ******************************************
    // Questao: o resultado de aplicarmos duas rotacoes em sequencia eh o mesmo de aplicar 
    // uma matriz de rotacao que contem ambas? Conclusao: Sim, o resultado eh o mesmo.
    // Isso eh curioso pq o yaw da 2a rotacao eh ao redor do z no sistema de coordenadas
    // original, nao ao redor do novo z apos a aplicacao de R1.
    // ******************************************
    cout << "--------------------------------------------------------" << endl;
    cout << "Analise de sequencias de rotacoes vs. matriz de rotacao:" << endl << endl;
    R1 = euler2rotation(-M_PI / 2., 0., 0.); // roll
    R2 = euler2rotation(0., 0., -M_PI / 2.); // yaw

    // saida esperada: 0, -1, 0
    axis << 1, 0, 0;
    cout << axis << " -> " << R2 * (R1 * axis) << endl << endl;

    // saida esperada: 0, 0, -1
    axis << 0, 1, 0;
    cout << axis << " -> " << R2 * (R1 * axis) << endl << endl;

    // saida esperada: 1, 0, 0
    axis << 0, 0, 1;
    cout << axis << " -> " << R2 * (R1 * axis) << endl << endl;   

    // ******************************************
    // Importante: Observe que ao aplicar uma rotacao a um vetor (ou ponto) NAO obtemos
    // as coordenadas dele no sistema rotacionado. O que obtemos sao as coordenadas do 
    // vetor rotacionado NO SISTEMA ORIGINAL. Questao: como obter as coordenadas no sistema
    // rotacionado? Resposta: a inversa da rotacao retorna as coordenadas do vetor no 
    // sistema rotacionado (veja o exemplo de rotacao inversa com esse olhar).
    // ******************************************


    // ******************************************
    // Questao: Uma rotacao com o negativo de roll, pitch, e yaw equivale a rotacao inversa? Eu
    // acho que nao pq a ordem das operacoes tb deveria ser invertida. Conclusao: De fato, aplicar
    // a inversa de uma rotacao nao equivale a aplicar uma rotacao com o negativo de roll, pitch, e yaw.
    // ******************************************
    cout << "--------------------------------------------------------" << endl;
    cout << "Analise da rotacao obtida negativando os angulos:" << endl << endl;
    R = euler2rotation(M_PI / 2., 0, M_PI / 2);

    // saida esperada: 0, 0, 1
    axis << 1, 0, 0;
    cout << axis << " -> " << R * axis << endl << endl;

    // saida esperada: -1, 0, 0
    axis << 0, 1, 0;
    cout << axis << " -> " << R * axis << endl << endl;

    // saida esperada: 0, -1, 0
    axis << 0, 0, 1;
    cout << axis << " -> " << R * axis << endl << endl << endl;
}


void
transformations_example()
{
    Matrix<double, 4, 1> point;
    Matrix<double, 4, 4> T;

    // ******************************************
    // Questao: Ao transformar um ponto, primeiro rotacionamos, depois transladamos. E se fizermos a inversa, 
    // primeiro transladamos, e depois rotacionamos? Conclusao: Sim. Quando aplicamos a transformada direta, fazemos
    //  a seguinte sequencia de operações: roll, pitch, yaw, t. Quando aplicamos a transformacao inversa, fazemos
    // a seguinte sequencia de operacoes: -t, -yaw, -pitch, -roll. Sejam T a transformacao criada usando a pose de 
    // sistema de coordenadas B com relacao a um sistema A, P as coordenadas de um ponto em A, e Q as coordenadas
    // do mesmo ponto em B. Entao: Q = T.inverse() * P, e P = T * Q.
    // ******************************************
    cout << "--------------------------------------------------------" << endl;
    cout << "Transformacoes:" << endl << endl;

    point << 1, 0, 0, 1;
    T = pose2transform(2, 0, 0, M_PI / 2, 0, M_PI / 2);

    // resultado esperado: [2, 1, 0]
    cout << "Transformacao de um ponto:" << endl;
    cout << "T:" << endl << T << endl << endl;
    cout << "point:" << endl << point << endl << endl;
    cout << "T * point:" << endl << T * point << endl << endl;

    // resultado esperado: [0, 0, -1]
    cout << "Obtencao das coordenadas de um ponto em um sistema de destino:" << endl;
    cout << "T.inverse()" << endl << T.inverse() << endl << endl;
    cout << "point:" << endl << point << endl << endl;
    cout << "T.inverse() * point:" << endl << T.inverse() * point << endl << endl << endl;

    // resultado esperado: [1, 0, 0] 
    point << 0, 0, -1, 1;
    cout << "Obtencao das coordenadas de um ponto no sistema de origem:" << endl;
    cout << "T" << endl << T << endl << endl;
    cout << "point:" << endl << point << endl << endl;
    cout << "T * point:" << endl << T * point << endl << endl << endl;

    // ******************************************
    // Questao: Pose do velodyne com respeito a camera.
    // ******************************************
    cout << "--------------------------------------------------------" << endl;
    cout << "Pose do velodyne com respeito a camera:" << endl << endl;

    T = pose2transform(0.115, -0.27, -0.1, M_PI/2, -M_PI/2, 0);

    // resultado esperado: [2.1, 0.115, -0.27]
    point << 0, 0, 2, 1;
    cout << "projection from cam to velodyne" << endl;
    cout << "point:" << endl << point << endl << endl;
    cout << "T.inverse() * point:" << endl << T.inverse() * point << endl << endl << endl;

    // resultado esperado: [0, 0, 2]
    point << 2.1, 0.115, -0.27, 1; 
    cout << "projection from velodyne to cam" << endl;
    cout << "point:" << endl << point << endl << endl;
    cout << "T * point:" << endl << T * point << endl << endl << endl;
}


int 
main()
{
    //ambiguity_example();
    //euler2rotation_example();
    transformations_example();

    return 0;    
}
