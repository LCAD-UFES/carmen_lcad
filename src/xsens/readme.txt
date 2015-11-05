O xsens possui 3 sensores: magnetometro, aceleromentro e giroscopio (velocidades).
Um filtro de Kalman combina estes sensores e gera uma saida de orientacao do xsens.

Nas mensagens geradas pelo modulo xsens, o valor das variaveis:
    carmen_xsens_axis m_acc;
    carmen_xsens_axis m_gyr;
    carmen_xsens_axis m_mag;
sao iguais aas saidas calibradas dos 3 sensores (calibradas pela fabrica e,
possivelmente, pela ferramenta Windows disponibilizada pela Xsens).

A saida do filtro de Kalman ee afetada pelos 3 tipos de reset de orientacao
disponiveis na ferramenta Windows da Xsens. Contudo, a saida do magnetometro nao.
Ainda nao foi possivel descobrir se as saidas do acelerometro e giroscopio 
sao afetadas.

O magnetometro ee afetado pela ferramenta de calibracao de interferencia 
magnetica disponivel na ferramenta Windows disponibilizada pela Xsens, mas, 
aparentemente, nao muito na sua orientacao para o norte. Ele nao ee afetado 
pela calibracao de declination 
(ver http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp).

A saida do filtro de Kalman oferece uma boa alternativa para medidas diferenciais
do angulo de orientacao do xsens (d_angulo/d_t).

Em resumo, aparentemente as saidas 
    carmen_xsens_axis m_acc;
    carmen_xsens_axis m_gyr;
    carmen_xsens_axis m_mag;
sao como que brutas. Apenas a saida do filtro de Kalman parace ser afetada
pelos ajustes da ferramenta Windows provida pela Xsens.
