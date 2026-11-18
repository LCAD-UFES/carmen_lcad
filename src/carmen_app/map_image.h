/* Decodificacao dos mapas do protocolo (grade zlib, 1 byte por celula) em imagem. */
#ifndef CARMEN_APP_MAP_IMAGE_H
#define CARMEN_APP_MAP_IMAGE_H

#include <QByteArray>
#include <QColor>
#include <QImage>
#include <QJsonObject>
#include <QRectF>

/* Tabela de cores do tema escuro: 0 = livre .. 254 = ocupado, 255 = desconhecido. */
QVector<QRgb> map_color_table();

/* Grade -> imagem ARGB (coluna = x, linha = y; a linha 0 e' y_origin) e retangulo em metros. */
bool decode_map(const QJsonObject &meta, const QByteArray &blob, QImage *out, QRectF *rect);

/* Igual ao decode_map, mas para desenhar POR CIMA do mapa offline: so' as celulas ocupadas aparecem,
   na cor dada (opacidade cresce com a probabilidade); livre e desconhecido ficam transparentes.
   E' o que deixa ver se o mapa ao vivo (mapper) encaixou no offline. */
bool decode_map_overlay(const QJsonObject &meta, const QByteArray &blob, const QColor &color, QImage *out, QRectF *rect);

#endif
