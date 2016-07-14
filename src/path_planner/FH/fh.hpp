/* 
 * File:   fh.h
 * Author: Leonardo
 *
 * Created on March 29, 2011, 9:58 PM
 */

#ifndef FH_H
#define	FH_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include <stdlib.h>
    #include <limits.h>
    #include <stdbool.h>

#ifndef INFINITO
#define INFINITO INT_MAX // + Inifinito
#endif

#ifndef MENOS_INFINITO
#define MENOS_INFINITO INT_MIN // - Infinito
#endif

    //Novos tipos para facilitar nossa vida
    typedef enum {Gauss, PivotacaoCompleta} METODO_PIVO;

    //Estrutura basica que representa cada no' em  uma heap de Fibonacci. Um no'
    //em si e' uma arvore (binomial ou nao)
    typedef struct T_FH_NODE {
        struct T_FH_NODE* PAI;
        struct T_FH_NODE* FILHO;
        struct T_FH_NODE* ESQUERDO;
        struct T_FH_NODE* DIREITO;
        int CHAVE;
        int GRAU;
        bool MARCA;
        void* PTR;
        //Ponteiro para uma estrutura qualqer, utilizado quando se usa essa heap
        //para processamento de uma outra estrutura, como grafo, por exemplo.
        //Ele aponta para o elemento gerador do no';
        void** SELFPTR;
        //Esse outro e' o seguinte: no endereco fornecido por esse ponteiro,
        //existe uma referencia para este T_FH_NODE. Quer dizer, se por algum motivo
        //essa celula mudar fisicamente de endereco na memoria, precisamos alterar
        //esse ponteiro (caso exista).
    } FH_NODE;

    //Uma heap Fibonacci
    typedef struct {
        FH_NODE* MIN;
        int N;
    } FH;

    //De acordo com: Cormen, Leiserson, Rivest & Stein
    //Operacoes essenciais para heaps intercambiaveis
    FH* FH_MAKE_HEAP(void);

    FH_NODE* FH_MAKE_NODE(int);
    FH_NODE* FH_MINIMUM(FH*);
    FH_NODE* FH_EXTRACT_MIN(FH*);

    void FH_INSERT(FH*, FH_NODE*);
    void FH_DECREASE_KEY(FH*, FH_NODE*, int);
    void FH_DELETE(FH*, FH_NODE*);
    void FH_LINK(FH_NODE*, FH_NODE*);
    void FH_SWAP_NODE(FH_NODE**, FH_NODE**);

    //Operacoes auxiliares para manutencao da heap
    int FH_MAX_DEGREE(FH*);
    void FH_CONSOLIDATE(FH*);
    void FH_ADD_ROOT(FH*, FH_NODE*);
    void FH_REMOVE_NODE(FH*, FH_NODE*);
    void FH_CUT(FH*, FH_NODE*);
    void FH_CASCADING_CUT(FH*, FH_NODE*);

    void FH_FREE_HEAP(FH*, bool);
    void FH_FREE_NODE(FH_NODE*, bool);

    void FH_PRINT_HEAP(FH*);
    void FH_PRINT_NODE(FH_NODE*, int);
    void FH_PRINT_A(FH_NODE**, int);

#ifdef	__cplusplus
}
#endif

#endif	/* FH_H */

