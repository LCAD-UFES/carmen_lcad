#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "fh.hpp"

FH* FH_MAKE_HEAP(void) {
    FH* heap = (FH*) malloc(sizeof(FH));

    if (!heap) {
        perror("malloc");
        exit(EXIT_FAILURE);
    }
    heap->MIN = NULL;
    heap->N = 0;

    return(heap);
}

FH_NODE* FH_MAKE_NODE(int chave) {
    FH_NODE* node = (FH_NODE*) malloc(sizeof(FH_NODE));

    if (!node) {
        perror("malloc");
        exit(EXIT_FAILURE);
    }
    node->PAI = NULL;
    node->DIREITO = NULL;
    node->ESQUERDO = NULL;
    node->FILHO = NULL;
    node->GRAU = 0;
    node->MARCA = false;
    node->PTR = NULL;
    node->CHAVE = chave;

    return(node);
}

FH_NODE* FH_MINIMUM(FH* heap) {
    return(heap->MIN);
}

void FH_INSERT(FH* heap, FH_NODE* x) {
    x->GRAU = 0;
    x->PAI = NULL;
    x->FILHO = NULL;
    x->ESQUERDO = x;
    x->DIREITO = x;
    x->MARCA = false;
    x->PTR = NULL;
    FH_ADD_ROOT(heap, x);
    heap->N++;

}

void FH_ADD_ROOT(FH* heap, FH_NODE* node) {
    if (heap->MIN) {
        heap->MIN->ESQUERDO->DIREITO = node;
        node->ESQUERDO = heap->MIN->ESQUERDO;
        heap->MIN->ESQUERDO = node;
        node->DIREITO = heap->MIN;
    }
    if (!heap->MIN || (node->CHAVE < heap->MIN->CHAVE)) {
        heap->MIN = node;
    }
    node->PAI = NULL;

}

FH_NODE* FH_EXTRACT_MIN(FH* heap) {
    if (!heap->MIN) return(NULL);
    FH_NODE* min = heap->MIN;
    FH_NODE* aux = min->FILHO;
    while (aux && (aux->PAI)) {
        aux->PAI = NULL;
        aux = aux->DIREITO;
    }
    if (aux) {
        min->ESQUERDO->DIREITO = aux->DIREITO;
        aux->DIREITO->ESQUERDO = min->ESQUERDO;
        min->ESQUERDO = aux;
        aux->DIREITO = min;
    }
    min->ESQUERDO->DIREITO = min->DIREITO;
    min->DIREITO->ESQUERDO = min->ESQUERDO;

    if (min == min->DIREITO) {
        heap->MIN = NULL;
    } else {
        heap->MIN = min->DIREITO;
        FH_CONSOLIDATE(heap);
    }
    heap->N--;
    return(min);
}

void FH_CONSOLIDATE(FH* heap) {
    int D = FH_MAX_DEGREE(heap);
    int i; //o bom e velho contador
    FH_NODE** A = (FH_NODE**) malloc(D * sizeof(FH_NODE*));
    if (!A) {
        perror("malloc");
        exit(EXIT_FAILURE);
    }
    for (i = 0; i < D; i++) {
        A[i] = NULL;
    }
    FH_NODE* w = NULL; //para caminhar na lista de raizes
    FH_NODE* x = NULL; //para ser inserido na estrutura auxiliar
    FH_NODE* y = NULL; //para armazenar o arvore que estava na estrutura auxiliar
    
    int d = 0; //indice de A (que e' o grau da arvore)
    while ((w = heap->MIN)) {
        x = w;
        FH_REMOVE_NODE(heap, w);
        d = x->GRAU;
        while (A[d]) {
            y = A[d];
            if (x->CHAVE > y->CHAVE) {
                FH_SWAP_NODE(&x, &y);
            }
            FH_LINK(y, x);
            A[d] = NULL;
            d++;
        }
        A[d] = x;
    }
    heap->MIN = NULL;
    for (i = 0; i < D; i++) {
        if (A[i]) {
            FH_ADD_ROOT(heap, A[i]);
        }
    }
    free(A);

}

int FH_MAX_DEGREE(FH* heap) {
    return(ceil(1.44 * log(heap->N) / log(2.0) + 1.0));
}

void FH_SWAP_NODE(FH_NODE** a, FH_NODE** b) {
    FH_NODE* c = *a;
    *a = *b;
    *b = c;
}

void FH_DECREASE_KEY(FH* heap, FH_NODE* x, int k) {
    if (k > x->CHAVE) return;
    x->CHAVE = k;
    FH_NODE* y = x->PAI;
    if (y && (x->CHAVE < y->CHAVE)) {
        FH_CUT(heap, x);
        FH_CASCADING_CUT(heap, y);
    }
    if (x->CHAVE < heap->MIN->CHAVE) {
        heap->MIN = x;
    }

}

void FH_CUT(FH* heap, FH_NODE* x) {
    FH_REMOVE_NODE(heap, x);
    FH_ADD_ROOT(heap, x);
    x->MARCA = false;
}

void FH_CASCADING_CUT(FH* heap, FH_NODE* y) {
    FH_NODE* z = y->PAI;
    if (z) {
        if (y->MARCA) {
            FH_CUT(heap, y);
            FH_CASCADING_CUT(heap, z);
        } else {
            y->MARCA = true;
        }
    }
}

void FH_REMOVE_NODE(FH* heap, FH_NODE* node) {
    FH_NODE* subs = (node->ESQUERDO == node?NULL:node->ESQUERDO);
    if (subs) {
        node->ESQUERDO->DIREITO = node->DIREITO;
        node->DIREITO->ESQUERDO = node->ESQUERDO;
    }
    if (node->PAI && (node->PAI->FILHO == node)) {
        node->PAI->FILHO = subs;
        node->PAI->GRAU--;
    }
    if (node == heap->MIN) {
        heap->MIN = subs;
    }
    node->PAI = NULL;
    node->DIREITO = node;
    node->ESQUERDO = node;
}

void FH_DELETE(FH* heap, FH_NODE* x) {
    FH_DECREASE_KEY(heap, x, MENOS_INFINITO);
    FH_EXTRACT_MIN(heap);
}



void FH_LINK(FH_NODE* filho, FH_NODE* pai) {
    filho->ESQUERDO->DIREITO = filho->DIREITO;
    filho->DIREITO->ESQUERDO = filho->ESQUERDO;
    filho->PAI = pai;
    if (pai->FILHO) {
        filho->DIREITO = pai->FILHO->DIREITO;
        pai->FILHO->DIREITO->ESQUERDO = filho;
        pai->FILHO->DIREITO = filho;
        filho->ESQUERDO = pai->FILHO;
    } else {
        filho->DIREITO = filho;
        filho->ESQUERDO = filho;
        pai->FILHO = filho;
    }
    filho->MARCA = false;
    pai->GRAU++;
}

void FH_FREE_HEAP(FH* heap, bool LiberaFilhos) {
    if (LiberaFilhos) {
        FH_NODE* N = heap->MIN;
        FH_NODE* Livre = NULL;
        FH_NODE* Inicio = N;
        while (N) {
            Livre = N;
            N = N->DIREITO;
            FH_FREE_NODE(Livre, LiberaFilhos);
            if (N == Inicio) break;
        }
    }

    free(heap);

}

void FH_FREE_NODE(FH_NODE* node, bool LiberaFilhos) {
    if (LiberaFilhos) {
        FH_NODE* N = node->FILHO;
        FH_NODE* Livre = NULL;
        FH_NODE* Inicio = N;
        while (N) {
            Livre = N;
            N = N->DIREITO;
            FH_FREE_NODE(Livre, LiberaFilhos);
            if (N == Inicio) break;
        }
    }

    free(node);
    
}



void FH_PRINT_HEAP(FH* heap) {
    printf("\n\nINICIO HEAP\n");
    printf("N=%d, MIN=%d",heap->N,(heap->MIN?heap->MIN->CHAVE:-1));
    FH_NODE* N = heap->MIN;
    while (N) {
        FH_PRINT_NODE(N,0);
        N = N->DIREITO;
        if (N == heap->MIN) {
            break;
        }
    }
    printf("\nFIM HEAP\n");
}

void FH_PRINT_NODE(FH_NODE* node, int level) {
    if (!node->PAI) printf("\n");

    int i = 0;
    for(i = 0; i < level; i++) printf("\\");

    printf("%d(%d)", node->CHAVE, node->GRAU);
    FH_NODE* N = node->FILHO;
    while (N) {
        FH_PRINT_NODE(N, level + 1);
        N = N->DIREITO;
        if (N == N->PAI->FILHO) {
            break;
        }
    }
}

void FH_PRINT_A(FH_NODE** A, int D) {
    int i = 0;
    for (i = 0; i < D; i++) {
        printf("\nA[%d]: ",i);
        if (!A[i]) {
            printf(" NULL");
        } else {
            FH_PRINT_NODE(A[i],0);
        }
    }
}
