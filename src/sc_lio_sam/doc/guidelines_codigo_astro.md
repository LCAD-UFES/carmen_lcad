# Diretrizes de escrita de código para Astro

## Objetivo e referências

Escreva para quem precisará entender, modificar e manter o módulo depois.
Clareza e consistência com o projeto têm prioridade sobre otimizações prematuras.

Este resumo combina as regras de `doc/program_with_Astro.md`, a organização de
`doc/template_for_astro_or_carmen.cpp` e as decisões adotadas neste chat. As
recomendações de arquitetura e integração abaixo são convenções deste trabalho,
não regras universais do ROS. No CARMEN, use os nomes e APIs equivalentes com o
prefixo `carmen`.

## Organização do módulo

- Parta do template e consulte módulos existentes; não invente uma arquitetura
  diferente sem necessidade.
- Prefira código procedural, funções pequenas e responsabilidades explícitas.
  Use classes quando a API ou a organização do estado justificar.
- Organize o arquivo nesta ordem: includes; constantes e estado do módulo;
  funções utilitárias; publishers; handlers; inicializações; `main`.
- Separe conversão de mensagens, publicação e processamento quando forem
  responsabilidades distintas. Não reúna conversores independentes em um
  handler grande.
- Mantenha o `main` curto: inicializar a comunicação, registrar encerramento,
  verificar a versão dos parâmetros, ler configuração, definir mensagens,
  criar publishers/subscriptions e iniciar o processamento de eventos.
- Centralize a leitura da configuração em uma única `read_parameters`, com
  partes identificáveis para Astro e ROS quando ambos forem utilizados.

## Nomes e formatação

- Use `lower_case_with_underscores` para funções e variáveis.
- Escolha nomes que indiquem finalidade, unidade ou sistema de coordenadas
  quando isso evitar ambiguidade. Contadores simples podem usar `i` e `j`.
- Coloque chaves de abertura e fechamento em linhas próprias.
- Use indentação consistente com o arquivo; a referência adota quatro espaços
  como configuração de indentação nos editores.
- Use espaços ao redor dos operadores e após vírgulas. Não separe o nome de uma
  função de seus parênteses: `publish_message(message)`.
- Quebre linhas longas em pontos lógicos. Evite várias operações independentes
  na mesma linha e expressões difíceis de acompanhar.
- Siga o template para definições de funções, com o tipo de retorno na linha
  anterior ao nome.

```cpp
void
publish_imu_message(const ImuMessage &message)
{
    imu_publisher->publish(message);
}
```

O exemplo ilustra apenas a formatação; os tipos e APIs devem ser os do módulo.

## Variáveis, unidades e geometria

- Declare e inicialize variáveis perto do primeiro uso, no menor escopo possível.
- Evite variáveis auxiliares desnecessárias e estado global sem necessidade.
  O estado compartilhado pelos handlers deve ter finalidade e dono claros.
- Prefira `double` para cálculos reais e `int` para inteiros comuns, conforme o
  guia. Respeite tipos exigidos por interfaces, tamanhos e formatos binários;
  não altere o layout de uma mensagem para impor essa preferência.
- Use unidades MKS: metros, segundos e radianos. Documente conversões na
  fronteira com sensores ou mensagens que usem outras unidades.
- Use sistemas de coordenadas de mão direita e deixe explícitos os frames.
- Use as funções Astro para conversão entre graus/radianos e normalização de
  ângulos: `astro_degrees_to_radians`, `astro_radians_to_degrees` e
  `astro_normalize_theta`.
- Para recuperar uma direção a partir de componentes, use `atan2(y, x)`.
- Não espalhe números mágicos. Use constantes nomeadas ou parâmetros, conforme
  o significado do valor; não escreva aproximações manuais de pi.

## Comentários e documentação

- Mantenha comentários curtos, em inglês, preferencialmente com `//`.
- Coloque o comentário antes do componente ao qual ele se refere.
- Explique propósito, restrições, unidades, ownership e decisões não óbvias;
  evite narrar operações já claras pelo código.
- Atualize os comentários quando o comportamento mudar.
- Para interfaces públicas ou funções com contrato complexo, documente entradas,
  retorno, pré-condições e efeitos relevantes.

O guia original pede documentação detalhada com `@brief`, `@param` e `@return`,
além de comentários para globais. Neste trabalho, a solicitação explícita de
comentários menores orientou uma documentação proporcional à complexidade,
sem repetir blocos extensos em funções triviais.

## Parâmetros e tratamento de erros

- Leia e valide a configuração na inicialização. Não repita a mesma consulta
  em cada callback quando os parâmetros forem estáticos.
- Parâmetros obrigatórios ausentes ou inválidos devem produzir um erro claro,
  com o nome do parâmetro e a condição esperada.
- Valide dimensões de vetores, valores finitos e intervalos antes de usá-los.
- Use defaults somente quando fizerem parte do comportamento definido. Não
  esconda a falta de calibração obrigatória com valores arbitrários.
- Quando houver uma exceção autorizada, torne-a explícita. Exemplo deste
  trabalho: ângulos horizontais do lidar ausentes podem assumir zero, com warning.
- Diferencie erros que impedem a inicialização de dados inválidos ocasionais,
  que podem ser descartados com diagnóstico adequado.
- Evite logs repetitivos em callbacks de alta frequência; use avisos únicos ou
  limitação de frequência quando apropriado.
- Deixe infraestrutura compartilhada seguir suas convenções: não sobrescreva
  `CENTRALHOST` no módulo sem uma necessidade explícita.

## Eventos, callbacks e concorrência

- Prefira processamento em uma thread. Não acrescente threads, mutexes ou
  executores concorrentes sem uma necessidade demonstrada.
- Uma assinatura ROS não implica, por si só, execução paralela: a concorrência
  depende do executor, spinner e configuração de callbacks.
- Em ROS, use processamento de eventos quando houver callbacks, timers ou
  serviços que precisem ser executados. Publicar mensagens, isoladamente, não
  exige um `spin` dedicado.
- Em módulos IPC, use o dispatch apropriado. Quando IPC e ROS coexistirem,
  integre os dois mecanismos sem bloquear indefinidamente o outro.
- Preserve workers necessários de algoritmos existentes, como os de mapping,
  e explicite acesso ao estado compartilhado e ordem de encerramento.
- Encerre de forma ordenada, liberando recursos e permitindo concluir gravações.

## Mensagens e interfaces

- Use as bibliotecas de interface do Astro para publicar e assinar mensagens.
- Ao criar uma interface IPC, separe definições de mensagens, declarações e
  implementação em `astro_<modulo>_messages.h`,
  `astro_<modulo>_interface.h` e `astro_<modulo>_interface.c`.
- Mensagens IPC novas devem conter dados, timestamp e hostname. Use as APIs do
  projeto para obter tempo e host quando a mensagem for originada no módulo.
- Em conversões, preserve o timestamp de aquisição e a semântica da origem;
  não substitua esse timestamp pelo instante de processamento.
- Respeite o ciclo de vida da memória recebida: callbacks IPC podem reutilizar
  o mesmo armazenamento. Copie dados se precisar retê-los além do callback.
- Evite estruturas intermediárias e cópias sem finalidade. Dimensione buffers
  conforme os dados e a configuração, sem limites fixos injustificados.
- Preserve campos, frames, unidades e formatos binários ao migrar interfaces.
  Configure QoS e retenção conforme a necessidade dos consumidores.

## Refatoração, migração e validação

- Separe mudanças de arquitetura de mudanças de algoritmo. Uma migração de ROS
  deve preservar o comportamento, salvo alteração solicitada explicitamente.
- Prefira nomes que descrevam a função atual do componente. Evite deixar nomes
  como “remaining” ou wrappers de transição no caminho normal de execução.
- Verifique também launches, processos, parâmetros e documentação: compilar o
  fonte não garante que o sistema esteja executando o nodo correto.
- Faça testes proporcionais à mudança: compilação, validação de configuração,
  comparação de mensagens e playback quando houver impacto no fluxo de dados.
- Use mapas temporários e ambientes isolados nos testes que gravam resultados.
- Registre o que foi validado e suas limitações. Um trecho de playback não
  equivale a um log completo, e abrir o RViz não valida visualmente todo o mapa.
