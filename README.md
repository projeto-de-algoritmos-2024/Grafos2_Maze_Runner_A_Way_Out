# Maze Runner: a Way Out

**Número da Lista**:29<br>
**Conteúdo da Disciplina**: Grafos 2<br>

## Alunos
|Matrícula | Aluno |
| -- | -- |
| 22/1008786  |  Mateus Villela Consorte |
| 22/1008679 |  Pablo Serra Carvalho |

## Sobre 
A palavra “labirinto” tem origem no latim “labyrinthus”, embora sua raiz mais antiga venha do grego. Essa palavra descreve um espaço artificial com várias passagens e caminhos, projetado para confundir as pessoas e dificultar a busca pela saída. Os labirintos existem há cerca de 4.000 anos e estão presentes em muitas religiões e culturas. Um dos mais famosos é o Labirinto de Creta, construído pelo artesão Dédalo a pedido do rei Minos para aprisionar o Minotauro. Essa lenda grega representa o labirinto como um lugar de confusão e perigo, onde é fácil se perder e difícil encontrar uma saída. Assim, emerge uma intrigantes questões: como poderia achar o caminho de qualquer labirinto solucionável? como garantir que ele seja o melhor caminho, se possível? E mais, como poderia ser gerado um labirinto e garantir que seja solucuonável?

Nesse contexto, surge os algoritimos para grafos ponderados, dentre eles, dois que serão aplicados na execução desse projeto: Algoritimo de Dijkstra e Algoritimo de Prim.

### Algoritimo de Dijkistra
#### Premissas
  * O grafo deve ser direcionado e ponderado.
  * As arestas devem ter pesos não negativos.
  * É necessário um nó inicial a partir do qual as distâncias serão calculadas.
  * Definição Lógica: encontra o menor caminho entre nós em um digrafo.
#### Pseudo-código
```
Dijkstra(G, s)
  para cada vértice v em G:
    dist[v] := infinito
    anterior[v] := indefinido
  dist[s] := 0
  Q := conjunto de todos os vértices em G
  
  enquanto Q não estiver vazio:
    u := vértice em Q com dist[u] mínimo
    remover u de Q
    
    para cada vizinho v de u:
      alt := dist[u] + peso(u, v)
      se alt < dist[v]:
        dist[v] := alt
        anterior[v] := u
        
  retornar dist[], anterior[]

```
#### Análise Assintótica
  * Tempo: Caso use file simples, o tempo é $O(V^2)$. Com heap binário, o tempo é $O((V + E)log V)$ e, com heap de Fibonacci,
o processo é amortizado para $O(Vlog V + E)$.
  * Espaço: para armazenar a distâncias e os predecessores, basta **V** slots, logo: $O(V)$.
#### Objetivo dentro do Projeto
  * Encontrar a saida do labirinto no menor caminho possível.

### Alogritimo de Prim
#### Premissas
  * O grafo deve ser conectado e ponderado.
  * Deve ser um grafo não direcional
  * As arestas podem eventualmente ter pesos negativos
  * Definição Lógica: O algoritmo de Prim encontra a árvore geradora mínima de um grafo, ou seja, a subárvore que conecta todos os nós com o menor custo total possível.
#### Pseudo-código
```
Prim(G, s)
  para cada vértice u em G:
    chave[u] := infinito
    anterior[u] := indefinido
  chave[s] := 0
  Q := conjunto de todos os vértices em G
  
  enquanto Q não estiver vazio:
    u := vértice em Q com chave[u] mínima
    remover u de Q
    
    para cada vizinho v de u:
      se v está em Q e peso(u, v) < chave[v]:
        anterior[v] := u
        chave[v] := peso(u, v)
        
  retornar anterior[]

```
#### Análise Assintótica
  * Tempo: Semelhante ao Dijkstra, o Prim também depende do tipo de estrutura será utilizado para armazenar a informação.
Caso seja utilizado fila simples, o tempo é $O(V^2)$;  com heap binário, $O((V + E) log V)$ e, com heap Fibonacci, $O(E + VlogV)$.
  * Espaço: o espaço necessário também é $O(V)$.
#### Objetivo dentro do Projeto
  * Assim como na mitologia graga, o objetivo desse algoritimo é fazer o que Dédalo fez, construir o labirinto.

## Screenshots

## Versão 1 
https://github.com/user-attachments/assets/d0574e0b-6606-4869-8db5-0d9b1778a1f3

## Versão 2
https://github.com/user-attachments/assets/8f90a6f5-de7f-46da-986b-4d71e719421f

## Versão 1 e versão 2
https://github.com/user-attachments/assets/4c2be9cf-b9c1-46aa-a434-93fb244e0f23



## Instalação 
**Linguagem**: Python<br>
**Framework**: (caso exista)<br>
Descreva os pré-requisitos para rodar o seu projeto e os comandos necessários.

## Uso 
Explique como usar seu projeto caso haja algum passo a passo após o comando de execução.


## Outros 
Quaisquer outras informações sobre seu projeto podem ser descritas abaixo.




