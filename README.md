![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![Matplotlib](https://img.shields.io/badge/Matplotlib-%23ffffff.svg?style=for-the-badge&logo=Matplotlib&logoColor=black)
# PathFinder - Resolvendo o Labirinto 2D com o Algoritmo A*

## Integrantes do Grupo
- [Renato Matos](https://github.com/RenatoMAP77)
- [Gabriel Ferreira](https://github.com/Druitti)

## Introdução

O PathFinder é um projeto desenvolvido para resolver o problema de encontrar o menor caminho em um labirinto 2D utilizando o algoritmo A*. Este projeto foi criado para simular um cenário onde um robô de resgate precisa navegar de um ponto inicial (S) até um ponto final (E) em um labirinto, evitando obstáculos e encontrando o caminho mais eficiente.

O algoritmo A* é um algoritmo de busca informada que combina as vantagens do algoritmo de Dijkstra (que garante encontrar o caminho mais curto) e a busca gulosa (que utiliza heurísticas para direcionar a busca). A combinação desses elementos permite que o A* encontre o caminho mais curto de forma eficiente, sendo amplamente utilizado em jogos, sistemas de navegação e robótica.

## Problema

O problema consiste em:
- Um labirinto representado por uma matriz 2D onde:
  - 0: Células livres (onde o robô pode se mover)
  - 1: Obstáculos (onde o robô não pode passar)
  - S: Ponto inicial (start)
  - E: Ponto final (end)
- O robô pode se mover para células adjacentes (cima, baixo, esquerda e direita)
- Cada movimento tem um custo de 1
- O objetivo é encontrar o menor caminho entre os pontos S e E

## Como o Algoritmo A* Funciona

O algoritmo A* utiliza uma função de avaliação f(n) para cada nó n do grafo, que é a soma de duas funções:

- g(n): o custo do caminho mais curto conhecido do nó inicial até o nó n
- h(n): uma função heurística que estima o custo do caminho mais curto do nó n até o nó objetivo

A função f(n) = g(n) + h(n) representa uma estimativa do custo total do caminho passando pelo nó n.

Neste projeto, utilizamos a Distância de Manhattan como heurística, calculada como:
h(n) = |x_atual - x_final| + |y_atual - y_final|

A Distância de Manhattan é particularmente apropriada para este problema porque o robô só pode se mover nas quatro direções cardinais (sem diagonais).

### O Processo do Algoritmo A*

1. Inicialize uma lista aberta (open list) com o nó inicial
2. Inicialize uma lista fechada (closed list) vazia
3. Enquanto a lista aberta não estiver vazia:
   - Encontre o nó com o menor valor f(n) na lista aberta
   - Remova este nó da lista aberta e adicione-o à lista fechada
   - Se este nó for o objetivo, reconstrua o caminho e retorne-o
   - Para cada vizinho do nó atual:
     - Se o vizinho estiver na lista fechada ou for um obstáculo, ignore-o
     - Calcule o valor g tentativo (g do nó atual + custo para mover para o vizinho)
     - Se o vizinho não estiver na lista aberta ou o valor g tentativo for menor que o atual, atualize os valores g, h e f do vizinho e defina o nó atual como seu pai
     - Adicione o vizinho à lista aberta se ainda não estiver lá
4. Se a lista aberta estiver vazia e o objetivo não foi encontrado, não há caminho possível

## Configuração e Execução

### Requisitos

- Python 3.6 ou superior
- NumPy
- Matplotlib (para visualização)

### Instalação das Dependências

```bash
pip install numpy matplotlib
```

### Execução do Projeto

1. Clone o repositório:
```bash
git clone https://github.com/seu-usuario/pathfinder.git
cd pathfinder
```

2. Execute o script principal:
```bash
python pathfinder.py
```

## Exemplos de Entrada e Saída

### Exemplo 1:

#### Entrada:
```
S 0 1 0 0
0 0 1 0 1
1 0 1 0 0
1 0 0 E 1
```

#### Saída:
```
Path found!
Coordinates: [(0, 0), (1, 0), (1, 1), (2, 1), (3, 1), (3, 2), (3, 3)]

Maze with path:
S 0 1 0 0
* * 1 0 1
1 * 1 0 0
1 * * E 1
```

### Exemplo 2 (Sem Solução):

#### Entrada:
```
S 0 1 0 0
0 0 1 0 1
1 1 1 0 0
1 0 0 E 1
```

#### Saída:
```
No path found.
```

## Funcionalidades do Projeto

1. **Leitura do Labirinto:**
   - O programa pode receber uma matriz representando o labirinto.

2. **Implementação do Algoritmo A*:**
   - Encontra o menor caminho entre os pontos S e E, usando a distância de Manhattan como heurística.

3. **Visualização:**
   - Mostra o labirinto original e o caminho encontrado, com os pontos do caminho destacados.

4. **Validação:**
   - Verifica se existe um caminho possível entre S e E e retorna uma mensagem apropriada caso não haja solução.

## Estrutura do Código

- `Node`: Classe que representa um nó no algoritmo A*
- `create_maze_from_string()`: Converte uma representação em string do labirinto para uma matriz 2D
- `manhattan_distance()`: Calcula a distância de Manhattan entre dois pontos
- `astar()`: Implementação do algoritmo A* para encontrar o menor caminho
- `visualize_maze_and_path()`: Visualiza o labirinto e o caminho usando matplotlib
- `print_maze_with_path()`: Imprime o labirinto com o caminho destacado
= `visualize_exploration_step():` Vizualiza um único passo do processo de exploração de caminhos do algorítmo A*
- `animate_exploration():` Executa o algoritmo A* com animação em tempo real, visualizando o processo de exploração dos nós.
- `run_pathfinder():` Função principal que inicializa um labirinto de exemplo e permite ao usuário escolher entre diferentes formas de execução do algoritmo A*.
- `create_custom_maze():` Permite ao usuário criar um labirinto customizado, definindo manualmente obstáculos, pesos, posições de início e fim, e escolhendo como o algoritmo A* será executado.

## Conclusão

O algoritmo A* é uma solução eficiente para o problema de encontrar caminhos em labirintos 2D. Ao combinar o custo do caminho já percorrido com uma heurística que estima a distância até o destino, o algoritmo consegue focar a busca na direção do objetivo, reduzindo significativamente o número de nós explorados em comparação com algoritmos como a busca em largura.

Este projeto demonstra a implementação do algoritmo A* em Python e sua aplicação na resolução de labirintos 2D. A visualização do caminho encontrado ajuda a compreender como o algoritmo explora o espaço de busca e encontra a solução ótima.
