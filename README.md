# Desafio de Navegação e Mapeamento Autônomo - ROS 2
Este repositório contém a solução para o desafio de navegação autônoma utilizando ROS 2 e C++. O projeto foi desenvolvido para controlar um robô em um labirinto simulado (pacote cg), dividido em duas etapas principais: navegação com mapa conhecido e exploração de mapa desconhecido.


## 📖 Sobre o Projeto
O objetivo deste projeto é aplicar algoritmos de busca em grafos para resolver problemas de robótica móvel.

- **Parte 1 (Navegação):** O robô recebe o mapa completo do labirinto via serviço ROS. O algoritmo deve processar esse mapa (transformando de Array 1D para Matriz/Grafo) e encontrar o caminho mais curto até o alvo.

- **Parte 2 (Mapeamento):** O robô começa sem conhecimento do mapa. Ele deve utilizar seus sensores locais para explorar o ambiente, memorizar o mapa e, ao final, traçar a rota otimizada do início ao fim.

## 🎥 Demonstração e Explicação
Confira o vídeo de demonstração e explicação dos algoritmos abaixo:

[CLIQUE AQUI PARA ASSISTIR AO VÍDEO NO YOUTUBE]()

### 📂 Estrutura do Projeto

```txt
culling_games/
├── src/
│   ├── cg/                      # Pacote principal
│   │   ├── maps/                # Labirintos
│   │   └── cg/                  # Nó ROS do jogo
│   ├── cg_interfaces/           # Mensagens e serviços customizados
│   ├── cg_teleop/               # Teleoperação
│   ├── ponderada/               # PONDERADA 1: BFS
│   │   └── src/                 # Implementação (.cpp)
│   │       └── bfs_main.cpp
│   └── ponderada2/              # PONDERADA 2: DFS + BFS
│       ├── package.xml          # arquivo xml para configurações
│       ├── CMakeLists.txt       # Cmake para gerar executavel
│       └── src/                 # Implementação (.cpp)
│           └── mapping_node.cpp 
├── build/                       # Arquivos de build 
├── install/                     # Executáveis instalados
├── log/                         # Logs de compilação
├── generate_maze.py             # Gerador de labirintos
├── .gitignore                   # Arquivo para ignorar outros
├── LICENSE                      # Licença
├── lobotomy_kaisen              # Assets
└── README.md                    # Documentação 
```
O workspace está organizado nos seguintes pacotes:

- **cg:** Pacote do simulador (fornecido pelo professor).

- **ponderada1:** Contém a solução da Parte 1 (Algoritmo BFS).

- **ponderada2:** Contém a solução da Parte 2 (Mapeamento DFS + BFS).

## 🛠 Pré-requisitos
- ROS 2 (Versão Rolling, Iron ou Humble) instalado.

- Colcon para buildar os pacotes.

- C++ .

## 🚀 Instalação e Compilação
Clone este repositório no seu ambiente ROS:

```Bash
git clone https://github.com/JvWandermurem/Navega--o_mapeamento_ros2.git
```

Instale as dependências e compile os pacotes:

```Bash
colcon build
```
Atualize o ambiente (Source):

```bash
source install/setup.bash ou zsh
```

## 🕹 Como Executar
Para rodar o projeto, você precisará de dois terminais.

### Parte 1: Navegação (BFS)
Nesta etapa, o robô recebe o mapa e vai direto ao alvo.

Terminal 1 (Simulador):

```Bash
ros2 run cg maze
```
Terminal 2 (Algoritmo):

```Bash
ros2 run ponderada1 bfs_node
```

O robô irá calcular a rota e se mover automaticamente até o alvo.

### Parte 2: Mapeamento (DFS + Backtracking)
Nesta etapa, o robô explora o desconhecido.

Terminal 1 (Simulador):
```Bash
ros2 run cg maze
```
(Você pode usar ros2 run cg maze -- --generate para gerar um labirinto aleatório novo).

Terminal 2 (Algoritmo):

```Bash
ros2 run ponderada2 mapping_node
```

O robô iniciará a exploração. Após mapear tudo e voltar à base, ele fará uma "corrida final" até a saída usando o mapa descoberto.

## 🧠 Explicação dos Algoritmos
### Parte 1: Breadth-First Search (BFS)
Para encontrar o caminho mais curto em um grid não ponderado (onde cada passo tem custo 1), utilizamos o algoritmo BFS (Busca em Largura).

- O nó solicita o mapa completo via serviço /get_map.

- O vetor 1D recebido é convertido para coordenadas 2D (Row-Major Order).

- Utiliza-se uma Fila para explorar os vizinhos em camadas a partir do robô.

- Uma matriz parent guarda de onde viemos, permitindo reconstruir o caminho reverso (Backtracking) do Alvo até o Início.

- O robô executa a lista de movimentos enviando comandos para /move_command.

### Parte 2: Depth-First Search (DFS) Online
Como o robô não conhece o mapa, utilizamos uma abordagem de Exploração com Backtracking.

- Sensores: O robô lê o tópico /robot_sensors para ver as células adjacentes.

- Memória: Utilizamos um std::set para armazenar células visitadas e outro para memorizar paredes/caminhos livres (Mapeamento).

Decisão:

- Se houver um caminho livre não visitado, o robô avança e empilha o movimento na Pilha (std::stack).

- Se encontrar um beco sem saída, ele desempilha o último movimento e volta (Backtracking).

- Otimização de Tempo: O robô possui velocidade variável. Na exploração (ida), ele espera 100ms para garantir leitura do sensor. No retorno (volta), ele reduz o delay para 15ms, agilizando o processo.

- Prova de Mapeamento: Ao terminar a exploração (pilha vazia), o robô usa o mapa memorizado para rodar um BFS interno e ir direto para a saída, provando que o mapa foi construído corretamente.