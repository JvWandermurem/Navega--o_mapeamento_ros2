//Includes do CPP
#include <iostream>
#include <vector>
#include <cstdlib>
#include <memory>
#include <chrono>
#include <queue>
#include <algorithm>
#include <thread>

//Include do ROS2
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

using namespace std::chrono_literals;

typedef struct{
    int x,y;
} Point;

int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "O node foi iniciado!");

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("parte1_bfs");   

    //Cria o client para chamar o getmap
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client =
    node->create_client<cg_interfaces::srv::GetMap>("get_map");

    //esperando a conexão, se não da erro
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Servico get_map nao disponivel... tentando novamente");
    }

    //Request para pegar o mapa passando nada como parâmetro já que no SRV não pede nada
    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    
    auto future = client->async_send_request(request);

   rclcpp::spin_until_future_complete(node, future);

   // O array de resposta tá contido aqui
    auto response = future.get();
    //Array com ronô
    auto &grid = response->occupancy_grid_flattened;
    uint8_t h = response->occupancy_grid_shape[0];
    uint8_t w = response->occupancy_grid_shape[1];

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Received map: shape=%u x %u, total_cells=%zu",
                (unsigned)h, (unsigned)w, grid.size());

    //Temos o array já, agora temos que achar primeiro o robô e depois o fim do labrinto para saber-mos onde queremos chegar.
    // Precisamos montar a matriz com o array grid, procurei no youtube e descobri que tem como simular a matriz com uma coisa chamada "Row-Major Order", aí é um calculo com módulo e divisão

    int robo_x = -1, robo_y = -1;
    int fim_x = -1, fim_y = -1;

    for (int i = 0; i<(int)grid.size(); i++){
        std::string cell = grid[i];

        if(cell == "r"){
            robo_y = i/w;
            robo_x = i% w;
        RCLCPP_INFO(node->get_logger(), "Loc ROBO: Linha %d, Coluna %d", robo_y, robo_x);
        }
        else if (cell == "t") {
            fim_y = i / w;
            fim_x = i % w;
        RCLCPP_INFO(node->get_logger(), "Loc Fim: Linha %d, Coluna %d", fim_y, fim_x);

        }
    }
    //timer para saber quanto tempo o robô vai gastar pra achar o fim do lbirinto
    auto inicio_timer = std::chrono::high_resolution_clock::now();
    // Vou usar o BFS porque foi o que está no autoestudo com um exemplo do labirinto e porque achei mais fácil de implementar.
    RCLCPP_INFO(node->get_logger(), "Iniciando busca BFS.");

    // Fila para guardar os pontos que o robo visitar
    std::queue<Point> fila;
    fila.push({robo_x, robo_y}); 

    // Matriz de visitados (inicia tudo como false, até pq ngm foi visitado ainda)

    // O tamanho é h (altura) por w (largura)
    std::vector<std::vector<bool>> visitado(h, std::vector<bool>(w, false));
    visitado[robo_y][robo_x] = true; // Marca o início como visitado

    // Matriz de Pais para reconstruir o caminho
    std::vector<std::vector<Point>> pai(h, std::vector<Point>(w, {-1, -1}));


    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0};

    bool encontrou = false;

    while(!fila.empty()) {
        Point atual = fila.front(); 
        fila.pop(); 

        if (atual.x == fim_x && atual.y == fim_y) {
            encontrou = true;
            RCLCPP_INFO(node->get_logger(), "Saída do labrinto encontrada!");
            break;
        }

        for(int i = 0; i < 4; i++) {
            int nx = atual.x + dx[i]; 
            int ny = atual.y + dy[i]; 

            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                
                // 2. Verifica se NÃO é parede
                int indice_linear = ny * w + nx;
                
                // Se não for parede ("b") E não tiver sido visitado ainda
                if (grid[indice_linear] != "b" && !visitado[ny][nx]) {
                     
                     // Marca como visitado
                    visitado[ny][nx] = true;
                    // Diz quem é o pai
                    pai[ny][nx] = atual;
                    // Adiciona na fila para visitar depois
                    fila.push({nx, ny});     
                }
            }
        }
    }
    if (!encontrou) {
        RCLCPP_ERROR(node->get_logger(), "Caminho nao encontrado!");
        return 0;
    }

    // Reconstruir o caminho voltando pelos pais
    std::vector<Point> caminho;
    Point p = {fim_x, fim_y};
    
    while(p.x != robo_x || p.y != robo_y) {
        caminho.push_back(p);
        p = pai[p.y][p.x];
    }
    caminho.push_back({robo_x, robo_y}); // Adiciona o ponto inicial
    
    // Inverte o caminho para ficar Inicio -> Fim
    std::reverse(caminho.begin(), caminho.end());

    RCLCPP_INFO(node->get_logger(), "Caminho pra saída com %zu passos.", caminho.size());

// Agora que o BFS achou o caminho o robo tem que ir até lá
    // Configurar o Client de Movimento
    auto client_move = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    while (!client_move->wait_for_service(std::chrono::seconds(1))) {
         RCLCPP_WARN(node->get_logger(), "Servico move_command...");
    }

    // Executar o movimento
    for(size_t i = 1; i < caminho.size(); i++) {
        Point atual = caminho[i-1];
        Point prox = caminho[i];

        int dif_x = prox.x - atual.x;
        int dif_y = prox.y - atual.y;
        std::string direcao;

        if(dif_x == 1) direcao = "right";
        else if(dif_x == -1) direcao = "left";
        else if(dif_y == 1) direcao = "down";
        else if(dif_y == -1) direcao = "up";

        auto req_move = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req_move->direction = direcao;

        auto result_future = client_move->async_send_request(req_move);
        
        // Espera o robô terminar de andar antes do próximo passo
        if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Moveu: %s", direcao.c_str());
        }
    }
    //lib chrono pra mostrar o tempo do robô pra achar o caminho
    auto fim_timer = std::chrono::high_resolution_clock::now();
    auto duracao = std::chrono::duration_cast<std::chrono::milliseconds>(fim_timer - inicio_timer);

    RCLCPP_INFO(node->get_logger(), "JUJUTSU KAISEN REFERENCE!");
    RCLCPP_INFO(node->get_logger(), "Tempo total: %ld milissegundos", duracao.count());
    rclcpp::shutdown();
    return 0;
}