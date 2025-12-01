//Includes do CPP
#include <iostream>
#include <vector>
#include <string>
#include <stack>
#include <set>
#include <queue>
#include <map>
#include <utility> 
#include <chrono>
#include <thread>
#include <algorithm>

//Include do ROS2
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

using namespace std::chrono_literals;

// Variáveis globais e typdef para melhorar a leitura do código/ sugestão do chatgpt
cg_interfaces::msg::RobotSensors g_sensors;
bool g_nova_leitura = false;

typedef std::pair<int, int> Coord;

//callback para atualizar os valores
void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
    g_sensors = *msg;
    g_nova_leitura = true;
}

// Função de movimento do robô, se conecta com o MoveCmd.srv. Então tá enviando os parâmetros do Node, o client, o direction e um delay de 100ms para que não dê um bug com o robô ser mais rápido que o tempo de resposta.
bool move_robot(rclcpp::Node::SharedPtr node, rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client, std::string direction,int delay_ms = 100) {
    
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction;
    
    auto result_future = client->async_send_request(request);
    
    // O spin espera o serviço do ROS responder, para que não dê um bug
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        bool sucesso = result_future.get()->success;
        if (sucesso) {
            if (delay_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms)); 
            }
        }
        return sucesso;
    }
    return false;
}

std::string get_opposite(std::string dir) {
    if (dir == "up") return "down";
    if (dir == "down") return "up";
    if (dir == "left") return "right";
    if (dir == "right") return "left";
    return "";
}

//BFS com o mapa já montado, é basicamente a parte 1
void ir_para_alvo(rclcpp::Node::SharedPtr node, 
                  rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client,
                  std::set<Coord> mapa_livre, 
                  Coord alvo) {
    
    RCLCPP_INFO(node->get_logger(), "--- CALCULANDO BFS---");
    
    std::queue<Coord> fila;
    fila.push({0,0});
    
    std::set<Coord> visitados_bfs;
    visitados_bfs.insert({0,0});
    
    std::map<Coord, Coord> pais; 

    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0};
    bool achou = false;

    while(!fila.empty()) {
        Coord atual = fila.front();
        fila.pop();

        if (atual == alvo) {
            achou = true;
            break;
        }

        for(int i=0; i<4; i++) {
            Coord vizinho = {atual.first + dx[i], atual.second + dy[i]};
            
            if (mapa_livre.count(vizinho) && visitados_bfs.find(vizinho) == visitados_bfs.end()) {
                visitados_bfs.insert(vizinho);
                pais[vizinho] = atual;
                fila.push(vizinho);
            }
        }
    }

    if (!achou) {
        RCLCPP_ERROR(node->get_logger(), "ERRO: Nao existe caminho conhecido ate a saída");
        return;
    }

    std::vector<Coord> caminho;
    Coord p = alvo;
    while(p != std::make_pair(0,0)) {
        caminho.push_back(p);
        p = pais[p];
    }
    std::reverse(caminho.begin(), caminho.end());

    RCLCPP_INFO(node->get_logger(), "Rota final: %zu passos. Executando...", caminho.size());

    Coord atual = {0,0};
    for(auto prox : caminho) {
        int dif_x = prox.first - atual.first;
        int dif_y = prox.second - atual.second;
        std::string dir = "";

        if (dif_x == 1) dir = "right";
        else if (dif_x == -1) dir = "left";
        else if (dif_y == 1) dir = "down";
        else if (dif_y == -1) dir = "up";

        move_robot(node, client, dir, 50); 
        atual = prox;
    }
    RCLCPP_INFO(node->get_logger(), "JUJUTSU KAISEN REFERENCE!");
}

// Main primeiras linhas são a mesma coisa do último arquivo, iniciar, criar node, client, etc...
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mapping_node");
    
    auto client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    while (!client->wait_for_service(1s)) {
        RCLCPP_WARN(node->get_logger(), "Esperando servico de movimento...");
    }

    auto sub = node->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors", 10, sensor_callback);
    
    int x = 0, y = 0;
    std::set<Coord> visitados; 
    visitados.insert({0,0});

    std::set<Coord> mapa_livre;
    mapa_livre.insert({0,0}); 

    std::stack<std::string> historico;
    
    Coord pos_alvo = {0,0};
    bool alvo_detectado = false;

    RCLCPP_INFO(node->get_logger(), "INICIANDO MAPEAMENTO ...");

    while(rclcpp::ok()) {
        
        g_nova_leitura = false;
        while (!g_nova_leitura && rclcpp::ok()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        struct Opcao { std::string dir; std::string val; int dx; int dy; };
        std::vector<Opcao> opcoes = {
            {"up",    g_sensors.up,    0, -1},
            {"down",  g_sensors.down,  0,  1},
            {"left",  g_sensors.left, -1,  0},
            {"right", g_sensors.right, 1,  0}
        };

        // Atualizar Mapa do robô
        for(auto op : opcoes) {
            Coord vizinho = {x + op.dx, y + op.dy};
            if (op.val != "b") mapa_livre.insert(vizinho);
            if (op.val == "t") {
                pos_alvo = vizinho;
                alvo_detectado = true;
            }
        }

        //  DFS
        bool moveu = false;
        std::string direcao_escolhida = "";

        for(auto op : opcoes) {
            Coord vizinho = {x + op.dx, y + op.dy};
            
            if (op.val == "f" && visitados.find(vizinho) == visitados.end()) {
                direcao_escolhida = op.dir;
                
                if (move_robot(node, client, direcao_escolhida, 100)) {
                    x += op.dx;
                    y += op.dy;
                    
                    visitados.insert({x, y});
                    historico.push(direcao_escolhida);
                    moveu = true;
                    
                    RCLCPP_INFO(node->get_logger(), "Explorando: %s", direcao_escolhida.c_str());
                }
                break; 
            }
        }

        //  Backtracking 
        if (!moveu) {
            if (!historico.empty()) {
                std::string ultimo = historico.top();
                historico.pop();
                std::string volta = get_opposite(ultimo);
            
                if (move_robot(node, client, volta, 15)) {
                    if (volta == "up") y--;
                    else if (volta == "down") y++;
                    else if (volta == "left") x--;
                    else if (volta == "right") x++;

                    RCLCPP_INFO(node->get_logger(), "backtranking: %s", volta.c_str());
                }
            } else {
                RCLCPP_INFO(node->get_logger(), "Mapeamento completo! Voltamos a base.");
                break; 
            }
        }
    }

    if (alvo_detectado) {
        ir_para_alvo(node, client, mapa_livre, pos_alvo);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Mapeei tudo e nao achei a sáida");
    }

    rclcpp::shutdown();
    return 0;
}