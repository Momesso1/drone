#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <map>

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode() : Node("motor_control_node")
    {
        // Criando publicadores para cada motor
        motor_publishers_["thruster1"] = create_publisher<std_msgs::msg::Float64>("/model/bluerov2_heavy/joint/thruster1_joint/cmd_thrust", 10);
        motor_publishers_["thruster2"] = create_publisher<std_msgs::msg::Float64>("/model/bluerov2_heavy/joint/thruster2_joint/cmd_thrust", 10);
        motor_publishers_["thruster3"] = create_publisher<std_msgs::msg::Float64>("/model/bluerov2_heavy/joint/thruster3_joint/cmd_thrust", 10);
        motor_publishers_["thruster4"] = create_publisher<std_msgs::msg::Float64>("/model/bluerov2_heavy/joint/thruster4_joint/cmd_thrust", 10);
        motor_publishers_["thruster5"] = create_publisher<std_msgs::msg::Float64>("/model/bluerov2_heavy/joint/thruster5_joint/cmd_thrust", 10);
        motor_publishers_["thruster6"] = create_publisher<std_msgs::msg::Float64>("/model/bluerov2_heavy/joint/thruster6_joint/cmd_thrust", 10);
        motor_publishers_["thruster7"] = create_publisher<std_msgs::msg::Float64>("/model/bluerov2_heavy/joint/thruster7_joint/cmd_thrust", 10);
        motor_publishers_["thruster8"] = create_publisher<std_msgs::msg::Float64>("/model/bluerov2_heavy/joint/thruster8_joint/cmd_thrust", 10);

        // Inicializando os valores de empuxo dos motores
        for (const auto &motor : motor_publishers_)
        {
            motor_thrust_[motor.first] = 0.0;
        }

        // Loop de controle
        control_loop();
    }

private:
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> motor_publishers_;
    std::map<std::string, double> motor_thrust_;

    // Função para capturar entrada do teclado
    char get_key()
    {
        char c;
        struct termios oldt, newt;

        // Desativa o buffer de entrada
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        c = getchar();

        // Restaura o terminal
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        return c;
    }

    // Loop de controle baseado no teclado
    void control_loop()
    {
        std::cout << "Controle dos motores do BlueROV2 Heavy:\n";
        std::cout << "W/S: Aumentar/Diminuir empuxo dos motores 1, 2, 3, 4\n";
        std::cout << "A/D: Girar motores 1, 2, 3, 4 para esquerda/direita\n";
        std::cout << "I/K: Aumentar/Diminuir empuxo dos motores 5, 6, 7, 8 (verticais)\n";
        std::cout << "Q: Encerrar o controle\n";

        while (rclcpp::ok())
        {
            char key = get_key();

            if (key == 'w' || key == 'W')
            {
                adjust_thrust("thruster1", -0.20);
                adjust_thrust("thruster2", -0.20);
                adjust_thrust("thruster3", 0.20);
                adjust_thrust("thruster4", 0.20);
            }
            else if (key == 's' || key == 'S')
            {
                adjust_thrust("thruster1", 0.20);
                adjust_thrust("thruster2", 0.20);
                adjust_thrust("thruster3", -0.20);
                adjust_thrust("thruster4", -0.20);
            }
            else if (key == 'a' || key == 'A')
            {
                adjust_thrust("thruster1", -0.20);
                adjust_thrust("thruster2", 0.20);
                adjust_thrust("thruster3", -0.20);
                adjust_thrust("thruster4", 0.20);
            }
            else if (key == 'd' || key == 'D')
            {
                adjust_thrust("thruster1", 0.20);
                adjust_thrust("thruster2", -0.20);
                adjust_thrust("thruster3", 0.20);
                adjust_thrust("thruster4", -0.20);
            }
            else if (key == 'i' || key == 'I')
            {
                adjust_thrust("thruster5", -0.20);
                adjust_thrust("thruster6", -0.20);
                adjust_thrust("thruster7", -0.20);
                adjust_thrust("thruster8", -0.20);
            }
            else if (key == 'k' || key == 'K')
            {
                adjust_thrust("thruster5", 0.20);
                adjust_thrust("thruster6", 0.20);
                adjust_thrust("thruster7", 0.20);
                adjust_thrust("thruster8", 0.20);
            }
            else if (key == 'q' || key == 'Q')
            {
                std::cout << "\nEncerrando controle..." << std::endl;
                break;
            }

            // Publica os valores atualizados de empuxo
            publish_thrusts();
        }
    }

    // Ajusta o empuxo de um motor específico
    void adjust_thrust(const std::string &motor, double increment)
    {
        motor_thrust_[motor] = std::clamp(motor_thrust_[motor] + increment, -7.0, 7.0);
        std::cout << "Empuxo do " << motor << ": " << motor_thrust_[motor] << "\r" << std::flush;
    }

    // Publica os valores de empuxo em todos os motores
    void publish_thrusts()
    {
        for (const auto &pair : motor_publishers_)
        {
            std_msgs::msg::Float64 msg;
            msg.data = motor_thrust_[pair.first];
            pair.second->publish(msg);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
