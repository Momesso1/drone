import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class DataPlotter(Node):
    def __init__(self):
        super().__init__('data_plotter')
        self.max_points = 200  # Limite para evitar crescimento excessivo da memória
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Variáveis para armazenar a posição atual e destino
        self.current_position = None   # Vindo do /rtabmap/odom
        self.destination = None        # Vindo do /destinations (usamos a primeira pose, se houver)

        # Dicionário para armazenar os dados dos tópicos de tempo e heuristic
        self.data = {
            # Tópicos relacionados a filter_time e time
            'a_star_filter_time': {'timestamps': [], 'values': []},
            'a_star_time': {'timestamps': [], 'values': []},
            'a_star_with_filter_time': {'timestamps': [], 'values': []},
            'bidirectional_a_star_filter_time': {'timestamps': [], 'values': []},
            'bidirectional_a_star_time': {'timestamps': [], 'values': []},
            'bidirectional_a_star_with_filter_time': {'timestamps': [], 'values': []},
            # Tópicos relacionados a heuristic distance
            'a_star_heuristic_distance': {'timestamps': [], 'values': []},
            'a_star_with_filter_heuristic_distance': {'timestamps': [], 'values': []},
            'bidirectional_a_star_heuristic_distance': {'timestamps': [], 'values': []},
            'bidirectional_a_star_with_filter_heuristic_distance': {'timestamps': [], 'values': []},
        }

        # Cria inscrições para os tópicos de dados
        for topic in self.data.keys():
            self.create_subscription(
                Float32, 
                '/' + topic, 
                self.generic_callback_factory(topic), 
                10
            )

        # Inscrição para posição atual (/rtabmap/odom)
        self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.odom_callback,
            10
        )

        # Inscrição para destino (/destinations)
        self.create_subscription(
            PoseArray,
            '/destinations',
            self.destinations_callback,
            10
        )

        # Configuração do Matplotlib: dois subplots (um para heuristic distance e outro para filter_time/time)
        plt.style.use('seaborn')
        self.fig, (self.ax_heuristic, self.ax_time) = plt.subplots(2, 1, figsize=(10, 12))

        # Cria animação para atualização dos gráficos
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)

    def generic_callback_factory(self, topic):
        """Retorna um callback que armazena os dados do tópico."""
        def callback(msg):
            current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
            self.data[topic]['timestamps'].append(current_time)
            self.data[topic]['values'].append(msg.data)
            if len(self.data[topic]['values']) > self.max_points:
                self.data[topic]['timestamps'].pop(0)
                self.data[topic]['values'].pop(0)
        return callback

    def odom_callback(self, msg):
        """Callback para atualizar a posição atual a partir de /rtabmap/odom."""
        self.current_position = msg.pose.pose.position

    def destinations_callback(self, msg):
        """
        Callback para atualizar a posição destino a partir de /destinations.
        Assume-se que a primeira pose (se existir) é o destino desejado.
        """
        if msg.poses:
            self.destination = msg.poses[0].position

    def update_plot(self, frame):
        # Limpa os subplots para redesenhar os gráficos
        self.ax_heuristic.clear()
        self.ax_time.clear()

        # Se não receber dados de /rtabmap/odom, define a posição atual como (0, 0, 0)
        if self.current_position is None:
            current_str = "(0.00, 0.00, 0.00)"
        else:
            current_str = f"({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f})"

        # Posição destino
        if self.destination is not None:
            dest_str = f"({self.destination.x:.2f}, {self.destination.y:.2f}, {self.destination.z:.2f})"
        else:
            dest_str = "N/A"

        # Atualiza o título geral com as posições (posição atual - destino)
        self.fig.suptitle(f"{current_str}  -  {dest_str}", fontsize=12)

        # Tópicos de heuristic distance
        heuristic_topics = [
            'a_star_heuristic_distance',
            'a_star_with_filter_heuristic_distance',
            'bidirectional_a_star_heuristic_distance',
            'bidirectional_a_star_with_filter_heuristic_distance'
        ]
        # Plota os dados de heuristic distance com zorder baixo
        for topic in heuristic_topics:
            self.ax_heuristic.plot(
                self.data[topic]['timestamps'], 
                self.data[topic]['values'], 
                label=topic,
                zorder=1
            )
        self.ax_heuristic.set_title('Heuristic Distance')
        self.ax_heuristic.set_xlabel('Tempo de Simulação (s)')
        self.ax_heuristic.set_ylabel('Distância (m)')
        # Cria a legenda e define seu zorder alto e fundo opaco
        leg1 = self.ax_heuristic.legend(loc='upper right', frameon=True)
        leg1.set_zorder(10)
        leg1.get_frame().set_alpha(1)

        # Ajusta o eixo Y para heuristic distance: inicia em 0 e vai até 10% acima do maior valor
        heuristic_values = []
        for topic in heuristic_topics:
            heuristic_values.extend(self.data[topic]['values'])
        if heuristic_values:
            max_val = max(heuristic_values)
            margin = max_val * 0.1 if max_val != 0 else 1.0
            self.ax_heuristic.set_ylim(0, max_val + margin)

        # Tópicos de filter_time e time
        time_topics = [
            'a_star_filter_time',
            'a_star_time',
            'a_star_with_filter_time',
            'bidirectional_a_star_filter_time',
            'bidirectional_a_star_time',
            'bidirectional_a_star_with_filter_time'
        ]
        # Plota os dados de filter_time/time com zorder baixo
        for topic in time_topics:
            self.ax_time.plot(
                self.data[topic]['timestamps'], 
                self.data[topic]['values'], 
                label=topic,
                zorder=1
            )
        self.ax_time.set_title('Filter Time e Time')
        self.ax_time.set_xlabel('Tempo de Simulação (s)')
        self.ax_time.set_ylabel('Tempo de execução (s)')
        # Cria a legenda para o subplot e define seu zorder alto e fundo opaco
        leg2 = self.ax_time.legend(loc='upper right', frameon=True)
        leg2.set_zorder(10)
        leg2.get_frame().set_alpha(1)

        # Ajuste dinâmico de escala para filter_time/time (pode ser modificado se necessário)
        time_values = []
        for topic in time_topics:
            time_values.extend(self.data[topic]['values'])
        if time_values:
            min_val = min(time_values)
            max_val = max(time_values)
            if min_val == max_val:
                margin = max_val * 0.1 if max_val != 0 else 1.0
                self.ax_time.set_ylim(0, max_val + margin)
            else:
                margin = (max_val - min_val) * 0.1
                self.ax_time.set_ylim(min_val - margin, max_val + margin)

        plt.tight_layout(rect=[0, 0, 1, 0.95])  # Ajusta layout para não sobrepor o suptitle

def start_plot():
    """Executa plt.show() em uma thread separada para manter o gráfico aberto."""
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    data_plotter = DataPlotter()

    # Inicia a thread do Matplotlib
    plot_thread = threading.Thread(target=start_plot, daemon=True)
    plot_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(data_plotter, timeout_sec=0.01)  # Atualiza callbacks
            plt.pause(0.001)  # Atualiza o gráfico em tempo real
    except KeyboardInterrupt:
        pass
    finally:
        data_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
