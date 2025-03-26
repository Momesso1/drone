import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class DataPlotter(Node):
    def __init__(self):
        super().__init__('data_plotter')
        
        # Initialize data storage with fixed maximum length to prevent memory growth
        self.max_points = 200  # Ajuste conforme necessário
        
        # Initialize data lists
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.processing_times = []
        self.processing_time_stamps = []
        self.robot_velocities = []
        self.velocity_time_stamps = []
        self.positions_x = []
        self.positions_y = []
        
        # Create subscriptions
        self.create_subscription(Float32, '/time', self.processing_time_callback, 10)
        self.create_subscription(Point, '/pose', self.robot_position_callback, 10)
        self.create_subscription(Float32, '/velocity', self.robot_velocity_callback, 10)
        
        # Configuração do Matplotlib para atualização em tempo real
        plt.style.use('seaborn')  
        # self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 12))

        self.fig, (self.ax1) = plt.subplots(1, 1, figsize=(10, 12))

        # Criar a animação para atualizar os gráficos
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)

    def processing_time_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.processing_time_stamps.append(current_time)
        self.processing_times.append(msg.data)
        
        if len(self.processing_times) > self.max_points:
            self.processing_time_stamps.pop(0)
            self.processing_times.pop(0)
    
    def robot_velocity_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.velocity_time_stamps.append(current_time)
        self.robot_velocities.append(msg.data)
        
        if len(self.robot_velocities) > self.max_points:
            self.velocity_time_stamps.pop(0)
            self.robot_velocities.pop(0)
    
    def robot_position_callback(self, msg):
        self.positions_x.append(msg.x)
        self.positions_y.append(msg.y)
        
        if len(self.positions_x) > self.max_points:
            self.positions_x.pop(0)
            self.positions_y.pop(0)
    
    def update_plot(self, frame):
        self.ax1.clear()
        # self.ax2.clear()
        # self.ax3.clear()
        
        # Processing Time
        self.ax1.plot(self.processing_time_stamps, self.processing_times, 
                      label='Processing Time', color='blue')
        self.ax1.set_title('Bidirectional A* - Origin: (0, 0, 0)  -  Destination (-17.5, -13.25, 2.55)\nNo obstacles')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Time (s)')
        self.ax1.legend()
        
        # Robot Velocity
        # self.ax2.plot(self.velocity_time_stamps, self.robot_velocities, 
        #               label='Robot Velocity', color='orange')
        # self.ax2.set_title('Robot Velocity')
        # self.ax2.set_xlabel('Time (s)')
        # self.ax2.set_ylabel('Velocity (m/s)')
        # self.ax2.legend()
        
        # # Robot Trajectory
        # self.ax3.plot(self.positions_x, self.positions_y, 
        #               label='Trajectory', marker='o', linestyle='-', color='green')
        # self.ax3.set_title('Robot Position')
        # self.ax3.set_xlabel('X Position (m)')
        # self.ax3.set_ylabel('Y Position (m)')
        # self.ax3.legend()
        
        plt.tight_layout()
    
def start_plot():
    """Função para rodar o plt.show() em uma thread separada"""
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    data_plotter = DataPlotter()

    # Iniciar a thread do Matplotlib para manter o gráfico aberto
    plot_thread = threading.Thread(target=start_plot, daemon=True)
    plot_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(data_plotter, timeout_sec=0.01)  # Atualiza os callbacks
            plt.pause(0.001)  # Atualiza o gráfico em tempo real
    except KeyboardInterrupt:
        pass
    finally:
        data_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
