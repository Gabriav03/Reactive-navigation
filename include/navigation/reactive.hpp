#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <vector>
#include "navigation/srv/power.hpp"

enum class States
{
    NORMAL_OPERATION,
    INMINENT_COLLISION
};

class Reactive : public rclcpp::Node
{
public:
    Reactive();           // Constructor
    ~Reactive();          // Destructor

    // Callback del laser
    void process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr laserx);

    // Bucle principal de control
    void FSM_Control_Loop();    

private:
    // ROS Communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;

    rclcpp::Service<navigation::srv::Power>::SharedPtr power_;
    void handle_power(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<navigation::srv::Power::Request> request,
        std::shared_ptr<navigation::srv::Power::Response> response);

    bool is_powered_on;

    // Almacenamiento de Datos
    // Puntero para guardar la ultima lectura y usarla en el bucle principal
    sensor_msgs::msg::LaserScan::SharedPtr last_laser_scan_; 
    int ranges_size_; // Tamaño del array del laser

    // Variables de Estado
    States current_state_;

    // Distancias acumuladas por sectores
    double forward_dist;
    double right_dist;
    double left_dist;

    // Flags booleanos de obstaculos cercanos
    bool near_obst_f;
    bool near_obst_r;
    bool near_obst_l;
    bool no_free_space;

    // Navegacion
    double desired_orientation; // El angulo objetivo calculado por los conos

    // PID y Control
    float pid_kp, pid_ki, pid_kd;
    float prev_error;
    float integral_error;
    
    // Parametros de movimiento
    float lin_vel_nominal;    // Velocidad lineal base
    float freq_ejecucion;  // Frecuencia del bucle (Hz)

    // Funciones Internas
    
    // Calcula la orientacion deseada basandose en last_laser_scan_
    void orientation_calculation(); 

    // Ejecuta el PID y publica la velocidad
    void navigation_control();

    // Funcion auxiliar para ponderar distancias
    double weight_assignation(double distance, int i, double phi, int ranges_size);
};