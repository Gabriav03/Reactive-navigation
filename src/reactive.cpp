#include "navigation/reactive.hpp" 

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// Constructor
Reactive::Reactive(): Node("reactive")
{
    // Init ROS service
    power_ = this->create_service<navigation::srv::Power>(
        "power_on_off",
        std::bind(&Reactive::handle_power, this, _1, _2, _3));
    is_powered_on = false; // Initially off

    // Init ROS publishers
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
 
    // Init ROS subscribers
    sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 1, std::bind(&Reactive::process_laser_info, this, _1));

    // Initialization of Logic Variables
    current_state_ = States::NORMAL_OPERATION;
    
    ranges_size_ = 0;
    
    near_obst_f = false;
    near_obst_r = false;
    near_obst_l = false;
    no_free_space = false;

    forward_dist = 0.0;
    right_dist = 0.0;
    left_dist = 0.0;
    desired_orientation = 0.0;

    // PID Configuration
    pid_kp = 1.5f; 
    pid_ki = 0.0f;
    pid_kd = 0.5f;
    
    prev_error = 0.0f;
    integral_error = 0.0f;
    
    lin_vel_nominal = 1.2f; // 1.2 m/s nominal speed
    freq_ejecucion = 5.0f; // 5 Hz loop rate
}

// Destructor
Reactive::~Reactive()
{
}

void Reactive::handle_power(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navigation::srv::Power::Request> request,
    std::shared_ptr<navigation::srv::Power::Response> response)
{
    (void)request_header;
    std::string command = request->power_command;
    if (command == "power on") {
        response->power_on = true;
        RCLCPP_INFO(this->get_logger(), "Navegacion Reactiva Iniciada");
    } else if (command == "power off") {
        response->power_on = false;
        RCLCPP_INFO(this->get_logger(), "Navegacion Reactiva Apagada");
    } else {
        RCLCPP_WARN(this->get_logger(), "Comando desconocido recibido: '%s'", command.c_str());
    }

    is_powered_on = response->power_on;
}

void Reactive::FSM_Control_Loop()
{
    if (!is_powered_on) {
        navigation_control();   // STOP
        return; 
    }
    // Make sure it doesn't run unless there's sensor data
    if (ranges_size_ == 0 || last_laser_scan_ == nullptr) {
        return;
    }

    // 1. Desired orientation calculation (Gap finding)
    orientation_calculation();

    // 2. Finite State Machine 
    switch (current_state_)
    {
    case States::NORMAL_OPERATION:
        if (near_obst_f)
        {
            current_state_ = States::INMINENT_COLLISION;
            RCLCPP_WARN(this->get_logger(), "ALERTA: Obstaculo Frontal - Cambiando a Evasion");
        }
        break;

    case States::INMINENT_COLLISION:
        if(!near_obst_f)
        {
            current_state_ = States::NORMAL_OPERATION;
            RCLCPP_INFO(this->get_logger(), "Recuperado: Vuelta a Operacion Normal");
        }
        break;

    default:
        break;
    }

    // 3. Run PID control and send commands
    navigation_control();
}

void Reactive::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr laserx)
{
    // Save pointer
    last_laser_scan_ = laserx;
    ranges_size_ = laserx->ranges.size();

    // Variable intialization
    forward_dist = 0;
    right_dist = 0;
    left_dist = 0;
    near_obst_f = false;
    near_obst_r = false;
    near_obst_l = false;
    no_free_space = false;
    
    double phi_min = laserx->angle_min;   
    double phi_increment = laserx->angle_increment;
    double phi = 0;
    double distance = 0;

    // First obstacle detection sweep
    for (int i = 0; i < ranges_size_ - 1; i++)    
    {
        phi = phi_min + i * phi_increment;
        distance = laserx->ranges[i];

        // Basic invalid readings filter  
        if(std::isinf(distance) || std::isnan(distance)) {
            distance = 5.0; // Asumir lejos si falla lectura
        }

        // Obstacle detection
        if(phi >= (-40*M_PI/180) && phi <= (40*M_PI/180))       // Front
        {
            if(distance <= 1.6) near_obst_f = true;
        }
        if(phi >= (-120*M_PI/180) && phi < (-40*M_PI/180))      // Right
        {
            if(distance <= 1.2) near_obst_r = true;
        }
        if(phi > (40*M_PI/180) && phi <= (120*M_PI/180))        // Left
        {
            if(distance <= 1.2) near_obst_l = true;
        }

        // Weight assignation and cumulative sum
        double weighted_dist = weight_assignation(distance, i, phi, ranges_size_);

        if(phi >= (-40*M_PI/180) && phi <= (40*M_PI/180))
        {
            forward_dist = forward_dist + weighted_dist;             
        }
        else if(phi >= (-120*M_PI/180) && phi < (-40*M_PI/180))
        {
            right_dist = right_dist + weighted_dist;
        }
        else if(phi > (40*M_PI/180) && phi <= (120*M_PI/180))
        {
            left_dist = left_dist + weighted_dist;
        }
    }
}

void Reactive::orientation_calculation()
{
    if (!last_laser_scan_) return;

    int sector = 2; // Central sector (default)
    int ranges_size = ranges_size_;
    double phi_min = last_laser_scan_->angle_min;
    double phi_increment = last_laser_scan_->angle_increment;

    // Sector selection logic (thirds division)
    if(forward_dist < (right_dist * 0.8) && forward_dist < (left_dist * 0.8))
    {
        // Choose the highest free-space value sector
        if (right_dist >= left_dist) sector = 1; // Right
        else sector = 3; // Left
    }
    else 
    {
        sector = 2; // Preference for going forwards
    }

    // Chosen sector indexes
    int start_idx = ((ranges_size * (sector-1)) / 3);
    int end_idx = (ranges_size * sector) / 3 - 1;
    
    // Valid indexes only
    if (start_idx < 0) start_idx = 0;
    if (end_idx > ranges_size) end_idx = ranges_size;

    // 10 readings window sweep
    double avg_cone = 0;
    double max_avg_cone = -1.0;
    int best_idx = (start_idx + end_idx) / 2; // Best sector's central index
    int cone_width = 10;
    int counter = 0;
    double current_cone_sum = 0;

    for(int i = start_idx; i < end_idx; i++)
    {
        if(no_free_space) break;

        double phi = phi_min + i * phi_increment;
        double distance = last_laser_scan_->ranges[i];

        // Weight assignation
        distance = weight_assignation(distance, i, phi, ranges_size_);

        current_cone_sum += distance;
        counter++;

        if (counter >= cone_width)
        {
            avg_cone = current_cone_sum / cone_width;
            
            if (avg_cone > max_avg_cone)
            {
                max_avg_cone = avg_cone;
                best_idx = i - (cone_width / 2); 
            }
            
            current_cone_sum = 0;
            counter = 0;
        }
    }

    // Define desired orientation
    desired_orientation = phi_min + best_idx * phi_increment;
}

void Reactive::navigation_control()
{
    // PID LOGIC
    float error = desired_orientation; 

    // Integral error calculation (with basic anti-windup)
    if (std::abs(error) > 0.1) 
        integral_error += error;
    else 
        integral_error = 0.0f;

    // Integral reset if error value crosses zero
    if ((prev_error > 0 && error < 0) || (prev_error < 0 && error > 0)) 
        integral_error = 0.0f;

    // Derivative error
    float derivative_error = error - prev_error;
    
    // PID output
    float w = (pid_kp * error) + (pid_ki * integral_error / freq_ejecucion) + (pid_kd * derivative_error);
    
    prev_error = error;

    // Linear velocity control
    float v = lin_vel_nominal;

    // Extra safety measure: if there's frontal obstacles, reduce linear velocity
    if(current_state_ == States::NORMAL_OPERATION)
    {
        if(std::abs(error) > (60.0f * M_PI/180)) v *= 0.4f;
    }

    if (current_state_ == States::INMINENT_COLLISION) 
    {
        v = v*0.1f; // Harder brake for inminent collisions
    }    
    // Recovery turns
    if(no_free_space)
    {
        v = 0.0f; // STOP
        // Force a fixed turn value to scape if PID calculation is not enough (local minimum escape)
        if (no_free_space && std::abs(w) < 0.3f) 
        {
            w = -0.9f;
        }
    }

    // Publish velocity commands
    geometry_msgs::msg::Twist cmd;
    if(!is_powered_on)
    {
        // When OFF, make sure it's stopped
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        pub_twist_->publish(cmd);
        return;
    }
    cmd.linear.x = v;
    cmd.angular.z = w;
    
    pub_twist_->publish(cmd);
}

double Reactive::weight_assignation(double distance, int i, double phi, int ranges_size)
{
    // Triangular weight assignation
    // No free-space condition
    if(near_obst_f && near_obst_r && near_obst_l)
    {
        no_free_space = true;
    }

    if(!near_obst_r && !near_obst_l) // Sides free space
    {
        if(!near_obst_f) // Front free space
        {
            if(phi >= (-120*M_PI/180) && phi <= 0)      
            {
                distance = distance*(1+(0.5*i)/(ranges_size/2.0));
            }
            else
            {
                distance = distance*(2-(0.5*i)/(ranges_size/2.0));
            }
        }
        else // If there are obstacles on the front side
        {
            if(phi >= (-120*M_PI/180) && phi < (-40*M_PI/180))
            {
                distance = distance*(1+(0.5*i)/(ranges_size/2.0));
            }
            else if (phi >= (-40*M_PI/180) && phi <= (40*M_PI/180))
            {
                distance = 0;
            }
            else
            {
                distance = distance*(2-(0.5*i)/(ranges_size/2.0));
            }
        }
        return distance;
    }

    if(near_obst_l && !near_obst_r) // Left blocked, right free
    {
        if(phi >= (-120*M_PI/180) && phi <= (-80*M_PI/180))
        {
            distance = distance*(1+(1.0*i)/(ranges_size/3.0));
        }
        else
        {
            distance = distance*(2-(1.0*i)/(ranges_size/3.0));
        }
    }
    
    if(near_obst_r && !near_obst_l) // Right blocked, left free
    {
        if(phi >= (-120*M_PI/180) && phi <= (80*M_PI/180))
        {
            distance = distance*(-1+(1.0*i)/(ranges_size/3.0));
        }
        else
        {
            distance = distance*(4-(1.0*i)/(ranges_size/3.0));
        }
    }

    return distance;
}

int main( int argc, char *argv[])
{
    //Init ROS2 node
    rclcpp::init( argc, argv );
    auto node = std::make_shared<Reactive>();
    rclcpp::Rate loop_rate(5.0);
    RCLCPP_INFO(node->get_logger(), "Iniciando Bucle Reactivo con PID...");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);    
        node->FSM_Control_Loop();   
        loop_rate.sleep();          
    }
    rclcpp::shutdown();

    return 0;
}