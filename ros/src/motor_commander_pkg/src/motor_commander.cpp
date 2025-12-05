#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <klask_interfaces/msg/stamped_polygon.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/msg/controller_status.hpp>
#include <random>
#include <cmath>
#include <algorithm>

// #include <odrive_can/srv/clear_errors.hpp>

const float MAX_TORQUE = 0.25f;
const float CALIB_TORQUE = 0.225f;
const float SHUTDOWN_TORQUE = 1.5f;
const float MAX_VEL = 1.0f;

std::string bufferToString(const std::vector<std::vector<bool>>& buffer) {
    std::stringstream ss;
    for (const auto& row : buffer) {
        for (bool val : row) {
            ss << (val ? "1" : "0") << " ";
        }
        ss << "\n";
    }
    return ss.str();
}

std::string sbufferToString(const std::vector<float>& buffer) {
    std::stringstream ss;
    
    for (float val : buffer) {
        ss << (val) << " ";
    }
    ss << "\n";

    return ss.str();
}


class Player : public rclcpp::Node {
public:
    Player(const std::string& name)
        :  Node(name), name_(name)
    {   

        std::string motor_ind_r;
        std::string motor_ind_l;
        if (name == "player"){
            motor_ind_r = "0";
            motor_ind_l = "1";
            home = {410.0f, 175.0f};
            EDGE = {283.0f, 528.0f, 5.0f, 367.0f};
            sign = 1.0f;
   
        }else{
            motor_ind_r = "2";
            motor_ind_l = "3";
            home = {110.0f, 175.0f};
            EDGE = {5.0f, 250.0f, 5.0f, 367.0f};
            sign = -1.0f;
        }
        group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::PublisherOptions pub_options;
        pub_options.callback_group = group_;

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = group_;

        motor_left_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(
            "/odrive_axis"+motor_ind_l+"/control_message", 10, pub_options);  
        motor_right_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(
            "/odrive_axis"+motor_ind_r+"/control_message", 10, pub_options);  
        
        position_magnet_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/" + name + "_magnet_position", 10,pub_options);

        request_axis_r_state_client_ = this->create_client<odrive_can::srv::AxisState>("/odrive_axis"+motor_ind_r+"/request_axis_state",
        rmw_qos_profile_services_default,
        group_ 
        );
        request_axis_l_state_client_ = this->create_client<odrive_can::srv::AxisState>("/odrive_axis"+motor_ind_l+"/request_axis_state",
        rmw_qos_profile_services_default,
        group_ 
        );

        set_motor_state(1);
        set_motor_state(8);

        controller_r_subscriber = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/odrive_axis"+motor_ind_r+"/controller_status", 1, std::bind(&Player::controller_status_callback_right, this, std::placeholders::_1),sub_options);
        controller_l_subscriber = this->create_subscription<odrive_can::msg::ControllerStatus>(
            "/odrive_axis"+motor_ind_l+"/controller_status", 1, std::bind(&Player::controller_status_callback_left, this, std::placeholders::_1),sub_options);
        
    }

    void velocity_targets(float v_x, float v_y){
        //RCLCPP_INFO(this->get_logger(), "Node %s is processing", name_);

        if (peg_mag_synchronized && !finding_peg)
        {
            publish_velocity(v_x, v_y);
        }else if(!finding_peg){
            find_peg();
        }

    }
    
    
    void is_synchronized() 
    {
        
        //float epsilon_desynch = 20.0f;
        //float epsilon_synch = 100.0f;
        float delta = 7.0f;
        //RCLCPP_INFO(this->get_logger(), "Buffer:%s", sbufferToString(synch_buffer).c_str());
        //RCLCPP_INFO(this->get_logger(), "Vel x:%f", velocity[0]);
        //RCLCPP_INFO(this->get_logger(), "Vel y:%f", velocity[1]);
        //RCLCPP_INFO(this->get_logger(), "MagVel x:%f", magnet_velocity[0]);
        //RCLCPP_INFO(this->get_logger(), "MagVel y:%f", magnet_velocity[1]);
        
        std::copy_backward(synch_buffer.begin(), synch_buffer.end() - 1, synch_buffer.end());
        if (velocity[0] < delta && velocity[1]<delta){
            
           
            if (magnet_velocity[0] < delta && magnet_velocity[1]< delta && peg_mag_synchronized){
                
                synch_buffer[0] = true;
            }else{
                 synch_buffer[0] = false;
            }
        }else{
            synch_buffer[0] = true;
        }
        
        //RCLCPP_INFO(this->get_logger(), "peg_mag_synch:%d", peg_mag_synchronized);
        //idea: depending on the velocity you can check a specified amount of elements in the buffer, the higher the vel the less elements to check before detecting desynch
        float mag_vel = std::sqrt(magnet_velocity[0]*magnet_velocity[0]+magnet_velocity[1]*magnet_velocity[1]);
        sync_elements_to_check = 0.4*mag_vel+0.6 * sync_elements_to_check;
        unsigned int elements_to_check = std::min( static_cast<unsigned int>(synch_buffer.size())-1, interpolate_elements(sync_elements_to_check)); 
        
        if (std::all_of(synch_buffer.begin(), synch_buffer.begin() + elements_to_check, [](float val) { return val == false; }))
        {   
            if(peg_mag_synchronized){
                RCLCPP_INFO(this->get_logger(), "Peg and magnet desynchronized");
            }
            peg_mag_synchronized = false; // it would desynchronize if it is hitting against a wall right now
            
            
        }else if(std::all_of(synch_buffer.begin(), synch_buffer.begin()+3,[](float val) { return val == true; })){
            if(!peg_mag_synchronized){
                RCLCPP_INFO(this->get_logger(), "Peg and magnet synchronized");
            }
            peg_mag_synchronized = true;
            
        }
        
    }

    bool finding_peg = false;
    bool peg_mag_synchronized = false;
    bool synchronized_once = false;
    std::vector<float> position = {0.0f, 0.0f};
    std::vector<float> velocity = {0.0f, 0.0f};
    std::vector<float> synchronized_state = {0.0f, 0.0f, 0.0f, 0.0f};  // encoder_pos_right , encoder_pos_left , camera x value, camera y value
    std::vector<float> encoder_values = {0.0f, 0.0f, 0.0f, 0.0f}; // 0.0f pos, 1 pos, 0.0f vel, 1 vel running values of encoder right(0.0f) and left(1) motor respectively

    

private:
    std::string name_;
    rclcpp::CallbackGroup::SharedPtr group_;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr motor_left_pub_;
    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr motor_right_pub_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_r_state_client_;
    rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr request_axis_l_state_client_;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr controller_r_subscriber;
    rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr controller_l_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_magnet_publisher_;

    float sign;

    
    std::vector<bool> synch_buffer = std::vector<bool>(100,false);
    float sync_elements_to_check = 0.0f; 
    
    bool motor_at_wall = false;
    bool motor_at_wall_calib = false;
    std::vector<std::vector<bool>> motor_at_wall_buffer = std::vector<std::vector<bool>>(4, std::vector<bool>(7, false));
    std::vector<bool> wall_prot_activated = std::vector<bool>(4,false);
    
    float torque_left_motor = 0.0f;
    float torque_right_motor = 0.0f;

    std::vector<float> home = std::vector<float>(4,0.0f);
    
    std::vector<float> EDGE = std::vector<float>(4,0.0f);

    const float DISTANCE_PER_REVOLUTION = 0.04f;                    // PITCH * #TEETH
    const float REAL_TO_CAM_FACTOR_X = 1.0f / 0.0008285f; // scaling factor from meter to camera units;
    const float REAL_TO_CAM_FACTOR_Y = 1150.0f;

    bool at_home = false;

    std::vector<float> magnet_position = {0.0f, 0.0f};      // x , y position of the magnet
    std::vector<float> magnet_velocity = {0.0f, 0.0f};

    
    void find_peg()
    {
        finding_peg = true;
        RCLCPP_INFO(this->get_logger(), "In Function");
        while (!peg_mag_synchronized)
        {

            move_to_corner(); // state messages are only published once the qr code is detectable (this is the case after exectution of this function)
            move_pattern();
        }
        synchronized_once = true;
        find_home();
        finding_peg = false;
        
    }
    

    void move_to_corner()
    {
        bool in_corner = false;
        bool at_edge = false;
        
        

        rclcpp::Rate loop_rate(50);

        while (rclcpp::ok() && (!in_corner && !peg_mag_synchronized))
        {
            //RCLCPP_INFO(this->get_logger(), "motor at wall var: %d", motor_at_wall_calib);
            if (motor_at_wall_calib && !at_edge)
            { // first time it hits a wall, find out which one
                at_edge = true;
                send_commands(-sign*0.008f, 0.0f);
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            else if (!motor_at_wall_calib && at_edge){
                
                send_commands(0.0f, sign*0.008f);
            }
            else if (motor_at_wall_calib && at_edge)
            {
                // If motor is at the wall and we are either at the right or left edge, we are in the corner
                
                send_commands(0.0f, -sign*0.008f);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                send_commands(0.0f, 0.0f); // Stop the motor
                
                magnet_position[0] = (name_ == "player") ? 530.0f: 0.0f;   // position of the bottom right corner
                magnet_position[1] = (name_ == "opponent") ? 370.0f: 0.0f;
                synchronized_state = {encoder_values[0],encoder_values[1],magnet_position[0],magnet_position[1]};
                RCLCPP_INFO(this->get_logger(), "Peg in corener");
                
                in_corner = true;
            }
            else
            {
                send_commands(sign*0.008f, 0.0f); // Move to the bottom right corner
                
            }

            //rclcpp::spin_some(this->get_node_base_interface());
                // RCLCPP_INFO(this->get_logger(),"looping");
            //RCLCPP_INFO(this->get_logger(), "Torque: %f", std::sqrt(std::pow(torque_right_motor, 2) + std::pow(torque_left_motor, 2)));
            is_synchronized();
            loop_rate.sleep();
        }
    }

    void move_pattern()
    {
        
        bool moving_in_x;
        bool moving_in_y;
        bool moving_in_pos_x;
        bool moving_in_pos_y;
        float x_target;
        float y_target;

        if(name_ == "player"){
            moving_in_x = true;
            moving_in_y = false;
            moving_in_pos_x = false;
            moving_in_pos_y = false;
            x_target = magnet_position[0] - 200.0f;
            y_target = magnet_position[1] - 30.0f;
        }else{
            moving_in_x = true;
            moving_in_y = false;
            moving_in_pos_x = true;
            moving_in_pos_y = true;
            x_target = magnet_position[0] + 200.0f;
            y_target = magnet_position[1] + 30.0f;
        }

        rclcpp::Rate loop_rate(50);
        while (rclcpp::ok() && !peg_mag_synchronized)
        { // if it doesnt work well here is no max torque functionality
            float v_x = moving_in_x ? (moving_in_pos_x ? 0.02f : -0.02f) : 0.0f;
            float v_y = moving_in_y ? (moving_in_pos_y ? 0.02f : -0.02f) : 0.0f;
            
            publish_velocity(v_x, v_y);
            // if it doesnt exit this loop it is prob. because states are not yet published. States are only published after all the tags have been detected
            //for (size_t i = 0; i < velocity.size(); ++i)
            //{   
            //    //RCLCPP_INFO(this->get_logger(), "velocity difference: %f, ",std::fabs(velocity[i] - last_velocity_values[i]));
            //    if (std::fabs(velocity[i] - last_velocity_values[i]) > epsilon)
            //    {   
            //        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //Imortant! The peg often falls, so the magnet better continue moving towards it
            //        send_commands(0.0, 0.0);
            //        peg_mag_synchronized = true;
            //        break;
            //    }
            //}
            if (moving_in_x && std::fabs(magnet_position[0] - x_target) < 10.0f)
            {   
                moving_in_pos_x = !moving_in_pos_x;
                moving_in_y = true;
                moving_in_x = false;
                x_target = magnet_position[0] + 190.0f * (moving_in_pos_x ? 1.0f : -1.0f);
            }
            else if (moving_in_y && magnet_position[1] - y_target < 5.0f)
            {
                moving_in_y = false;
                moving_in_x = true;  
                y_target = magnet_position[1] - 30.0f;
                if (y_target < 0.0f)
                {
                    break;
                }
            }
            is_synchronized();//rclcpp::spin_some(this->get_node_base_interface());
            loop_rate.sleep();
        }
    }
    void find_home()
    {
        
        const float epsilon = 4; // Allow slight movement tolerance
        float v_x = 0.0;
        float v_y = 0.0;
        rclcpp::Rate loop_rate(20);
        while (!at_home && peg_mag_synchronized)
        {
            if (std::abs(position[0] - home[0]) > epsilon)
            {
                v_x = std::copysign(0.005f, home[0] - position[0]);
            }
            else
            {
                v_x = 0.0;
            }
            if (std::abs(position[1] - home[1]) > epsilon)
            {
                v_y = std::copysign(0.005f, home[1] - position[1]);
            }
            else
            {
                v_y = 0.0;
            }
            publish_velocity(v_x, v_y);
            //is_synchronized();
            loop_rate.sleep();
            //rclcpp::spin_some(this->get_node_base_interface());
            if (v_x == 0.0 && v_y == 0.0)
            {   
                publish_velocity(0.0f,0.0f);
                RCLCPP_INFO(this->get_logger(), "Peg is at home");
                at_home = true;
                synchronized_state = {0.0f,0.0f,0.0f,0.0f};
                synchronized_state[0] = encoder_values[0];
                synchronized_state[1] = encoder_values[1];
                synchronized_state[2] = position[0];
                synchronized_state[3] = position[1];
                
                
                
            }
        }
        
    }

    

    unsigned int  interpolate_elements(float x){
        static float x1 = 5.875f;
        static float x2 = 235.0f;
        static float y1 = 50.0f;
        static float y2 = 7.0f;
        unsigned int y = std::ceil(((y2-y1)/(x2-x1))*x + y1-x1*((y2-y1)/(x2-x1)));
        return y;
    }

    void publish_velocity(float v_x, float v_y)
    {   
        float mag_pos = 0.0f;
        int counter = 0;
        for (auto& row : motor_at_wall_buffer) {
            std::copy_backward(row.begin(), row.end() - 1, row.end());
            row[0] = false;
            if(counter > 1){
                mag_pos = magnet_position[1];
            }else{
                mag_pos = magnet_position[0];
            }
            if (motor_at_wall && std::fabs(mag_pos - EDGE[counter]) < 15.0f) {
                row[0] = true;
                //RCLCPP_INFO(this->get_logger(), "COUNTER %u, MAG_POS %f", counter,mag_pos);
            }else if(std::fabs(mag_pos - EDGE[counter]) > 35.0f){
                if (wall_prot_activated[counter]){
                    RCLCPP_INFO(this->get_logger(), "Stopped hitting against %u st wall", counter);
                }
                wall_prot_activated[counter] = false;
                
            }
            counter++;
        }
        deaceleration_profile(v_x,v_y);
        move_away_from_wall(v_x, v_y);
        
            
        send_commands(v_x, v_y);
    }

    void deaceleration_profile(float& v_x, float &v_y){

        /*major factor here is that the state estimator is a kalman filter. thereafter if the peg comes with a high vel towards the boundary
        the position will be outside of the field and the vel direction changes which leads to undesired oscilations at the edges. to prevent it
        the factor kf_safety_with is introduced, so that this function only affects the performance slightly*/
        float distance = 35.0f;
        float kf_safety_width = 5.0f;
        float speed_limit_weight = 20.0f;
        if(position[0] <= EDGE[0] + distance){
            v_x = std::max(v_x,std::abs(v_x)*std::tanh((EDGE[0]-position[0]-kf_safety_width)/speed_limit_weight));
            
            //RCLCPP_INFO(this->get_logger(), "v_x is modified to: %f",v_x);
        }
        else if(position[0]+distance>=EDGE[1]){
            v_x = std::min(std::abs(v_x)*std::tanh((EDGE[1]-position[0]+kf_safety_width) / speed_limit_weight), v_x);
            
        }
        if(position[1] <= EDGE[2] + distance){
            v_y = std::max(v_y, std::abs(v_y)*std::tanh((EDGE[2]-position[1]-kf_safety_width)/speed_limit_weight));
            
        }
        else if(position[1]+distance>=EDGE[3]){
            v_y = std::min(v_y,std::abs(v_y)*std::tanh((EDGE[3]-position[1]+kf_safety_width) / speed_limit_weight));
            
        }
        
        
    }
    
    void move_away_from_wall(float &v_x, float &v_y)
    {
        //RCLCPP_INFO(this->get_logger(), "Buffer:\n%s", bufferToString(motor_at_wall_buffer).c_str());
        
        if (wall_prot_activated[0] || std::any_of(motor_at_wall_buffer[0].begin(), motor_at_wall_buffer[0].end(), [](bool b) { return b; })) {
            v_x = std::max(0.0f, v_x);
            if (!wall_prot_activated[0]){
                RCLCPP_INFO(this->get_logger(), "Hitting against 1st wall");
            }
            wall_prot_activated[0] = true;
            
        }
        
        else if (wall_prot_activated[1] || std::any_of(motor_at_wall_buffer[1].begin(), motor_at_wall_buffer[1].end(), [](bool b) { return b; })) {
            v_x = std::min(0.0f, v_x);
            if (!wall_prot_activated[1]){
                RCLCPP_INFO(this->get_logger(), "Hitting against 2st wall");
            }
            wall_prot_activated[1] = true;
            
        }
        if (wall_prot_activated[2] || std::any_of(motor_at_wall_buffer[2].begin(), motor_at_wall_buffer[2].end(), [](bool b) { return b; })) {
            v_y = std::max(0.0f, v_y);
            if (!wall_prot_activated[2]){
                RCLCPP_INFO(this->get_logger(), "Hitting against 3st wall");
            }
            wall_prot_activated[2] = true;
            
        }
        else if (wall_prot_activated[3] || std::any_of(motor_at_wall_buffer[3].begin(), motor_at_wall_buffer[3].end(), [](bool b) { return b; })) {
            v_y = std::min(0.0f, v_y);
            if (!wall_prot_activated[3]){
                RCLCPP_INFO(this->get_logger(), "Hitting against 4st wall");
            }
            wall_prot_activated[3] = true;
            
        }
    }

    void send_commands(float v_x, float v_y)
    {
        // Update motor control messages
        
        
        odrive_can::msg::ControlMessage control_r_msg;
        odrive_can::msg::ControlMessage control_l_msg;
        // if (peg_mag_synchronized){
        //     apply_vel_hard_limits(v_x,v_y);
        // }
        control_r_msg.input_vel = sign*(v_x + v_y)*(2/DISTANCE_PER_REVOLUTION); // Remo Convention: s1
        control_r_msg.control_mode = 2;        // Velocity control mode
        control_r_msg.input_mode = 1;          // Velocity input mode
        control_l_msg.input_vel = sign*(v_y - v_x)*(2/DISTANCE_PER_REVOLUTION); // Remo Convention: s1
        control_l_msg.control_mode = 2;        // Velocity control mode
        control_l_msg.input_mode = 1;          // Velocity input mode
        motor_right_pub_->publish(control_r_msg);
        motor_left_pub_->publish(control_l_msg);

    }
    

    void controller_status_callback_right(const odrive_can::msg::ControllerStatus::SharedPtr msg)
    {
        
        encoder_values[0] = msg->pos_estimate;
        encoder_values[2] = msg->vel_estimate;
        torque_right_motor = msg->torque_estimate;
        
        if (std::sqrt(std::pow(torque_right_motor, 2) + std::pow(torque_left_motor, 2)) > MAX_TORQUE)
        {   
            if(std::sqrt(std::pow(torque_right_motor, 2) + std::pow(torque_left_motor, 2)) > SHUTDOWN_TORQUE){
                send_commands(0.0f,0.0f);
                RCLCPP_ERROR(this->get_logger(), "Shutdown torque surpassed");
                set_motor_state(1);
            }
            motor_at_wall = true;
            if (std::sqrt(std::pow(torque_right_motor, 2) + std::pow(torque_left_motor, 2)) > CALIB_TORQUE){
                motor_at_wall_calib = true;
            }
        }
        else
        {
            motor_at_wall = false;
            motor_at_wall_calib = false;
        }
        
        
        // During synchronization it has no synchronized state and can not update the value
        update_magnet_position(encoder_values);
        
        
        if (msg->active_errors != 0)
        {   
            send_commands(0.0f,0.0f);
            
            RCLCPP_ERROR(this->get_logger(), "Error detected in controller status: %d", msg->active_errors);
            std::this_thread::sleep_for(std::chrono::milliseconds(100000));
            set_motor_state(1);
            
        }
        
    }
    void controller_status_callback_left(const odrive_can::msg::ControllerStatus::SharedPtr msg)
    {
        
        encoder_values[1] = msg->pos_estimate;
        encoder_values[3] = msg->vel_estimate;
        torque_left_motor = msg->torque_estimate;
        
        if (std::sqrt(std::pow(torque_right_motor, 2) + std::pow(torque_left_motor, 2)) > MAX_TORQUE)
        {   
            if(std::sqrt(std::pow(torque_right_motor, 2) + std::pow(torque_left_motor, 2)) > SHUTDOWN_TORQUE){
                send_commands(0.0f,0.0f);
                RCLCPP_ERROR(this->get_logger(), "Shutdown torque surpassed");
                set_motor_state(1);
            }
            motor_at_wall = true;
            if (std::sqrt(std::pow(torque_right_motor, 2) + std::pow(torque_left_motor, 2)) > CALIB_TORQUE){
                motor_at_wall_calib = true;
            }
            
        }
        else
        {
            motor_at_wall = false;
            motor_at_wall_calib = false;
        }
        
        
        // During synchronization it has no synchronized state and can not update the value
        update_magnet_position(encoder_values);
        
        
        if (msg->active_errors != 0)
        {   
            send_commands(0.0f,0.0f);
            RCLCPP_ERROR(this->get_logger(), "Error detected in controller status: %d", msg->active_errors);
            set_motor_state(1);
            
        }
        //std_msgs::msg::Float32MultiArray moin;
        //moin.data = {std::sqrt(std::pow(torque_right_motor, 2) + std::pow(torque_left_motor, 2)), torque_left_motor, torque_right_motor};
        
        //position_magnet_publisher_->publish(moin);
    }
    void update_magnet_position(std::vector<float> encoder_values)
    {
        if(!synchronized_state.empty()){
            magnet_position[0] = synchronized_state[2] + (1.0f/2.0f)*((encoder_values[0] - synchronized_state[0]) * DISTANCE_PER_REVOLUTION - (encoder_values[1] - synchronized_state[1]) * DISTANCE_PER_REVOLUTION) * REAL_TO_CAM_FACTOR_X;
            magnet_position[1] = synchronized_state[3] + (1.0f/2.0f)*((encoder_values[0] - synchronized_state[0]) * DISTANCE_PER_REVOLUTION + (encoder_values[1] - synchronized_state[1]) * DISTANCE_PER_REVOLUTION) * REAL_TO_CAM_FACTOR_Y;
        }
        magnet_velocity[0] = (1.0f/2.0f)*(encoder_values[2] * DISTANCE_PER_REVOLUTION - encoder_values[3] * DISTANCE_PER_REVOLUTION) * REAL_TO_CAM_FACTOR_X;
        magnet_velocity[1] = (1.0f/2.0f)*(encoder_values[2] * DISTANCE_PER_REVOLUTION + encoder_values[3] * DISTANCE_PER_REVOLUTION) * REAL_TO_CAM_FACTOR_Y;
        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(4);
        msg.data[0] = magnet_position[0]; // prob too much cummulated inaccuracy to be useful
        msg.data[1] = magnet_position[1]; // prob too much cummulated inaccuracy to be useful
        msg.data[2] = magnet_velocity[0];
        msg.data[3] = magnet_velocity[1];
        position_magnet_publisher_->publish(msg);
    }
    void set_motor_state(int state)
    {
        auto request_r = std::make_shared<odrive_can::srv::AxisState::Request>();
        request_r->axis_requested_state = state;
        auto request_l = std::make_shared<odrive_can::srv::AxisState::Request>();
        request_l->axis_requested_state = state;
        
        // Wait for the service to be available and then call it
        while (!request_axis_r_state_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service /odrive_axis0/request_axis_state...");
        }
        while (!request_axis_l_state_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service /odrive_axis1/request_axis_state...");
        }
        
        auto future_r = request_axis_r_state_client_->async_send_request(request_r);
        auto future_l = request_axis_l_state_client_->async_send_request(request_l);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_r);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_l);
    }
};



class ODriveController : public rclcpp::Node
{
public:
    ODriveController() : Node("odrive_controller")
    {
        
        group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        player_ = std::make_shared<Player>("player");
        opponent_ = std::make_shared<Player>("opponent");
        //CONVENTION: RIGHT SIDE IS THE PLAYER, LEFT SIDE IS THE OPPONENT. TOP SIDE IS RIGHT MOTOR FOR THE RIGHT SIDE AND LEFT MOTOR FOR THE LEFT SIDE
        
    

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = group_;


        states_subscriber_ = this->create_subscription<klask_interfaces::msg::StampedPolygon>("/ball_peg_states", 1, std::bind(&ODriveController::state_callback, this, std::placeholders::_1),sub_options);

        
        velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "velocity_requests_checked", 1, std::bind(&ODriveController::velocity_callback, this, std::placeholders::_1),sub_options);

        

        // Shared subscriptions or logic can go here
    }
    Player::SharedPtr get_player_node(){
        return this->player_;
    }
    Player::SharedPtr get_opponent_node(){
        return this->opponent_;
    }

private:
    std::shared_ptr<Player> player_;
    std::shared_ptr<Player> opponent_;

    rclcpp::CallbackGroup::SharedPtr group_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<klask_interfaces::msg::StampedPolygon>::SharedPtr states_subscriber_;

    void state_callback(klask_interfaces::msg::StampedPolygon::SharedPtr msg)
    {   
        player_->position = {msg->polygon.points[4].x, msg->polygon.points[4].y};
        player_->velocity = {msg->polygon.points[5].x, msg->polygon.points[5].y};
        opponent_->position = {msg->polygon.points[2].x, msg->polygon.points[2].y};
        opponent_->velocity = {msg->polygon.points[3].x, msg->polygon.points[3].y};
        if(player_->synchronized_once){
            player_->is_synchronized(); //before synchronization the cam. gives bad measurements and this would see the peg synchronized
        };
        if(opponent_->synchronized_once){
            opponent_->is_synchronized();
        };
        if(player_->peg_mag_synchronized &&!player_->synchronized_state.empty()){ //this code snipped can only be used once a reliable synch detecion is implemented
            player_->synchronized_state[0] = player_->encoder_values[0];
            player_->synchronized_state[1] = player_->encoder_values[1];
            player_->synchronized_state[2] = player_->position[0];
            player_->synchronized_state[3] = player_->position[1];
        }
        if(opponent_->peg_mag_synchronized &&!opponent_->synchronized_state.empty()){ //this code snipped can only be used once a reliable synch detecion is implemented
            opponent_->synchronized_state[0] = opponent_->encoder_values[0];
            opponent_->synchronized_state[1] = opponent_->encoder_values[1];
            opponent_->synchronized_state[2] = opponent_->position[0];
            opponent_->synchronized_state[3] = opponent_->position[1];
        }
    }
    

    void velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {   
        //player_->v_y = msg->data[3]; // Remo Convention: y for peg 1 (no minus has to be added, because the motors are facing to the ground.)
        //player_->v_x = msg->data[2]; // Remo Convention: x for peg 1 (no minus has to be added, because the motors are facing to the ground.)
        //opponent_->v_y = msg->data[1]; 
        //opponent_->v_x = msg->data[0];
        player_->velocity_targets(msg->data[2], msg->data[3]);
        opponent_->velocity_targets(msg->data[0], msg->data[1]);

    }

    
};
    
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ODriveController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    executor.add_node(node->get_player_node());
    
    executor.add_node(node->get_opponent_node());

    executor.spin();
    rclcpp::shutdown();
    return 0;
}