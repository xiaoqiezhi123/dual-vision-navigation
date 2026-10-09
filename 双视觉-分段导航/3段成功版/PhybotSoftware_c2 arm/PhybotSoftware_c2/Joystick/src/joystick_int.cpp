#include "Joystick/include/joystick_int.h"
#include <iostream>

Joystick::Joystick() {
    std::string file_path = "../Joystick/config/joystick.yaml";
    YAML::Node config = YAML::LoadFile(file_path);
    acc_max = config["acc_max"].as<double>();
    kp_js_vx = config["kp_js_vx"].as<double>();
    angacc_max = config["angacc_max"].as<double>();
}

Joystick::~Joystick() {
        // if (xbox_thread.joinable()) {
        //     xbox_thread.join();
        // }
}

void Joystick::SetDataToPackage(DataPackage &DataPackage){

    DataPackage.NextState = NextState;


    DataPackage.js_vx_desire = js_vx_desire;
    DataPackage.js_OmegaZ_desire = js_OmegaZ_desire;
    DataPackage.js_vy_desire = js_vy_desire;
    DataPackage.control_mode = control_mode;


}

void Joystick::GetDataFromPackage(DataPackage &DataPackage){

    control_period = DataPackage.control_period;
    speed_forward_avg = DataPackage.speed_forward_avg;
    js_OmegaZ_control = DataPackage.js_OmegaZ_desire;

}

int Joystick::xbox_open(const char *file_name) {
    int xbox_fd;

    xbox_fd = open(file_name, O_RDONLY);
    if (xbox_fd < 0) {
        perror("open");
        return -1;
    }

    return xbox_fd;
}

int Joystick::xbox_map_read(xbox_map_t *map) {
    int len, type, number, value;
    struct js_event js;

    len = read(xbox_fd, &js, sizeof(struct js_event));
    if (len < 0) {
        perror("read");
        return -1;
    }

    type = js.type;
    number = js.number;
    value = js.value;

    map->time = js.time;

    if (type == JS_EVENT_BUTTON) {
        switch (number) {
        case XBOX_BUTTON_A:
            map->a = value;
            break;

        case XBOX_BUTTON_B:
            map->b = value;
            break;

        case XBOX_BUTTON_X:
            map->x = value;
            break;

        case XBOX_BUTTON_Y:
            map->y = value;
            break;

        case XBOX_BUTTON_LB:
            map->lb = value;
            break;

        case XBOX_BUTTON_RB:
            map->rb = value;
            break;

        case XBOX_BUTTON_START:
            map->start = value;
            break;

        case XBOX_BUTTON_BACK:
            map->back = value;
            break;

        case XBOX_BUTTON_HOME:
            map->home = value;
            break;

        case XBOX_BUTTON_LO:
            map->lo = value;
            break;

        case XBOX_BUTTON_RO:
            map->ro = value;
            break;

        default:
            break;
        }
    } else if (type == JS_EVENT_AXIS) {
        switch (number) {
        case XBOX_AXIS_LX:
            map->lx = value;
            break;

        case XBOX_AXIS_LY:
            map->ly = value;
            break;

        case XBOX_AXIS_RX:
            map->rx = value;
            break;

        case XBOX_AXIS_RY:
            map->ry = value;
            break;

        case XBOX_AXIS_LT:
            map->lt = value;
            break;

        case XBOX_AXIS_RT:
            map->rt = value;
            break;

        case XBOX_AXIS_XX:
            map->xx = value;
            break;

        case XBOX_AXIS_YY:
            map->yy = value;
            break;

        default:
            break;
        }
    } else {
        /* Init do nothing */
    }

    return len;
}

void Joystick::get_state_change() {


    if (xbox_m.y == 1.0) {

        NextState = State::ZERO;

    } 
    else if (xbox_m.b == 1.0) {

        NextState = State::RL_walk;

    } 

    else if (xbox_m.x == 1.0) {

        NextState = State::RL_mimic;

    } 

}

void Joystick::get_control_mode() {
    if (xbox_m.a == 1.0) {
        control_mode = 0;
    } else if (xbox_m.y == 1.0) {
        control_mode = 1;
}
}

void Joystick::xbox_close(void) {
    close(xbox_fd);
    return;
}

int Joystick::xbox_init(void) {
    int len, type;
    int axis_value, button_value;
    int number_of_axis, number_of_buttons;

    xbox_fd = xbox_open("/dev/input/js0");
    if (xbox_fd < 0) {
        return -1;
    }

    return 0;
}

int Joystick::xbox_read(xbox_map_t *xbox_m) {

    int len = xbox_map_read(xbox_m);
    if (len < 0) {
        return -1;
    }
    return 0;
}

void Joystick::init() {
    int ret = -1;
    xbox_m.a = 0.0;
    xbox_m.b = 0.0;
    xbox_m.x = 0.0;
    xbox_m.y = 0.0;
    xbox_m.lx = 0.0;
    xbox_m.ly = 0.0;
    xbox_m.rx = 0.0;
    xbox_m.ry = 0.0;
    xbox_m.xx = 0.0;
    xbox_m.yy = 0.0;
    xbox_m.lt = -32767;
    current_fsmstate_command = "";
    // ret = pthread_create(&Joystick::xbox_thread, NULL, Joystick::xbox_run, NULL);
    xbox_thread = std::thread(&Joystick::xbox_run, this);
}

void Joystick::xbox_run() {
    int len, ret;

    ret = xbox_init();
    if (ret < 0) {
        printf("xbox init fail\n");
    }

    while (ret == 0) {

        std::lock_guard<std::mutex> lock(data_mutex); // 加锁
        len = xbox_read(&xbox_m);
        if (len < 0) {
            // usleep(10*1000);
            continue;
        }

        // printf("Time:%8d A:%d B:%d X:%d Y:%d LB:%  m.b, xbox_m.x, xbox_m.y, xbox_m.lb, xbox_m.rb, xbox_m.start, xbox_m.back, xbox_m.home, xbox_m.lo, xbox_m.ro,
        //         xbox_m.xx, xbox_m.yy, xbox_m.lx, xbox_m.ly, xbox_m.rx, xbox_m.ry, xbox_m.lt, xbox_m.rt);
        // fflush(stdout);

        usleep(10 * 1000);
    }
}


double Joystick::get_walk_x_direction_speed() {
    int lt_value = xbox_m.lt;
    if (lt_value < 1000) {
        int x_value = xbox_m.ly;
        if ((abs(x_value) > deadarea) && (abs(x_value) <= maxvalue)) {
            if (x_value > 0) {
                return ly_dir * minspeed_x * ((double)(abs(x_value) - deadarea) / (maxvalue - deadarea));
            } else {
                return ly_dir * maxspeed_x * ((double)(abs(x_value) - deadarea) / (maxvalue - deadarea));
            }
        } else {
            return 0.0;
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_walk_yaw_direction_speed() {
    int yaw_value = xbox_m.rx;
    if ((abs(yaw_value) > deadarea) && (abs(yaw_value) <= maxvalue)) {
        if (yaw_value > 0) {
            return rx_dir * maxspeed_yaw * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
        } else {
            return rx_dir * minspeed_yaw * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_stand_up_pos() {
    int pos_value = xbox_m.ry;
    if ((abs(pos_value) > deadarea) && (abs(pos_value) <= maxvalue)) {
        if (pos_value > 0) {
            return rx_dir * max_stand_up_pos * ((double)(abs(pos_value) - deadarea) / (maxvalue - deadarea));
        } else {
            return rx_dir * min_stand_up_pos * ((double)(abs(pos_value) - deadarea) / (maxvalue - deadarea));
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_stand_forward_pos() {
    int pos_value = xbox_m.ly;
    if ((abs(pos_value) > deadarea) && (abs(pos_value) <= maxvalue)) {
        if (pos_value > 0) {
            return ly_dir * max_stand_forward_pos * ((double)(abs(pos_value) - deadarea) / (maxvalue - deadarea));
        } else {
            return ly_dir * min_stand_forward_pos * ((double)(abs(pos_value) - deadarea) / (maxvalue - deadarea));
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_stand_left_pos() {
    int pos_value = xbox_m.lx;
    if ((abs(pos_value) > deadarea) && (abs(pos_value) <= maxvalue)) {
        if (pos_value > 0) {
            return lx_dir * max_stand_left_pos * ((double)(abs(pos_value) - deadarea) / (maxvalue - deadarea));
        } else {
            return lx_dir * min_stand_left_pos * ((double)(abs(pos_value) - deadarea) / (maxvalue - deadarea));
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_stand_yaw_pos() {
    int yaw_value = xbox_m.rx;
    if ((abs(yaw_value) > deadarea) && (abs(yaw_value) <= maxvalue)) {
        if (yaw_value > 0) {
            return rx_dir * max_stand_yaw_pos * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
        } else {
            return rx_dir * min_stand_yaw_pos * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_stand_up_vel() {
    int vel_value = xbox_m.ry;
    if ((abs(vel_value) > deadarea) && (abs(vel_value) <= maxvalue)) {
        if (vel_value > 0) {
            // return rx_dir * max_stand_up_vel * ((double)(abs(vel_value) - deadarea) / (maxvalue - deadarea));
            return rx_dir * max_stand_up_vel ;
        } else {
            // return rx_dir * min_stand_up_vel * ((double)(abs(vel_value) - deadarea) / (maxvalue - deadarea));
            return rx_dir * min_stand_up_vel ;
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_stand_forward_vel() {
    int vel_value = xbox_m.ly;
    if ((abs(vel_value) > deadarea) && (abs(vel_value) <= maxvalue)) {
        if (vel_value > 0) {
            return ly_dir * max_stand_forward_vel * ((double)(abs(vel_value) - deadarea) / (maxvalue - deadarea));
        } else {
            return ly_dir * min_stand_forward_vel * ((double)(abs(vel_value) - deadarea) / (maxvalue - deadarea));
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_stand_left_vel() {
    int vel_value = xbox_m.lx;
    if ((abs(vel_value) > deadarea) && (abs(vel_value) <= maxvalue)) {
        if (vel_value > 0) {
            return lx_dir * max_stand_left_vel * ((double)(abs(vel_value) - deadarea) / (maxvalue - deadarea));
        } else {
            return lx_dir * min_stand_left_vel * ((double)(abs(vel_value) - deadarea) / (maxvalue - deadarea));
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_stand_yaw_vel() {
    int yaw_value = xbox_m.rx;
    if ((abs(yaw_value) > deadarea) && (abs(yaw_value) <= maxvalue)) {
        if (yaw_value > 0) {
            return rx_dir * max_stand_yaw_vel * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
        } else {
            return rx_dir * min_stand_yaw_vel * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_walk_x_direction_speed_offset() {
    int x_value = xbox_m.yy;
    if ((x_value < -30000) && (abs(last_value_x) < 500)) {
        last_value_x = x_value;
        return deltoffset_x;
    } else if ((x_value > 30000) && (abs(last_value_x) < 500)) {
        last_value_x = x_value;
        return -deltoffset_x;
    } else {
        last_value_x = x_value;
        return 0.0;
    }
}

double Joystick::get_walk_y_direction_speed() {
    int y_value = xbox_m.lx;
    if ((abs(y_value) > deadarea) && (abs(y_value) <= maxvalue)) {
        if (y_value > 0) {
            return lx_dir * maxspeed_y * ((double)(abs(y_value) - deadarea) / (maxvalue - deadarea));
        } else {
            return lx_dir * minspeed_y * ((double)(abs(y_value) - deadarea) / (maxvalue - deadarea));
        }
    } else {
        return 0.0;
    }
}

double Joystick::get_walk_y_direction_speed_offset() {
    int y_value = xbox_m.xx;
    if ((y_value < -30000) && (abs(last_value_y) < 500)) {
        last_value_y = y_value;
        return deltoffset_y;
    } else if ((y_value > 30000) && (abs(last_value_y) < 500)) {
        last_value_y = y_value;
        return -deltoffset_y;
    } else {
        last_value_y = y_value;
        return 0.0;
    }
}

void Joystick::get_speed(){

    js_vx_desire = get_walk_x_direction_speed() ;


    double delta_vx_max = acc_max * control_period;
    double delta_vx_control = js_vx_feedback - js_vx_control;

    if(abs(delta_vx_control) <= delta_vx_max){

        js_vx_control = js_vx_feedback;

    }
    else if(delta_vx_control>0){

        js_vx_control += delta_vx_max;

    }else{
        js_vx_control -= delta_vx_max;
    }

    js_OmegaZ_desire = get_walk_yaw_direction_speed();

    double delta_OmegaZ_max = angacc_max * control_period;
    double delta_OmegaZ_control = js_OmegaZ_desire - js_OmegaZ_control;

    if(abs(delta_OmegaZ_control) <= delta_OmegaZ_max){

        js_OmegaZ_control = js_OmegaZ_desire;

    }
    else if(delta_OmegaZ_control>0){

        js_OmegaZ_control += delta_OmegaZ_max;

    }else{
        js_OmegaZ_control -= delta_OmegaZ_max;
    }

    js_vy_offset = js_vy_offset + get_walk_y_direction_speed_offset();
    js_vy_desire = get_walk_y_direction_speed() + js_vy_offset;


}



void Joystick::run(){
    get_speed();
    get_state_change();
    get_control_mode();
    // change_state_by_speed();

    
}

void Joystick::SetNextState(State state) {
    std::lock_guard<std::mutex> lock(data_mutex);
    NextState = state;
}
