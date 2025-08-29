#include <rclcpp/rclcpp.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include "get_Regression.hpp"
#include "OpenXLSX/OpenXLSX.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace OpenXLSX;

class Flint_Node : public rclcpp::Node
{
    public:
        Flint_Node(std::string name) : Node(name)
        {
            char error[1024] = {0};
            // load ur5e model
            model_ = mj_loadXML("/home/xiatenghui/work_space/mujoco_ws/src/flintrobot_description/urdf/flintrobot.xml", nullptr, error, sizeof(error));
            if(!model_)
            {
                // 模型加载失败处理
                RCLCPP_ERROR(this->get_logger(), "无法加载模型: %s", error);
                throw std::runtime_error("模型加载失败");
            }

            // 读取xlsx文件
            XLDocument PA;
            XLDocument fourier_series;
            PA.open("/home/xiatenghui/work_space/mujoco_ws/src/mujoco_sim/src/PA.xlsx");
            fourier_series.open("/home/xiatenghui/work_space/mujoco_ws/src/mujoco_sim/src/fourier_series_flint.xlsx");

            XLWorksheet PA_sheet = PA.workbook().worksheet("Sheet1");
            XLWorksheet fourier_series_sheet = fourier_series.workbook().worksheet("Sheet1");

            timer_ = this->create_wall_timer(10ms,std::bind(&Flint_Node::timer_callback, this));

            int PA_Row = PA_sheet.rowCount();
            for(int row = 1;row <= PA_Row;row++)
            {
                auto cell = PA_sheet.cell(row, 1);
                auto& value = cell.value();
                if (value.type() == OpenXLSX::XLValueType::Empty) continue;

                // 处理数值
                if (value.type() == OpenXLSX::XLValueType::Float || 
                    value.type() == OpenXLSX::XLValueType::Integer) {
                    double num = value.get<double>();
                    PA_[row-1] = num;
                }                 
            }

            int fourier_series_Row = fourier_series_sheet.rowCount();
            for(int row = 1;row <= fourier_series_Row;row++)
            {
                auto cell = fourier_series_sheet.cell(row, 1);
                auto& value = cell.value();
                if (value.type() == OpenXLSX::XLValueType::Empty) continue;

                // 处理数值
                if (value.type() == OpenXLSX::XLValueType::Float || 
                    value.type() == OpenXLSX::XLValueType::Integer) {
                    double num = value.get<double>();
                    fourier_series_.push_back(num);
                }
            }

            Eigen::Map<Eigen::Matrix<double, 11, 8, Eigen::ColMajor>> fourier_series_data(fourier_series_.data());
            for(int i = 0;i < joint_nums;i++)
            {
                std::array<double, 5> A;
                std::array<double, 5> B;
                for(int j = 0;j < 5;j++)
                {
                    A[j] = fourier_series_data(j, i);
                    B[j] = fourier_series_data(j+5, i);
                }
                A_.push_back(A);
                B_.push_back(B);
                C_[i] = fourier_series_data(10, i);
            }

            // 创建仿真数据结构
            data_ = mj_makeData(model_);
            RCLCPP_INFO(this->get_logger(), "模型加载成功");

            // 初始化仿真参数
            model_->opt.gravity[2] = -9.81;
            force_scale = 5;

            // 初始化位置到初始状态
            joint_names = {
                "joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
            };

            sensor_names = {
                "sensor_joint0_torque", "sensor_joint1_torque", "sensor_joint2_torque", "sensor_joint3_torque",
                "sensor_joint4_torque", "sensor_joint5_torque", "sensor_joint6_torque", "sensor_joint7_torque"
            };

            // 控制器回调函数注册
            data_->userdata = (mjtNum*)this;
            mjcb_control = [](const mjModel* m, mjData* d) {
                auto* self = (Flint_Node*)(d->userdata);
                self->apply_position_control();
            };

            for (int i = 0; i < joint_names.size(); i++)
            {
                int joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_names[i].c_str());
                int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, sensor_names[i].c_str());
                if (joint_id != -1) 
                {
                    // 设置初始位置为0
                    data_->qpos[model_->jnt_qposadr[joint_id]] = 0.0;
                    
                    // 初始化控制目标值
                    target_positions_[i] = 0.0;
                    target_velocities_[i] = 0.0;

                    // 关节ID映射
                    joint_id_map[i] = joint_id;
                    sensor_id_map[i] = sensor_id;

                    qpos_adr_[i] = model_->jnt_qposadr[joint_id];
                    qvel_adr_[i] = model_->jnt_dofadr[joint_id];
                }
            }
            target_positions_[1] = target_positions_[1] + M_PI/2.0;
            target_positions_[2] = target_positions_[2] + M_PI/2.0;
            target_positions_[3] = target_positions_[3] + M_PI/2.0;
            target_positions_[4] = target_positions_[4] + M_PI/2.0;
            target_positions_[6] = target_positions_[6] + M_PI/2.0;

            // 创建状态发布器
            joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            target_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("target_states", 10);
        }

        ~Flint_Node() 
        {
            if(data_) mj_deleteData(data_);
            if(model_) mj_deleteModel(model_);
            if(window_) glfwDestroyWindow(window_);
            glfwTerminate();
        }

        void run()
        {
            std::thread render_thread(&Flint_Node::rendering_thread, this);
            // 主仿真循环
            double last_update = glfwGetTime();
            rclcpp::Time current_time = this->now();
            start_time = current_time.seconds();
            while (render_active_)
            {
                // 计算时间步长
                double now = glfwGetTime();
                double dt = now - last_update;
                last_update = now;
                // RCLCPP_INFO(get_logger(), "dt : %f", dt);
                
                {
                    std::lock_guard<std::mutex> lock();
                    // 物理仿真步进
                    mj_step(model_, data_);
                }

                // 500Hz的控制频率
                double elapse = glfwGetTime() - now;
                if(elapse < 0.002)
                {
                    std::this_thread::sleep_for(std::chrono::duration<double>(0.002- elapse));
                }
                // RCLCPP_INFO(get_logger(), "施加扰动力");
                // RCLCPP_INFO(get_logger(), "force_vector[0] %f", force_vector[0]);
                // RCLCPP_INFO(get_logger(), "force_vector[1] %f", force_vector[1]);
            }

            // 关闭glfw
            glfwTerminate();
            // 清理渲染资源
            mjr_freeContext(&con);
            mjv_freeScene(&scn);
        }

        void rendering_thread()
        {
            // 创建GLFW窗口
            if (!glfwInit()) 
            {
                RCLCPP_ERROR(get_logger(), "GLFW初始化失败");
                return;
            }

            window_ = glfwCreateWindow(1200, 900, "UR5e Simulation", NULL, NULL);
            glfwSetWindowUserPointer(window_, this);
            if (!window_) 
            {
                RCLCPP_ERROR(get_logger(), "无法创建GLFW窗口");
                glfwTerminate();
                return;
            }

            // 设置OpenGL上下文
            glfwMakeContextCurrent(window_);
            glfwSwapInterval(1);  // 开启垂直同步
            
            // 初始化渲染
            init_rendering(&scn, &con, &cam);

            // 初始化扰动器
            mjv_defaultPerturb(&pert);
            pert.active = mjPERT_TRANSLATE; // 激活扰动
            pert.select = mj_name2id(model_, mjOBJ_BODY, "right_finger4_joint2"); // 目标部位ID
            mjv_initPerturb(model_, data_, &scn, &pert);
            mjv_applyPerturbForce(model_, data_, &pert);

            // 注册回调函数
            auto keyboard = [](GLFWwindow* window, int key, int scancode, int act, int mods) {
                auto* self = static_cast<Flint_Node*>(glfwGetWindowUserPointer(window));
                self->keyboard(window, key, scancode, act, mods);
            };
            auto mouse_button = [](GLFWwindow* window, int button, int act, int mods) {
                auto* self = static_cast<Flint_Node*>(glfwGetWindowUserPointer(window));
                self->mouse_button(window, button, act, mods);
            };
            auto mouse_move = [](GLFWwindow* window, double xpos, double ypos) {
                auto* self = static_cast<Flint_Node*>(glfwGetWindowUserPointer(window));
                self->mouse_move(window, xpos, ypos);
            };
            glfwSetKeyCallback(window_, keyboard);
            glfwSetMouseButtonCallback(window_, mouse_button);
            glfwSetCursorPosCallback(window_, mouse_move);
            while (!glfwWindowShouldClose(window_))
            {
                {
                    std::lock_guard<std::mutex> lock();
                    // 渲染
                    render_frame(&scn, &con, &cam);
                }
            }
            render_active_ = 0;
        }

        void apply_position_control() 
        {
            // 读取当前角度和速度
            for(uint8_t i = 0;i < joint_nums;i++)
            {
                current_positions[i] = data_->qpos[qpos_adr_[i]];
                current_velocities[i] = data_->qvel[qvel_adr_[i]];
                effort[i] = data_->sensordata[sensor_id_map[i]];
            }
            current_positions[1] = current_positions[1] + M_PI/2.0;
            current_positions[2] = current_positions[2] + M_PI/2.0;
            current_positions[3] = current_positions[3] + M_PI/2.0;
            current_positions[4] = current_positions[4] + M_PI/2.0;
            current_positions[6] = current_positions[6] + M_PI/2.0;
            rclcpp::Time control_time = this->now();
            double current_time = control_time.seconds();
            double time = current_time - start_time;
            for(int i = 0;i < joint_nums;i++)
            {
                std::array<double, 3> target;
                target = fourier_serise(time, A_[i], B_[i], C_[i]);
                target_positions_[i] = target[0];
                target_velocities_[i] = target[1];
            }

            // for(uint8_t i = 0;i < joint_nums;i++)
            // {
            //     RCLCPP_INFO(get_logger(), "current_positions[%d] : %f", i, current_positions[i]);
            // }
            // double calculate_result[288];
            // get_Regression(calculate_result, current_positions, {0,0,0,0,0,0}, {0,0,0,0,0,0});
            // Eigen::Map<Eigen::Matrix<double, 6, 48, Eigen::RowMajor>> Regression_Matrix(calculate_result);
            // Eigen::VectorXd force = Regression_Matrix * PA_;
            // RCLCPP_INFO(get_logger(), "calculate force result");
            // for(uint8_t i = 0;i < joint_nums;i++)
            // {
            //     RCLCPP_INFO(get_logger(), "force %d : %f", i, force[i]);
            //     data_->ctrl[joint_id_map[i]] = force[i];
            // }
            PID_control(0, 50, 5);
            PID_control(1, 40, 20);
            PID_control(2, 20, 2);
            PID_control(3, 30, 5);
            PID_control(4, 30, 3);
            PID_control(5, 20, 2);
            PID_control(6, 10, 1.2);
            PID_control(7, 10, 1.2);

            // RCLCPP_INFO(get_logger(), "force0 %f", data_->ctrl[joint_id_map[0]]);
            // RCLCPP_INFO(get_logger(), "force1 %f", data_->ctrl[joint_id_map[1]]);
        }

        void PID_control(uint8_t id,uint8_t kp, uint8_t kd)
        {
            double position_error = target_positions_[id] - current_positions[id];
            double velocity_error = target_velocities_[id] - current_velocities[id];
            double force = kp * position_error + kd * velocity_error;

            // 应用力到执行器
            data_->ctrl[joint_id_map[id]] = force;

        }

        void init_rendering(mjvScene* scn, mjrContext* con, mjvCamera* cam) 
        {
            // 初始化场景            
            mjv_defaultScene(scn);
            mjv_defaultCamera(cam);
            mjv_defaultOption(&vopt_);
            mjr_defaultContext(con);

            // 打开扰动力的可视化
            vopt_.flags[mjVIS_PERTFORCE] = 1;
            // 调整相机位置
            cam->distance = 3;
            cam->elevation = -40;
            cam->azimuth = 45;
            cam->type = mjCAMERA_FREE;  // 自由相机模式

            // 创建OpenGL上下文
            mjv_makeScene(model_, scn, 1000);
            mjr_makeContext(model_, con, mjFONTSCALE_100);
            
            // 设置视角
            mjv_updateScene(model_, data_, &vopt_, NULL, cam, mjCAT_ALL, scn);

        }

        void render_frame(mjvScene* scn, mjrContext* con, mjvCamera* cam) 
        {
            // 获取窗口尺寸
            glfwGetFramebufferSize(window_, &rect_.width, &rect_.height);
            
            // 更新场景
            mjv_updateScene(model_, data_, &vopt_, NULL, cam, mjCAT_ALL, scn);
            
            // 渲染场景
            mjr_render(rect_, scn, con);
            
            // 交换缓冲区
            glfwSwapBuffers(window_);

            // 处理用户输出与窗口事件
            glfwPollEvents();
        }

        void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) 
        {
            if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) 
            {
                mj_resetData(model_, data_);
                mj_forward(model_, data_);
            }
            else if(act==GLFW_PRESS && key==GLFW_KEY_ENTER)
            {
                int body_id = mj_name2id(model_, mjOBJ_BODY, "right_finger4_joint2");
                data_->xfrc_applied[6 * body_id + 0] = 0;
                data_->xfrc_applied[6 * body_id + 1] = 0;
                data_->xfrc_applied[6 * body_id + 2] = 0;
            }
        }

        // 鼠标按键回调
        void mouse_button(GLFWwindow* window, int button, int act, int mods) 
        {
            // update button state
            button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
            button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
            button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

            // update mouse position
            glfwGetCursorPos(window, &lastx, &lasty);
        }

        // mouse move callback
        void mouse_move(GLFWwindow* window, double xpos, double ypos) 
        {
            // no buttons down: nothing to do
            if (!button_left && !button_middle && !button_right) {
                return;
            }

            // compute mouse displacement, save
            double dx = xpos - lastx;
            double dy = ypos - lasty;
            lastx = xpos;
            lasty = ypos;

            // get current window size
            int width, height;
            glfwGetWindowSize(window, &width, &height);

            // get shift key state
            bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                                glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

            // get ctrl key state
            bool mod_ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL)==GLFW_PRESS ||
                                glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL)==GLFW_PRESS);
            
            // determine action based on mouse button
            mjtMouse action;
            if (button_right) {
                action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            } else if (button_left) {
                action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            } else {
                action = mjMOUSE_ZOOM;
            }

            int body_id = mj_name2id(model_, mjOBJ_BODY, "right_finger4_joint2");
            // 施加扰动力   
            if(mod_ctrl && body_id != -1)
            {
                force_vector[0] = dx * force_scale;
                force_vector[1] = dy * force_scale;
                force_vector[2] = 0.0;
                data_->xfrc_applied[6 * body_id + 0] = force_vector[0];  // Fx
                data_->xfrc_applied[6 * body_id + 1] = force_vector[1];  // Fy
                data_->xfrc_applied[6 * body_id + 2] = force_vector[2];  // Fz
            }
            else
            {
                // move camera
                mjv_moveCamera(model_, action, dx/height, dy/height, &scn, &cam);
            }
            
        }

        void timer_callback() 
        {
            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->now();
            message.position.resize(8);
            message.velocity.resize(8);
            message.effort.resize(8);

            auto message1 = sensor_msgs::msg::JointState();
            message1.header.stamp = this->now();
            message1.position.resize(8);
            message1.velocity.resize(8);

            for(int i = 0;i < joint_nums;i++)
            {
                message.position[i] = current_positions[i];
                message.velocity[i] = current_velocities[i];
                message.effort[i] = effort[i];
                message1.position[i] = target_positions_[i];
                message1.velocity[i] = target_velocities_[i];
            }
            
            joint_state_pub_->publish(message);
            target_state_pub_->publish(message1);
        }

        std::array<double, 3> fourier_serise(double t, std::array<double, 5> A, std::array<double, 5> B,double C)
        {
            double q = C, qd = 0, qdd = 0;
            double w = 2 * M_PI * 0.1;
            for(int k = 1;k <= 5;k++)
            {
                q +=  A[k-1]/(w*k) * sin(k*w*t) - B[k-1]/(w*k) * cos(k*w*t);
                qd += A[k-1] * cos(k*w*t) + B[k-1] * sin(k*w*t);
                qdd += -A[k-1] * k * w * sin(k*w*t) + B[k-1] * k * w * cos(k*w*t);
            }
            std::array<double, 3> target;
            target[0] = q;
            target[1] = qd;
            target[2] = qdd;
            return target;
        }

        std::array<double, 8> target_positions_;
        std::array<double, 8> target_velocities_;

    private:
        mjModel* model_ = nullptr;
        mjData* data_ = nullptr;
        GLFWwindow* window_ = nullptr;

        std::vector<std::string> joint_names;
        std::vector<std::string> sensor_names;
        uint8_t joint_nums = 8;
        double start_time;
        // 关节ID映射表
        std::map<int, int> joint_id_map;
        std::map<int, int> sensor_id_map;

        // ROS 接口
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_state_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr control_sub_;
        
        // 渲染配置变量
        mjvCamera cam;
        mjvScene scn;
        mjrContext con;
        mjvPerturb pert;
        mjvOption vopt_;  // 使用默认选项

        // 渲染区域
        mjrRect rect_ = {0};

        // 鼠标交互
        bool button_left = false;
        bool button_middle = false;
        bool button_right =  false;
        double lastx = 0;
        double lasty = 0;

        // 控制变量
        uint8_t force_scale;  // 力缩放系数
        double force_vector[3];  //扰动力

        // 关节的位置和速度地址
        std::array<int, 8> qpos_adr_;
        std::array<int, 8> qvel_adr_;
        std::array<double, 8> current_positions;
        std::array<double, 8> current_velocities;
        std::array<double, 8> effort;
        
        // 互斥锁保护共享数据访问
        std::mutex data_mutex_;
        uint8_t render_active_ = 1;

        // 最小惯性参数集和傅里叶级数系数
        Eigen::VectorXd PA_ = Eigen::VectorXd::Zero(48);
        std::vector<double> fourier_series_;
        std::vector<std::array<double, 5>> A_;
        std::vector<std::array<double, 5>> B_;
        std::array<double, 5> C_;

        rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto flint_Node = std::make_shared<Flint_Node>("Flint_Node");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(flint_Node);
    std::thread spin_thread([&executor]() {
        executor.spin();
    });
    flint_Node->run();
    rclcpp::shutdown();
    return 0;
}