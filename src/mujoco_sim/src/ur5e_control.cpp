#include <rclcpp/rclcpp.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

class Ur5e_Node : public rclcpp::Node
{
    public:
        Ur5e_Node(std::string name) : Node(name)
        {
            char error[1024] = {0};
            // load ur5e model
            model_ = mj_loadXML("/home/xiatenghui/mujoco/ur5e_description/urdf/ur5e_description.xml", nullptr, error, sizeof(error));
            if(!model_)
            {
                // 模型加载失败处理
                RCLCPP_ERROR(this->get_logger(), "无法加载模型: %s", error);
                throw std::runtime_error("模型加载失败");
            }

            // 创建仿真数据结构
            data_ = mj_makeData(model_);
            RCLCPP_INFO(this->get_logger(), "模型加载成功");

            // 初始化仿真参数
            model_->opt.gravity[2] = -9.81;

            // 初始化位置到初始状态
            joint_names = {
                "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
            };

            // 控制器回调函数注册
            data_->userdata = (mjtNum*)this;
            mjcb_control = [](const mjModel* m, mjData* d) {
                auto* self = (Ur5e_Node*)(d->userdata);
                self->apply_position_control();
            };

            for (int i = 0; i < joint_names.size(); i++)
            {
                int joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_names[i].c_str());
                if (joint_id != -1) 
                {
                // 设置初始位置为0
                    data_->qpos[model_->jnt_qposadr[joint_id]] = 0.0;
                    
                    // 初始化控制目标值
                    target_positions_[joint_id] = 0.0;
                    target_velocities_[joint_id] = 0.0;
                }
            }

            // 创建状态发布器
            joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        }

        ~Ur5e_Node() 
        {
            if(data_) mj_deleteData(data_);
            if(model_) mj_deleteModel(model_);
            if(window_) glfwDestroyWindow(window_);
            glfwTerminate();
        }

        void run()
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

            // 注册回调函数
            auto keyboard = [](GLFWwindow* window, int key, int scancode, int act, int mods) {
                auto* self = static_cast<Ur5e_Node*>(glfwGetWindowUserPointer(window));
                self->keyboard(window, key, scancode, act, mods);
            };
            glfwSetKeyCallback(window_, keyboard);

            // 主仿真循环
            double last_update = glfwGetTime();
            while (!glfwWindowShouldClose(window_))
            {
                // 计算时间步长
                double now = glfwGetTime();
                double dt = now - last_update;
                last_update = now;
                // RCLCPP_INFO(get_logger(), "dt : %f", dt);
                
                // 物理仿真步进
                mj_step(model_, data_);

                // 渲染
                render_frame(&scn, &con, &cam);
            }

            // 关闭glfw
            glfwTerminate();
            // 清理渲染资源
            mjr_freeContext(&con);
            mjv_freeScene(&scn);
        }

        void apply_position_control() 
        {
            // 位置控制器
            for (auto& [joint_id, target] : target_positions_) 
            {
                int qposadr = model_->jnt_qposadr[joint_id];
                int dofadr = model_->jnt_dofadr[joint_id];
                
                double current_pos = data_->qpos[qposadr];
                double error = target - current_pos;
                
                // PD控制器
                double kp = 30.0;  // 比例增益
                double kd = 0.0;   // 微分增益
                
                double velocity = data_->qvel[dofadr];
                double target_vel = target_velocities_[joint_id];
                double force = kp * error + kd * (target_vel - velocity);
                
                // 应用力到执行器
                data_->ctrl[joint_id] = force;
            }
        }

        void init_rendering(mjvScene* scn, mjrContext* con, mjvCamera* cam) 
        {
            // 初始化场景            
            mjv_defaultScene(scn);
            mjv_defaultCamera(cam);
            mjv_defaultOption(&vopt_);
            mjr_defaultContext(con);

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


    private:
        mjModel* model_ = nullptr;
        mjData* data_ = nullptr;
        GLFWwindow* window_ = nullptr;

        std::vector<std::string> joint_names;

        // ROS 接口
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
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
        std::map<int, double> target_positions_;
        std::map<int, double> target_velocities_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto ur5e_node = std::make_shared<Ur5e_Node>("ur5e_node");
    ur5e_node->run();
    rclcpp::shutdown();
    return 0;
}