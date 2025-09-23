#include "Y2RobMotion/robot_measure.hpp"

// Constructor
UrMeasure::UrMeasure(Mode mode,
                     const rclcpp::Node::SharedPtr& node,
                     const std::string& robot_name, const std::string& basic_dir)
    : node_(node), robot_name_(robot_name), mode_(mode), basic_dir_(basic_dir)
{
    // Initialize files
    if (!initializeFiles()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize files. Exiting.");
        rclcpp::shutdown();
        return;
    }

    // Topics
    std::string current_j_topic = robot_name_ + std::string("/currentJ");
    std::string current_p_topic = robot_name_ + std::string("/currentP");
    std::string current_f_topic = robot_name_ + std::string("/currentF");
    std::string target_j_topic  = robot_name_ + std::string("/targetJ");
    std::string target_p_topic  = robot_name_ + std::string("/targetP");
    std::string target_f_topic  = robot_name_ + std::string("/targetF");

    // Subscribers
    current_j_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_j_topic, 10,
        std::bind(&UrMeasure::currentJCallback, this, std::placeholders::_1));

    current_p_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_p_topic, 10,
        std::bind(&UrMeasure::currentPCallback, this, std::placeholders::_1));

    current_f_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_f_topic, 10,
        std::bind(&UrMeasure::currentFCallback, this, std::placeholders::_1));

    target_j_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_j_topic, 10,
        std::bind(&UrMeasure::targetJCallback, this, std::placeholders::_1));

    target_p_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_p_topic, 10,
        std::bind(&UrMeasure::targetPCallback, this, std::placeholders::_1));

    target_f_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_f_topic, 10,
        std::bind(&UrMeasure::targetFCallback, this, std::placeholders::_1));

    // Timer
    timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(MEASURE_PERIOD),
        std::bind(&UrMeasure::timerCallback, this));

    // Mode banner
    if (mode_ == Mode::Continuous) {
        RCLCPP_INFO(node_->get_logger(), "\033[32mRecording started (CONTINUOUS)\033[0m");
        RCLCPP_INFO(node_->get_logger(), "Measurement frequency: %.1f Hz", 1.0 / MEASURE_PERIOD);
    } else {
        RCLCPP_INFO(node_->get_logger(), "\033[33mRecording ready (DISCRETE)\033[0m");
        RCLCPP_INFO(node_->get_logger(), "Press ENTER to snapshot; type -1 then ENTER to exit.");
        startInputThreadIfDiscrete();
    }

    RCLCPP_INFO(node_->get_logger(), "UrMeasure initialized for robot: %s", robot_name_.c_str());
}

// Destructor
UrMeasure::~UrMeasure()
{
    if (input_thread_.joinable()) input_thread_.join();

    // Close files safely
    if (cp_file_ && cp_file_->is_open()) cp_file_->close();
    if (cj_file_ && cj_file_->is_open()) cj_file_->close();
    if (cf_file_ && cf_file_->is_open()) cf_file_->close();
    if (tp_file_ && tp_file_->is_open()) tp_file_->close();
    if (tj_file_ && tj_file_->is_open()) tj_file_->close();
    if (tf_file_ && tf_file_->is_open()) tf_file_->close();

    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "\033[31mProgram was terminated\033[0m");
    }
}

// Initialize files
bool UrMeasure::initializeFiles()
{
    // Create measured directory if it doesn't exist
    std::string measured_dir = basic_dir_+robot_name_;
    if (!std::filesystem::exists(measured_dir)) {
        try {
            std::filesystem::create_directories(measured_dir);
            RCLCPP_INFO(node_->get_logger(), "Created directory: %s", measured_dir.c_str());
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create directory %s: %s",
                         measured_dir.c_str(), e.what());
            return false;
        }
    }

    // Initialize file streams
    try {
        cp_file_ = std::make_unique<std::ofstream>(measured_dir + CP_PATH);
        cj_file_ = std::make_unique<std::ofstream>(measured_dir + CJ_PATH);
        cf_file_ = std::make_unique<std::ofstream>(measured_dir + CF_PATH);

        tp_file_ = std::make_unique<std::ofstream>(measured_dir + TP_PATH);
        tj_file_ = std::make_unique<std::ofstream>(measured_dir + TJ_PATH);
        tf_file_ = std::make_unique<std::ofstream>(measured_dir + TF_PATH);

        if (!cp_file_->is_open() || !cj_file_->is_open() || !cf_file_->is_open() ||
            !tp_file_->is_open() || !tj_file_->is_open() || !tf_file_->is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "Error opening one or more files.");
            return false;
        }

        // Set precision for output files
        cp_file_->precision(3); cj_file_->precision(3); cf_file_->precision(3);
        tp_file_->precision(3); tj_file_->precision(3); tf_file_->precision(3);
        cp_file_->setf(std::ios::fixed); cj_file_->setf(std::ios::fixed); cf_file_->setf(std::ios::fixed);
        tp_file_->setf(std::ios::fixed); tj_file_->setf(std::ios::fixed); tf_file_->setf(std::ios::fixed);

        RCLCPP_INFO(node_->get_logger(), "All measurement files opened successfully.");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while opening files: %s", e.what());
        return false;
    }
}

// Input thread for discrete mode
void UrMeasure::startInputThreadIfDiscrete()
{
    if (mode_ != Mode::Discrete) return;

    input_thread_ = std::thread([this]() {
        std::string line;
        while (rclcpp::ok()) {
            if (!std::getline(std::cin, line)) {
                // stdin closed -> graceful stop
                rclcpp::shutdown();
                break;
            }
            if (line == "-1") {
                RCLCPP_INFO(node_->get_logger(), "Exit requested (-1). Shutting down...");
                rclcpp::shutdown();
                break;
            } else if (line.empty()) {
                snap_trigger_.store(true);
            } else {
                std::cout << "[Discrete] Press ENTER to record, or -1 then ENTER to exit.\n";
            }
        }
    });
}

// ---- Continuous writers
void UrMeasure::writeContinuousCJ(const std_msgs::msg::Float64MultiArray::SharedPtr& msg)
{
    if (!cj_file_ || !cj_file_->is_open()) return;
    for (int i = 0; i < NUMBER_OF_JOINTS && i < static_cast<int>(msg->data.size()); i++) {
        *cj_file_ << msg->data[i] << "\t";
    }
    *cj_file_ << "\n";
    cj_file_->flush();
}
void UrMeasure::writeContinuousCP(const std_msgs::msg::Float64MultiArray::SharedPtr& msg)
{
    if (!cp_file_ || !cp_file_->is_open()) return;
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++) {
        *cp_file_ << msg->data[i] << "\t";
    }
    *cp_file_ << "\n";
    cp_file_->flush();
}
void UrMeasure::writeContinuousCF(const std_msgs::msg::Float64MultiArray::SharedPtr& msg)
{
    if (!cf_file_ || !cf_file_->is_open()) return;
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++) {
        *cf_file_ << msg->data[i] << "\t";
    }
    *cf_file_ << "\n";
    cf_file_->flush();
}
void UrMeasure::writeContinuousTJ(const std_msgs::msg::Float64MultiArray::SharedPtr& msg)
{
    if (!tj_file_ || !tj_file_->is_open()) return;
    for (int i = 0; i < NUMBER_OF_JOINTS && i < static_cast<int>(msg->data.size()); i++) {
        *tj_file_ << msg->data[i] << "\t";
    }
    *tj_file_ << "\n";
    tj_file_->flush();
}
void UrMeasure::writeContinuousTP(const std_msgs::msg::Float64MultiArray::SharedPtr& msg)
{
    if (!tp_file_ || !tp_file_->is_open()) return;
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++) {
        *tp_file_ << msg->data[i] << "\t";
    }
    *tp_file_ << "\n";
    tp_file_->flush();
}
void UrMeasure::writeContinuousTF(const std_msgs::msg::Float64MultiArray::SharedPtr& msg)
{
    if (!tf_file_ || !tf_file_->is_open()) return;
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++) {
        *tf_file_ << msg->data[i] << "\t";
    }
    *tf_file_ << "\n";
    tf_file_->flush();
}

// ---- Subscriber callbacks
void UrMeasure::currentJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    {   std::lock_guard<std::mutex> lk(buf_mtx_);
        for (int i = 0; i < NUMBER_OF_JOINTS && i < static_cast<int>(msg->data.size()); i++)
            last_cj_[i] = msg->data[i];
        have_cj_ = true;
    }
    if (mode_ == Mode::Continuous) writeContinuousCJ(msg);
}
void UrMeasure::currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    {   std::lock_guard<std::mutex> lk(buf_mtx_);
        for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++)
            last_cp_[i] = msg->data[i];
        have_cp_ = true;
    }
    if (mode_ == Mode::Continuous) writeContinuousCP(msg);
}
void UrMeasure::currentFCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    {   std::lock_guard<std::mutex> lk(buf_mtx_);
        for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++)
            last_cf_[i] = msg->data[i];
        have_cf_ = true;
    }
    if (mode_ == Mode::Continuous) writeContinuousCF(msg);
}
void UrMeasure::targetJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    {   std::lock_guard<std::mutex> lk(buf_mtx_);
        for (int i = 0; i < NUMBER_OF_JOINTS && i < static_cast<int>(msg->data.size()); i++)
            last_tj_[i] = msg->data[i];
        have_tj_ = true;
    }
    if (mode_ == Mode::Continuous) writeContinuousTJ(msg);
}
void UrMeasure::targetPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    {   std::lock_guard<std::mutex> lk(buf_mtx_);
        for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++)
            last_tp_[i] = msg->data[i];
        have_tp_ = true;
    }
    if (mode_ == Mode::Continuous) writeContinuousTP(msg);
}
void UrMeasure::targetFCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    {   std::lock_guard<std::mutex> lk(buf_mtx_);
        for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++)
            last_tf_[i] = msg->data[i];
        have_tf_ = true;
    }
    if (mode_ == Mode::Continuous) writeContinuousTF(msg);
}

// ---- Discrete snapshot writer
void UrMeasure::writeDiscreteSnapshot()
{
    std::lock_guard<std::mutex> lk(buf_mtx_);
    if (cj_file_ && cj_file_->is_open() && have_cj_) {
        for (int i = 0; i < NUMBER_OF_JOINTS; ++i) *cj_file_ << last_cj_[i] << "\t";
        *cj_file_ << "\n"; cj_file_->flush();
    }
    if (cp_file_ && cp_file_->is_open() && have_cp_) {
        for (int i = 0; i < 6; ++i) *cp_file_ << last_cp_[i] << "\t";
        *cp_file_ << "\n"; cp_file_->flush();
    }
    if (cf_file_ && cf_file_->is_open() && have_cf_) {
        for (int i = 0; i < 6; ++i) *cf_file_ << last_cf_[i] << "\t";
        *cf_file_ << "\n"; cf_file_->flush();
    }

    if (tj_file_ && tj_file_->is_open() && have_tj_) {
        for (int i = 0; i < NUMBER_OF_JOINTS; ++i) *tj_file_ << last_tj_[i] << "\t";
        *tj_file_ << "\n"; tj_file_->flush();
    }
    if (tp_file_ && tp_file_->is_open() && have_tp_) {
        for (int i = 0; i < 6; ++i) *tp_file_ << last_tp_[i] << "\t";
        *tp_file_ << "\n"; tp_file_->flush();
    }
    if (tf_file_ && tf_file_->is_open() && have_tf_) {
        for (int i = 0; i < 6; ++i) *tf_file_ << last_tf_[i] << "\t";
        *tf_file_ << "\n"; tf_file_->flush();
    }
    RCLCPP_INFO(node_->get_logger(), "Snapshot saved (discrete).");
}

// Timer callback
void UrMeasure::timerCallback()
{
    static int counter = 0;
    counter++;

    if (mode_ == Mode::Discrete) {
        if (snap_trigger_.exchange(false)) {
            writeDiscreteSnapshot();
        }
    } else {
        // Continuous: optional periodic debug
        if (counter % static_cast<int>(10.0 / MEASURE_PERIOD) == 0) {
            RCLCPP_DEBUG(node_->get_logger(), "Measurement running... (count: %d)", counter);
        }
    }
}


