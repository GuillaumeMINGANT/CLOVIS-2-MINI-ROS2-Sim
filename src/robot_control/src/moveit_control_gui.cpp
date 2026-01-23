/// @file moveit_control_gui.cpp
/// @brief A ROS2 node providing a GUI to control a humanoid robot leg using MoveIt.

#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>
#include <cmath> // For M_PI
#include <geometry_msgs/msg/pose_stamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <map>
#include <vector>
#include <memory>
#include <cstdlib> // For rand() and RAND_MAX
#include <ctime>   // For time()
#include <iostream> // For debugging output

/**
 * @brief A GUI class to control a humanoid robot leg using MoveIt.
 */
class MoveItControlGui : public QWidget {
public:
    MoveItControlGui(rclcpp::Node::SharedPtr node)
        : node_(node),
          tf_buffer_(node_->get_clock()),
          tf_listener_(tf_buffer_),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),

          // Start the executor in a separate thread
          executor_thread_([this]() { executor_->spin(); }) {

        // Planning groups for selection
        std::vector<std::string> default_groups = {
            "torso",
            "left_leg",
            "right_leg",
            "left_arm",
            "right_arm",
        };
        node_->declare_parameter<std::vector<std::string>>(
            "planning_groups", default_groups);
        node_->declare_parameter<std::string>("planning_group", "left_leg");

        auto groups = node_->get_parameter("planning_groups").as_string_array();
        auto initial_group = node_->get_parameter("planning_group").as_string();
        if (std::find(groups.begin(), groups.end(), initial_group) == groups.end()) {
            groups.push_back(initial_group);
        }

        // GUI Setup
        QVBoxLayout *layout = new QVBoxLayout(this);

        group_selector_ = new QComboBox(this);
        for (const auto &group : groups) {
            group_selector_->addItem(QString::fromStdString(group));
        }
        group_selector_->setCurrentText(QString::fromStdString(initial_group));
        layout->addWidget(new QLabel("Planning Group:", this));
        layout->addWidget(group_selector_);

        QPushButton *move_to_home_btn = new QPushButton("Move to Home Position", this);
        QPushButton *move_to_predefined_btn = new QPushButton("Move to Predefined Position", this);
        QPushButton *move_to_random_pose_btn = new QPushButton("Move to Random Pose", this);
        QPushButton *move_to_tf_btn = new QPushButton("Move to TF Position", this);
        QPushButton *refresh_frames_btn = new QPushButton("Refresh TF Frames", this);
        QPushButton *send_joint_targets_btn = new QPushButton("Send Joint Targets", this);

        joint_sliders_container_ = new QWidget(this);
        joint_sliders_layout_ = new QVBoxLayout(joint_sliders_container_);
        joint_sliders_layout_->setContentsMargins(0, 0, 0, 0);

        QScrollArea *joint_scroll = new QScrollArea(this);
        joint_scroll->setWidgetResizable(true);
        joint_scroll->setWidget(joint_sliders_container_);

        tf_frame_selector_ = new QComboBox(this);
        layout->addWidget(tf_frame_selector_);

        status_label_ = new QLabel("Status: Waiting for action", this);
        layout->addWidget(status_label_);

        layout->addWidget(move_to_home_btn);
        layout->addWidget(move_to_predefined_btn);
        layout->addWidget(move_to_random_pose_btn);
        layout->addWidget(move_to_tf_btn);
        layout->addWidget(refresh_frames_btn);
        layout->addWidget(send_joint_targets_btn);
        layout->addWidget(new QLabel("Joint Sliders:", this));
        layout->addWidget(joint_scroll);

        connect(group_selector_, &QComboBox::currentTextChanged, this, &MoveItControlGui::onGroupChanged);
        connect(move_to_home_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToHomePosition);
        connect(move_to_predefined_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToPredefinedPosition);
        connect(move_to_random_pose_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToRandomPose);
        connect(move_to_tf_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToTf);
        connect(refresh_frames_btn, &QPushButton::clicked, this, &MoveItControlGui::refreshTfFrames);
        connect(send_joint_targets_btn, &QPushButton::clicked, this, &MoveItControlGui::sendJointTargets);

        setLayout(layout);
        setPlanningGroup(initial_group);
        refreshTfFrames(); // Refresh frames on startup
    }

    ~MoveItControlGui() {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    QComboBox *group_selector_;
    QComboBox *tf_frame_selector_;
    QLabel *status_label_;
    QWidget *joint_sliders_container_;
    QVBoxLayout *joint_sliders_layout_;
    std::vector<QSlider *> joint_sliders_;
    std::vector<QLabel *> joint_labels_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;

    void setPlanningGroup(const std::string &group) {
        move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, group);
        status_label_->setText(QString("Status: Planning group set to %1").arg(QString::fromStdString(group)));
        rebuildJointSliders();
    }

    void onGroupChanged(const QString &group) {
        setPlanningGroup(group.toStdString());
    }

    void moveToHomePosition() {
        if (!move_group_interface_) {
            status_label_->setText("Status: No planning group selected");
            return;
        }
        move_group_interface_->setNamedTarget("home");
        if (move_group_interface_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            status_label_->setText("Status: Moved to home");
        } else {
            status_label_->setText("Status: Move to home failed");
        }
    }

    void moveToPredefinedPosition() {
        moveToPosition(0.37441, 0.12162, 0.51234, 0.61579, 0.77367, 0.072197, 0.13048);
    }

void moveToRandomPose() {
    // Seed the random number generator
    static bool seeded = false;
    if (!seeded) {
        srand(static_cast<unsigned int>(time(0))); // Seed with current time
        seeded = true;
    }
    // Define limits for random pose generation
    double x_min = 0.3; // Minimum x position
    double x_max = 0.8;  // Maximum x position
    double y_min = 0.3; // Minimum y position
    double y_max = 0.8;  // Maximum y position
    double z_min = 0.3;  // Minimum z position (e.g., ground level)
    double z_max = 0.8;  // Maximum z position (e.g., above ground)

    // Generate random pose within specified limits
    double random_x = x_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (x_max - x_min)));
    double random_y = y_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (y_max - y_min)));
    double random_z = z_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (z_max - z_min)));


    // Define a random orientation (roll, pitch, yaw) and convert to quaternion
    double random_roll = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2.0 * M_PI)));
    double random_pitch = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2.0 * M_PI)));
    double random_yaw = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2.0 * M_PI)));

    tf2::Quaternion q;
    q.setRPY(random_roll, random_pitch, random_yaw);

    // Move to the generated random pose
    moveToPosition(random_x, random_y, random_z, q.x(), q.y(), q.z(), q.w());
}

    void moveToTf() {
        // Logic to move to a pose based on TF frame selection
        std::string selected_frame = tf_frame_selector_->currentText().toStdString();
        // Retrieve the transform and move to the corresponding pose
    }

    void refreshTfFrames() {
        // Logic to refresh the TF frames in the combo box
        tf_frame_selector_->clear();
        // Populate tf_frame_selector_ with available frames
    }

    void moveToPosition(double x, double y, double z, double ox, double oy, double oz, double ow) {
        if (!move_group_interface_) {
            status_label_->setText("Status: No planning group selected");
            return;
        }
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        double norm = std::sqrt(ox * ox + oy * oy + oz * oz + ow * ow);
        if (norm <= 1e-6) {
            target_pose.orientation.x = 0.0;
            target_pose.orientation.y = 0.0;
            target_pose.orientation.z = 0.0;
            target_pose.orientation.w = 1.0;
        } else {
            target_pose.orientation.x = ox / norm;
            target_pose.orientation.y = oy / norm;
            target_pose.orientation.z = oz / norm;
            target_pose.orientation.w = ow / norm;
        }

        move_group_interface_->setPoseTarget(target_pose);
        if (move_group_interface_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            status_label_->setText("Status: Moved to target position");
        } else {
            status_label_->setText("Status: Move failed");
        }
    }

    void rebuildJointSliders() {
        for (auto *slider : joint_sliders_) {
            joint_sliders_layout_->removeWidget(slider);
            slider->deleteLater();
        }
        for (auto *label : joint_labels_) {
            joint_sliders_layout_->removeWidget(label);
            label->deleteLater();
        }
        joint_sliders_.clear();
        joint_labels_.clear();

        if (!move_group_interface_) {
            return;
        }

        const auto *joint_model_group = move_group_interface_->getRobotModel()->getJointModelGroup(
            move_group_interface_->getName());
        if (!joint_model_group) {
            return;
        }

        const auto &joint_names = joint_model_group->getActiveJointModelNames();
        for (const auto &name : joint_names) {
            const auto *joint = move_group_interface_->getRobotModel()->getJointModel(name);
            if (!joint || joint->getVariableBounds().empty()) {
                continue;
            }
            const auto &bounds = joint->getVariableBounds().front();
            double min_pos = bounds.min_position_;
            double max_pos = bounds.max_position_;
            if (!std::isfinite(min_pos) || !std::isfinite(max_pos)) {
                min_pos = -3.14159;
                max_pos = 3.14159;
            }

            QLabel *label = new QLabel(QString::fromStdString(name), joint_sliders_container_);
            QSlider *slider = new QSlider(Qt::Horizontal, joint_sliders_container_);
            slider->setMinimum(0);
            slider->setMaximum(1000);
            slider->setValue(500);
            slider->setProperty("joint_name", QString::fromStdString(name));
            slider->setProperty("min_pos", min_pos);
            slider->setProperty("max_pos", max_pos);

            joint_labels_.push_back(label);
            joint_sliders_.push_back(slider);
            joint_sliders_layout_->addWidget(label);
            joint_sliders_layout_->addWidget(slider);
        }
    }

    void sendJointTargets() {
        if (!move_group_interface_) {
            status_label_->setText("Status: No planning group selected");
            return;
        }
        std::map<std::string, double> targets;
        for (auto *slider : joint_sliders_) {
            const auto name = slider->property("joint_name").toString().toStdString();
            const double min_pos = slider->property("min_pos").toDouble();
            const double max_pos = slider->property("max_pos").toDouble();
            const double t = static_cast<double>(slider->value()) / 1000.0;
            const double value = min_pos + (max_pos - min_pos) * t;
            targets[name] = value;
        }
        move_group_interface_->setJointValueTarget(targets);
        if (move_group_interface_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            status_label_->setText("Status: Joint targets executed");
        } else {
            status_label_->setText("Status: Joint target move failed");
        }
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_control_gui");
    MoveItControlGui gui(node);
    gui.show();
    return app.exec();
}
