// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Stéphane Caron

#include "upkie/cpp/actuation/BulletInterface.h"

#include <algorithm>
#include <memory>
#include <string>

#include "tools/cpp/runfiles/runfiles.h"
#include "upkie/cpp/actuation/bullet/gravity.h"
#include "upkie/cpp/actuation/bullet/read_imu_data.h"
#include "upkie/cpp/actuation/bullet/utils.h"

namespace upkie::cpp::actuation {

using bazel::tools::cpp::runfiles::Runfiles;
using bullet::bullet_from_eigen;
using bullet::eigen_from_bullet;

/*! Find ground plane URDF from Bazel runfiles.
 *
 * \param[in] argv0 Value of argv[0] used to locate runfiles.
 */
std::string find_plane_urdf(const std::string argv0) {
  std::string error;
  std::unique_ptr<Runfiles> runfiles(Runfiles::Create(argv0, &error));
  if (runfiles == nullptr) {
    throw std::runtime_error(
        "Could not retrieve the package path to plane.urdf: " + error);
  }
  return runfiles->Rlocation(
      "upkie/upkie/cpp/actuation/bullet/plane/plane.urdf");
}

BulletInterface::BulletInterface(const Parameters& params)
    : Interface(), params_(params), rng_(std::random_device()()) {
  // Start simulator
  auto flag = (params.gui ? eCONNECT_GUI : eCONNECT_DIRECT);
  bool is_connected = bullet_.connect(flag);
  if (!is_connected) {
    throw std::runtime_error("Could not connect to the Bullet GUI");
  }

  // Setup simulation scene
  bullet_.configureDebugVisualizer(COV_ENABLE_GUI, 0);
  bullet_.configureDebugVisualizer(COV_ENABLE_RENDERING, 0);
  bullet_.configureDebugVisualizer(COV_ENABLE_SHADOWS, 0);
  if (params.gravity) {
    bullet_.setGravity(btVector3(0, 0, -bullet::kGravity));
  }
  bullet_.setRealTimeSimulation(false);  // making sure

  // Load robot model
  robot_ = bullet_.loadURDF(params.robot_urdf_path);
  imu_link_index_ = get_link_index("imu");
  if (imu_link_index_ < 0) {
    throw std::runtime_error("Robot does not have a link named \"imu\"");
  }
  // If params has an attribute inertia_randomization, store masses
  if (params.inertia_randomization) {
    inertia_randomization_ = params.inertia_randomization;
    save_nominal_masses();
  }

  // Read servo layout
  for (const auto& id_joint : servo_name_map()) {
    const auto& servo_id = id_joint.first;
    const std::string& joint_name = id_joint.second;
    moteus::ServoReply reply;
    reply.id = servo_id;
    reply.result.temperature = 20.0;  // ['C], simulation room temperature
    reply.result.voltage = 18.0;      // [V], nominal voltage of a RYOBI battery
    joint_index_map_.try_emplace(joint_name, -1);
    servo_reply_.try_emplace(joint_name, reply);
  }

  // Map servo layout to Bullet
  b3JointInfo joint_info;
  const int nb_joints = bullet_.getNumJoints(robot_);
  for (int joint_index = 0; joint_index < nb_joints; ++joint_index) {
    bullet_.getJointInfo(robot_, joint_index, &joint_info);
    std::string joint_name = joint_info.m_jointName;
    if (joint_index_map_.find(joint_name) != joint_index_map_.end()) {
      joint_index_map_[joint_name] = joint_index;

      bullet::JointProperties props;
      props.maximum_torque = joint_info.m_jointMaxForce;
      joint_properties_.try_emplace(joint_name, props);
    }
  }

  // Load plane URDF
  if (params.floor) {
    if (bullet_.loadURDF(find_plane_urdf(params.argv0)) < 0) {
      throw std::runtime_error("Could not load the plane URDF!");
    }
  } else {
    spdlog::info("Not loading the plane URDF");
    if (params.gravity) {
      spdlog::warn(
          "No ground plane was loaded, but gravity is enabled. The robot will "
          "fall!");
    }
  }
  // Load environment URDFs
  for (const auto& urdf_path : params.env_urdf_paths) {
    spdlog::info("Loading environment URDF: {}", urdf_path);
    std::string body_name = (urdf_path.substr(urdf_path.find_last_of("/") + 1));
    body_name = body_name.substr(0, body_name.find_last_of("."));
    int body_id = bullet_.loadURDF(urdf_path);
    if (body_id < 0) {
      throw std::runtime_error("Could not load the environment URDF: " +
                               urdf_path);
    }
    body_names[body_name] = body_id;
  }

  // Start visualizer and configure simulation
  bullet_.configureDebugVisualizer(COV_ENABLE_RENDERING, 1);
  reset(Dictionary{});
}

BulletInterface::~BulletInterface() { bullet_.disconnect(); }

void BulletInterface::reset(const Dictionary& config) {
  params_.configure(config);
  bullet_.setTimeStep(params_.dt);
  reset_base_state(params_.position_base_in_world,
                   params_.orientation_base_in_world,
                   params_.linear_velocity_base_to_world_in_world,
                   params_.angular_velocity_base_in_base);
  reset_contact_data();
  reset_joint_angles(params_.joint_configuration);
  reset_joint_properties();
  if (std::abs(params_.inertia_randomization) < 1e-10) {
    randomize_masses();
  }
}

void BulletInterface::reset_base_state(
    const Eigen::Vector3d& position_base_in_world,
    const Eigen::Quaterniond& orientation_base_in_world,
    const Eigen::Vector3d& linear_velocity_base_to_world_in_world,
    const Eigen::Vector3d& angular_velocity_base_in_base) {
  bullet_.resetBasePositionAndOrientation(
      robot_, bullet_from_eigen(position_base_in_world),
      bullet_from_eigen(orientation_base_in_world));

  const auto& rotation_base_to_world =
      orientation_base_in_world;  // transforms are coordinates
  const Eigen::Vector3d angular_velocity_base_in_world =
      rotation_base_to_world * angular_velocity_base_in_base;
  bullet_.resetBaseVelocity(
      robot_, bullet_from_eigen(linear_velocity_base_to_world_in_world),
      bullet_from_eigen(angular_velocity_base_in_world));

  // Reset the extra velocity field used to compute IMU accelerations
  b3LinkState link_state;
  bullet_.getLinkState(robot_, imu_link_index_, /* computeVelocity = */ true,
                       /* computeForwardKinematics = */ true, &link_state);
  imu_data_.linear_velocity_imu_in_world[0] =
      link_state.m_worldLinearVelocity[0];
  imu_data_.linear_velocity_imu_in_world[1] =
      link_state.m_worldLinearVelocity[1];
  imu_data_.linear_velocity_imu_in_world[2] =
      link_state.m_worldLinearVelocity[2];
}

void BulletInterface::reset_contact_data() {
  for (const auto& link_name : params_.monitor_contacts) {
    contact_data_.try_emplace(link_name, bullet::ContactData());
  }
}

void BulletInterface::reset_joint_angles(
    const Eigen::VectorXd& joint_configuration) {
  b3JointInfo joint_info;
  const int nb_joints = bullet_.getNumJoints(robot_);
  const size_t nq = joint_configuration.size();
  size_t nb_revolute_joints = 0;
  for (int joint_index = 0; joint_index < nb_joints; ++joint_index) {
    bullet_.getJointInfo(robot_, joint_index, &joint_info);
    if (joint_info.m_jointType == eRevoluteType) {
      const int revolute_joint_index = nb_revolute_joints;
      nb_revolute_joints++;
      double joint_angle = 0.0;
      if (nq > 0 && revolute_joint_index < joint_configuration.size()) {
        joint_angle = joint_configuration(revolute_joint_index);
      }
      bullet_.resetJointState(robot_, joint_index, joint_angle);
    }
  }
  if (0 < nq && nq < nb_revolute_joints) {
    throw std::runtime_error(
        "Invalid joint configuration: vector has dimension " +
        std::to_string(nq) + " whereas the robot has " +
        std::to_string(nb_revolute_joints) + " revolute joints");
  }
}

void BulletInterface::reset_joint_properties() {
  b3JointInfo joint_info;
  const int nb_joints = bullet_.getNumJoints(robot_);
  for (int joint_index = 0; joint_index < nb_joints; ++joint_index) {
    bullet_.getJointInfo(robot_, joint_index, &joint_info);
    std::string joint_name = joint_info.m_jointName;
    if (joint_index_map_.find(joint_name) != joint_index_map_.end()) {
      const auto params_it = params_.joint_properties.find(joint_name);
      if (params_it != params_.joint_properties.end()) {
        joint_properties_[joint_name].update_configurable(params_it->second);
      } else /* no configuration for joint properties */ {
        joint_properties_[joint_name].reset_configurable();
      }
    }
  }
}

void BulletInterface::observe(Dictionary& observation) const {
  Interface::observe_imu(observation);

  Dictionary& sim = observation("sim");
  sim("imu")("linear_velocity") = imu_data_.linear_velocity_imu_in_world;
  for (const auto& link_name : params_.monitor_contacts) {
    sim("contact")(link_name)("num_contact_points") =
        contact_data_.at(link_name).num_contact_points;
  }
  sim("env_collision") = environment_collision_;

  // Observe base pose
  Eigen::Matrix4d T = get_transform_base_to_world();
  sim("base")("position") = Eigen::Vector3d(T(0, 3), T(1, 3), T(2, 3));  // [m]
  sim("base")("orientation") =
      Eigen::Quaterniond(T.block<3, 3>(0, 0));  // [w, x, y, z]

  // Observe base velocity
  btVector3 linear_velocity_base_to_world_in_world;
  btVector3 angular_velocity_base_to_world_in_world;
  bullet_.getBaseVelocity(robot_, linear_velocity_base_to_world_in_world,
                          angular_velocity_base_to_world_in_world);
  Eigen::Vector3d v = eigen_from_bullet(linear_velocity_base_to_world_in_world);
  Eigen::Vector3d omega =
      eigen_from_bullet(angular_velocity_base_to_world_in_world);
  sim("base")("linear_velocity") = v;       // [m] / [s]
  sim("base")("angular_velocity") = omega;  // [rad] / [s]

  // Observe the environment URDF states
  Dictionary& bodies = sim("bodies");
  for (const auto& key_child : body_names) {
    const auto& body_name = key_child.first;
    const auto& body_id = key_child.second;
    Eigen::Matrix4d T = get_transform_body_to_world(body_id);
    bodies(body_name)("position") =
        Eigen::Vector3d(T(0, 3), T(1, 3), T(2, 3));  // [m]
    bodies(body_name)("orientation") =
        Eigen::Quaterniond(T.block<3, 3>(0, 0));  // [w, x, y, z]
  }
}

void BulletInterface::process_action(const Dictionary& action) {
  if (!action.has("bullet")) {
    return;
  }
  const Dictionary& bullet_action = action("bullet");
  if (bullet_action.has("external_forces")) {
    process_forces(bullet_action("external_forces"));
  }
}

void BulletInterface::process_forces(const Dictionary& external_forces) {
  for (const auto& link_name : external_forces.keys()) {
    // Check that link name is valid and get link index
    int link_index = get_link_index(link_name);
    if (link_index < 0) {
      if (link_name == "base") {
        link_index = -1;
      } else {
        spdlog::warn(
            "Link \"{}\" not found in the robot description, cannot apply an "
            "external force to it",
            link_name);
        continue;
      }
    }

    // Read external force from the dictionary
    const auto& params = external_forces(link_name);
    const bool local_frame = params.get<bool>("local_frame", false);
    Eigen::Vector3d force_eigen = params("force");

    // Save external force, to be applied before stepSimulation()
    bullet::ExternalForce& ext_force = external_forces_[link_index];
    ext_force.flags = (local_frame) ? EF_LINK_FRAME : EF_WORLD_FRAME;
    ext_force.force = bullet_from_eigen(force_eigen);
  }
}

void BulletInterface::cycle(
    std::function<void(const moteus::Output&)> callback) {
  assert(data_.commands.size() == data_.replies.size());
  assert(!std::isnan(params_.dt));
  if (!bullet_.isConnected()) {
    throw std::runtime_error("simulator is not running any more");
  }

  environment_collision_ = environment_collision();
  read_joint_sensors();
  read_imu();
  read_contacts();
  send_commands();
  apply_external_forces();
  bullet_.stepSimulation();

  if (params_.follower_camera) {
    translate_camera_to_robot();
  }

  moteus::Output output;
  for (size_t i = 0; i < data_.replies.size(); ++i) {
    const auto servo_id = data_.commands[i].id;
    const std::string& joint_name = servo_name_map().at(servo_id);
    data_.replies[i].id = servo_id;
    data_.replies[i].result = servo_reply_[joint_name].result;
    output.query_result_size = i + 1;
  }
  callback(output);
}

void BulletInterface::read_imu() {
  bullet::read_imu_data(imu_data_, bullet_, robot_, imu_link_index_,
                        params_.dt);
  params_.imu_uncertainty.apply(imu_data_.linear_acceleration_imu_in_imu,
                                imu_data_.angular_velocity_imu_in_imu, rng_);
  params_.imu_uncertainty.apply(imu_data_.raw_linear_acceleration,
                                imu_data_.raw_angular_velocity, rng_);
}

void BulletInterface::read_contacts() {
  b3ContactInformation contact_info;
  b3RobotSimulatorGetContactPointsArgs contact_args;
  for (const auto& link_name : params_.monitor_contacts) {
    contact_args.m_bodyUniqueIdA = robot_;
    contact_args.m_linkIndexA = get_link_index(link_name);
    bullet_.getContactPoints(contact_args, &contact_info);
    contact_data_[link_name].num_contact_points =
        contact_info.m_numContactPoints;
  }
}
int BulletInterface::environment_collision()  {
  b3ContactInformation contact_info;
  b3RobotSimulatorGetContactPointsArgs contact_args;
  const int nb_links = bullet_.getNumJoints(robot_);
  for (const auto& key_child : body_names) {
    const auto& env_id = key_child.second;
    for (int link_id = 0; link_id < nb_links; ++link_id) {
      if (get_link_index("left_wheel_tire") != link_id
      && get_link_index("right_wheel_tire") != link_id) {
        contact_args.m_bodyUniqueIdA = robot_;
        contact_args.m_bodyUniqueIdB = env_id;
        contact_args.m_linkIndexA = link_id;
        bullet_.getContactPoints(contact_args, &contact_info);
        if (contact_info.m_numContactPoints != 0) {
          return 1;
        }
      }
    }
  }
  return 0;
}
void BulletInterface::read_joint_sensors() {
  b3JointSensorState sensor_state;
  for (const auto& name_index : joint_index_map_) {
    const auto& joint_name = name_index.first;
    const auto joint_index = name_index.second;
    bullet_.getJointState(robot_, joint_index, &sensor_state);
    auto& result = servo_reply_[joint_name].result;
    result.position = sensor_state.m_jointPosition / (2.0 * M_PI);
    result.velocity = sensor_state.m_jointVelocity / (2.0 * M_PI);

    // m_jointMotorTorque is set to a non-zero value when the joint is velocity
    // controlled (command mode == moteus::Mode::kStopped), and is zero when
    // the joint is torque controlled (command mode == moteus::Mode::kPosition)
    result.torque = sensor_state.m_jointMotorTorque;
  }
}
void BulletInterface::save_nominal_masses() {
  const int nb_links = bullet_.getNumJoints(robot_);
  b3DynamicsInfo info;
  for (int link_id = 0; link_id < nb_links; ++link_id) {
    bullet_.getDynamicsInfo(robot_, link_id, &info);
    nominal_masses[link_id] = info.m_mass;
    nominal_inertia[link_id][0] = info.m_localInertialDiagonal[0];
    nominal_inertia[link_id][1] = info.m_localInertialDiagonal[1];
    nominal_inertia[link_id][2] = info.m_localInertialDiagonal[2];
  }
}
void BulletInterface::randomize_masses() {
  std::default_random_engine generator;
  for (const auto& link_id : nominal_masses) {
    std::uniform_real_distribution<double> distribution(-inertia_randomization_,
                                                        inertia_randomization_);
    double epsilon = distribution(generator);
    RobotSimulatorChangeDynamicsArgs change_dyn_args;
    change_dyn_args.m_mass = nominal_masses[link_id.first] * (1 + epsilon);
    // We assume a uniform distribution of the mass density
    // The new mass is (1+\epsilon)previous_mass
    // As the distribution of mass is uniform, the inertia is linear
    // in mass, so we can multiply the original inertia by (1+\epsilon)
    change_dyn_args.m_localInertiaDiagonal[0] =
        nominal_inertia[link_id.first][0] * (1 + epsilon);
    change_dyn_args.m_localInertiaDiagonal[1] =
        nominal_inertia[link_id.first][1] * (1 + epsilon);
    change_dyn_args.m_localInertiaDiagonal[2] =
        nominal_inertia[link_id.first][2] * (1 + epsilon);
    bullet_.changeDynamics(robot_, link_id.first, change_dyn_args);
  }
}

void BulletInterface::send_commands() {
  b3RobotSimulatorJointMotorArgs motor_args(CONTROL_MODE_VELOCITY);

  for (const auto& command : data_.commands) {
    const auto servo_id = command.id;
    const std::string& joint_name = servo_name_map().at(servo_id);
    const int joint_index = joint_index_map_[joint_name];

    const auto previous_mode = servo_reply_[joint_name].result.mode;
    if (previous_mode == moteus::Mode::kStopped &&
        command.mode != moteus::Mode::kStopped) {
      // disable velocity controllers to enable torque control
      motor_args.m_controlMode = CONTROL_MODE_VELOCITY;
      motor_args.m_maxTorqueValue = 0.;  // [N m]
      bullet_.setJointMotorControl(robot_, joint_index, motor_args);
    }
    servo_reply_[joint_name].result.mode = command.mode;

    if (command.mode == moteus::Mode::kStopped) {
      motor_args.m_controlMode = CONTROL_MODE_VELOCITY;
      motor_args.m_maxTorqueValue = 100.;  // [N m]
      motor_args.m_targetVelocity = 0.;    // [rad] / [s]
      bullet_.setJointMotorControl(robot_, joint_index, motor_args);
      continue;
    }

    if (command.mode != moteus::Mode::kPosition) {
      throw std::runtime_error(
          "Bullet interface does not support command mode " +
          std::to_string(static_cast<unsigned>(command.mode)));
    }

    const double target_position = command.position.position * (2.0 * M_PI);
    const double target_velocity = command.position.velocity * (2.0 * M_PI);
    const double feedforward_torque = command.position.feedforward_torque;
    const double kp_scale = command.position.kp_scale;
    const double kd_scale = command.position.kd_scale;
    const double maximum_torque = command.position.maximum_torque;
    const double joint_torque = compute_joint_torque(
        joint_name, feedforward_torque, target_position, target_velocity,
        kp_scale, kd_scale, maximum_torque);
    motor_args.m_controlMode = CONTROL_MODE_TORQUE;
    motor_args.m_maxTorqueValue = joint_torque;
    bullet_.setJointMotorControl(robot_, joint_index, motor_args);

    // m_jointMotorTorque processed in read_joint_sensors() will be set to zero
    // since we just torque controlled the joint, hence we measure it here:
    const bullet::JointProperties& joint_props = joint_properties_[joint_name];
    servo_reply_[joint_name].result.torque = joint_torque;
    if (joint_props.torque_measurement_noise > 1e-10) {
      std::normal_distribution<double> white_noise(
          0.0, joint_props.torque_control_noise);
      servo_reply_[joint_name].result.torque += white_noise(rng_);
    }
  }
}

double BulletInterface::compute_joint_torque(
    const std::string& joint_name, const double feedforward_torque,
    const double target_position, const double target_velocity,
    const double kp_scale, const double kd_scale, const double maximum_torque) {
  assert(!std::isnan(target_velocity));
  const bullet::JointProperties& joint_props = joint_properties_[joint_name];
  const auto& measurements = servo_reply_[joint_name].result;
  const double measured_position = measurements.position * (2.0 * M_PI);
  const double measured_velocity = measurements.velocity * (2.0 * M_PI);
  const double kp = kp_scale * params_.torque_control_kp;
  const double kd = kd_scale * params_.torque_control_kd;
  const double tau_max = std::min(maximum_torque, joint_props.maximum_torque);
  double torque = feedforward_torque;
  torque += kd * (target_velocity - measured_velocity);
  if (!std::isnan(target_position)) {
    torque += kp * (target_position - measured_position);
  }
  constexpr double kMaxStictionVelocity = 1e-3;  // rad/s
  if (std::abs(measured_velocity) > kMaxStictionVelocity) {
    torque += joint_props.friction * ((measured_velocity > 0.0) ? -1.0 : +1.0);
  }
  if (joint_props.torque_control_noise > 1e-10) {
    std::normal_distribution<double> white_noise(
        0.0, joint_props.torque_control_noise);
    torque += white_noise(rng_);
  }
  torque = std::max(std::min(torque, tau_max), -tau_max);
  return torque;
}

Eigen::Matrix4d BulletInterface::get_transform_base_to_world() const noexcept {
  btVector3 position_base_in_world;
  btQuaternion orientation_base_in_world;
  bullet_.getBasePositionAndOrientation(robot_, position_base_in_world,
                                        orientation_base_in_world);
  auto quat = eigen_from_bullet(orientation_base_in_world);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = eigen_from_bullet(position_base_in_world);
  return T;
}

Eigen::Matrix4d BulletInterface::get_transform_body_to_world(
    int body_id) const noexcept {
  btVector3 position_base_in_world;
  btQuaternion orientation_base_in_world;
  bullet_.getBasePositionAndOrientation(body_id, position_base_in_world,
                                        orientation_base_in_world);
  auto quat = eigen_from_bullet(orientation_base_in_world);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = eigen_from_bullet(position_base_in_world);
  return T;
}

Eigen::Vector3d BulletInterface::get_linear_velocity_base_to_world_in_world()
    const noexcept {
  btVector3 linear_velocity_base_to_world_in_world;
  btVector3 _;  // anonymous, we only get the linear velocity
  bullet_.getBaseVelocity(robot_, linear_velocity_base_to_world_in_world, _);
  return eigen_from_bullet(linear_velocity_base_to_world_in_world);
}

Eigen::Vector3d BulletInterface::get_angular_velocity_base_in_base()
    const noexcept {
  btVector3 angular_velocity_base_to_world_in_world;
  btVector3 _;  // anonymous, we only get the angular velocity
  bullet_.getBaseVelocity(robot_, _, angular_velocity_base_to_world_in_world);
  Eigen::Matrix4d T = get_transform_base_to_world();
  Eigen::Matrix3d rotation_base_to_world = T.block<3, 3>(0, 0);
  return rotation_base_to_world.transpose() *
         eigen_from_bullet(angular_velocity_base_to_world_in_world);
}

Eigen::VectorXd BulletInterface::get_joint_angles() noexcept {
  Eigen::VectorXd joint_angles(6);
  b3JointInfo joint_info;
  b3JointSensorState joint_state;
  const int nb_joints = bullet_.getNumJoints(robot_);
  for (int joint_index = 0; joint_index < nb_joints; ++joint_index) {
    bullet_.getJointInfo(robot_, joint_index, &joint_info);
    if (joint_info.m_jointType == eRevoluteType) {
      const int idx_q = joint_info.m_qIndex - 7;
      bullet_.getJointState(robot_, joint_index, &joint_state);
      joint_angles(idx_q) = joint_state.m_jointPosition;
    }
  }
  return joint_angles;
}

void BulletInterface::translate_camera_to_robot() {
  b3OpenGLVisualizerCameraInfo camera_info;
  btVector3 position_base_in_world;
  btQuaternion orientation_base_in_world;
  bullet_.getBasePositionAndOrientation(robot_, position_base_in_world,
                                        orientation_base_in_world);
  bullet_.getDebugVisualizerCamera(&camera_info);
  bullet_.resetDebugVisualizerCamera(camera_info.m_dist, camera_info.m_pitch,
                                     camera_info.m_yaw, position_base_in_world);
}

int BulletInterface::get_link_index(const std::string& link_name) {
  if (link_index_.find(link_name) != link_index_.end()) {
    return link_index_[link_name];
  }
  int link_index = bullet::find_link_index(bullet_, robot_, link_name);
  link_index_[link_name] = link_index;
  return link_index;
}

Eigen::Vector3d BulletInterface::get_position_link_in_world(
    const std::string& link_name) {
  int link_index = get_link_index(link_name);
  return bullet::get_position_link_in_world(bullet_, robot_, link_index);
}

double BulletInterface::compute_robot_mass() {
  return bullet::compute_robot_mass(bullet_, robot_);
}

Eigen::Vector3d BulletInterface::compute_position_com_in_world() {
  return bullet::compute_position_com_in_world(bullet_, robot_);
}

void BulletInterface::apply_external_forces() {
  for (auto& link_force : external_forces_) {
    const int link_index = link_force.first;
    const int flags = link_force.second.flags;
    btVector3& force = link_force.second.force;
    Eigen::Vector3d position_eigen;
    if (flags == EF_LINK_FRAME) {
      position_eigen.setZero();
    } else /* (flags == EF_WORLD_FRAME) */ {
      position_eigen =
          bullet::get_position_link_in_world(bullet_, robot_, link_index);
    }
    btVector3 position = bullet_from_eigen(position_eigen);
    bullet_.applyExternalForce(robot_, link_index, force, position, flags);
  }
}

}  // namespace upkie::cpp::actuation
