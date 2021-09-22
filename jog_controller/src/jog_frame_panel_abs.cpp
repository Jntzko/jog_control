#include <ros/package.h>
#include <ros/ros.h>

#include "jog_frame_panel_abs.h"

#include <rviz/config.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace jog_controller {

JogFramePanelAbs::JogFramePanelAbs(QWidget *parent) : rviz::Panel(parent) {
  ros::NodeHandle nh;
  // Get groups parameter of jog_frame_node

  std::vector<std::string> all_param_names;
  nh.getParamNames(all_param_names);

  std::vector<std::string> all_ns;
  for (auto &param : all_param_names) {
    if (param.find("jog_frame_node") != std::string::npos) {
      std::string ns = param.substr(0, param.find("jog_frame_node"));
      if (std::find(all_ns.begin(), all_ns.end(), ns) == all_ns.end())
        all_ns.push_back(ns);
    }
  }
  for (auto &ns : all_ns) {
    struct MoveGroup mg;
    nh.getParam(ns + "jog_frame_node/group_names", mg.group_names);
    nh.getParam(ns + "jog_frame_node/link_names", mg.link_names);
    nh.getParam(ns + "jog_frame_node/base_frame", mg.base_frame);
    mg.name_space = ns;
    move_groups_.push_back(mg);

    jog_frame_abs_pub_.push_back(
        nh.advertise<jog_msgs::JogFrameAbs>(ns + "jog_frame_abs", 1));
  }

  if (!move_groups_.empty()) {
    current_mg_ = 0;
    group_names_ = move_groups_[0].group_names;
    link_names_ = move_groups_[0].link_names;
    base_frame_ = move_groups_[0].base_frame;
    target_link_ = link_names_[0];
    frame_id_ = base_frame_;
  } else {
    ROS_WARN("No move groups available");
  }

  QLayout *root_layout = initUi(parent);
  setLayout(root_layout);

  master_on_publish_ = false;
  avoid_collisions_ = true;
}

JogFramePanelAbs::~JogFramePanelAbs() {
  server_->clear();
  server_->applyChanges();
  delete int_marker_;
  delete server_;
}

void JogFramePanelAbs::onInitialize() {
  ROS_WARN("ON Initial");
  initInteractiveMarkers();
  connect(vis_manager_, SIGNAL(preUpdate()), this, SLOT(update()));
  updateBaseFrame(base_frame_qcb_);
  updateLinkNames(target_link_qcb_);
  resetInteractiveMarker();

  // initialize repeating timer
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));

  // send 20 jog commands per second
  double msg_rate = 20;
  int milliseconds = (1.0 / msg_rate) * 1000;
  timer->start(milliseconds);
}

/**
 * Gets repeatedley called by QTimer
 */
void JogFramePanelAbs::update() {
  if (master_on_publish_ && on_publish_marker_) {
    jog_frame_abs_pub_[current_mg_].publish(marker_msg_);
  }
}

void JogFramePanelAbs::load(const rviz::Config &config) {}

void JogFramePanelAbs::save(rviz::Config config) const {}

void JogFramePanelAbs::hideEvent(QHideEvent *event) {
  server_->clear();
  server_->applyChanges();
}

void JogFramePanelAbs::showEvent(QShowEvent *event) {
  server_->insert(*int_marker_);
  server_->setCallback(
      int_marker_->name,
      boost::bind(&JogFramePanelAbs::interactiveMarkerFeedback, this, _1));
  server_->applyChanges();
  resetInteractiveMarker();
}

geometry_msgs::Pose *JogFramePanelAbs::getTargetLinkPose() {
  std::shared_ptr<tf2_ros::Buffer> tf = vis_manager_->getTF2BufferPtr();
  try {
    geometry_msgs::TransformStamped transform;
    transform = tf->lookupTransform(frame_id_, target_link_, ros::Time());

    geometry_msgs::Pose *pose = new geometry_msgs::Pose();
    pose->position.x = transform.transform.translation.x;
    pose->position.y = transform.transform.translation.y;
    pose->position.z = transform.transform.translation.z;
    pose->orientation = transform.transform.rotation;
    return pose;
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
  return nullptr;
}

void JogFramePanelAbs::resetInteractiveMarker() {
  geometry_msgs::Pose *pose_ptr = getTargetLinkPose();
  if (pose_ptr != nullptr) {
    geometry_msgs::Pose markerPose = geometry_msgs::Pose(*pose_ptr);
    server_->setPose(int_marker_->name, markerPose);
    server_->applyChanges();
  }
}

void JogFramePanelAbs::interactiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  if (feedback != nullptr) {
    marker_msg_.header.stamp = ros::Time::now();
    marker_msg_.header.frame_id = frame_id_;
    marker_msg_.group_name = group_name_;
    marker_msg_.link_name = target_link_;
    marker_msg_.avoid_collisions = avoid_collisions_;
    marker_msg_.damping_factor = damping_fac_;

    // reset the marker to the endeffectors position when mouse_up and stop
    // the end effector from moving by setting marker_msg_ to it's actual
    // pose.
    if (feedback->event_type == feedback->MOUSE_UP) {
      on_publish_marker_ = false;
      // tell target link to stay where it is
      resetInteractiveMarker();
      geometry_msgs::Pose *pose = getTargetLinkPose();
      if (pose != nullptr) {
        marker_msg_.pose = *pose;
        if (master_on_publish_) {
          jog_frame_abs_pub_[current_mg_].publish(marker_msg_);
        }
      }
    } else {
      on_publish_marker_ = true;
      marker_msg_.pose = feedback->pose;
    }
  }
} // namespace jog_controller

void JogFramePanelAbs::respondOnOffCb(bool isChecked) {
  master_on_publish_ = isChecked;
}

void JogFramePanelAbs::respondTargetLink(QString text) {
  target_link_ = text.toStdString();
  resetInteractiveMarker();
}

void JogFramePanelAbs::respondGroupName(QString text) {
  group_name_ = text.toStdString();
  for (int i = 0; i < move_groups_.size(); i++) {
    for (auto &group_name : move_groups_[i].group_names) {
      if (group_name_ == group_name)
        current_mg_ = i;
    }
  }
  updateLinkNames(target_link_qcb_);
  resetInteractiveMarker();
}

void JogFramePanelAbs::respondFrameId(QString text) {
  frame_id_ = text.toStdString();
  resetInteractiveMarker();
}

void JogFramePanelAbs::respondDamping(double value) { damping_fac_ = value; }

void JogFramePanelAbs::respondCollision(bool isChecked) {
  avoid_collisions_ = isChecked;
}

QLayout *JogFramePanelAbs::initUi(QWidget *parent) {
  QTreeWidget *tree = new QTreeWidget();
  tree->setColumnCount(2);
  tree->setColumnWidth(0, 200);
  QStringList headers = {"Preferences", ""};
  tree->setHeaderLabels(headers);
  tree->setStyleSheet("QTreeView::item { border: 0px; margin: 2px; }");
  tree->setSelectionMode(QAbstractItemView::NoSelection);
  tree->setFocusPolicy(Qt::NoFocus);

  QStringList prefs = {"Enable Jogging", "Move Group",
                       "Base Frame",     "End-Effector link",
                       "Damping Factor", "Collision Check"};

  QList<QTreeWidgetItem *> items;
  for (int i = 0; i < prefs.size(); ++i) {
    QTreeWidgetItem *item =
        new QTreeWidgetItem((QTreeWidget *)0, QStringList(prefs[i]));
    items.append(item);
  }
  tree->insertTopLevelItems(0, items);

  // Enable Jogging
  QCheckBox *on_off_master_ = new QCheckBox();
  tree->setItemWidget(items.value(0), 1, on_off_master_);

  // Move Group
  QComboBox *groupBox = new QComboBox();
  groupBox->setEditable(true);
  for (auto mg : move_groups_) {
    for (auto &group_name : mg.group_names)
      groupBox->addItem(group_name.c_str());
  }
  group_name_ = groupBox->currentText().toStdString();
  tree->setItemWidget(items.value(1), 1, groupBox);

  // Base Frame
  base_frame_qcb_ = new QComboBox();
  base_frame_qcb_->setEditable(true);
  tree->setItemWidget(items.value(2), 1, base_frame_qcb_);

  // End-Effector Link
  target_link_qcb_ = new QComboBox();
  target_link_qcb_->setEditable(true);
  tree->setItemWidget(items.value(3), 1, target_link_qcb_);

  // Damping Factor
  QDoubleSpinBox *damping_factor_qdsb = new QDoubleSpinBox();
  damping_factor_qdsb->setRange(0.1, 1.0);
  damping_factor_qdsb->setSingleStep(0.05);
  damping_factor_qdsb->setValue(0.95);
  damping_fac_ = damping_factor_qdsb->value();
  tree->setItemWidget(items.value(4), 1, damping_factor_qdsb);

  // Collision Check
  QCheckBox *collision_qcb = new QCheckBox();
  collision_qcb->setCheckState(Qt::Checked);
  tree->setItemWidget(items.value(5), 1, collision_qcb);

  QVBoxLayout *root_layout = new QVBoxLayout;
  root_layout->addWidget(tree);

  connect(on_off_master_, SIGNAL(toggled(bool)), this,
          SLOT(respondOnOffCb(bool)));
  connect(collision_qcb, SIGNAL(toggled(bool)), this,
          SLOT(respondCollision(bool)));
  connect(damping_factor_qdsb, SIGNAL(valueChanged(double)), this,
          SLOT(respondDamping(double)));
  connect(target_link_qcb_, SIGNAL(currentTextChanged(QString)), this,
          SLOT(respondTargetLink(QString)));
  connect(groupBox, SIGNAL(currentTextChanged(QString)), this,
          SLOT(respondGroupName(QString)));
  connect(base_frame_qcb_, SIGNAL(currentTextChanged(QString)), this,
          SLOT(respondFrameId(QString)));

  return root_layout;
}

void JogFramePanelAbs::updateBaseFrame(QComboBox *base_frame_qcb) {

  std::vector<std::string> frames;
  vis_manager_->getTF2BufferPtr()->_getFrameStrings(frames);
  std::sort(frames.begin(), frames.end());
  base_frame_qcb->clear();
  for (auto &frame : frames) {
    base_frame_qcb->addItem(frame.c_str());
  }
  base_frame_ = move_groups_[current_mg_].base_frame;
  base_frame_qcb->setCurrentIndex(
      base_frame_qcb->findText(QString::fromStdString(base_frame_)));
  frame_id_ = base_frame_qcb->currentText().toStdString();
}

void JogFramePanelAbs::updateLinkNames(QComboBox *target_link_qcb_) {
  target_link_qcb_->clear();
  for (auto &link_name : move_groups_[current_mg_].link_names)
    target_link_qcb_->addItem(link_name.c_str());
  target_link_qcb_->setCurrentIndex(0);
  target_link_ = target_link_qcb_->currentText().toStdString();
}

void JogFramePanelAbs::initInteractiveMarkers() {
  server_ = new interactive_markers::InteractiveMarkerServer(
      "jog_frame_node_abs", "", true);

  // create an interactive marker for our server, frame_id will be overwritten
  // later on, it's just necessary to initialize it
  int_marker_ = new visualization_msgs::InteractiveMarker();
  int_marker_->header.frame_id = base_frame_;
  // int_marker.header.stamp=ros::Time::now();
  int_marker_->name = "jog_frame_marker";
  int_marker_->description = "6-DOF Jogging Control";
  int_marker_->scale = 0.15;

  // create a marker
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.075;
  marker.scale.y = 0.075;
  marker.scale.z = 0.075;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl marker_control;
  marker_control.always_visible = true;
  marker_control.markers.push_back(marker);
  marker_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_3D;

  // add the control to the interactive marker
  int_marker_->controls.push_back(marker_control);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl arrow_control;
  arrow_control.name = "move_x";
  arrow_control.orientation.w = 1;
  arrow_control.orientation.x = 1;
  arrow_control.orientation.y = 0;
  arrow_control.orientation.z = 0;
  arrow_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_->controls.push_back(arrow_control);
  arrow_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_->controls.push_back(arrow_control);

  arrow_control.name = "move_y";
  arrow_control.orientation.w = 1;
  arrow_control.orientation.x = 0;
  arrow_control.orientation.y = 1;
  arrow_control.orientation.z = 0;
  arrow_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_->controls.push_back(arrow_control);
  arrow_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_->controls.push_back(arrow_control);

  arrow_control.name = "move_z";
  arrow_control.orientation.w = 1;
  arrow_control.orientation.x = 0;
  arrow_control.orientation.y = 0;
  arrow_control.orientation.z = 1;
  arrow_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker_->controls.push_back(arrow_control);
  arrow_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_->controls.push_back(arrow_control);

  // add the control to the interactive marker
  int_marker_->pose.position.z = 1;

  // add the interactive marker to our collection &
  // tell the server to call processMarkerFeedback() when feedback arrives for
  // it
  server_->insert(*int_marker_);
  server_->setCallback(
      int_marker_->name,
      boost::bind(&JogFramePanelAbs::interactiveMarkerFeedback, this, _1));
  server_->applyChanges();

  ROS_WARN_STREAM("Interactive Marker initialized: " << server_->size());
}

} // namespace jog_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jog_controller::JogFramePanelAbs, rviz::Panel)
