# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python import get_resource
from geometry_msgs.msg import Twist, TwistStamped
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QEvent, QObject, Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QApplication, QCheckBox, QShortcut, QWidget
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rqt_gui_py.plugin import Plugin


class _ArrowShortcutEnabler(QObject):
    """Swallow ShortcutOverride for arrow keys so QShortcut wins over QSlider's built-in arrow handling."""

    _ARROWS = (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.ShortcutOverride and event.key() in self._ARROWS:
            return True
        return False


class _TeleopKeyFilter(QObject):
    """Application-level key filter for direct-mode autobrake and Alt-toggle."""

    _LINEAR = (Qt.Key_W, Qt.Key_S, Qt.Key_Up, Qt.Key_Down)
    _ANGULAR = (Qt.Key_A, Qt.Key_D, Qt.Key_Left, Qt.Key_Right)

    def __init__(self, plugin):
        super().__init__()
        self._plugin = plugin

    def eventFilter(self, obj, event):
        et = event.type()
        if et == QEvent.ShortcutOverride and not event.isAutoRepeat():
            key = event.key()
            if event.modifiers() & Qt.AltModifier and key != Qt.Key_Alt:
                self._plugin._alt_pristine = False
            if self._plugin._direct:
                if key in self._LINEAR:
                    self._plugin._save_linear()
                elif key in self._ANGULAR:
                    self._plugin._save_angular()
            return False
        if et == QEvent.KeyPress and not event.isAutoRepeat():
            key = event.key()
            if key == Qt.Key_Alt:
                self._plugin._alt_pristine = True
            elif event.modifiers() & Qt.AltModifier:
                self._plugin._alt_pristine = False
            return False
        if et == QEvent.KeyRelease and not event.isAutoRepeat():
            key = event.key()
            if key == Qt.Key_Alt and self._plugin._alt_pristine:
                self._plugin._alt_pristine = False
                self._plugin._toggle_direct()
                return True
            if self._plugin._direct:
                if key in self._LINEAR:
                    self._plugin._restore_linear()
                    return True
                if key in self._ANGULAR:
                    self._plugin._restore_angular()
                    return True
        return False


class RobotSteering(Plugin):

    slider_factor = 1000.0

    def __init__(self, context):
        super(RobotSteering, self).__init__(context)
        self.setObjectName('RobotSteering')

        self._node = context.node

        self._node.declare_parameter('default_topic', Parameter.Type.STRING)
        self._node.declare_parameter('default_stamped', Parameter.Type.BOOL)
        self._node.declare_parameter('default_direct', Parameter.Type.BOOL)
        self._node.declare_parameter('default_vx_min', Parameter.Type.DOUBLE)
        self._node.declare_parameter('default_vx_max', Parameter.Type.DOUBLE)
        self._node.declare_parameter('default_vw_min', Parameter.Type.DOUBLE)
        self._node.declare_parameter('default_vw_max', Parameter.Type.DOUBLE)

        self._publisher = None
        self._publisher_stamped = None
        self._use_stamped = True
        self._direct = False
        self._alt_pristine = False
        self._saved_linear = None
        self._saved_angular = None

        self._widget = QWidget()
        _, package_path = get_resource('packages', 'rqt_robot_steering')
        ui_file = os.path.join(
            package_path, 'share', 'rqt_robot_steering', 'resource', 'RobotSteering.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RobotSteeringUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self._widget.direct_check_box = QCheckBox(self.tr('direct'), self._widget)
        self._widget.direct_check_box.setToolTip(
            self.tr('jump-to-limit, restore on release (Alt toggles, Space stops)'))
        self._widget.horizontalLayout.insertWidget(
            self._widget.horizontalLayout.indexOf(self._widget.stop_push_button) + 1,
            self._widget.direct_check_box)

        self._widget.topic_line_edit.textChanged.connect(
            self._on_topic_changed)
        self._widget.stamped_check_box.stateChanged.connect(
            self._on_stamped_cb_changed)
        self._widget.direct_check_box.stateChanged.connect(
            self._on_direct_cb_changed)
        self._widget.stop_push_button.pressed.connect(self._on_stop_pressed)

        self._widget.x_linear_slider.valueChanged.connect(
            self._on_x_linear_slider_changed)
        self._widget.z_angular_slider.valueChanged.connect(
            self._on_z_angular_slider_changed)

        self._widget.increase_x_linear_push_button.pressed.connect(
            self._on_strong_increase_x_linear_pressed)
        self._widget.reset_x_linear_push_button.pressed.connect(
            self._on_reset_x_linear_pressed)
        self._widget.decrease_x_linear_push_button.pressed.connect(
            self._on_strong_decrease_x_linear_pressed)
        self._widget.increase_z_angular_push_button.pressed.connect(
            self._on_strong_increase_z_angular_pressed)
        self._widget.reset_z_angular_push_button.pressed.connect(
            self._on_reset_z_angular_pressed)
        self._widget.decrease_z_angular_push_button.pressed.connect(
            self._on_strong_decrease_z_angular_pressed)

        self._widget.max_x_linear_double_spin_box.valueChanged.connect(
            self._on_max_x_linear_changed)
        self._widget.min_x_linear_double_spin_box.valueChanged.connect(
            self._on_min_x_linear_changed)
        self._widget.max_z_angular_double_spin_box.valueChanged.connect(
            self._on_max_z_angular_changed)
        self._widget.min_z_angular_double_spin_box.valueChanged.connect(
            self._on_min_z_angular_changed)

        self._arrow_enabler = _ArrowShortcutEnabler(self._widget)
        self._widget.x_linear_slider.installEventFilter(self._arrow_enabler)
        self._widget.z_angular_slider.installEventFilter(self._arrow_enabler)

        self._teleop_filter = _TeleopKeyFilter(self)
        QApplication.instance().installEventFilter(self._teleop_filter)

        self.shortcut_w = QShortcut(QKeySequence(Qt.Key_W), self._widget)
        self.shortcut_w.setContext(Qt.ApplicationShortcut)
        self.shortcut_w.activated.connect(self._on_increase_x_linear_pressed)
        self.shortcut_x = QShortcut(QKeySequence(Qt.Key_X), self._widget)
        self.shortcut_x.setContext(Qt.ApplicationShortcut)
        self.shortcut_x.activated.connect(self._on_reset_x_linear_pressed)
        self.shortcut_s = QShortcut(QKeySequence(Qt.Key_S), self._widget)
        self.shortcut_s.setContext(Qt.ApplicationShortcut)
        self.shortcut_s.activated.connect(self._on_decrease_x_linear_pressed)
        self.shortcut_a = QShortcut(QKeySequence(Qt.Key_A), self._widget)
        self.shortcut_a.setContext(Qt.ApplicationShortcut)
        self.shortcut_a.activated.connect(self._on_increase_z_angular_pressed)
        self.shortcut_z = QShortcut(QKeySequence(Qt.Key_Z), self._widget)
        self.shortcut_z.setContext(Qt.ApplicationShortcut)
        self.shortcut_z.activated.connect(self._on_reset_z_angular_pressed)
        self.shortcut_d = QShortcut(QKeySequence(Qt.Key_D), self._widget)
        self.shortcut_d.setContext(Qt.ApplicationShortcut)
        self.shortcut_d.activated.connect(self._on_decrease_z_angular_pressed)

        self.shortcut_shift_w = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_W), self._widget)
        self.shortcut_shift_w.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_w.activated.connect(
            self._on_strong_increase_x_linear_pressed)
        self.shortcut_shift_x = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_X), self._widget)
        self.shortcut_shift_x.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_x.activated.connect(
            self._on_reset_x_linear_pressed)
        self.shortcut_shift_s = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_S), self._widget)
        self.shortcut_shift_s.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_s.activated.connect(
            self._on_strong_decrease_x_linear_pressed)
        self.shortcut_shift_a = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_A), self._widget)
        self.shortcut_shift_a.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_a.activated.connect(
            self._on_strong_increase_z_angular_pressed)
        self.shortcut_shift_z = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Z), self._widget)
        self.shortcut_shift_z.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_z.activated.connect(
            self._on_reset_z_angular_pressed)
        self.shortcut_shift_d = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_D), self._widget)
        self.shortcut_shift_d.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_d.activated.connect(
            self._on_strong_decrease_z_angular_pressed)

        self.shortcut_up = QShortcut(QKeySequence(Qt.Key_Up), self._widget)
        self.shortcut_up.setContext(Qt.ApplicationShortcut)
        self.shortcut_up.activated.connect(self._on_increase_x_linear_pressed)
        self.shortcut_down = QShortcut(QKeySequence(Qt.Key_Down), self._widget)
        self.shortcut_down.setContext(Qt.ApplicationShortcut)
        self.shortcut_down.activated.connect(self._on_decrease_x_linear_pressed)
        self.shortcut_left = QShortcut(QKeySequence(Qt.Key_Left), self._widget)
        self.shortcut_left.setContext(Qt.ApplicationShortcut)
        self.shortcut_left.activated.connect(self._on_increase_z_angular_pressed)
        self.shortcut_right = QShortcut(QKeySequence(Qt.Key_Right), self._widget)
        self.shortcut_right.setContext(Qt.ApplicationShortcut)
        self.shortcut_right.activated.connect(self._on_decrease_z_angular_pressed)

        self.shortcut_shift_up = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Up), self._widget)
        self.shortcut_shift_up.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_up.activated.connect(
            self._on_strong_increase_x_linear_pressed)
        self.shortcut_shift_down = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Down), self._widget)
        self.shortcut_shift_down.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_down.activated.connect(
            self._on_strong_decrease_x_linear_pressed)
        self.shortcut_shift_left = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Left), self._widget)
        self.shortcut_shift_left.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_left.activated.connect(
            self._on_strong_increase_z_angular_pressed)
        self.shortcut_shift_right = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Right), self._widget)
        self.shortcut_shift_right.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_right.activated.connect(
            self._on_strong_decrease_z_angular_pressed)

        self.shortcut_space = QShortcut(
            QKeySequence(Qt.Key_Space), self._widget)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
        self.shortcut_space.activated.connect(self._on_stop_pressed)
        self.shortcut_space = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Space), self._widget)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
        self.shortcut_space.activated.connect(self._on_stop_pressed)

        self._widget.stop_push_button.setToolTip(
            self._widget.stop_push_button.toolTip() + ' ' + self.tr('([Shift +] Space)'))
        self._widget.increase_x_linear_push_button.setToolTip(
            self._widget.increase_x_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] W / Up)'))
        self._widget.reset_x_linear_push_button.setToolTip(
            self._widget.reset_x_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] X)'))
        self._widget.decrease_x_linear_push_button.setToolTip(
            self._widget.decrease_x_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] S / Down)'))
        self._widget.increase_z_angular_push_button.setToolTip(
            self._widget.increase_z_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] A / Left)'))
        self._widget.reset_z_angular_push_button.setToolTip(
            self._widget.reset_z_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] Z)'))
        self._widget.decrease_z_angular_push_button.setToolTip(
            self._widget.decrease_z_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] D / Right)'))

        # timer to consecutively send twist messages
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(
            self._on_parameter_changed)
        self._update_parameter_timer.start(100)
        self.zero_cmd_sent = False

        self._update_topic_type_timer = None

    @Slot(str)
    def _on_topic_changed(self, topic):
        topic = str(topic)
        self._unregister_publisher()
        if topic == '':
            return
        try:
            if self._use_stamped:
                self._publisher_stamped = self._node.create_publisher(
                    TwistStamped, topic, qos_profile=QoSProfile(depth=10))
            else:
                self._publisher = self._node.create_publisher(
                    Twist, topic, qos_profile=QoSProfile(depth=10))
        except Exception as e:
            print('Error creating publisher: %s' % e)

    @Slot(int)
    def _on_stamped_cb_changed(self, state):
        state = int(state)
        self._unregister_publisher()
        self._use_stamped = state
        # we can't change this in the same slot or we get a type error from rcl
        self._update_topic_type_timer = QTimer(self)
        self._update_topic_type_timer.timeout.connect(
            self._on_topic_type_changed)
        self._update_topic_type_timer.start(100)

    def _on_topic_type_changed(self):
        topic = self._widget.topic_line_edit.text()
        if self._update_topic_type_timer is not None:
            self._update_topic_type_timer.stop()
        if topic == '':
            return
        try:
            if self._use_stamped:
                self._publisher_stamped = self._node.create_publisher(
                    TwistStamped, topic, qos_profile=QoSProfile(depth=10))
            else:
                self._publisher = self._node.create_publisher(
                    Twist, topic, qos_profile=QoSProfile(depth=10))
        except Exception as e:
            print('Error creating publisher: %s' % e)

    def _on_stop_pressed(self):
        self._saved_linear = None
        self._saved_angular = None
        # If the current value of sliders is zero directly send stop twist msg
        if self._widget.x_linear_slider.value() == 0 and \
                self._widget.z_angular_slider.value() == 0:
            self.zero_cmd_sent = False
            self._on_parameter_changed()
        else:
            self._widget.x_linear_slider.setValue(0)
            self._widget.z_angular_slider.setValue(0)

    def _on_x_linear_slider_changed(self):
        self._widget.current_x_linear_label.setText(
            '%0.2f m/s' % (self._widget.x_linear_slider.value() / RobotSteering.slider_factor))
        self._on_parameter_changed()

    def _on_z_angular_slider_changed(self):
        self._widget.current_z_angular_label.setText(
            '%0.2f rad/s' % (self._widget.z_angular_slider.value() / RobotSteering.slider_factor))
        self._on_parameter_changed()

    def _on_increase_x_linear_pressed(self):
        slider = self._widget.x_linear_slider
        slider.setValue(slider.maximum() if self._direct else slider.value() + slider.singleStep())

    def _on_reset_x_linear_pressed(self):
        self._widget.x_linear_slider.setValue(0)

    def _on_decrease_x_linear_pressed(self):
        slider = self._widget.x_linear_slider
        slider.setValue(slider.minimum() if self._direct else slider.value() - slider.singleStep())

    def _on_increase_z_angular_pressed(self):
        slider = self._widget.z_angular_slider
        slider.setValue(slider.maximum() if self._direct else slider.value() + slider.singleStep())

    def _on_reset_z_angular_pressed(self):
        self._widget.z_angular_slider.setValue(0)

    def _on_decrease_z_angular_pressed(self):
        slider = self._widget.z_angular_slider
        slider.setValue(slider.minimum() if self._direct else slider.value() - slider.singleStep())

    def _on_max_x_linear_changed(self, value):
        self._widget.x_linear_slider.setMaximum(
            int(value * RobotSteering.slider_factor))

    def _on_min_x_linear_changed(self, value):
        self._widget.x_linear_slider.setMinimum(
            int(value * RobotSteering.slider_factor))

    def _on_max_z_angular_changed(self, value):
        self._widget.z_angular_slider.setMaximum(
            int(value * RobotSteering.slider_factor))

    def _on_min_z_angular_changed(self, value):
        self._widget.z_angular_slider.setMinimum(
            int(value * RobotSteering.slider_factor))

    def _on_strong_increase_x_linear_pressed(self):
        slider = self._widget.x_linear_slider
        slider.setValue(slider.maximum() if self._direct else slider.value() + slider.pageStep())

    def _on_strong_decrease_x_linear_pressed(self):
        slider = self._widget.x_linear_slider
        slider.setValue(slider.minimum() if self._direct else slider.value() - slider.pageStep())

    def _on_strong_increase_z_angular_pressed(self):
        slider = self._widget.z_angular_slider
        slider.setValue(slider.maximum() if self._direct else slider.value() + slider.pageStep())

    def _on_strong_decrease_z_angular_pressed(self):
        slider = self._widget.z_angular_slider
        slider.setValue(slider.minimum() if self._direct else slider.value() - slider.pageStep())

    @Slot(int)
    def _on_direct_cb_changed(self, state):
        self._direct = bool(int(state))
        self._saved_linear = None
        self._saved_angular = None

    def _toggle_direct(self):
        self._widget.direct_check_box.setChecked(not self._widget.direct_check_box.isChecked())

    def _save_linear(self):
        if self._saved_linear is None:
            self._saved_linear = self._widget.x_linear_slider.value()

    def _save_angular(self):
        if self._saved_angular is None:
            self._saved_angular = self._widget.z_angular_slider.value()

    def _restore_linear(self):
        if self._saved_linear is not None:
            self._widget.x_linear_slider.setValue(self._saved_linear)
            self._saved_linear = None

    def _restore_angular(self):
        if self._saved_angular is not None:
            self._widget.z_angular_slider.setValue(self._saved_angular)
            self._saved_angular = None

    def _on_parameter_changed(self):
        self._send_twist(
            self._widget.x_linear_slider.value() / RobotSteering.slider_factor,
            self._widget.z_angular_slider.value() / RobotSteering.slider_factor)

    def _send_twist(self, x_linear, z_angular):
        if self._publisher is None and self._publisher_stamped is None:
            return

        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_angular

        twist_stamped = TwistStamped()
        twist_stamped.twist = twist
        twist_stamped.header.stamp = self._node.get_clock().now().to_msg()
        twist_stamped.header.frame_id = ''

        # Only send the zero command once so other devices can take control
        if x_linear == 0.0 and z_angular == 0.0:
            if self.zero_cmd_sent:
                return
            else:
                self.zero_cmd_sent = True
        else:
            self.zero_cmd_sent = False

        if self._use_stamped:
            self._publisher_stamped.publish(twist_stamped)
        else:
            self._publisher.publish(twist)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._node.destroy_publisher(self._publisher)
            self._publisher = None
        if self._publisher_stamped is not None:
            self._node.destroy_publisher(self._publisher_stamped)
            self._publisher_stamped = None

    def shutdown_plugin(self):
        self._update_parameter_timer.stop()
        if self._update_topic_type_timer is not None:
            self._update_topic_type_timer.stop()
        app = QApplication.instance()
        if app is not None:
            app.removeEventFilter(self._teleop_filter)
        self._unregister_publisher()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value(
            'topic', self._widget.topic_line_edit.text())
        instance_settings.set_value(
            'stamped', self._widget.stamped_check_box.isChecked())
        instance_settings.set_value(
            'direct', self._widget.direct_check_box.isChecked())
        instance_settings.set_value(
            'vx_max', self._widget.max_x_linear_double_spin_box.value())
        instance_settings.set_value(
            'vx_min', self._widget.min_x_linear_double_spin_box.value())
        instance_settings.set_value(
            'vw_max', self._widget.max_z_angular_double_spin_box.value())
        instance_settings.set_value(
            'vw_min', self._widget.min_z_angular_double_spin_box.value())

    def restore_settings(self, plugin_settings, instance_settings):
        value = instance_settings.value('topic', '/cmd_vel')
        value = self._node.get_parameter_or('default_topic', value)
        if isinstance(value, Parameter):
            value = value.get_parameter_value().string_value
        self._widget.topic_line_edit.setText(value)

        value = self._widget.stamped_check_box.isChecked()
        if instance_settings.contains('stamped'):
            value = instance_settings.value('stamped', value) in ['true', 'True']
        value = self._node.get_parameter_or('default_stamped', value)
        if isinstance(value, Parameter):
            value = value.get_parameter_value().bool_value
        self._widget.stamped_check_box.setChecked(value)

        value = self._widget.direct_check_box.isChecked()
        if instance_settings.contains('direct'):
            value = instance_settings.value('direct', value) in ['true', 'True']
        value = self._node.get_parameter_or('default_direct', value)
        if isinstance(value, Parameter):
            value = value.get_parameter_value().bool_value
        self._widget.direct_check_box.setChecked(value)

        value = self._widget.max_x_linear_double_spin_box.value()
        if instance_settings.contains('vx_max'):
            value = float(instance_settings.value('vx_max', value))
        value = self._node.get_parameter_or('default_vx_max', value)
        if isinstance(value, Parameter):
            value = value.get_parameter_value().double_value
        self._widget.max_x_linear_double_spin_box.setValue(value)

        value = self._widget.min_x_linear_double_spin_box.value()
        if instance_settings.contains('vx_min'):
            value = float(instance_settings.value('vx_min', value))
        value = self._node.get_parameter_or('default_vx_min', value)
        if isinstance(value, Parameter):
            value = value.get_parameter_value().double_value
        self._widget.min_x_linear_double_spin_box.setValue(value)

        value = self._widget.max_z_angular_double_spin_box.value()
        if instance_settings.contains('vw_max'):
            value = float(instance_settings.value('vw_max', value))
        value = self._node.get_parameter_or('default_vw_max', value)
        if isinstance(value, Parameter):
            value = value.get_parameter_value().double_value
        self._widget.max_z_angular_double_spin_box.setValue(value)

        value = self._widget.min_z_angular_double_spin_box.value()
        if instance_settings.contains('vw_min'):
            value = float(instance_settings.value('vw_min', value))
        value = self._node.get_parameter_or('default_vw_min', value)
        if isinstance(value, Parameter):
            value = value.get_parameter_value().double_value
        self._widget.min_z_angular_double_spin_box.setValue(value)
