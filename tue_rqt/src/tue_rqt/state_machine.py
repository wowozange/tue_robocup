import rospy
import rosgraph

from qt_gui.plugin import Plugin

from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

import shlex
import re
import os
import subprocess
import glob

import roslaunch, rospkg


def ping(host):
    sub = subprocess.Popen(["/bin/ping", "-c1", "-w100", host], stdout=subprocess.PIPE)
    return sub.returncode == 0


class StateMachinePlugin(Plugin):

    def __init__(self, context):
        super(StateMachinePlugin, self).__init__(context)

        # Add widget with layout
        self._widget = QWidget()
        self._layout = QVBoxLayout()
        self._widget.setLayout(self._layout)
        # Add widget to content
        context.add_widget(self._widget)
        self._layout_specification = ""

    @staticmethod
    def _execute_service(service):
        if service:
            print "hjio"

    def _timer_callback(self, event):
        rosnode_hostname = None

        try:
            state_machine_node = self._ros_master.lookupNode("state_machine")
            rosnode_hostname = re.search("http://(.+?):\d+", state_machine_node).group(1)
        except Exception as e:
            print e

        if rosnode_hostname:
            try:
                process = subprocess.Popen(["ssh", rosnode_hostname, "ps aux"], stdout=subprocess.PIPE);
                output = process.communicate()[0]

                # Now find the state machine process launch file
                launch_file = re.search("launch/state_machines/(.+?).launch", output).group(1)

                self._running_state_machine_edit.setText("%s (%s)" % (launch_file, rosnode_hostname))
            except Exception as e:
                print e

            # self._running_state_machine_edit.setText(state_machine_node)
        else:
            self._running_state_machine_edit.setText("No state_machine node running ..")

    def _get_button(self, service):
        button = QPushButton(service)
        button.clicked.connect(lambda: self._execute_service(service))
        return button

    def _setup_gui(self, host, layout_specification):
        self._host = host
        self._layout_specification = layout_specification
        QWidget().setLayout(self._layout)
        self._layout = QVBoxLayout()
        self._widget.setLayout(self._layout)

        host_grid_layout = QGridLayout()
        host_grid_layout.addWidget(QLabel("Host = %s" % host), 0, 0)

        self._host_state_edit = QLineEdit()
        self._host_state_edit.setDisabled(True)
        host_grid_layout.addWidget(self._host_state_edit, 0, 1)

        self._layout.addLayout(host_grid_layout)

        grid_layout = QGridLayout()
        for row_index, row_spec in enumerate(layout_specification.split("\n")):
            for column_index, service in enumerate(shlex.split(row_spec)):
                grid_layout.addWidget(self._get_button(service), row_index, column_index)
        self._layout.addLayout(grid_layout)

        status_grid_layout = QGridLayout()

        self._bringup_path_edit = QLineEdit()
        self._bringup_path_edit.setDisabled(True)
        status_grid_layout.addWidget(self._bringup_path_edit, 0, 0)

        self._robot_bringup_path = os.getenv("ROBOT_BRINGUP_PATH")
        self._bringup_path_edit.setText(self._robot_bringup_path)

        self._running_state_machine_edit = QLineEdit()
        self._running_state_machine_edit.setDisabled(True)
        status_grid_layout.addWidget(self._running_state_machine_edit, 0, 1)

        self._layout.addLayout(status_grid_layout)

        self._running_state_machine_timer = rospy.Timer(rospy.Duration(2), self._timer_callback)
        self._ros_master = rosgraph.Master("")

    def trigger_configuration(self):
        result, ok = SettingsDialog.get_settings(self._host, self._layout_specification)
        if ok:
            topic_name, layout_specification = result
            self._setup_gui(topic_name, layout_specification)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("host", self._host)
        instance_settings.set_value("layout", self._layout_specification)

    def restore_settings(self, plugin_settings, instance_settings):
        host = str(instance_settings.value("host", "localhost"))
        layout_specification = str(instance_settings.value("layout", "messageA messageB\nmessageC messageD"))
        self._setup_gui(host, layout_specification)


class SettingsDialog(QDialog):
    def __init__(self, host, layout_specification):
        super(SettingsDialog, self).__init__(None)
        # Create inputs
        self._host_edit = QLineEdit()
        self._layout_specification_edit = QTextEdit()
        # Set defaults
        self._host_edit.setText(host)

        self._layout_specification_edit.setText(layout_specification)
        layout = QGridLayout(self)
        layout.addWidget(QLabel("Host name:"), 0, 0)
        layout.addWidget(self._host_edit, 0, 1)
        layout.addWidget(QLabel("Layout specification:"), 1, 0)
        layout.addWidget(self._layout_specification_edit, 1, 1)
        # OK and Cancel buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get(self):
        return self._host_edit.text(), \
               self._layout_specification_edit.toPlainText()

    # static method to create the dialog and return (date, time, accepted)
    @staticmethod
    def get_settings(host, layout_specification):
        dialog = SettingsDialog(host, layout_specification)
        result = dialog.exec_()
        return dialog.get(), result == QDialog.Accepted
