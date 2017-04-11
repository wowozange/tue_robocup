import rospy
import rosgraph

from qt_gui.plugin import Plugin

from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

import re
import subprocess


class ROSNodeMonitorPlugin(Plugin):

    def __init__(self, context):
        super(ROSNodeMonitorPlugin, self).__init__(context)
        self._widget = QWidget()
        context.add_widget(self._widget)

        self._layout = QGridLayout()
        self._widget.setLayout(self._layout)

        self._running_state_machine_edit = QLineEdit()
        self._running_state_machine_edit.setDisabled(True)
        self._layout.addWidget(self._running_state_machine_edit, 0, 0)

        self._running_state_machine_timer = rospy.Timer(rospy.Duration(2), self._timer_callback)
        self._ros_master = rosgraph.Master("")

    def _timer_callback(self, event):

        try:
            state_machine_node = self._ros_master.lookupNode(self._node_name)
            rosnode_hostname = re.search("http://(.+?):\d+", state_machine_node).group(1)
        except Exception as e:
            self._running_state_machine_edit.setText(str(e))
            return

        try:
            process = subprocess.Popen(["ssh", rosnode_hostname, "ps aux"], stdout=subprocess.PIPE)
            output = process.communicate()[0]

            # Now find the state machine process launch file
            regex_result = re.search(self._regex, output).group(1)

            self._running_state_machine_edit.setText("%s (%s)" % (regex_result, rosnode_hostname))
        except Exception as e:
            self._running_state_machine_edit.setText(str(e))

    def _setup_gui(self, node_name, regex):
        self._node_name = node_name
        self._regex = regex

    def trigger_configuration(self):
        result, ok = SettingsDialog.get_settings(self._node_name, self._regex)
        if ok:
            self._setup_gui(*result)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("node_name", self._node_name)
        instance_settings.set_value("regex", self._regex)

    def restore_settings(self, plugin_settings, instance_settings):
        node_name = str(instance_settings.value("node_name", "/node_name"))
        regex = str(instance_settings.value("regex", "/(.+?)\.launch"))
        self._setup_gui(node_name, regex)


class SettingsDialog(QDialog):
    def __init__(self, node_name, regex):
        super(SettingsDialog, self).__init__(None)
        # Create inputs
        self._node_name_edit = QLineEdit()
        self._node_name_edit.setText(node_name)

        self._regex_edit = QLineEdit()
        self._regex_edit.setText(regex)

        layout = QGridLayout(self)
        layout.addWidget(QLabel("Node name:"), 0, 0)
        layout.addWidget(self._node_name_edit, 0, 1)

        layout.addWidget(QLabel("Regex on 'ps aux' of the node machine (the first group will be extracted):"), 1, 0)
        layout.addWidget(self._regex_edit, 1, 1)

        # OK and Cancel buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get(self):
        return self._node_name_edit.text(), self._regex_edit.text()

    # static method to create the dialog and return (date, time, accepted)
    @staticmethod
    def get_settings(node_name, regex):
        dialog = SettingsDialog(node_name, regex)
        result = dialog.exec_()
        return dialog.get(), result == QDialog.Accepted
