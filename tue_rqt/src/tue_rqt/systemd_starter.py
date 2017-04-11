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


def error_dialog(title, text):
    """
    Helper function for creating a error dialog
    :param title: Title of the dialog
    :param text: Text of the dialog
    """
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Critical)
    msg.setText(text)
    msg.setWindowTitle(title)
    msg.exec_()


def info_dialog(title, text):
    """
    Helper function for creating a info dialog
    :param title: Title of the dialog
    :param text: Text of the dialog
    """
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Information)
    msg.setText(text)
    msg.setWindowTitle(title)
    msg.exec_()


def systemctl_start(host, service):
    cmd = ['ssh', host, 'systemctl', '--user', 'start', service]

    print "Executing command %s" % " ".join(cmd)

    try:
        sub = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        out, err = sub.communicate()
    except Exception as e:
        return False, str(e)

    if err:
        return False, err

    return True, out


class SystemdStarterPlugin(Plugin):

    def __init__(self, context):
        super(SystemdStarterPlugin, self).__init__(context)

        # Add widget with layout
        self._widget = QWidget()
        self._layout = QVBoxLayout()
        self._widget.setLayout(self._layout)
        # Add widget to content
        context.add_widget(self._widget)
        self._layout_specification = ""

    def _execute_service(self, service):
        if service:
            ok, msg = systemctl_start(self._host, service)
            if ok:
                info_dialog("Succesfully started service %s" % service, msg if msg else "Started service")
            else:
                error_dialog("Failed to start service %s" % service, msg if msg else "Unknown error")

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

        self._layout.addLayout(host_grid_layout)

        grid_layout = QGridLayout()
        for row_index, row_spec in enumerate(layout_specification.split("\n")):
            for column_index, service in enumerate(shlex.split(row_spec)):
                grid_layout.addWidget(self._get_button(service), row_index, column_index)
        self._layout.addLayout(grid_layout)

        status_grid_layout = QGridLayout()

        self._layout.addLayout(status_grid_layout)

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
