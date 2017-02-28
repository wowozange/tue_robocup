import rospy
import rosgraph

from qt_gui.plugin import Plugin

from python_qt_binding.QtWidgets import * 
from python_qt_binding.QtGui import * 
from python_qt_binding.QtCore import * 

import re
import os
import subprocess
import glob

import roslaunch, rospkg


class StateMachinePlugin(Plugin):

    def __init__(self, context):
        super(StateMachinePlugin, self).__init__(context)

        # Widget setup
        self.setObjectName('StateMachinePlugin')

        self._widget = QWidget()
        context.add_widget(self._widget)

        # Layout and attach to widget
        layout = QVBoxLayout()  
        self._widget.setLayout(layout)
        
        # Layout and attach to widget
        top_grid_layout = QGridLayout()  
        layout.addLayout(top_grid_layout)

        self._bringup_path_edit = QLineEdit()
        self._bringup_path_edit.setDisabled(True)
        top_grid_layout.addWidget(self._bringup_path_edit, 0, 0)

        self._robot_bringup_path = os.getenv("ROBOT_BRINGUP_PATH")
        self._bringup_path_edit.setText(self._robot_bringup_path)

        self._running_state_machine_edit = QLineEdit()
        self._running_state_machine_edit.setDisabled(True)
        top_grid_layout.addWidget(self._running_state_machine_edit, 0, 1)

        self._running_state_machine_timer = rospy.Timer(rospy.Duration(1), self._running_state_machine_timer_callback)

        self._ros_master = rosgraph.Master("")

        # Layout and attach to widget
        bottom_grid_layout = QGridLayout()  
        layout.addLayout(bottom_grid_layout)

        # Now index all state_machine launch files in the bringup path
        launch_files = glob.glob(self._robot_bringup_path + "/launch/state_machines/*.launch")
        for i, launch_file in enumerate(launch_files):
            btn = QPushButton(re.search("/([^/]+)\.launch$", launch_file).group(1))
            bottom_grid_layout.addWidget(btn, 1, i)
            btn.clicked.connect(self._button_clicked)

    def _button_clicked(self):
        launch_file_path = "%s/launch/state_machines/%s.launch" % (self._robot_bringup_path, self.sender().text())

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
        launch.start()         

    def _running_state_machine_timer_callback(self, event):
        hostname = None

        try:
            state_machine_node = self._ros_master.lookupNode("state_machine")
            hostname = re.search("http://(.+?):\d+", state_machine_node).group(1)
        except Exception as e:
            print e

        if hostname:
            try:
                process = subprocess.Popen(["ssh", hostname, "ps aux"], stdout=subprocess.PIPE);
                output = process.communicate()[0]

                # Now find the state machine process launch file
                launch_file = re.search("launch/state_machines/(.+?).launch", output).group(1)

                self._running_state_machine_edit.setText("%s (%s)" % (launch_file, hostname))
            except Exception as e:
                print e

            # self._running_state_machine_edit.setText(state_machine_node)
        else:
            self._running_state_machine_edit.setText("No state_machine node running ..")

    def trigger_configuration(self):
        pass

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
