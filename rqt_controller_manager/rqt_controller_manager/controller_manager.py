# Copyright 2013 Georgia Tech
# Copyright 2015 PAL Robotics S.L.
# Copyright 2022 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from controller_manager.controller_manager_services import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
    unload_controller,
)
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import SwitchController
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractTableModel, Qt, QTimer
from python_qt_binding.QtGui import QCursor, QFont, QIcon, QStandardItem, QStandardItemModel
from python_qt_binding.QtWidgets import QHeaderView, QMenu, QStyledItemDelegate, QWidget
from qt_gui.plugin import Plugin
from ros2param.api import call_get_parameters, call_list_parameters
from ros2service.api import get_service_names_and_types

from .update_combo import update_combo


class ControllerManager(Plugin):
    """Graphical frontend for interacting with the controller manager."""

    _cm_update_freq = 1  # Hz

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("ControllerManager")

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory("rqt_controller_manager"),
            "resource",
            "controller_manager.ui",
        )
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("ControllerManagerUi")

        # Pop-up that displays controller information
        self._popup_widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory("rqt_controller_manager"), "resource", "controller_info.ui"
        )
        loadUi(ui_file, self._popup_widget)
        self._popup_widget.setObjectName("ControllerInfoUi")

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(f"{self._widget.windowTitle()} {context.serial_number()}")
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Initialize members
        self._cm_name = ""  # Name of the selected controller manager's node
        self._controllers = []  # State of each controller
        self._table_model = None

        # Store reference to node
        self._node = context.node

        # Controller state icons
        path = get_package_share_directory("rqt_controller_manager")
        self._icons = {
            "active": QIcon(f"{path}/resource/led_green.png"),
            "finalized": QIcon(f"{path}/resource/led_off.png"),
            "inactive": QIcon(f"{path}/resource/led_red.png"),
            "unconfigured": QIcon(f"{path}/resource/led_off.png"),
        }

        # Controllers display
        table_view = self._widget.table_view
        table_view.setContextMenuPolicy(Qt.CustomContextMenu)
        table_view.customContextMenuRequested.connect(self._on_ctrl_menu)

        table_view.doubleClicked.connect(self._on_ctrl_info)

        header = table_view.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        header.setContextMenuPolicy(Qt.CustomContextMenu)
        header.customContextMenuRequested.connect(self._on_header_menu)

        # Timer for controller manager updates
        self._update_cm_list_timer = QTimer(self)
        self._update_cm_list_timer.setInterval(int(1000.0 / self._cm_update_freq))
        self._update_cm_list_timer.timeout.connect(self._update_cm_list)
        self._update_cm_list_timer.start()

        # Timer for running controller updates
        self._update_ctrl_list_timer = QTimer(self)
        self._update_ctrl_list_timer.setInterval(int(1000.0 / self._cm_update_freq))
        self._update_ctrl_list_timer.timeout.connect(self._update_controllers)
        self._update_ctrl_list_timer.start()

        # Signal connections
        w = self._widget
        w.cm_combo.currentIndexChanged[str].connect(self._on_cm_change)

    def shutdown_plugin(self):
        self._update_cm_list_timer.stop()
        self._update_ctrl_list_timer.stop()
        self._popup_widget.hide()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("cm_name", self._cm_name)

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore last session's controller_manager, if present
        self._update_cm_list()
        cm_name = instance_settings.value("cm_name")
        cm_combo = self._widget.cm_combo
        cm_list = [cm_combo.itemText(i) for i in range(cm_combo.count())]
        try:
            idx = cm_list.index(cm_name)
            cm_combo.setCurrentIndex(idx)
        except ValueError:
            pass

    def _update_cm_list(self):
        update_combo(self._widget.cm_combo, _list_controller_managers(self._node))

    def _on_cm_change(self, cm_name):
        self._cm_name = cm_name

        if cm_name:
            self._update_controllers()

    def _update_controllers(self):
        if not self._cm_name:
            return

        # Find controllers associated to the selected controller manager
        controllers = self._list_controllers()

        # Update controller display, if necessary
        if self._controllers != controllers:
            self._controllers = controllers
            self._show_controllers()  # NOTE: Model is recomputed from scratch

    def _list_controllers(self):
        """
        List the controllers associated to a controller manager node.

        @return List of controllers associated to a controller manager
        node. Contains both stopped/running controllers, as returned by
        the C{list_controllers} service, plus uninitialized controllers with
        returned by the C{list_parameters} service..
        @rtype [str]
        """
        # Add loaded controllers first
        controllers = list_controllers(self._node, self._cm_name).controller

        # Append potential controller configs found in the node's parameters
        for name in _get_parameter_controller_names(self._node, self._cm_name):
            add_ctrl = all(name != ctrl.name for ctrl in controllers)
            if add_ctrl:
                type_str = _get_controller_type(self._node, self._cm_name, name)
                uninit_ctrl = ControllerState(name=name, type=type_str)
                controllers.append(uninit_ctrl)
        return controllers

    def _show_controllers(self):
        table_view = self._widget.table_view
        self._table_model = ControllerTable(self._controllers, self._icons)
        table_view.setModel(self._table_model)

    def _on_ctrl_menu(self, pos):
        # Get data of selected controller
        row = self._widget.table_view.rowAt(pos.y())
        if row < 0:
            return  # Cursor is not under a valid item

        ctrl = self._controllers[row]

        # Show context menu
        menu = QMenu(self._widget.table_view)
        if ctrl.state == "active":
            action_deactivate = menu.addAction(self._icons["inactive"], "Deactivate")
            action_kill = menu.addAction(self._icons["finalized"], "Deactivate and Unload")
        elif ctrl.state == "inactive":
            action_activate = menu.addAction(self._icons["active"], "Activate")
            action_unload = menu.addAction(self._icons["unconfigured"], "Unload")
        elif ctrl.state == "unconfigured":
            action_configure = menu.addAction(self._icons["inactive"], "Configure")
            action_spawn = menu.addAction(self._icons["active"], "Configure and Activate")
        else:
            # Controller isn't loaded
            action_load = menu.addAction(self._icons["unconfigured"], "Load")
            action_configure = menu.addAction(self._icons["inactive"], "Load and Configure")
            action_activate = menu.addAction(self._icons["active"], "Load, Configure and Activate")

        action = menu.exec_(self._widget.table_view.mapToGlobal(pos))

        # Evaluate user action
        if ctrl.state == "active":
            if action is action_deactivate:
                self._deactivate_controller(ctrl.name)
            elif action is action_kill:
                self._deactivate_controller(ctrl.name)
                unload_controller(self._node, self._cm_name, ctrl.name)
        elif ctrl.state in ("finalized", "inactive"):
            if action is action_activate:
                self._activate_controller(ctrl.name)
            elif action is action_unload:
                unload_controller(self._node, self._cm_name, ctrl.name)
        elif ctrl.state == "unconfigured":
            if action is action_configure:
                configure_controller(self._node, self._cm_name, ctrl.name)
            elif action is action_spawn:
                load_controller(self._node, self._cm_name, ctrl.name)
                self._activate_controller(ctrl.name)
        else:
            # Assume controller isn't loaded
            if action is action_load:
                load_controller(self._node, self._cm_name, ctrl.name)
            elif action is action_configure:
                load_controller(self._node, self._cm_name, ctrl.name)
                configure_controller(self._node, self._cm_name, ctrl.name)
            elif action is action_activate:
                load_controller(self._node, self._cm_name, ctrl.name)
                configure_controller(self._node, self._cm_name, ctrl.name)
                self._activate_controller(ctrl.name)

    def _on_ctrl_info(self, index):
        popup = self._popup_widget

        ctrl = self._controllers[index.row()]
        popup.ctrl_name.setText(ctrl.name)
        popup.ctrl_type.setText(ctrl.type)

        res_model = QStandardItemModel()
        model_root = QStandardItem("Claimed Interfaces")
        res_model.appendRow(model_root)
        for claimed_interface in ctrl.claimed_interfaces:
            hw_iface_item = QStandardItem(claimed_interface)
            model_root.appendRow(hw_iface_item)

        popup.resource_tree.setModel(res_model)
        popup.resource_tree.setItemDelegate(FontDelegate(popup.resource_tree))
        popup.resource_tree.expandAll()
        popup.move(QCursor.pos())
        popup.show()

    def _on_header_menu(self, pos):
        header = self._widget.table_view.horizontalHeader()

        # Show context menu
        menu = QMenu(self._widget.table_view)
        action_toggle_auto_resize = menu.addAction("Toggle Auto-Resize")
        action = menu.exec_(header.mapToGlobal(pos))

        # Evaluate user action
        if action is action_toggle_auto_resize:
            if header.resizeMode(0) == QHeaderView.ResizeToContents:
                header.setSectionResizeMode(QHeaderView.Interactive)
            else:
                header.setSectionResizeMode(QHeaderView.ResizeToContents)

    def _activate_controller(self, name):
        switch_controllers(
            node=self._node,
            controller_manager_name=self._cm_name,
            deactivate_controllers=[],
            activate_controllers=[name],
            strict=SwitchController.Request.STRICT,
            activate_asap=False,
            timeout=0.3,
        )

    def _deactivate_controller(self, name):
        switch_controllers(
            node=self._node,
            controller_manager_name=self._cm_name,
            deactivate_controllers=[name],
            activate_controllers=[],
            strict=SwitchController.Request.STRICT,
            activate_asap=False,
            timeout=0.3,
        )


class ControllerTable(QAbstractTableModel):
    """
    Model containing controller information for tabular display.

    The model allows display of basic read-only information like controller
    name and state.
    """

    def __init__(self, controller_info, icons, parent=None):
        QAbstractTableModel.__init__(self, parent)
        self._data = controller_info
        self._icons = icons

    def rowCount(self, parent):
        return len(self._data)

    def columnCount(self, parent):
        return 2

    def headerData(self, col, orientation, role):
        if orientation != Qt.Horizontal or role != Qt.DisplayRole:
            return None
        if col == 0:
            return "controller"
        elif col == 1:
            return "state"

    def data(self, index, role):
        if not index.isValid():
            return None

        ctrl = self._data[index.row()]

        if role == Qt.DisplayRole:
            if index.column() == 0:
                return ctrl.name
            elif index.column() == 1:
                return ctrl.state or "not loaded"

        if role == Qt.DecorationRole and index.column() == 0:
            return self._icons.get(ctrl.state)

        if role == Qt.FontRole and index.column() == 0:
            bf = QFont()
            bf.setBold(True)
            return bf

        if role == Qt.TextAlignmentRole and index.column() == 1:
            return Qt.AlignCenter


class FontDelegate(QStyledItemDelegate):
    """
    Simple delegate for customizing font weight and italization.

    Simple delegate for customizing font weight and italization when displaying resources claimed
    by a controller.
    """

    def paint(self, painter, option, index):
        if not index.parent().isValid():
            # Root level
            option.font.setWeight(QFont.Bold)
        if index.parent().isValid() and not index.parent().parent().isValid():
            # Hardware interface level
            option.font.setItalic(True)
            option.font.setWeight(QFont.Bold)
        QStyledItemDelegate.paint(self, painter, option, index)


def _get_controller_type(node, node_name, ctrl_name):
    """
    Get the controller's type from the controller manager node with the call_get_parameter service.

    @param node_name Controller manager node's name
    @type node_name str
    @param ctrl_name Controller name
    @type ctrl_name str
    @return Controller type
    @rtype str
    """
    response = call_get_parameters(node=node, node_name=node_name, parameter_names=[ctrl_name])
    return response.values[0].string_value if response.values else ""


def _list_controller_managers(node):
    """
    List controller manager nodes that are active.

    Does this by looking for a service that should be exclusive to a controller manager node.
    The "list_controllers" service is used to determine if a node is a controller manager.
    @return List of controller manager node names
    @rtype list of str
    """
    return [
        name.rstrip("list_controllers").rstrip("/")
        for name, _ in get_service_names_and_types(node=node)
        if name.endswith("list_controllers")
    ]


def _get_parameter_controller_names(node, node_name):
    """Get list of ROS parameter names that potentially represent a controller configuration."""
    parameter_names = call_list_parameters(node=node, node_name=node_name)
    suffix = ".type"
    return [n[: -len(suffix)] for n in parameter_names.result().result.names if n.endswith(suffix)]
