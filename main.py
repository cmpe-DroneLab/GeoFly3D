import json
import math
import sys

import folium
from PyQt6 import QtWebEngineCore
from PyQt6.QtCore import pyqtSignal, QSize
from PyQt6.QtWidgets import QApplication, QMainWindow, QDialog, QListWidgetItem, QWidget, QLabel
from PyQt6.QtWebEngineWidgets import QWebEngineView

from UI import draw
from UI.drone_dialog import Ui_drone_dialog
from UI.preflight2 import Ui_MainWindow
from UI.drone import Ui_Form

from RoutePlanner import route_planner


class GeoFly3D(QMainWindow):

    def __init__(self):
        super().__init__()
        self.pf_ui = Ui_MainWindow()
        self.pf_ui.setupUi(self)

        self.webView = QWebEngineView()
        self.load_map(41.085974, 29.044456)
        self.pf_ui.v_lay_right.addWidget(self.webView)

        self.pf_ui.slider_altitude.valueChanged.connect(self.slider_altitude_changed)
        self.pf_ui.spinbox_altitude.valueChanged.connect(self.spinbox_altitude_changed)
        self.pf_ui.btn_add_drone.clicked.connect(self.add_drone)
        self.pf_ui.btn_delete_drone.clicked.connect(self.delete_drone)
        self.pf_ui.btn_start.clicked.connect(self.start_mission)

        # Connect the signal from WebEnginePage to slot in GeoFly3D
        self.webView.page().coords_printed.connect(self.update_label_area)
        self.coords = []

    def load_map(self, lat, lon):
        m = folium.Map(location=[lat, lon],
                       zoom_start=18,
                       control_scale=True,
                       )
        drw = draw.Draw(export=True,
                        show_geometry_on_click=False,
                        filename='my_data.geojson',
                        draw_options={'polyline': False,
                                      'polygone': True,
                                      'rectangle': False,
                                      'circle': False,
                                      'marker': False,
                                      'circlemarker': False})

        m.add_child(drw)
        m.save('./UI/map.html')
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(open('./UI/map.html').read())
        self.webView.show()

    def add_drone(self):
        # Open Dialog
        dialog_win = QDialog()
        dialog_ui = Ui_drone_dialog()
        dialog_ui.setupUi(dialog_win)
        dialog_win.setWindowTitle("Add Drone")
        dialog_ui.btn_save.setText("Add")
        dialog_ui.spin_spare_batt.setValue(0)
        response = dialog_win.exec()

        if response == 1:
            # Get values from dialog
            new_drone_type = dialog_ui.combo_model.currentText()
            new_drone_spare = dialog_ui.spin_spare_batt.value()

            new_drone_item = QListWidgetItem()
            new_drone_item.setSizeHint(QSize(self.pf_ui.listWidget.lineWidth(), 60))
            new_drone_widget = QWidget()
            new_drone_ui = Ui_Form()
            new_drone_ui.setupUi(new_drone_widget)
            new_drone_ui.label_drone_model.setText(new_drone_type)
            new_drone_ui.label_spare_batt.setText(str(new_drone_spare))
            self.pf_ui.listWidget.addItem(new_drone_item)
            self.pf_ui.listWidget.setItemWidget(new_drone_item, new_drone_widget)

            self.update_metrics()

    def delete_drone(self):
        if len(self.pf_ui.listWidget.selectedIndexes()):
            selected_items = self.pf_ui.listWidget.selectedItems()
            for item in selected_items:
                self.pf_ui.listWidget.removeItemWidget(item)
                self.pf_ui.listWidget.takeItem(self.pf_ui.listWidget.row(item))
            self.update_metrics()

    def slider_altitude_changed(self):
        value = self.pf_ui.slider_altitude.value()
        self.pf_ui.spinbox_altitude.setValue(value)
        self.update_metrics()

    def spinbox_altitude_changed(self):
        value = self.pf_ui.spinbox_altitude.value()
        self.pf_ui.slider_altitude.setValue(value)
        self.update_metrics()

    def update_label_area(self, coords):
        # Update the label_area text with the coordinates
        area = self.calculate_area(coords)
        self.pf_ui.selected_area_value.setText(f"{area:.0f}")
        self.update_metrics()
        self.coords = coords.copy()


    def update_metrics(self):
        self.calculate_required_capacity()
        self.calculate_mission_time()
        self.calculate_provided_capacity()

    def start_mission(self):
        altitude_val = self.pf_ui.slider_altitude.value()
        route_planner.plan_route(self.coords, altitude=altitude_val, intersection_ratio=0.8, angle_deg=20)

    @staticmethod
    def calculate_area(coords):
        area = 0.0
        if len(coords) > 2:
            for i in range(len(coords) - 1):
                p1 = coords[i]
                p2 = coords[i + 1]
                area += math.radians(p2[0] - p1[0]) * (
                            2 + math.sin(math.radians(p1[1])) + math.sin(math.radians(p2[1])))
            area = area * 6378137.0 * 6378137.0 / 2.0
        return abs(area)

    def calculate_mission_time(self):
        num_of_drones = self.pf_ui.listWidget.count()
        required_capacity = self.pf_ui.batt_required_value.text()
        if len(required_capacity) and num_of_drones:
            mission_time = int(required_capacity) / num_of_drones
            self.pf_ui.mission_time_value.setText(f"{mission_time:.0f}")
        else:
            self.pf_ui.mission_time_value.setText("")

    def calculate_required_capacity(self):
        altitude = self.pf_ui.slider_altitude.value()
        if len(self.pf_ui.selected_area_value.text()):
            area = float(self.pf_ui.selected_area_value.text())
            required_capacity = 3 + area / altitude ** 2
            self.pf_ui.batt_required_value.setText(f"{required_capacity:.0f}")
        else:
            self.pf_ui.batt_required_value.setText("")

    def calculate_provided_capacity(self):
        total_battery = 0
        num_of_drones = self.pf_ui.listWidget.count()
        if num_of_drones:
            for i in range(num_of_drones):
                drone_item = self.pf_ui.listWidget.item(i)
                drone_widget = self.pf_ui.listWidget.itemWidget(drone_item)
                label_spare_batt = drone_widget.findChild(QLabel, "label_spare_batt")
                spare_battery_count = int(label_spare_batt.text())
                total_battery = total_battery + spare_battery_count
            self.pf_ui.batt_provided_value.setText(str(15 * total_battery))
        else:
            self.pf_ui.batt_provided_value.setText("")


class WebEnginePage(QtWebEngineCore.QWebEnginePage):
    coords_printed = pyqtSignal(list)

    def javaScriptConsoleMessage(self, level, msg, line, sourceID):
        # Check if msg is not empty and is a string
        if msg and isinstance(msg, str):
            try:
                coords_dict = json.loads(msg)
                # Check if the parsed JSON contains the expected structure
                if 'geometry' in coords_dict and 'coordinates' in coords_dict['geometry']:
                    coords = coords_dict['geometry']['coordinates'][0]
                    print(coords)
                    self.coords_printed.emit(coords)  # Emit the coordinates
                else:
                    print("Invalid JSON structure: 'geometry' or 'coordinates' key not found.")
            except json.JSONDecodeError as e:
                print("Error decoding JSON:", e)
        else:
            print("Invalid message:", msg)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GeoFly3D()
    window.show()
    sys.exit(app.exec())
