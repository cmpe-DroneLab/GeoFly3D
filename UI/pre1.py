import folium
from PyQt6.QtWidgets import QWidget, QListWidgetItem
from PyQt6.QtWebEngineWidgets import QWebEngineView

from UI.web_engine_page import WebEnginePage
import UI.pre1_design

from UI.database import Mission, session, get_all_missions


class Pre1(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.pre1_design.Ui_Form()
        self.ui.setupUi(self)

        self.webView = QWebEngineView()
        self.setup_map(41, 29, 12)
        self.ui.v_lay_right.addWidget(self.webView)
        self.ui.btn_delete_mission.clicked.connect(self.delete_mission)
        self.ui.listWidget.itemClicked.connect(self.enable_buttons)
        self.ui.listWidget.itemSelectionChanged.connect(self.enable_buttons)
        self.refresh_mission_list()

    def setup_map(self, lat, lon, zoom):
        self.m = folium.Map(location=[lat, lon],
                            zoom_start=zoom,
                            control_scale=True,
                            )

        self.save_map()

    def save_map(self):
        self.m.save('./UI/pre1map.html')
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(open('./UI/pre1map.html').read())
        self.webView.show()

    def add_marker(self, lat, lon, popup_text):
        folium.Marker([lat, lon], popup=popup_text).add_to(self.m)
        self.save_map()

    def delete_mission(self):
        selected_items = self.ui.listWidget.selectedItems()
        for item in selected_items:
            # Mission ID'sini al
            mission_id = int(item.text().split(":")[1].split(",")[0].strip())
            # Veritabanından görevi bul
            mission = session.query(Mission).filter_by(mission_id=mission_id).first()
            if mission:
                # Görevi veritabanından sil
                session.delete(mission)
                session.commit()
                # Listeden görevi kaldır
                # self.ui.listWidget.takeItem(self.ui.listWidget.row(item))

                self.refresh_mission_list()

    def refresh_mission_list(self):
        self.ui.listWidget.clear()

        # Get all missions from the database
        missions = get_all_missions()

        # Clear existing markers from the map
        self.m = folium.Map(location=[39, 35], zoom_start=5, control_scale=True)


        for mission in missions:
            # Add missions to the Mission List
            item = QListWidgetItem(f"Mission ID: {mission.mission_id}, Status: {mission.mission_status}")
            self.ui.listWidget.addItem(item)

            # Get mission center coordinates
            lat = mission.center_lat
            lon = mission.center_lon

            # Add marker to the map
            if lat is not None:
                popup_text = f"Mission ID: {mission.mission_id}, Status: {mission.mission_status}"
                self.add_marker(lat, lon, popup_text)

            # Save and display the updated map
            self.save_map()

            # Disable buttons
            self.disable_buttons()

    def enable_buttons(self):
        self.ui.btn_edit_mission.setEnabled(True)
        self.ui.btn_delete_mission.setEnabled(True)

    def disable_buttons(self):
        self.ui.btn_edit_mission.setEnabled(False)
        self.ui.btn_delete_mission.setEnabled(False)
