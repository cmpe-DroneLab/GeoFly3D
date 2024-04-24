import json

from PyQt6 import QtWebEngineCore
from PyQt6.QtCore import pyqtSignal


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
                    self.coords_printed.emit(coords)  # Emit the coordinates
                else:
                    print("Invalid JSON structure: 'geometry' or 'coordinates' key not found.")
            except json.JSONDecodeError as e:
                print("Error decoding JSON:", e)
        else:
            print("Invalid message:", msg)
