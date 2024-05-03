import json

from PyQt6 import QtWebEngineCore
from PyQt6.QtCore import pyqtSignal


class WebEnginePage(QtWebEngineCore.QWebEnginePage):
    coords_printed = pyqtSignal(list)  # Signal emitted when coordinates are printed
    shapes_deleted = pyqtSignal()  # Signal emitted when shapes are deleted

    def javaScriptConsoleMessage(self, level, msg, line, sourceID):
        # Check if the message is a non-empty string
        if msg and isinstance(msg, str):
            try:

                # Check if the message is "drawings deleted"
                if msg == "drawings deleted":
                    # Emit the shapes deleted signal
                    self.shapes_deleted.emit()

                # Attempt to parse the message as JSON
                else:
                    coords_dict = json.loads(msg)
                    # Check if the JSON structure contains the expected keys
                    if 'geometry' in coords_dict and 'coordinates' in coords_dict['geometry']:
                        # Extract coordinates from the parsed JSON
                        coords = coords_dict['geometry']['coordinates'][0]
                        # Emit the coordinates signal with the extracted coordinates
                        self.coords_printed.emit(coords)
                    else:
                        # Print an error message if the expected keys are not found
                        print("Invalid JSON structure: 'geometry' or 'coordinates' key not found.")
            except json.JSONDecodeError as e:
                # Print an error message if JSON decoding fails
                print("Error decoding JSON:", e)
        else:
            # Print an error message for invalid messages
            print("Invalid message:", msg)
