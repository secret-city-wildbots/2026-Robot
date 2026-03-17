import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtWebEngineWidgets import QWebEngineView
import ctypes

# ---------------- CONFIG ----------------
URL = "http://localhost:5804"
BOTTOM_GAP = 200  # space for FRC Driver Station
# ----------------------------------------

# Define RECT manually
class RECT(ctypes.Structure):
    _fields_ = [("left", ctypes.c_long),
                ("top", ctypes.c_long),
                ("right", ctypes.c_long),
                ("bottom", ctypes.c_long)]

app = QApplication(sys.argv)

# Get screen size
screen_geometry = app.primaryScreen().geometry()
screen_width = screen_geometry.width()
screen_height = screen_geometry.height()

# Get taskbar height
SPI_GETWORKAREA = 0x0030
work_area = RECT()
ctypes.windll.user32.SystemParametersInfoW(SPI_GETWORKAREA, 0, ctypes.byref(work_area), 0)
taskbar_height = screen_height - work_area.bottom

# Calculate overlay height
overlay_height = screen_height - BOTTOM_GAP - taskbar_height

# Create main window
window = QMainWindow()
window.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
window.setGeometry(0, 0, screen_width, overlay_height)

# Create webview
webview = QWebEngineView()
webview.load(QUrl(URL))
window.setCentralWidget(webview)

window.show()
sys.exit(app.exec())