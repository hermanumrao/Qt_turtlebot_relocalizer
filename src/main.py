import sys
from PyQt6.QtWidgets import QApplication
from dashboard_window import TurtleBotDashboard

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TurtleBotDashboard()
    window.show()
    sys.exit(app.exec())
