import logging
from PyQt5 import QtCore, QtWidgets, QtGui, uic
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtGui import QColor, QImage, QPixmap


# Thread-safe Version
class QPlainTextEditLogger(logging.Handler, QObject):
    appendPlainText = pyqtSignal(str)

    def __init__(self, parent):
        super(QPlainTextEditLogger, self).__init__()
        QObject.__init__(self)
        self.widget = QtWidgets.QPlainTextEdit(parent)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget.sizePolicy().hasHeightForWidth())
        self.widget.setSizePolicy(sizePolicy)
        self.widget.setMinimumSize(QtCore.QSize(0, 80))
        self.widget.setMaximumSize(QtCore.QSize(16777215, 300))
        self.widget.setObjectName("QPlainTextEditLogger")
        self.widget.setReadOnly(True)
        self.appendPlainText.connect(self.widget.appendPlainText)

    def setCurrentFontColor(self, color):
        try:
            old_format = self.widget.currentCharFormat()
            old_format.setForeground(color)
            self.widget.setCurrentCharFormat(old_format)
        except Exception:
            pass

    def emit(self, record):
        msg = self.format(record)
        if '[DEBUG]' in msg:
            self.setCurrentFontColor(QColor('#1f2015'))
        elif '[INFO]' in msg:
            self.setCurrentFontColor(QColor('#0640b2'))
        elif '[WARNING]' in msg:
            self.setCurrentFontColor(QColor('#ffbd2f'))
        elif '[ERROR]' in msg:
            self.setCurrentFontColor(QColor('#ff6058'))
        elif '[CRITICAL]' in msg:
            self.setCurrentFontColor(QColor('#d41226'))
        try:
            self.appendPlainText.emit(msg)
        except Exception:
            pass

    def write(self, m):
        pass
