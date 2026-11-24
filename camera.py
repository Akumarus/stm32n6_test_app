#!/usr/bin/env python3
"""Просмотр кадров по USB CDC от STM32N6 (IMX335) на Qt.

Протокол: magic(4) | w u16 | h u16 | size u32 | id u32 | данные
    'FRM1' — RGB565 little-endian
    'JPG1' — JPEG

В списке портов показываются только ACM/USB устройства:
ttyACM*, ttyUSB*, usbmodem*, usbserial*, COM* с USB VID/PID.

pip install pyserial numpy pillow PySide6
(также работает с PyQt6/PyQt5, если PySide6 не установлен)
"""
import io
import struct
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports
from PIL import Image

try:                                    # Qt-биндинги
    from PySide6 import QtCore, QtGui, QtWidgets
    _Signal = QtCore.Signal
except ImportError:
    try:
        from PyQt6 import QtCore, QtGui, QtWidgets
        _Signal = QtCore.pyqtSignal
    except ImportError:
        try:
            from PyQt5 import QtCore, QtGui, QtWidgets
            _Signal = QtCore.pyqtSignal
        except ImportError as e:
            raise SystemExit(
                "Не найден Qt для Python. Установите: pip install PySide6") from e


# Совместимость PyQt5/PyQt6/PySide6: scoped-enum или обычный enum
def _qt_enum(owner, scoped, plain):
    obj = owner
    for part in scoped.split("."):
        obj = getattr(obj, part, None)
        if obj is None:
            break
    return obj if obj is not None else getattr(owner, plain)


ALIGN_CENTER = _qt_enum(QtCore.Qt, "AlignmentFlag.AlignCenter", "AlignCenter")
HORIZONTAL = _qt_enum(QtCore.Qt, "Orientation.Horizontal", "Horizontal")
KEEP_ASPECT = _qt_enum(QtCore.Qt, "AspectRatioMode.KeepAspectRatio", "KeepAspectRatio")
SMOOTH = _qt_enum(QtCore.Qt, "TransformationMode.SmoothTransformation", "SmoothTransformation")
IGNORED = _qt_enum(QtWidgets.QSizePolicy, "Policy.Ignored", "Ignored")
RGB888 = _qt_enum(QtGui.QImage, "Format.Format_RGB888", "Format_RGB888")

MAGICS = (b"FRM1", b"JPG1")
HDR = struct.Struct("<4sHHII")


# --------------------------------------------------------------------------
# Порты: только ACM и USB
# --------------------------------------------------------------------------
_ACM_USB_MARKERS = ("ttyacm", "ttyusb", "usbmodem", "usbserial", "usb")


def _is_acm_or_usb(info):
    dev = (info.device or "").lower()
    hwid = (info.hwid or "").lower()
    desc = (info.description or "").lower()
    if any(k in dev for k in _ACM_USB_MARKERS):
        return True
    if getattr(info, "vid", None) is not None:      # USB VID/PID определён
        return True
    return "usb" in hwid or "acm" in hwid or "acm" in desc or "usb" in desc


def _port_priority(info):
    dev = (info.device or "").lower()
    hwid = (info.hwid or "").lower()
    # CDC ACM (STM32) — выше остальных USB-серийников
    return (0 if ("ttyacm" in dev or "acm" in hwid) else 1, info.device)


def list_ports():
    ports = [p for p in serial.tools.list_ports.comports() if _is_acm_or_usb(p)]
    return sorted(ports, key=_port_priority)


# --------------------------------------------------------------------------
# Фоновое чтение порта
# --------------------------------------------------------------------------
class Reader(QtCore.QThread):
    frame_received = _Signal(object)    # (magic, fid, w, h, data)
    error = _Signal(str)

    def __init__(self, port, parent=None):
        super().__init__(parent)
        self.port = port
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    @staticmethod
    def _find(buf):
        idx = [i for i in (buf.find(m) for m in MAGICS) if i >= 0]
        return min(idx) if idx else -1

    def run(self):
        try:
            ser = serial.Serial(self.port, timeout=0.2)
        except Exception as e:
            self.error.emit(f"Не удалось открыть {self.port}: {e}")
            return

        buf = bytearray()
        try:
            while not self._stop.is_set():
                chunk = ser.read(max(1, ser.in_waiting))
                if chunk:
                    buf += chunk
                while True:
                    i = self._find(buf)
                    if i < 0:
                        del buf[:-3]                    # хвост на случай разрыва magic
                        break
                    if i > 0:
                        del buf[:i]
                    if len(buf) < HDR.size:
                        break
                    magic, w, h, size, fid = HDR.unpack_from(buf)
                    ok = (size == w * h * 2) if magic == b"FRM1" \
                        else (0 < size < 8 * 1024 * 1024)
                    if not ok:
                        del buf[:4]                     # ложный magic
                        continue
                    if len(buf) < HDR.size + size:
                        break
                    data = bytes(buf[HDR.size:HDR.size + size])
                    del buf[:HDR.size + size]
                    if magic == b"JPG1" and not data.startswith(b"\xff\xd8"):
                        continue                        # битый JPEG
                    self.frame_received.emit((magic, fid, w, h, data))
        except Exception as e:
            self.error.emit(f"Ошибка порта: {e}")
        finally:
            try:
                ser.close()
            except Exception:
                pass


# --------------------------------------------------------------------------
# Декодирование кадра
# --------------------------------------------------------------------------
def rgb565_to_rgb(data, w, h, gain):
    v = np.frombuffer(data, dtype="<u2").reshape(h, w)
    r = ((v >> 11) & 0x1F).astype(np.uint16)
    g = ((v >> 5) & 0x3F).astype(np.uint16)
    b = (v & 0x1F).astype(np.uint16)
    r = (r << 3) | (r >> 2)
    g = (g << 2) | (g >> 4)
    b = (b << 3) | (b >> 2)
    rgb = np.dstack([r, g, b]) * gain
    return np.clip(rgb, 0, 255).astype(np.uint8), int(v.min()), int(v.max())


def pil_to_qimage(img):
    img = img.convert("RGB")
    w, h = img.size
    return QtGui.QImage(img.tobytes(), w, h, 3 * w, RGB888).copy()


# --------------------------------------------------------------------------
# Виджеты
# --------------------------------------------------------------------------
class ImageView(QtWidgets.QLabel):
    """QLabel, который масштабирует картинку под размер окна."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(ALIGN_CENTER)
        self.setMinimumSize(320, 240)
        self.setStyleSheet("background:#202020;")
        self.setSizePolicy(IGNORED, IGNORED)
        self._image = None

    def set_image(self, image):
        self._image = image
        self._update_pixmap()

    def _update_pixmap(self):
        if self._image is None or self._image.isNull():
            return
        pm = QtGui.QPixmap.fromImage(self._image)
        pm = pm.scaled(self.size(), KEEP_ASPECT, SMOOTH)
        self.setPixmap(pm)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()


class PortCombo(QtWidgets.QComboBox):
    """QComboBox, который обновляет список портов при открытии."""

    popup_about_to_show = _Signal()

    def showPopup(self):
        self.popup_about_to_show.emit()
        super().showPopup()


class App(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMX335 viewer (JPEG / RGB565)")
        self.resize(960, 720)

        self.reader = None
        self.frame = None           # последний сырой кадр
        self.qimage = None          # текущая картинка (для сохранения)
        self.fps = 0.0
        self._fps_t = time.time()
        self._fps_n = 0

        self._build_ui()
        self._refresh_ports()

    # ---------------- UI ----------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        bar = QtWidgets.QHBoxLayout()

        bar.addWidget(QtWidgets.QLabel("Порт (ACM/USB):"))
        self.port_cb = PortCombo()
        self.port_cb.setMinimumWidth(220)
        self.port_cb.popup_about_to_show.connect(self._refresh_ports)
        bar.addWidget(self.port_cb)

        self.btn_refresh = QtWidgets.QPushButton("Обновить")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        bar.addWidget(self.btn_refresh)

        self.btn_conn = QtWidgets.QPushButton("Подключиться")
        self.btn_conn.setCheckable(True)
        self.btn_conn.clicked.connect(self._toggle)
        bar.addWidget(self.btn_conn)

        bar.addSpacing(16)
        bar.addWidget(QtWidgets.QLabel("Яркость x (RGB565):"))
        self.gain = QtWidgets.QSlider(HORIZONTAL)
        self.gain.setRange(10, 160)             # 1.0 .. 16.0
        self.gain.setValue(10)
        self.gain.setFixedWidth(140)
        self.gain.valueChanged.connect(self._gain_changed)
        bar.addWidget(self.gain)
        self.gain_lbl = QtWidgets.QLabel("1.0")
        self.gain_lbl.setFixedWidth(32)
        bar.addWidget(self.gain_lbl)

        bar.addStretch(1)
        self.btn_save = QtWidgets.QPushButton("Сохранить")
        self.btn_save.clicked.connect(self._save)
        bar.addWidget(self.btn_save)

        root.addLayout(bar)

        self.view = ImageView()
        root.addWidget(self.view, 1)

        self.status = QtWidgets.QLabel("Не подключено")
        self.status.setContentsMargins(4, 0, 4, 0)
        root.addWidget(self.status)

    def _gain(self):
        return self.gain.value() / 10.0

    # ---------------- Порты ----------------
    def _refresh_ports(self):
        ports = list_ports()
        current = self.port_cb.currentData()

        self.port_cb.blockSignals(True)
        self.port_cb.clear()
        for p in ports:
            label = f"{p.device} — {p.description}" if p.description else p.device
            self.port_cb.addItem(label, p.device)
        if current:
            idx = self.port_cb.findData(current)
            if idx >= 0:
                self.port_cb.setCurrentIndex(idx)
        self.port_cb.blockSignals(False)

        if self.reader is None:
            self.btn_conn.setEnabled(bool(ports))
            if not ports:
                self.status.setText("Не найдено ACM/USB портов (ttyACM / ttyUSB / COM-USB)")

    # ---------------- Подключение ----------------
    def _toggle(self):
        if self.reader is not None:
            self._disconnect()
            return
        port = self.port_cb.currentData()
        if not port:
            self.btn_conn.setChecked(False)
            QtWidgets.QMessageBox.warning(self, "Порт", "Выберите ACM/USB порт")
            return
        self._connect(port)

    def _connect(self, port):
        self.reader = Reader(port)
        self.reader.frame_received.connect(self._on_frame)
        self.reader.error.connect(self._on_error)
        self.reader.start()

        self.btn_conn.setChecked(True)
        self.btn_conn.setText("Отключиться")
        self.port_cb.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        self.status.setText(f"Подключено к {port}, жду кадр...")

    def _disconnect(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader.wait()          # read() имеет timeout 0.2 c
            self.reader = None
        self.btn_conn.setChecked(False)
        self.btn_conn.setText("Подключиться")
        self.port_cb.setEnabled(True)
        self.btn_refresh.setEnabled(True)
        self.btn_conn.setEnabled(self.port_cb.count() > 0)
        self.status.setText("Не подключено")

    def _on_error(self, message):
        if self.reader is not None:
            self._disconnect()
        QtWidgets.QMessageBox.critical(self, "Ошибка", message)

    # ---------------- Кадры ----------------
    def _on_frame(self, frame):
        self.frame = frame
        self._fps_n += 1
        now = time.time()
        if now - self._fps_t >= 1.0:
            self.fps = self._fps_n / (now - self._fps_t)
            self._fps_n = 0
            self._fps_t = now
        self._render()

    def _render(self):
        if self.frame is None:
            return
        magic, fid, w, h, data = self.frame
        try:
            if magic == b"JPG1":
                self.qimage = pil_to_qimage(Image.open(io.BytesIO(data)))
                extra = f"JPEG {len(data) // 1024} KB"
            else:
                rgb, mn, mx = rgb565_to_rgb(data, w, h, self._gain())
                hh, ww, _ = rgb.shape
                self.qimage = QtGui.QImage(rgb.tobytes(), ww, hh, 3 * ww, RGB888).copy()
                extra = f"RGB565 min {mn} max {mx}"
        except Exception as e:
            self.status.setText(f"Ошибка декодирования кадра {fid}: {e}")
            return

        self.view.set_image(self.qimage)
        self.status.setText(f"кадр {fid}   {w}x{h}   {self.fps:.1f} fps   {extra}")

    def _gain_changed(self, _value):
        self.gain_lbl.setText(f"{self._gain():.1f}")
        if self.frame is not None and self.frame[0] == b"FRM1":
            self._render()

    # ---------------- Сохранение ----------------
    def _save(self):
        if self.qimage is None:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Сохранить кадр", "",
            "PNG (*.png);;JPEG (*.jpg *.jpeg)")
        if not path:
            return
        fmt = "JPEG" if path.lower().endswith((".jpg", ".jpeg")) else "PNG"
        if not self.qimage.save(path, fmt, 95):
            QtWidgets.QMessageBox.warning(self, "Сохранение",
                                          f"Не удалось сохранить {path}")

    # ---------------- Закрытие ----------------
    def closeEvent(self, event):
        self._disconnect()
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication([])
    win = App()
    win.show()
    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()
