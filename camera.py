#!/usr/bin/env python3
"""
Camera viewer для STM32H743 + OV2640 (USB CDC).

Протокол (текстовые команды, строки заканчиваются \n):
    size <N>   -> "OK\r\n"            выбрать разрешение (N = значение enum framesize_t)
    mono <0|1> -> "OK\r\n"            цвет / чёрно-белый режим (встроенный эффект OV2640)
    snap       -> "IMG <len>\r\n" + <len байт JPEG>      один кадр
    start      -> поток кадров "IMG <len>\r\n" + JPEG, один за другим
    stop       -> "OK\r\n"            остановить поток (приходит между кадрами)

В окне показываются: размер кадра (последний / avg / min / max), битрейт
(за последнюю секунду / avg / peak) и качество сжатия JPEG. Качество берётся
из таблиц квантования (DQT) самого JPEG и пересчитывается в эквивалент IJG, %.

Зависимости:  pip install PyQt6 pyserial
"""
import os
import struct
import sys
import threading
import time
from collections import deque
from functools import lru_cache

# обход падения Qt+GTK на некоторых системах (GLib: no key 'antialiasing')
os.environ.setdefault("QT_QPA_PLATFORMTHEME", "generic")

import serial
import serial.tools.list_ports
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QHBoxLayout, QLabel, QMainWindow,
    QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)

# (подпись, значение framesize_t из camera.h) - только размеры, которые разрешает OV2640
RESOLUTIONS = [
    ("160 x 120  (QQVGA)", 9),
    ("320 x 240  (QVGA)", 10),
    ("352 x 240  (SIF)", 6),
    ("352 x 288  (CIF)", 3),
    ("480 x 320  (HVGA)", 15),
    ("640 x 480  (VGA)", 11),
    ("752 x 480  (WVGA2)", 23),
    ("800 x 600  (SVGA)", 24),
    ("1024 x 768  (XGA)", 25),
    ("1280 x 720  (HD 720p, 16:9)", 28),
    ("1280 x 1024  (SXGA)", 26),
    ("1600 x 1200  (UXGA)", 27),
]

MAX_JPEG = 1_000_000          # защита от мусора в заголовке
SNAP_TIMEOUT = 5.0            # сек на получение кадра
SIZE_TIMEOUT = 5.0            # сек на смену разрешения (камера переинициализируется)
FRAME_TIMEOUT = 3.0           # сек без кадров в режиме видео -> ошибка
STOP_TIMEOUT = 3.0            # сек на подтверждение stop


# --------------------------------------------------------------------------
# Качество JPEG: читаем таблицу квантования (DQT) прямо из кадра
# --------------------------------------------------------------------------
# Стандартная таблица яркости из спецификации JPEG (Annex K), q = 50
_STD_LUMA = [
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77,
    24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103, 99,
]


def _zigzag_order():
    pts = sorted(((r, c) for r in range(8) for c in range(8)),
                 key=lambda p: (p[0] + p[1], p[0] if (p[0] + p[1]) % 2 else -p[0]))
    return [r * 8 + c for r, c in pts]


_STD_LUMA_ZZ = [_STD_LUMA[i] for i in _zigzag_order()]    # в DQT значения лежат в zigzag-порядке


def _ijg_table(q):
    scale = 5000 // q if q < 50 else 200 - 2 * q
    return [min(255, max(1, (v * scale + 50) // 100)) for v in _STD_LUMA_ZZ]


_IJG_TABLES = {q: _ijg_table(q) for q in range(1, 101)}


def read_dqt(data):
    """Таблицы квантования JPEG: {id: [64 значения в zigzag-порядке]}."""
    tables = {}
    i, n = 2, len(data)
    while i + 4 <= n and data[i] == 0xFF:
        marker = data[i + 1]
        if marker == 0xFF:                       # заполняющие байты
            i += 1
            continue
        if marker in (0xDA, 0xD9):               # SOS / EOI - дальше только данные
            break
        seglen = (data[i + 2] << 8) | data[i + 3]
        if marker == 0xDB:
            p, end = i + 4, i + 2 + seglen
            while p < end:
                precision, table_id = data[p] >> 4, data[p] & 0x0F
                p += 1
                size = 128 if precision else 64
                if p + size > end:
                    break
                if precision:
                    tables[table_id] = list(struct.unpack_from(">64H", data, p))
                else:
                    tables[table_id] = list(data[p:p + 64])
                p += size
        i += 2 + seglen
    return tables


@lru_cache(maxsize=32)
def _match_ijg(luma):
    best_q, best_err = 50, None
    for q, table in _IJG_TABLES.items():
        err = sum(abs(a - b) for a, b in zip(luma, table))
        if best_err is None or err < best_err:
            best_q, best_err = q, err
    return best_q, sum(luma) / 64.0


def jpeg_quality(data):
    """(эквивалент IJG quality 1..100, среднее значение таблицы яркости) или None.

    Чем больше число, тем лучше качество. Среднее значение таблицы - «сырой»
    параметр: чем оно меньше, тем слабее сжатие. У OV2640 таблицы не строго
    стандартные, поэтому процент приблизительный.
    """
    try:
        luma = read_dqt(data).get(0)
    except (IndexError, struct.error):
        return None
    if not luma or len(luma) != 64:
        return None
    return _match_ijg(tuple(luma))


# --------------------------------------------------------------------------
# Статистика размера кадра и битрейта (без Qt)
# --------------------------------------------------------------------------
class StreamStats:
    """Считает по размеру JPEG (без заголовка «IMG <len>»).

    Битрейт: rate - за последнее окно ~1 с, avg - от первого кадра, peak - максимум по окнам.
    """
    WINDOW = 1.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.n = 0
        self.total = 0                  # все байты
        self.last = 0
        self.min = None
        self.max = 0
        self.t_first = None
        self.t_last = None
        self.bytes_after_first = 0      # байты после первого кадра (для avg-битрейта)
        self.win_start = None
        self.win_bytes = 0
        self.rate_bps = None            # бит/с, последнее завершённое окно
        self.peak_bps = None

    def add(self, size, now):
        self.n += 1
        self.total += size
        self.last = size
        self.min = size if self.min is None else min(self.min, size)
        self.max = max(self.max, size)
        self.t_last = now
        if self.t_first is None:        # отсчёт идёт от прихода первого кадра
            self.t_first = self.win_start = now
            return
        self.bytes_after_first += size
        self.win_bytes += size
        dt = now - self.win_start
        if dt >= self.WINDOW:
            self.rate_bps = self.win_bytes * 8 / dt
            self.peak_bps = max(self.peak_bps or 0.0, self.rate_bps)
            self.win_start = now
            self.win_bytes = 0

    def snapshot(self):
        avg_bps = None
        if self.n >= 2 and self.t_last > self.t_first:
            avg_bps = self.bytes_after_first * 8 / (self.t_last - self.t_first)
        return {
            "last": self.last,
            "avg": self.total / self.n if self.n else None,
            "min": self.min,
            "max": self.max if self.n else None,
            "rate_bps": self.rate_bps,
            "avg_bps": avg_bps,
            "peak_bps": self.peak_bps,
        }


def format_stats(s, quality):
    """Текст для панели статистики. s - snapshot() или None, quality - jpeg_quality() или None."""
    kb = lambda v: f"{v / 1024:.1f}"
    kbit = lambda v: "—" if v is None else f"{v / 1000:.0f}"

    if s is None:
        size_line = "Кадр:      — КБ"
        rate_line = "Битрейт:   — кбит/с"
    else:
        size_line = f"Кадр:      {kb(s['last'])} КБ"
        if s["avg"] is not None and s["min"] is not None:
            size_line += (f"   (avg {kb(s['avg'])} | min {kb(s['min'])} | max {kb(s['max'])})")
        if s["rate_bps"] is None and s["avg_bps"] is None:
            rate_line = "Битрейт:   — кбит/с"
        else:
            now_part = "—" if s["rate_bps"] is None else \
                f"{kbit(s['rate_bps'])} кбит/с ({s['rate_bps'] / 8 / 1024:.0f} КБ/с)"
            rate_line = (f"Битрейт:   {now_part}   "
                         f"(avg {kbit(s['avg_bps'])} | peak {kbit(s['peak_bps'])})")

    if quality is None:
        q_line = "Качество:  — (нет DQT в кадре)"
    else:
        q, mean_q = quality
        q_line = f"Качество:  ~{q} %  (экв. IJG; ср. значение таблицы квантования {mean_q:.1f})"
    return "\n".join((size_line, rate_line, q_line))


# --------------------------------------------------------------------------
# Протокол (без Qt, чтобы можно было тестировать отдельно)
# --------------------------------------------------------------------------
def _read_line(ser, deadline):
    buf = bytearray()
    while time.monotonic() < deadline:
        c = ser.read(1)
        if not c:
            continue
        if c == b"\n":
            return buf.decode(errors="replace").strip()
        buf += c
    raise TimeoutError("нет ответа от устройства")


def set_size(ser, size_id):
    ser.reset_input_buffer()
    ser.write(f"size {size_id}\n".encode())
    deadline = time.monotonic() + SIZE_TIMEOUT
    while True:
        line = _read_line(ser, deadline)
        if line == "OK":
            return
        if line == "ERR":
            raise RuntimeError("камера отклонила разрешение")


def set_mono(ser, on):
    ser.reset_input_buffer()
    ser.write(f"mono {1 if on else 0}\n".encode())
    deadline = time.monotonic() + SIZE_TIMEOUT
    while True:
        line = _read_line(ser, deadline)
        if line == "OK":
            return
        if line == "ERR":
            raise RuntimeError("камера отклонила ч/б режим (нужна OV2640)")


def snap(ser):
    """Запросить кадр, вернуть bytes с JPEG."""
    ser.reset_input_buffer()
    ser.write(b"snap\n")
    deadline = time.monotonic() + SNAP_TIMEOUT

    # пропускаем всё лишнее, ждём заголовок "IMG <len>"
    while True:
        line = _read_line(ser, deadline)
        if line.startswith("IMG "):
            break
        if line == "ERR":
            raise RuntimeError("устройство ответило ERR")
    try:
        length = int(line.split()[1])
    except (IndexError, ValueError):
        raise RuntimeError(f"битый заголовок: {line!r}")
    if not 0 < length <= MAX_JPEG:
        raise RuntimeError(f"недопустимая длина кадра: {length}")

    data = bytearray()
    while len(data) < length:
        if time.monotonic() > deadline:
            raise TimeoutError(f"получено {len(data)} из {length} байт")
        chunk = ser.read(length - len(data))
        data += chunk

    data = bytes(data)
    if not (data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9"):
        raise RuntimeError("данные не похожи на JPEG (нет FFD8/FFD9)")
    return data


# --------------------------------------------------------------------------
# Поток, чтобы GUI не зависал на время приёма
# --------------------------------------------------------------------------
class CaptureThread(QThread):
    done = pyqtSignal(bytes, float)     # jpeg, время приёма в секундах
    failed = pyqtSignal(str)

    def __init__(self, ser, size_id, need_size_change, mono=None):
        super().__init__()
        self.ser = ser
        self.size_id = size_id
        self.need_size_change = need_size_change
        self.mono = mono                # None - не менять, True/False - выставить перед съёмкой

    def run(self):
        try:
            if self.need_size_change:
                set_size(self.ser, self.size_id)
                # первые кадры после смены размера могут быть битыми
                time.sleep(0.3)
            if self.mono is not None:
                set_mono(self.ser, self.mono)
            t0 = time.monotonic()
            jpeg = snap(self.ser)
            self.done.emit(jpeg, time.monotonic() - t0)
        except Exception as e:                      # noqa: BLE001
            self.failed.emit(str(e))


class StreamThread(QThread):
    """Видео: start -> читаем кадры, пока не попросят остановиться -> stop -> ждём OK."""
    frame = pyqtSignal(bytes, QImage)
    stats = pyqtSignal(float, object, object)    # fps, snapshot (dict), качество (tuple | None)
    size_applied = pyqtSignal()
    mono_applied = pyqtSignal()
    failed = pyqtSignal(str)

    def __init__(self, ser, size_id, need_size_change, mono=None):
        super().__init__()
        self.ser = ser
        self.size_id = size_id
        self.need_size_change = need_size_change
        self.mono = mono                # None - не менять, True/False - выставить перед стартом
        self._stop = threading.Event()
        self._reset = threading.Event()

    def request_stop(self):
        self._stop.set()

    def request_reset(self):
        self._reset.set()

    def run(self):
        ser = self.ser
        try:
            if self.need_size_change:
                set_size(ser, self.size_id)
                self.size_applied.emit()
            if self.mono is not None:
                set_mono(ser, self.mono)
                self.mono_applied.emit()
            ser.reset_input_buffer()
            ser.write(b"start\n")

            stats = StreamStats()
            window = deque()                    # (время, байт) за последние ~2 с
            line = bytearray()
            stopping = False
            stop_deadline = 0.0
            idle_deadline = time.monotonic() + SNAP_TIMEOUT

            while True:
                now = time.monotonic()
                if self._stop.is_set() and not stopping:
                    ser.write(b"stop\n")
                    stopping = True
                    stop_deadline = now + STOP_TIMEOUT
                if stopping and now > stop_deadline:
                    raise TimeoutError("устройство не подтвердило остановку")
                if not stopping and now > idle_deadline:
                    raise TimeoutError("нет кадров от устройства")

                c = ser.read(1)
                if not c:
                    continue
                if c != b"\n":
                    if len(line) < 64:
                        line += c
                    continue
                text = line.decode(errors="replace").strip()
                line.clear()

                if text == "OK" and stopping:
                    break
                if not text.startswith("IMG "):
                    continue                    # мусор / лишний OK
                try:
                    length = int(text.split()[1])
                except (IndexError, ValueError):
                    continue
                if not 0 < length <= MAX_JPEG:
                    continue

                data = bytearray()
                read_deadline = time.monotonic() + 3.0
                while len(data) < length:
                    if time.monotonic() > read_deadline:
                        raise TimeoutError(f"кадр оборван: {len(data)} из {length} байт")
                    data += ser.read(length - len(data))
                data = bytes(data)

                if stopping:
                    continue                    # кадр, прилетевший после stop - выбрасываем
                if not (data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9"):
                    continue
                img = QImage.fromData(data, "JPG")
                if img.isNull():
                    continue

                t = time.monotonic()
                idle_deadline = t + FRAME_TIMEOUT
                window.append((t, len(data)))
                while window and t - window[0][0] > 2.0:
                    window.popleft()
                if self._reset.is_set():
                    self._reset.clear()
                    stats.reset()
                stats.add(len(data), t)
                if len(window) >= 2:
                    dt = window[-1][0] - window[0][0]
                    fps = (len(window) - 1) / dt
                    self.stats.emit(fps, stats.snapshot(), jpeg_quality(data))
                self.frame.emit(data, img)
        except Exception as e:                      # noqa: BLE001
            self.failed.emit(str(e))


# --------------------------------------------------------------------------
# GUI
# --------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Camera")
        self.resize(900, 700)

        self.ser = None
        self.thread = None
        self.stream = None
        self.current_size = None        # какой size_id сейчас в камере
        self.current_mono = None        # ч/б в камере сейчас (None - неизвестно)
        self.jpeg = None
        self.pixmap = None

        # --- виджеты
        self.port_box = QComboBox()
        self.port_box.setMinimumWidth(260)
        self.refresh_btn = QPushButton("Обновить")
        self.connect_btn = QPushButton("Подключить")
        self.res_box = QComboBox()
        for name, _ in RESOLUTIONS:
            self.res_box.addItem(name)
        self.res_box.setCurrentIndex(5)             # VGA
        self.mono_box = QCheckBox("Ч/Б")
        self.snap_btn = QPushButton("Снимок")
        self.snap_btn.setEnabled(False)
        self.video_btn = QPushButton("▶ Видео")
        self.video_btn.setEnabled(False)
        self.fps_label = QLabel("FPS: —")
        self.fps_label.setMinimumWidth(260)
        self.statusBar().addPermanentWidget(self.fps_label)
        self.save_btn = QPushButton("Сохранить…")
        self.save_btn.setEnabled(False)
        self.reset_btn = QPushButton("Сброс статистики")
        self.stats_label = QLabel()
        self.stats_label.setStyleSheet(
            "font-family: monospace; background:#101010; color:#7CFC7C; padding:6px;")
        self.stats_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.show_stats(None, None)

        self.view = QLabel("Нет изображения")
        self.view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view.setMinimumSize(320, 240)
        self.view.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.view.setStyleSheet("background:#202020; color:#aaa;")

        # --- раскладка
        top = QHBoxLayout()
        top.addWidget(QLabel("Порт:"))
        top.addWidget(self.port_box)
        top.addWidget(self.refresh_btn)
        top.addWidget(self.connect_btn)
        top.addStretch()

        mid = QHBoxLayout()
        mid.addWidget(QLabel("Разрешение:"))
        mid.addWidget(self.res_box)
        mid.addWidget(self.mono_box)
        mid.addWidget(self.snap_btn)
        mid.addWidget(self.video_btn)
        mid.addWidget(self.save_btn)
        mid.addWidget(self.reset_btn)
        mid.addStretch()

        root = QVBoxLayout()
        root.addLayout(top)
        root.addLayout(mid)
        root.addWidget(self.stats_label)
        root.addWidget(self.view, 1)
        central = QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)

        # --- сигналы
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.snap_btn.clicked.connect(self.capture)
        self.video_btn.clicked.connect(self.toggle_video)
        self.save_btn.clicked.connect(self.save)
        self.reset_btn.clicked.connect(self.reset_stats)

        self.refresh_ports()

    # ---- порты
    def refresh_ports(self):
        self.port_box.clear()
        best = 0
        for i, p in enumerate(sorted(serial.tools.list_ports.comports(),
                                     key=lambda x: x.device)):
            self.port_box.addItem(f"{p.device} - {p.description}", p.device)
            if p.vid == 0x0483:                      # STMicroelectronics
                best = i
        if self.port_box.count():
            self.port_box.setCurrentIndex(best)
        else:
            self.port_box.addItem("порты не найдены", None)

    def toggle_connection(self):
        if self.ser:
            self.disconnect_port()
            return
        dev = self.port_box.currentData()
        if not dev:
            self.statusBar().showMessage("Выберите порт")
            return
        try:
            self.ser = serial.Serial(dev, 115200, timeout=0.2)
        except serial.SerialException as e:
            self.statusBar().showMessage(f"Не удалось открыть {dev}: {e}")
            return
        self.current_size = None
        self.current_mono = None
        # если устройство ещё гонит поток с прошлого сеанса - остановить и вычистить
        self.ser.write(b"stop\n")
        time.sleep(0.4)
        self.ser.reset_input_buffer()
        self.connect_btn.setText("Отключить")
        self.port_box.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        self.snap_btn.setEnabled(True)
        self.video_btn.setEnabled(True)
        self.statusBar().showMessage(f"Подключено: {dev}")

    def disconnect_port(self):
        if self.stream and self.stream.isRunning():
            self.stream.request_stop()
            self.stream.wait(5000)
        if self.thread and self.thread.isRunning():
            self.thread.wait()
        if self.ser:
            self.ser.close()
        self.ser = None
        self.connect_btn.setText("Подключить")
        self.port_box.setEnabled(True)
        self.refresh_btn.setEnabled(True)
        self.snap_btn.setEnabled(False)
        self.video_btn.setEnabled(False)
        self.fps_label.setText("FPS: —")
        self.statusBar().showMessage("Отключено")

    # ---- съёмка
    def capture(self):
        if not self.ser:
            return
        size_id = RESOLUTIONS[self.res_box.currentIndex()][1]
        need_change = size_id != self.current_size
        mono = self.mono_box.isChecked()
        mono_arg = mono if mono != self.current_mono else None

        self.snap_btn.setEnabled(False)
        self.res_box.setEnabled(False)
        self.mono_box.setEnabled(False)
        self.statusBar().showMessage("Смена разрешения и съёмка…" if need_change
                                     else "Съёмка…")

        self.thread = CaptureThread(self.ser, size_id, need_change, mono_arg)
        self.thread.done.connect(lambda j, t: self.on_done(j, t, size_id, mono))
        self.thread.failed.connect(self.on_failed)
        self.thread.finished.connect(self.on_finished)
        self.thread.start()

    def on_done(self, jpeg, elapsed, size_id, mono):
        img = QImage.fromData(jpeg, "JPG")
        if img.isNull():
            self.statusBar().showMessage("Не удалось декодировать JPEG")
            return
        self.current_size = size_id
        self.current_mono = mono
        self.jpeg = jpeg
        self.pixmap = QPixmap.fromImage(img)
        self.save_btn.setEnabled(True)
        self.update_view()
        self.show_stats({"last": len(jpeg), "avg": None, "min": None, "max": None,
                         "rate_bps": None, "avg_bps": None, "peak_bps": None},
                        jpeg_quality(jpeg))
        self.statusBar().showMessage(
            f"{img.width()}x{img.height()}, {len(jpeg) / 1024:.1f} КБ, {elapsed:.2f} с")

    def on_failed(self, msg):
        self.statusBar().showMessage(f"Ошибка: {msg}")

    def on_finished(self):
        self.res_box.setEnabled(True)
        self.mono_box.setEnabled(True)
        self.snap_btn.setEnabled(self.ser is not None)

    # ---- видео
    def toggle_video(self):
        if not self.ser:
            return
        if self.stream and self.stream.isRunning():        # пауза
            self.stream.request_stop()
            self.video_btn.setEnabled(False)
            self.statusBar().showMessage("Остановка…")
            return

        size_id = RESOLUTIONS[self.res_box.currentIndex()][1]
        need_change = size_id != self.current_size
        mono = self.mono_box.isChecked()
        mono_arg = mono if mono != self.current_mono else None
        self.snap_btn.setEnabled(False)
        self.res_box.setEnabled(False)
        self.mono_box.setEnabled(False)
        self.video_btn.setText("⏸ Пауза")
        self.show_stats(None, None)
        self.statusBar().showMessage("Запуск видео…")

        self.stream = StreamThread(self.ser, size_id, need_change, mono_arg)
        self.stream.size_applied.connect(lambda: setattr(self, "current_size", size_id))
        self.stream.mono_applied.connect(lambda: setattr(self, "current_mono", mono))
        self.stream.frame.connect(self.on_frame)
        self.stream.stats.connect(self.on_stats)
        self.stream.failed.connect(self.on_failed)
        self.stream.finished.connect(self.on_stream_finished)
        self.stream.start()

    def on_frame(self, jpeg, img):
        self.jpeg = jpeg
        self.pixmap = QPixmap.fromImage(img)
        self.save_btn.setEnabled(True)
        self.update_view()
        self.statusBar().showMessage(
            f"{img.width()}x{img.height()}, {len(jpeg) / 1024:.1f} КБ/кадр")

    def on_stats(self, fps, snapshot, quality):
        self.fps_label.setText(f"FPS: {fps:.1f}")
        self.show_stats(snapshot, quality)

    def show_stats(self, snapshot, quality):
        self.stats_label.setText(format_stats(snapshot, quality))

    def reset_stats(self):
        if self.stream and self.stream.isRunning():
            self.stream.request_reset()         # поток обнулит счётчики перед следующим кадром
        else:
            self.show_stats(None, None)

    def on_stream_finished(self):
        self.video_btn.setText("▶ Видео")
        self.video_btn.setEnabled(self.ser is not None)
        self.snap_btn.setEnabled(self.ser is not None)
        self.res_box.setEnabled(True)
        self.mono_box.setEnabled(True)
        self.fps_label.setText("FPS: —")

    # ---- отображение / сохранение
    def update_view(self):
        if not self.pixmap:
            return
        self.view.setPixmap(self.pixmap.scaled(
            self.view.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self.update_view()

    def save(self):
        if not self.jpeg:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Сохранить", "frame.jpg", "JPEG (*.jpg)")
        if path:
            with open(path, "wb") as f:
                f.write(self.jpeg)
            self.statusBar().showMessage(f"Сохранено: {path}")

    def closeEvent(self, e):
        self.disconnect_port()
        super().closeEvent(e)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())