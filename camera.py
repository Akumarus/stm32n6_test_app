#!/usr/bin/env python3
"""Просмотр кадров RGB565 по USB CDC от STM32N6 (IMX335).

pip install pyserial numpy
(tkinter входит в стандартную поставку Python; в conda: conda install tk)

Протокол: 'FRM1' | w u16 | h u16 | size u32 | id u32 | данные RGB565 LE
"""
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np
import serial
import serial.tools.list_ports

MAGIC = b"FRM1"
HDR = struct.Struct("<4sHHII")      # magic, w, h, size, id


def list_ports():
    return [p.device for p in sorted(serial.tools.list_ports.comports(),
                                     key=lambda p: p.device)]


class Reader(threading.Thread):
    """Читает порт, собирает кадры, кладёт последний в очередь."""

    def __init__(self, port, out_q, err_q):
        super().__init__(daemon=True)
        self.port, self.out_q, self.err_q = port, out_q, err_q
        self.stop_evt = threading.Event()

    def run(self):
        try:
            ser = serial.Serial(self.port, timeout=0.2)
        except Exception as e:
            self.err_q.put(f"Не удалось открыть {self.port}: {e}")
            return
        buf = bytearray()
        try:
            while not self.stop_evt.is_set():
                chunk = ser.read(max(1, ser.in_waiting))
                if chunk:
                    buf += chunk
                while True:
                    i = buf.find(MAGIC)
                    if i < 0:
                        del buf[:-3]          # хвост на случай разрыва magic
                        break
                    if i > 0:
                        del buf[:i]
                    if len(buf) < HDR.size:
                        break
                    _, w, h, size, fid = HDR.unpack_from(buf)
                    if size != w * h * 2 or size > 64 * 1024 * 1024:
                        del buf[:4]           # ложный magic
                        continue
                    if len(buf) < HDR.size + size:
                        break
                    data = bytes(buf[HDR.size:HDR.size + size])
                    del buf[:HDR.size + size]
                    frame = np.frombuffer(data, dtype="<u2").reshape(h, w)
                    if self.out_q.full():
                        try:
                            self.out_q.get_nowait()
                        except queue.Empty:
                            pass
                    self.out_q.put((fid, frame))
        except Exception as e:
            self.err_q.put(f"Ошибка порта: {e}")
        finally:
            try:
                ser.close()
            except Exception:
                pass

    def stop(self):
        self.stop_evt.set()


def rgb565_to_rgb(v, gain=1.0):
    r = ((v >> 11) & 0x1F).astype(np.uint16)
    g = ((v >> 5) & 0x3F).astype(np.uint16)
    b = (v & 0x1F).astype(np.uint16)
    r = (r << 3) | (r >> 2)
    g = (g << 2) | (g >> 4)
    b = (b << 3) | (b >> 2)
    rgb = np.dstack([r, g, b])
    if gain != 1.0:
        rgb = rgb * gain
    return np.clip(rgb, 0, 255).astype(np.uint8)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("IMX335 viewer")
        self.frame_q = queue.Queue(maxsize=1)
        self.err_q = queue.Queue()
        self.reader = None
        self.last = None           # (fid, raw frame)
        self.photo = None
        self.t_fps = time.time()
        self.n_fps = 0
        self.fps = 0.0

        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")

        ttk.Label(top, text="Порт:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var,
                                    width=18, state="readonly")
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(top, text="Обновить", command=self.refresh_ports).pack(side="left")
        self.btn_conn = ttk.Button(top, text="Подключиться", command=self.toggle)
        self.btn_conn.pack(side="left", padx=6)

        ttk.Label(top, text="Яркость x").pack(side="left", padx=(12, 2))
        self.gain = tk.DoubleVar(value=1.0)
        ttk.Scale(top, from_=1, to=16, variable=self.gain, length=140,
                  command=lambda _: self.redraw()).pack(side="left")
        self.gain_lbl = ttk.Label(top, text="1.0", width=5)
        self.gain_lbl.pack(side="left")

        ttk.Button(top, text="Сохранить", command=self.save).pack(side="right")

        self.canvas = tk.Label(self, background="#202020")
        self.canvas.pack(fill="both", expand=True)

        self.status = tk.StringVar(value="Не подключено")
        ttk.Label(self, textvariable=self.status, padding=4,
                  anchor="w").pack(fill="x")

        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(30, self.poll)

    # --- порты ---
    def refresh_ports(self):
        ports = list_ports()
        self.port_cb["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[-1])
        elif not ports:
            self.port_var.set("")

    # --- подключение ---
    def toggle(self):
        if self.reader:
            self.disconnect()
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showwarning("Порт", "Выберите порт")
                return
            self.reader = Reader(port, self.frame_q, self.err_q)
            self.reader.start()
            self.btn_conn.config(text="Отключиться")
            self.port_cb.config(state="disabled")
            self.status.set(f"Подключено к {port}, жду кадр...")

    def disconnect(self):
        if self.reader:
            self.reader.stop()
            self.reader.join(timeout=1)
            self.reader = None
        self.btn_conn.config(text="Подключиться")
        self.port_cb.config(state="readonly")
        self.status.set("Не подключено")

    # --- отрисовка ---
    def redraw(self):
        g = float(self.gain.get())
        self.gain_lbl.config(text=f"{g:.1f}")
        if self.last is None:
            return
        _, raw = self.last
        rgb = rgb565_to_rgb(raw, g)
        h, w, _ = rgb.shape
        ppm = b"P6 %d %d 255\n" % (w, h) + rgb.tobytes()
        self.photo = tk.PhotoImage(data=ppm, format="PPM")
        self.canvas.config(image=self.photo)

    def poll(self):
        try:
            msg = self.err_q.get_nowait()
            self.disconnect()
            messagebox.showerror("Ошибка", msg)
        except queue.Empty:
            pass

        try:
            fid, raw = self.frame_q.get_nowait()
            self.last = (fid, raw)
            self.redraw()
            self.n_fps += 1
            now = time.time()
            if now - self.t_fps >= 1.0:
                self.fps = self.n_fps / (now - self.t_fps)
                self.n_fps, self.t_fps = 0, now
            self.status.set(
                f"кадр {fid}   {raw.shape[1]}x{raw.shape[0]}   {self.fps:.1f} fps   "
                f"raw min {raw.min()} max {raw.max()} mean {raw.mean():.0f}")
        except queue.Empty:
            pass
        self.after(30, self.poll)

    # --- сохранение ---
    def save(self):
        if self.photo is None:
            return
        path = filedialog.asksaveasfilename(defaultextension=".png",
                                            filetypes=[("PNG", "*.png")])
        if path:
            self.photo.write(path, format="png")

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()