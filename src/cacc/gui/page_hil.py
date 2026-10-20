"""Hardware-in-the-loop page: drive a controller on real hardware.

Pick how the board is connected (UDP for a Raspberry Pi, serial for an
Arduino, CAN for an ECU, or a demo device started on this PC), which cars
run on it and the maneuver. :func:`cacc.adapters.hil.run` drives the
simulated platoon against the board, runs the same law in software next to
it, and the page shows both gap traces, the round-trip times and any missed
or late replies.
"""

from __future__ import annotations

import numpy as np
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout,
                               QLineEdit, QMessageBox, QPlainTextEdit, QProgressBar,
                               QSplitter)
from PySide6.QtCore import Qt

from cacc.gui.common import (BLUE, MUTED, ORANGE, Page, PlotCanvas, card, label,
                             primary_button, run_job)

TRANSPORTS = (("spawn", "Demo device on this PC (no hardware)", ""),
              ("udp", "Raspberry Pi / Linux board (UDP)", "192.168.1.20:5005"),
              ("serial", "Arduino (serial)", "COM5"),
              ("can", "ECU on a CAN bus", "can0"))


def run_hil(transport, address, cars, maneuver, condition, step, realtime, law):
    from cacc.adapters import hil

    spawn = transport == "spawn"
    return hil.run(maneuver, condition, "udp" if spawn else transport, address,
                   cars, step, 0.05, realtime, law, spawn_device=spawn)


class HILPage(Page):
    def __init__(self):
        super().__init__(
            "Hardware in the loop",
            "Run the controller on real hardware while the platoon, the road and the "
            "radio stay simulated. Copy src/cacc/adapters/hil_device.py to a "
            "Raspberry Pi (python3 hil_device.py --udp 0.0.0.0:5005), or flash "
            "examples/hil/arduino_cacc onto an Arduino. Every reply is timed, and "
            "the run is compared with the same law in pure software.")
        from cacc.campaign import CONDITIONS, MANEUVERS

        top, tl = card()
        row = QHBoxLayout()
        self.transport = QComboBox()
        for key, text, _addr in TRANSPORTS:
            self.transport.addItem(text, key)
        self.address = QLineEdit()
        self.transport.currentIndexChanged.connect(self._transport_changed)
        self.cars = QLineEdit("1")
        self.cars.setToolTip("followers running on the board, e.g. 1 3 (blank = all)")
        self.cars.setMaximumWidth(90)
        for text, w in (("Connection", self.transport), ("Address", self.address),
                        ("Cars on hardware", self.cars)):
            row.addWidget(label(text, wrap=False))
            row.addWidget(w, 1 if w is self.address else 0)
        tl.addLayout(row)
        row2 = QHBoxLayout()
        self.man = QComboBox()
        for k, v in MANEUVERS.items():
            self.man.addItem(v[0], k)
        self.man.setCurrentIndex(self.man.findData("hard-brake"))
        self.cond = QComboBox()
        for k, v in CONDITIONS.items():
            self.cond.addItem(v[0], k)
        self.step = QDoubleSpinBox()
        self.step.setRange(1, 200)
        self.step.setValue(20)
        self.step.setSuffix(" ms")
        self.law = QComboBox()
        self.law.addItems(["cacc", "acc"])
        self.realtime = QCheckBox("Real time")
        self.realtime.setToolTip("pace the simulation to the wall clock, so the board "
                                 "sees the real timing")
        for text, w in (("Maneuver", self.man), ("Condition", self.cond),
                        ("Control period", self.step), ("Law", self.law),
                        (None, self.realtime)):
            if text:
                row2.addWidget(label(text, wrap=False))
            row2.addWidget(w)
        tl.addLayout(row2)
        act = QHBoxLayout()
        self.run_btn = primary_button("Run with hardware")
        self.run_btn.clicked.connect(self._run)
        self.bar = QProgressBar()
        self.bar.setRange(0, 0)
        self.bar.hide()
        self.status = label("", "subtitle")
        act.addWidget(self.run_btn)
        act.addWidget(self.bar, 1)
        act.addWidget(self.status, 2)
        tl.addLayout(act)
        self.body.addWidget(top)
        split = QSplitter(Qt.Vertical)
        self.plot = PlotCanvas(rows=2, height=4.0)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        split.addWidget(self.plot)
        split.addWidget(self.text)
        split.setSizes([480, 160])
        self.body.addWidget(split, 1)
        self._transport_changed()

    def _transport_changed(self) -> None:
        i = self.transport.currentIndex()
        self.address.setText(TRANSPORTS[i][2])
        self.address.setEnabled(TRANSPORTS[i][0] != "spawn")

    def _cars(self):
        txt = self.cars.text().replace(",", " ").split()
        return [int(c) for c in txt] if txt else None

    def _run(self) -> None:
        try:
            cars = self._cars()
        except ValueError:
            QMessageBox.warning(self, "Hardware in the loop", "Cars: numbers like 1 3.")
            return
        self.run_btn.setEnabled(False)
        self.bar.show()
        self.status.setText("running" + (" in real time…" if self.realtime.isChecked()
                                         else "…"))
        run_job(run_hil, self.transport.currentData(), self.address.text().strip(), cars,
                self.man.currentData(), self.cond.currentData(), self.step.value() / 1e3,
                self.realtime.isChecked(), self.law.currentText(),
                on_done=self.show_result, on_error=self._failed)

    def _failed(self, msg: str) -> None:
        self.run_btn.setEnabled(True)
        self.bar.hide()
        self.status.setText("")
        QMessageBox.critical(self, "Hardware run failed", msg)

    def show_result(self, r: dict) -> None:
        from cacc.adapters.hil import format_text

        self.run_btn.setEnabled(True)
        self.bar.hide()
        missed = sum(s["misses"] for s in r["timing"])
        self.status.setText(("no reply from the board" if not r["timing"] or
                             missed == sum(s["samples"] for s in r["timing"]) else
                             f"{missed} missed replies") if missed or not r["timing"]
                            else "every reply on time · matches software")
        self.text.setPlainText(format_text(r))
        hw, sw = r["hw"], r["sw"]
        a1, a2 = self.plot.clear(rows=2)
        gap_hw = -np.diff(hw.pos, axis=1) - hw.config.vehicle.length
        gap_sw = -np.diff(sw.pos, axis=1) - sw.config.vehicle.length
        for i in range(gap_hw.shape[1]):
            a1.plot(hw.t, gap_hw[:, i], color=BLUE, lw=1.2,
                    label="hardware" if i == 0 else None)
            a1.plot(sw.t, gap_sw[:, i], color=ORANGE, lw=1.0, ls="--",
                    label="software" if i == 0 else None)
        a1.set_ylabel("gap [m]", fontsize=8)
        a1.legend(fontsize=7, frameon=False)
        a1.set_title(r["cell"], fontsize=9, color=MUTED)
        a2.plot(hw.t, hw.u, lw=1.0)
        a2.set_ylabel("command [m/s²]", fontsize=8)
        a2.set_xlabel("time [s]", fontsize=8)
        self.plot.draw()
