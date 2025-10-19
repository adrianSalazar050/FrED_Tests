import sys
import platform
import serial
import serial.tools.list_ports
import numpy as np
import csv
import cv2

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QFileDialog
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# -----------------------
# Serial: auto-detect puerto
# -----------------------
def encontrar_puerto_arduino():
    puertos = serial.tools.list_ports.comports()
    for p in puertos:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if ("arduino" in desc) or ("usb serial" in desc) or ("ch340" in desc) or ("1a86" in hwid) or ("2341" in hwid):
            return p.device  # Found a likely Arduino port

    # Fallbacks if not found
    if platform.system() == "Windows":
        return "COM9"  # Ajustar según tu configuración
    elif platform.system() == "Linux":
        return "/dev/ttyACM0"
    elif platform.system() == "Darwin":  # macOS
        return "/dev/tty.usbmodem1101"
    else:
        return None

try:
    puerto = encontrar_puerto_arduino() or "/dev/ttyACM0"
    arduino = serial.Serial(puerto, 115200, timeout=1)
except Exception as e:
    print(f"Error al abrir el puerto serial: {e}")
    # En un caso real: mostraría una ventana de error al usuario.
    sys.exit(1)

# -----------------------
# PlotCanvas: contenedor de gráficas
# -----------------------
class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=2, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)
        self.setParent(parent)

    def plot(self, data, ylabel=""):
        self.axes.cla()
        if len(data) > 0:
            self.axes.plot(data, linestyle='-')
        if ylabel:
            self.axes.set_ylabel(ylabel)
        self.axes.grid(True)
        self.draw()

# -----------------------
# GUI principal
# -----------------------
class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AI-FrED0 Control Interface")
        self.resize(1200, 780)

        # Estados y datos
        self.estado = ['0','0','0','0']            # [Motor DC / Spool, Fan, Extruder, Heater]
        self.velocidad_extrusor = 100
        self.temperatura_objetivo = 190
        self.velocidad_dc_objetivo = 20           # motor DC
        self.temp_data     = []
        self.motor_data    = []
        self.fan_data      = []
        self.extruder_data = []
        self.external_data = []

        # Parámetros vision
        self.FACTOR_CONVERSION = 0.25 / 43
        self.VALOR_UMBRAL = 127

        # Layout principal
        main_layout = QHBoxLayout(self)

        # --- Panel de gráficas (izquierda) ---
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        self.canvas_temp     = PlotCanvas(self, width=5, height=2)
        self.canvas_motor    = PlotCanvas(self, width=5, height=2)
        self.canvas_fan      = PlotCanvas(self, width=5, height=2)
        self.canvas_extruder = PlotCanvas(self, width=5, height=2)
        self.canvas_external = PlotCanvas(self, width=5, height=2)  # datos externos
        left_layout.addWidget(self.canvas_temp)
        left_layout.addWidget(self.canvas_motor)
        left_layout.addWidget(self.canvas_fan)
        left_layout.addWidget(self.canvas_extruder)
        left_layout.addWidget(self.canvas_external)
        main_layout.addWidget(left_widget, 3)

        # --- Panel de controles (derecha) ---
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        # Botones ON/OFF
        self.btn_spool   = QPushButton("Motor DC (OFF)")
        self.btn_fan     = QPushButton("Fan (OFF)")
        self.btn_extrude = QPushButton("Extrusor (OFF)")
        self.btn_heater  = QPushButton("Heater (OFF)")
        for idx, btn, name in [
            (0, self.btn_spool,   "Motor DC"),
            (1, self.btn_fan,     "Fan"),
            (2, self.btn_extrude, "Extrusor"),
            (3, self.btn_heater,  "Heater")
        ]:
            btn.clicked.connect(lambda ch, i=idx, b=btn, n=name: self.toggle(i, b, n))
            right_layout.addWidget(btn)

        # Slider de velocidad de extrusor
        self.lbl_slider = QLabel(f"Velocidad Extrusor: {self.velocidad_extrusor}")
        self.slider     = QSlider(Qt.Horizontal)
        self.slider.setRange(10, 100)
        self.slider.setValue(self.velocidad_extrusor)
        self.slider.valueChanged.connect(self.actualizar_velocidad_extrusor)
        right_layout.addWidget(self.lbl_slider)
        right_layout.addWidget(self.slider)

        # Slider de temperatura
        self.lbl_temp = QLabel(f"Temperatura objetivo: {self.temperatura_objetivo} °C")
        self.slider_temp = QSlider(Qt.Horizontal)
        self.slider_temp.setRange(0, 300)
        self.slider_temp.setValue(self.temperatura_objetivo)
        self.slider_temp.valueChanged.connect(self.actualizar_temperatura)
        right_layout.addWidget(self.lbl_temp)
        right_layout.addWidget(self.slider_temp)

        # Slider de velocidad de Motor DC
        self.lbl_dc = QLabel(f"Velocidad Motor DC (RPM): {self.velocidad_dc_objetivo}")
        self.slider_dc = QSlider(Qt.Horizontal)
        self.slider_dc.setRange(0, 100)
        self.slider_dc.setValue(self.velocidad_dc_objetivo)
        self.slider_dc.valueChanged.connect(self.actualizar_velocidad_dc)
        right_layout.addWidget(self.lbl_dc)
        right_layout.addWidget(self.slider_dc)

        # Botón exportar CSV
        self.export_button = QPushButton("Exportar CSV")
        self.export_button.clicked.connect(self.export_csv)
        right_layout.addWidget(self.export_button)

        # Separador visual
        right_layout.addStretch()

        # ----------------- Video de Extrusión -----------------
        self.label_video = QLabel("Video de Extrusión")
        self.label_video.setStyleSheet("font-weight: bold;")
        right_layout.addWidget(self.label_video)

        # Espacio para mostrar las imágenes de la cámara
        self.label_camara = QLabel("Sin señal")
        self.label_camara.setFixedSize(420, 315)
        self.label_camara.setAlignment(Qt.AlignCenter)
        self.label_camara.setStyleSheet("border: 2px solid gray; background-color: #111; color: white;")
        right_layout.addWidget(self.label_camara)

        # Inicializa la cámara (usar CAP_DSHOW en Windows para evitar warning)
        if platform.system() == "Windows":
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            print("⚠️ No se pudo abrir la cámara. Verifica la conexión.")
            self.label_camara.setText("Cámara no disponible")

        # Timer para actualizar la imagen cada 500 ms
        self.timer_camara = QTimer()
        self.timer_camara.timeout.connect(self.actualizar_imagen_camara)
        self.timer_camara.start(500)

        main_layout.addWidget(right_widget, 1)

        # Timer para refrescar gráficas y comunicar con Arduino cada 1s
        self.timer = QTimer()
        self.timer.timeout.connect(self.actualizar)
        self.timer.start(250)

    #------------------------------------------------
    # Cierre ordenado
    #------------------------------------------------
    def closeEvent(self, event):
        """Cierra la cámara correctamente al salir."""
        try:
            if hasattr(self, 'cap') and self.cap.isOpened():
                self.cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()
        event.accept()

    #------------------------------------------------
    # Actualizar imagen de cámara (pipeline de visión)
    #------------------------------------------------
    def actualizar_imagen_camara(self):
        if not (hasattr(self, 'cap') and self.cap.isOpened()):
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.label_camara.setText("Error al capturar imagen")
            return

        # ---------- PIPELINE ----------
        FACTOR_CONVERSION = self.FACTOR_CONVERSION
        VALOR_UMBRAL = self.VALOR_UMBRAL

        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gris, (5, 5), 0)
        _, umbral = cv2.threshold(blur, VALOR_UMBRAL, 255, cv2.THRESH_BINARY)

        frame_resultado = cv2.cvtColor(umbral, cv2.COLOR_GRAY2BGR)

        contornos, _ = cv2.findContours(umbral, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contornos:
            # Tomar el contorno más grande
            contorno_principal = max(contornos, key=cv2.contourArea)
            if cv2.contourArea(contorno_principal) > 100:
                todos_los_puntos = contorno_principal.squeeze()
                if todos_los_puntos.ndim != 1 and len(todos_los_puntos) >= 10:
                    centro_x = frame.shape[1] / 2
                    puntos_izquierdos = todos_los_puntos[todos_los_puntos[:, 0] < centro_x]
                    puntos_derechos   = todos_los_puntos[todos_los_puntos[:, 0] >= centro_x]

                    if len(puntos_izquierdos) > 0 and len(puntos_derechos) > 0:
                        distancias_px = []
                        # muestreo para eficiencia
                        for punto_izq in puntos_izquierdos[::5]:
                            distancias = np.linalg.norm(puntos_derechos - punto_izq, axis=1)
                            min_dist = np.min(distancias)
                            distancias_px.append(min_dist)
                            punto_cercano_der = puntos_derechos[np.argmin(distancias)]
                            cv2.line(frame_resultado, tuple(punto_izq), tuple(punto_cercano_der), (0, 255, 0), 1)

                        if distancias_px:
                            distancia_promedio_px = np.mean(distancias_px)
                            distancia_mm = distancia_promedio_px * FACTOR_CONVERSION
                            texto = f"Grosor: {distancia_mm:.3f} mm"
                            cv2.putText(frame_resultado, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
                            cv2.drawContours(frame_resultado, [contorno_principal], -1, (0, 0, 255), 2)

        # ---------- Mostrar en el QLabel ----------
        vis = cv2.resize(frame_resultado, (420, 315), interpolation=cv2.INTER_AREA)
        vis_rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        h, w, ch = vis_rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(vis_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.label_camara.setPixmap(QPixmap.fromImage(qimg))

    #------------------------------------------------
    # Funciones de control
    #------------------------------------------------
    def toggle(self, index, boton, nombre):
        self.estado[index] = '1' if self.estado[index]=='0' else '0'
        estado_txt = "ON" if self.estado[index]=='1' else "OFF"
        boton.setText(f"{nombre} ({estado_txt})")

    def actualizar_velocidad_extrusor(self, val):
        self.velocidad_extrusor = val
        self.lbl_slider.setText(f"Velocidad Extrusor: {val}")

    def actualizar_temperatura(self, val):
        self.temperatura_objetivo = val
        self.lbl_temp.setText(f"Temperatura objetivo: {val} °C")

    def actualizar_velocidad_dc(self, val):
        self.velocidad_dc_objetivo = val
        self.lbl_dc.setText(f"Velocidad Motor DC (RPM): {val}")

    #------------------------------------------------
    # Loop de comunicación y gráfica
    #------------------------------------------------
    def actualizar(self):
        # Enviar comandos al Arduino
        try:
            cmd_act = "ACTUATE:" + ''.join(self.estado) + "\n"
            arduino.write(cmd_act.encode())
        except Exception:
            pass

        try:
            cmd_vel = f"SPEED:{self.velocidad_extrusor}\n"
            arduino.write(cmd_vel.encode())
        except Exception:
            pass

        try:
            cmd_temp = f"TEMP:{self.temperatura_objetivo}\n"
            arduino.write(cmd_temp.encode())
        except Exception:
            pass

        # Enviar velocidad del motor DC (nuevo)
        try:
            cmd_dc_speed = f"DCSPEED:{self.velocidad_dc_objetivo}\n"
            arduino.write(cmd_dc_speed.encode())
        except Exception:
            pass

        # Leer y procesar datos del Arduino (compatibilidades)
        try:
            while arduino.in_waiting:
                try:
                    line = arduino.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    # Temp:
                    if line.startswith("Temp:"):
                        try:
                            t = float(line.split(':',1)[1])
                            self.temp_data.append(t)
                        except Exception:
                            pass
                    # Motor DC RPM:  <-- formato primer script
                    elif line.startswith("Motor DC RPM:"):
                        try:
                            rpm = float(line.split(":",1)[1])
                            self.motor_data.append(rpm)
                        except Exception:
                            pass
                    # Motor Spool: <-- formato segundo script (encendido / apagado)
                    elif line.startswith("Motor Spool:"):
                        v = 1 if "Encendido" in line or "ON" in line else 0
                        self.motor_data.append(v)
                    # Fan:
                    elif line.startswith("Fan:"):
                        v = 1 if "Encendido" in line or "ON" in line else 0
                        self.fan_data.append(v)
                    # Extruder:
                    elif line.startswith("Extruder:"):
                        v = 1 if "Encendido" in line or "ON" in line else 0
                        self.extruder_data.append(v)
                except Exception:
                    # Ignorar líneas malformadas
                    break
        except Exception:
            pass

        # Limitar longitud de buffers
        max_len = 100
        self.temp_data     = self.temp_data[-max_len:]
        self.motor_data    = self.motor_data[-max_len:]
        self.fan_data      = self.fan_data[-max_len:]
        self.extruder_data = self.extruder_data[-max_len:]

        # Actualizar gráficas
        self.canvas_temp.plot(self.temp_data, ylabel="Temp (°C)")
        self.canvas_motor.plot(self.motor_data, ylabel="Motor DC (RPM/ON)")
        self.canvas_fan.plot(self.fan_data, ylabel="Fan ON/OFF")
        self.canvas_extruder.plot(self.extruder_data, ylabel="Extrusor ON/OFF")

        # ------------------- Datos externos -------------------
        try:
            with open("external_data.csv", "r") as f:
                lines = f.readlines()
                new_data = [float(line.strip()) for line in lines if line.strip()]
                # Append only new elements (simple approach: extend)
                self.external_data.extend(new_data)
        except Exception:
            # archivo no existe o error -> ignorar
            pass
        self.external_data = self.external_data[-max_len:]
        self.canvas_external.plot(self.external_data, ylabel="Datos externos")

    #------------------------------------------------
    # Exportar CSV
    #------------------------------------------------
    def export_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "Guardar CSV", "data.csv", "CSV Files (*.csv)")
        if not path:
            return

        # Asegurar la misma longitud para filas (basado en temp_data como referencia)
        max_rows = len(self.temp_data)
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index','Temp','Motor_DC','Fan','Extrusor'])
            for i in range(max_rows):
                temp = self.temp_data[i]
                m = self.motor_data[i]    if i < len(self.motor_data)    else ''
                fan = self.fan_data[i]    if i < len(self.fan_data)      else ''
                e = self.extruder_data[i] if i < len(self.extruder_data) else ''
                writer.writerow([i, temp, m, fan, e])
        print(f"CSV guardado en {path}")

# -----------------------
# Inicio de la aplicación
# -----------------------
def main():
    app = QApplication(sys.argv)
    gui = ControlGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
