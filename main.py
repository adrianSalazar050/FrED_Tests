import sys
import platform
import serial
import serial.tools.list_ports
import numpy as np
import csv
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QFileDialog
)
from PyQt5.QtCore import Qt, QTimer, QUrl
from PyQt5.QtGui import QIcon
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# -----------------------
# Serial: auto-detect puerto
# -----------------------

def encontrar_puerto_arduino():
    puertos = serial.tools.list_ports.comports()
    for p in puertos:
        desc = p.description.lower()
        hwid = p.hwid.lower()
        if ("arduino" in desc) or ("usb serial" in desc) or ("ch340" in desc) or ("1a86" in hwid) or ("2341" in hwid):
            return p.device  # Found a likely Arduino port

    # Fallbacks if not found
    if platform.system() == "Windows":
        return "COM9"  # You can change this to your usual COM
    elif platform.system() == "Linux":
        return "/dev/ttyACM0"
    elif platform.system() == "Darwin":  # macOS
        return "/dev/tty.usbmodem1101"
    else:
        return None

try:
    puerto = encontrar_puerto_arduino() or "/dev/ttyACM0"
    arduino = serial.Serial(puerto, 115200, timeout=1)
except serial.SerialException as e:
    print(f"Error al abrir el puerto serial: {e}")
    # En un caso real, mostrarías una ventana de error al usuario
    sys.exit(1) # Salir si no se puede conectar al Arduino


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
        self.resize(1100, 700)

        # Estados y datos
        self.estado = ['0','0','0','0']
        self.velocidad_extrusor = 100
        self.temperatura_objetivo = 190
        self.velocidad_dc_objetivo = 20 # <--- NUEVO: Valor inicial para motor DC
        self.temp_data     = []
        self.motor_data    = []
        self.fan_data      = []
        self.extruder_data = []

        # Layout principal
        main_layout = QHBoxLayout(self)

        # --- Panel de gráficas ---
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        self.canvas_temp     = PlotCanvas(self, width=5, height=2)
        self.canvas_motor    = PlotCanvas(self, width=5, height=2)
        self.canvas_fan      = PlotCanvas(self, width=5, height=2)
        self.canvas_extruder = PlotCanvas(self, width=5, height=2)
        left_layout.addWidget(self.canvas_temp)
        left_layout.addWidget(self.canvas_motor)
        left_layout.addWidget(self.canvas_fan)
        left_layout.addWidget(self.canvas_extruder)
        main_layout.addWidget(left_widget, 3)
       
        # --- Panel de controles ---
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        # Botones ON/OFF
        self.btn_spool   = QPushButton("Motor DC (OFF)") # Texto cambiado
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
       
        # --- Slider de velocidad de Motor DC (NUEVO) ---
        self.lbl_dc = QLabel(f"Velocidad Motor DC (RPM): {self.velocidad_dc_objetivo}")
        self.slider_dc = QSlider(Qt.Horizontal)
        self.slider_dc.setRange(0, 100) # Puedes ajustar este rango según necesites
        self.slider_dc.setValue(self.velocidad_dc_objetivo)
        self.slider_dc.valueChanged.connect(self.actualizar_velocidad_dc)
        right_layout.addWidget(self.lbl_dc)
        right_layout.addWidget(self.slider_dc)

        # Botón exportar CSV
        self.export_button = QPushButton("Exportar CSV")
        self.export_button.clicked.connect(self.export_csv)
        right_layout.addWidget(self.export_button)

        # Placeholder para el video, para que el layout no se rompa
        right_layout.addStretch()

        main_layout.addWidget(right_widget, 1)

        # Timer para refrescar
        self.timer = QTimer()
        self.timer.timeout.connect(self.actualizar)
        self.timer.start(1000)

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
   
    # --- NUEVA FUNCIÓN PARA EL SLIDER DEL MOTOR DC ---
    def actualizar_velocidad_dc(self, val):
        self.velocidad_dc_objetivo = val
        self.lbl_dc.setText(f"Velocidad Motor DC (RPM): {val}")

    #------------------------------------------------
    # Loop de comunicación y gráfica
    #------------------------------------------------
    def actualizar(self):
        # Mandar comandos al Arduino
        cmd_act = "ACTUATE:" + ''.join(self.estado) + "\n"
        arduino.write(cmd_act.encode())

        cmd_vel = f"SPEED:{self.velocidad_extrusor}\n"
        arduino.write(cmd_vel.encode())

        cmd_temp = f"TEMP:{self.temperatura_objetivo}\n"
        arduino.write(cmd_temp.encode())
       
        # --- NUEVO COMANDO ENVIADO AL ARDUINO ---
        cmd_dc_speed = f"DCSPEED:{self.velocidad_dc_objetivo}\n"
        arduino.write(cmd_dc_speed.encode())

        # Leer y procesar datos del Arduino
        while arduino.in_waiting:
            try:
                line = arduino.readline().decode(errors='ignore').strip()
                if line.startswith("Temp:"):
                    t = float(line.split(':')[1])
                    self.temp_data.append(t)
                elif line.startswith("Motor DC RPM:"):
                    try:
                        rpm = float(line.split(":")[1])
                        self.motor_data.append(rpm)
                    except ValueError:
                        pass
                elif line.startswith("Fan:"):
                    v = 1 if "Encendido" in line else 0
                    self.fan_data.append(v)
                elif line.startswith("Extruder:"):
                    v = 1 if "Encendido" in line else 0
                    self.extruder_data.append(v)
            except (ValueError, IndexError):
                # Ignorar líneas malformadas que pueden ocurrir durante la conexión inicial
                pass

        # Limitar la cantidad de datos a mostrar
        max_len = 100
        self.temp_data     = self.temp_data[-max_len:]
        self.motor_data    = self.motor_data[-max_len:]
        self.fan_data      = self.fan_data[-max_len:]
        self.extruder_data = self.extruder_data[-max_len:]

        # Actualizar gráficas
        self.canvas_temp.plot(self.temp_data, ylabel="Temp (°C)")
        self.canvas_motor.plot(self.motor_data, ylabel="Motor DC (RPM)")
        self.canvas_fan.plot(self.fan_data, ylabel="Fan ON/OFF")
        self.canvas_extruder.plot(self.extruder_data, ylabel="Extrusor ON/OFF")

    #------------------------------------------------
    # Exportar CSV
    #------------------------------------------------
    def export_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "Guardar CSV", "data.csv", "CSV Files (*.csv)")
        if not path:
            return
       
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index','Temp','Motor_DC','Fan','Extrusor'])
           
            # Asegurarse de que todas las listas tengan la misma longitud para un CSV consistente
            max_rows = len(self.temp_data)
           
            for i in range(max_rows):
                temp = self.temp_data[i]
                m = self.motor_data[i] if i < len(self.motor_data) else ''
                f_ = self.fan_data[i]   if i < len(self.fan_data)   else ''
                e = self.extruder_data[i] if i < len(self.extruder_data) else ''
                writer.writerow([i, temp, m, f_, e])
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
