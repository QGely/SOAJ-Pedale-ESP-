#!/usr/bin/env python3
"""
Visualisation temps réel du traitement audio SOAJ Pédale
Affiche les courbes entrée/sortie du signal guitare

Usage:
  python3 plot_audio.py [port] [baudrate]

Exemples:
  python3 plot_audio.py /dev/ttyUSB0 115200  (Linux)
  python3 plot_audio.py COM3 115200           (Windows)
  python3 plot_audio.py /dev/cu.SLAB_USBtoUART 115200  (macOS)
"""

import serial
import sys
import matplotlib.pyplot as plt
from collections import deque
from matplotlib.animation import FuncAnimation
import argparse

# Configuration
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200
BUFFER_SIZE = 2000  # points à afficher

class AudioVisualizer:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.x_in = deque(maxlen=BUFFER_SIZE)
        self.y_out = deque(maxlen=BUFFER_SIZE)
        self.time_axis = deque(maxlen=BUFFER_SIZE)
        self.sample_count = 0
        self.setup()

    def setup(self):
        """Initialise la connexion série et le graphique"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            print(f"✓ Connecté à {self.port} @ {self.baudrate} baud")
        except Exception as e:
            print(f"✗ Erreur de connexion : {e}")
            sys.exit(1)

        # Configure le graphique
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle('SOAJ Pédale - Visualisation Audio (entrée/sortie)', fontsize=14)

        # Graphique 1 : entrée/sortie
        self.line_in, = self.ax1.plot([], [], label='Entrée (ADC normalisé)', color='#2E86AB', linewidth=1, alpha=0.8)
        self.line_out, = self.ax1.plot([], [], label='Sortie (filtrée)', color='#A23B72', linewidth=1, alpha=0.8)
        self.ax1.set_ylabel('Amplitude (±1)')
        self.ax1.set_ylim(-2, 2)
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True, alpha=0.3)

        # Graphique 2 : différence (gain)
        self.line_diff, = self.ax2.plot([], [], label='Sortie / Entrée (gain)', color='#F18F01', linewidth=1)
        self.ax2.set_xlabel('Temps (échantillons)')
        self.ax2.set_ylabel('Gain')
        self.ax2.set_ylim(0, 100)
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True, alpha=0.3)

        # Animation
        self.anim = FuncAnimation(self.fig, self.update, interval=100, blit=True)
        plt.show()

    def read_data(self):
        """Lit les données du port série"""
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if ',' in line:
                    parts = line.split(',')
                    x_val = float(parts[0])
                    y_val = float(parts[1])

                    self.x_in.append(x_val)
                    self.y_out.append(y_val)
                    self.time_axis.append(self.sample_count)
                    self.sample_count += 1
            except (ValueError, UnicodeDecodeError):
                pass

    def update(self, frame):
        """Mise à jour des graphiques"""
        self.read_data()

        if len(self.x_in) > 0:
            # Graphique 1 : signaux
            self.line_in.set_data(range(len(self.x_in)), list(self.x_in))
            self.line_out.set_data(range(len(self.y_out)), list(self.y_out))
            self.ax1.set_xlim(max(0, len(self.x_in) - 500), len(self.x_in))

            # Graphique 2 : gain (rapport sortie/entrée, évite division par 0)
            gains = []
            for x, y in zip(self.x_in, self.y_out):
                if abs(x) > 0.01:  # évite division par zéro
                    gains.append(abs(y) / abs(x))
                else:
                    gains.append(0)

            if gains:
                self.line_diff.set_data(range(len(gains)), gains)
                self.ax2.set_xlim(max(0, len(gains) - 500), len(gains))

        return self.line_in, self.line_out, self.line_diff

def main():
    parser = argparse.ArgumentParser(
        description='Visualise l\'audio SOAJ Pédale en temps réel',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  python3 plot_audio.py /dev/ttyUSB0 115200  (Linux)
  python3 plot_audio.py COM3 115200           (Windows)
  python3 plot_audio.py /dev/cu.SLAB_USBtoUART 115200  (macOS)

Avant de lancer:
  1. Assurez-vous que DEBUG_PLOT = 1 dans PedaleEsclave.ino
  2. Téléversez le sketch sur l'ESP32
  3. Identifiez le port série (Arduino IDE, Device Manager, etc.)
  4. Lancez ce script
        """
    )
    parser.add_argument('port', nargs='?', default=DEFAULT_PORT, help=f'Port série (défaut: {DEFAULT_PORT})')
    parser.add_argument('baud', nargs='?', type=int, default=DEFAULT_BAUD, help=f'Vitesse en baud (défaut: {DEFAULT_BAUD})')

    args = parser.parse_args()

    print(f"\n╔═══════════════════════════════════════════════════════════╗")
    print(f"║  SOAJ Pédale - Visualisation Audio Temps Réel            ║")
    print(f"╚═══════════════════════════════════════════════════════════╝\n")
    print(f"Configuration:")
    print(f"  Port : {args.port}")
    print(f"  Baud : {args.baud}")
    print(f"\nAssurez-vous que DEBUG_PLOT = 1 dans PedaleEsclave.ino !")
    print(f"Appuyez sur Ctrl+C pour arrêter.\n")

    viz = AudioVisualizer(args.port, args.baud)

if __name__ == '__main__':
    main()
