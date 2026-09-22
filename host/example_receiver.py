#!/usr/bin/env python3
"""
Récepteur et visualiseur des données ARS
envoyées par le firmware STM32H723ZG.

Usage:
    python example_receiver.py --port /dev/ttyACM0
    python example_receiver.py --port COM3
"""

import argparse
import serial
import numpy as np
import matplotlib.pyplot as plt


def parse_block(ser: serial.Serial, timeout: float = 60.0):
    """Lit un bloc complet START ... END depuis le port série."""
    timestamps = []
    values = []
    in_block = False
    ser.timeout = 1.0

    print("En attente du marqueur START...")
    while True:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            continue

        if line == "START":
            in_block = True
            timestamps.clear()
            values.clear()
            print("→ START reçu — réception en cours...")
            continue

        if line == "END" and in_block:
            print(f"→ END reçu — {len(timestamps)} points collectés")
            return np.array(timestamps, dtype=np.int64), np.array(values, dtype=np.int32)

        if in_block:
            try:
                t_str, v_str = line.split(",")
                timestamps.append(int(t_str))
                values.append(int(v_str))
            except ValueError:
                pass


def plot_results(timestamps_us, adc_values):
    """Affiche le signal temporel et l'histogramme des intervalles τn."""
    t_s = timestamps_us * 1e-6

    tau = np.diff(timestamps_us) if len(timestamps_us) > 1 else np.array([])

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle("Acquisition ARS — STM32H723ZG", fontsize=14, fontweight="bold")

    # Signal temporel
    axes[0].plot(t_s, adc_values, "b.-", markersize=2, linewidth=0.6)
    axes[0].set_xlabel("Temps (s)")
    axes[0].set_ylabel("Valeur ADC (12 bits)")
    axes[0].set_title("Signal acquis")
    axes[0].grid(True, alpha=0.3)

    # Histogramme des intervalles
    if len(tau) > 0:
        axes[1].hist(tau, bins=60, color="steelblue", edgecolor="black", alpha=0.75)
        axes[1].axvline(1200, color="crimson", linestyle="--", linewidth=1.5, label="a = 1200 µs")
        axes[1].axvline(2800, color="crimson", linestyle="--", linewidth=1.5, label="b = 2800 µs")
        axes[1].set_xlabel("Intervalle τₙ (µs)")
        axes[1].set_ylabel("Nombre d'occurrences")
        axes[1].set_title("Distribution des intervalles d'échantillonnage")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Récepteur ARS STM32H723ZG")
    parser.add_argument("--port", required=True, help="Port série (COM3 ou /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (défaut 115200)")
    args = parser.parse_args()

    print(f"Ouverture de {args.port} @ {args.baud} bauds...")
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        print("En attente d'un bloc de données...\n")
        timestamps, values = parse_block(ser)

        if len(timestamps) == 0:
            print("Aucun point reçu.")
            return

        duration = timestamps[-1] * 1e-6
        fs_mean = len(timestamps) / duration if duration > 0 else 0

        print("\n========== Statistiques ==========")
        print(f"  Points reçus      : {len(timestamps)}")
        print(f"  Durée totale      : {duration:.3f} s")
        print(f"  Fréquence moyenne : {fs_mean:.1f} Hz")
        print(f"  ADC min / max     : {values.min()} / {values.max()}")
        print("==================================\n")

        plot_results(timestamps, values)


if __name__ == "__main__":
    main()
