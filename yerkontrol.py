import tkinter as tk
from tkinter import ttk
import serial
import threading
import time

# Arduino bağlantısı
arduino_port = 'COM3'  # Arduino'nun bağlı olduğu portu belirtin
baud_rate = 9600


class ArduinoInterface:
    def __init__(self, master):
        self.master = master
        self.master.title("UAV Kontrol Arayüzü")
        self.master.geometry("1920x1080")  # Boyutu 1920x1080 olarak ayarladım
        self.master.resizable(False, False)  # Tam ekran olmasını engelle
        self.master.configure(bg="#2E3B4E")

        # Serial bağlantıyı kur
        try:
            self.serial_connection = serial.Serial(arduino_port, baud_rate)
        except serial.SerialException as e:
            print(f"Serial bağlantı hatası: {e}")
            self.master.quit()  # Hata durumunda programdan çık

        self.data = ""

        # Hedef açı girişi
        self.target_pitch_label = ttk.Label(master, text="Hedef Pitch Açısı:", background="#2E3B4E", foreground="white")
        self.target_pitch_label.pack(pady=15)

        self.target_pitch_entry = ttk.Entry(master, font=("Helvetica", 12))
        self.target_pitch_entry.pack(pady=10)

        self.target_roll_label = ttk.Label(master, text="Hedef Roll Açısı:", background="#2E3B4E", foreground="white")
        self.target_roll_label.pack(pady=15)

        self.target_roll_entry = ttk.Entry(master, font=("Helvetica", 12))
        self.target_roll_entry.pack(pady=10)

        # Kontrol butonları
        self.send_button = ttk.Button(master, text="Veri Gönder", command=self.send_data, width=20)
        self.send_button.pack(pady=20)

        self.motor_on_button = ttk.Button(master, text="Motor Aç", command=self.engine_on, width=20)
        self.motor_on_button.pack(pady=10)

        self.motor_off_button = ttk.Button(master, text="Motor Kapat", command=self.engine_off, width=20)
        self.motor_off_button.pack(pady=10)

        # Acil durum butonu
        self.emergency_button = ttk.Button(master, text="Acil Durum", command=self.emergency_stop, width=20,
                                           style="Danger.TButton")
        self.emergency_button.pack(pady=20)

        # Veri görüntüleme alanı
        self.data_display = tk.Text(master, height=20, width=100, bg="#1F232A", fg="white", insertbackground='white',
                                    font=("Helvetica", 12))
        self.data_display.pack(pady=20)

        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.update_data_thread = threading.Thread(target=self.update_data)
        self.update_data_thread.start()

    def send_data(self):
        target_pitch = self.target_pitch_entry.get()
        target_roll = self.target_roll_entry.get()

        if target_pitch:
            self.serial_connection.write(f"burun:{target_pitch}\n".encode())

        if target_roll:
            self.serial_connection.write(f"roll:{target_roll}\n".encode())

    def engine_on(self):
        self.serial_connection.write(b"motor_on\n")

    def engine_off(self):
        self.serial_connection.write(b"motor_off\n")

    def emergency_stop(self):
        self.serial_connection.write(b"emergency_stop\n")  # Acil durumu bildir
        self.data_display.insert(tk.END, "Acil durum aktif edildi!\n")
        self.data_display.see(tk.END)

    def update_data(self):
        while True:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().decode().strip()
                self.data_display.insert(tk.END, data + "\n")
                self.data_display.see(tk.END)
            time.sleep(0.1)

    def on_closing(self):
        self.serial_connection.close()
        self.master.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ArduinoInterface(root)
    root.mainloop()
