u#!/usr/bin/env python3
import serial
import time
import sys

def test_serial_commands():
    # ESP32'nin bağlı olduğu portu bul (genellikle /dev/ttyUSB0 veya /dev/ttyACM0)
    ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/tty.usbserial-0001']

    ser = None
    for port in ports:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"✅ Port {port} açıldı")
            break
        except:
            continue

    if ser is None:
        print("❌ ESP32 portu bulunamadı. Lütfen ESP32'yi bağlayın.")
        return False

    try:
        # ESP32'nin boot etmesini bekle
        print("⏳ ESP32'nin hazır olmasını bekliyorum...")
        time.sleep(3)

        # Buffer'ı temizle
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Test komutları
        test_commands = [
            "help",
            "status",
            "sensor",
            "reset",
            "otonom",
            "manuel",
            "motor ileri",
            "motor dur",
            "servo orta"
        ]

        print("\n🧪 SERİ KOMUT TESTLERİ BAŞLIYOR...\n")

        for cmd in test_commands:
            print(f"📤 Gönderilen: {cmd}")

            # Komutu gönder
            ser.write((cmd + '\n').encode())
            time.sleep(0.1)  # Kısa bekleme

            # Yanıtı oku
            response = ""
            start_time = time.time()
            while time.time() - start_time < 2:  # 2 saniye bekle
                if ser.in_waiting > 0:
                    char = ser.read().decode('utf-8', errors='ignore')
                    response += char
                    if '\n' in response and len(response.strip()) > 0:
                        break

            if response.strip():
                print(f"📥 Yanıt: {response.strip()}")
            else:
                print("❌ Yanıt alınamadı")

            print("-" * 50)
            time.sleep(0.5)  # Komutlar arası bekleme

        print("✅ Test tamamlandı!")
        return True

    except Exception as e:
        print(f"❌ Hata: {e}")
        return False

    finally:
        if ser:
            ser.close()

if __name__ == "__main__":
    success = test_serial_commands()
    sys.exit(0 if success else 1)