from multiprocessing import Process
import micro_serial.stm_serial as stm
import micro_serial.pico_serial as pico
import main_control.fusion_sensor as fusi

def main():
    # Membuat proses untuk masing-masing node
    process_a = Process(target=stm.main)
    process_b = Process(target=pico.main)
    process_c = Process(target=fusi.main)

    # Menjalankan proses
    process_a.start()
    process_b.start()
    process_c.start()

    # Menunggu proses selesai
    process_a.join()
    process_b.join()
    process_c.join()

if __name__ == '__main__':
    main()