📡 BLE Central UART Uygulaması – Nordic SDK (nRF52) 🔍 Proje Tanımı Bu proje, Nordic Semiconductor'ın nRF5 SDK'sı kullanılarak geliştirilen bir BLE Central (Merkez) cihaz uygulamasıdır. Uygulamanın amacı, etraftaki BLE cihazlarını taramak, filtreleme yaparak belirli cihazlara bağlanmak ve bu cihazlardan alınan verileri UART üzerinden terminale aktarmaktır. Uygulama, özellikle Nordic UART Service (NUS) protokolü ile haberleşen cihazlara yöneliktir.

🧰 Kullanılan Teknolojiler ve Araçlar Nordic nRF52 Serisi (test için: nRF52832)

nRF5 SDK v17.1.0

Segger Embedded Studio IDE

SoftDevice: S132

nRF Log, UART, GATT, NUS, Scan, DB Discovery, Power Management modülleri

⚙️ Temel Özellikler BLE cihazları tarar (aktif tarama)

UUID veya cihaz adı filtresi ile eşleşen cihazlara otomatik bağlanır

Nordic UART Service (NUS) üzerinden gelen verileri UART ile PC'ye iletir

GATT MTU güncellemelerini loglar

Düşük güç tüketimi için enerji yönetimi içerir

Hata durumlarında sistem sıfırlanır

📁 Proje Yapısı Dosya / Modül Açıklama main.c Uygulamanın ana dosyası; tüm modüllerin başlatılması ve çalışma döngüsü ble_stack_init SoftDevice ve BLE yığınının başlatılması gatt_init GATT modülü ve MTU yapılandırması scan_init BLE tarama başlatma ve filtre ayarları nus_c_init Nordic UART Service istemcisinin başlatılması db_discovery_init Servis keşif modülünün yapılandırılması uart_init UART haberleşmesinin başlatılması start_scan Tarama işleminin başlatılması connect_to_device Belirli bir cihaza bağlanma log_float_values Float veri loglama yardımıcı fonksiyonu error_handler Hataları yakalayan ve sistemi sıfırlayan fonksiyon

🔧 Donanım Ayarları Proje donanım bağımlıdır. UART pinleri aşağıdaki gibi ayarlanmıştır:

c Kopyala Düzenle #define RX_PIN_NUMBER 8 #define TX_PIN_NUMBER 6 #define RTS_PIN_NUMBER 5 #define CTS_PIN_NUMBER 7 Bağlantı parametreleri:

Bağlantı Aralığı: 15ms - 30ms

Gecikme: 0

Zaman Aşımı: 4000ms

MTU Boyutu: NRF_SDH_BLE_GATT_MAX_MTU_SIZE

🚀 Başlatma Adımları main.c dosyasını Segger Embedded Studio’da açın.

Gerekli SDK yolunu ve SoftDevice yapılandırmalarını kontrol edin.

Gerekli tanımlamaları ve bağlantı ayarlarını yapın (örneğin DEVICE_NAME, filtre UUID’si).

Derleyin ve nRF52 kartınıza yükleyin.

UART terminal (örneğin PuTTY) ile 115200 baud üzerinden bağlantı kurun.

BLE cihazı etrafa reklam verdiğinde, merkezi cihaz otomatik bağlanır ve gelen veriler UART’a yazdırılır.

📎 Notlar Bu proje BLE Central rolündedir. Peripheral (çevresel) cihazlar tarafından reklam yapılması beklenir.

NUS servisi dışında cihazlara bağlanmak isteniyorsa ble_uuid_t user_data_uuid değeri değiştirilmelidir.

BLE bağlantı güvenliği bu örnekte aktif değildir; istenirse Security Manager modülleri entegre edilebilir.

Enerji verimliliği nrf_pwr_mgmt_run() ile sağlanmaktadır.

Gerekli tüm dosyalar zip dosyasında bulunmaktadır

🧑‍💻 Katkıda Bulunmak Bu proje üzerinde geliştirme yapmak, kodu genişletmek ya da hataları bildirmek isterseniz issue açabilir veya pull request gönderebilirsiniz.

📄 Lisans Bu proje açık kaynak değildir ancak Nordic SDK bileşenleriyle geliştirilmiştir. Kendi sisteminizde test ve öğrenme amaçlı kullanabilirsiniz.
