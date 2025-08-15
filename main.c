/**
* @file main.c
* @brief A BLE Central application to connect to a custom service. 
* @note SD kartın düzgün çalışılabilmesi için logların kapalı olması gerekmektedir.
*/  
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "ble_db_discovery.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_gap.h"
#include "ble_advdata.h"
#include "nrf_ble_gq.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_delay.h"

//Bu fonksiyon zamanı rtc yapmak için kullanılmaya çalışılacak veya düzenlenecek.
DWORD get_fattime(void) {
    // Örnek olarak sabit bir tarih döndürülüyor (7 Temmuz 2023, 12:00:00)
    return ((DWORD)(2023 - 1980) << 25)    // Yıl içi bit alanı
           | ((DWORD)7 << 21)             // Ay (Temmuz)
           | ((DWORD)7 << 16)             // Gün
           | ((DWORD)12 << 11)            // Saat
           | ((DWORD)0 << 5)              // Dakika
           | ((DWORD)0 >> 1);             // Saniye / 2
}
#define FILE_NAME   "NORDIC.TXT" ///< SD karta açılan dosyanın adı
#define TEST_STRING "SD card example."///< SD karta  açılan dosyanın içerisine yazılacak test cümlesi

#define SDC_SCK_PIN     ARDUINO_13_PIN  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    ARDUINO_11_PIN  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    ARDUINO_12_PIN  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      ARDUINO_10_PIN  ///< SDC chip select (CS) pin.
/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);
/**
 * @brief UART üzerinden iletir .
 *
 * @details `app_uart_put()` çağrısını en fazla 10 kez dener.
 * Başarısız olursa uyarı logu basar ve iletimi sonlandırır.
 * Denemeler arasında çok kısa bekleme yapılır.
 *
 * @param byte Gönderilecek 8-bit veri.
 *
 * @note Bu makro bloklayıcıdır; UART tamponu doluysa gecikme yaşanabilir.
 */
#define UART_PUT_BYTE(byte)                                           \
    do {                                                              \
        uint8_t __retry = 0;                                          \
        while (app_uart_put(byte) != NRF_SUCCESS) {                   \
            if (++__retry > 10) {                                     \
                NRF_LOG_WARNING("UART Overflow at byte 0x%02X", byte);\
                break;                                                \
            }                                                         \
            nrf_delay_us(50); /* Çok kısa bekleme burada yapılır*/                  \
        }                                                             \
    } while(0)

/**
 * @brief Uygulamanın temel BLE ve UART parametreleri.
 *
 * @details Bu sabitler, cihazın tarama (scan), bağlantı (connection),
 * UART haberleşmesi ve cihaz listesi ile ilgili konfigürasyonunu tanımlar.
 */

/** @brief BLE tarama aralığı (0x00A0 = 100 ms, 0.625 ms birim). */
#define SCAN_INTERVAL       0x00A0

/** @brief BLE tarama penceresi (0x0050 = 50 ms, 0.625 ms birim). */
#define SCAN_WINDOW         0x0050

/** @brief BLE bağlantı slave latency (kaç bağlantı aralığı atlanabilir). */
#define SLAVE_LATENCY       0

/** @brief BLE bağlantı denetim zaman aşımı (4 saniye). */
#define CONN_SUP_TIMEOUT    MSEC_TO_UNITS(4000, UNIT_10_MS)

/** @brief UART TX tampon boyutu (bayt). */
#define UART_TX_BUF_SIZE    1024  

/** @brief UART RX tampon boyutu (bayt). */
#define UART_RX_BUF_SIZE    1024

/** @brief Maksimum takip edilecek cihaz sayısı. */
#define MAX_DEVICES         15

/** @brief Cihaz adı için ayrılan maksimum uzunluk (karakter). */
#define DEVICE_NAME_MAX_LEN 32

/** @brief BLE bağlantı konfigürasyon etiketi (SoftDevice için). */
#define APP_BLE_CONN_CFG_TAG 1

/** @brief BLE gözlemci (observer) önceliği. */
#define APP_BLE_OBSERVER_PRIO 3

/** @brief İstenen eş zamanlı bağlantı sayısı. */
#define DESIRED_CONNECTION_COUNT 1

/** @brief UART RX pini (nRF GPIO numarası).RX: Karşı cihazdan gelen veriyi alır. */
#define RX_PIN_NUMBER  8 

/** @brief UART TX pini (nRF GPIO numarası).TX: Karşı cihaza veri gönderir. */
#define TX_PIN_NUMBER  6

/** @brief UART RTS pini (nRF GPIO numarası).RTS: Karşı cihaza veri göndermesini durdurmasını veya başlatmasını söyler. */
#define RTS_PIN_NUMBER 5

/** @brief UART CTS pini (nRF GPIO numarası).CTS: Karşı cihazdan veri gönderilmesine izin verilip verilmediğini bildirir. */
#define CTS_PIN_NUMBER 7

/**
 * @brief IMU servis ve karakteristik UUID tanımları.
 */

/** @brief IMU özel servis UUID tabanı (128-bit, LSB format). */
#define CUSTOM_SERVICE_UUID_BASE    {0x1C, 0x2B, 0x13, 0xD5, 0x15, 0x52, 0x38, 0xA8, 0x4B, 0x4B, 0x6C, 0x3C, 0xE5, 0x17, 0x39, 0x38}

/** @brief IMU özel servis kısa UUID'si (16-bit). */
#define CUSTOM_SERVICE_UUID           0x1500

/** @brief IMU ivmeölçer (accelerometer) karakteristiği UUID'si. */
#define CUSTOM_ACCEL_CHAR_UUID        0x1502

/** @brief IMU jiroskop (gyroscope) karakteristiği UUID'si. */
#define CUSTOM_GYRO_CHAR_UUID         0x1503

/** @brief IMU yönelim (orientation) karakteristiği UUID'si. */
#define CUSTOM_ORIENTATION_CHAR_UUID  0x1401

/** @brief IMU hareketsizlik (stillness) karakteristiği UUID'si. */
#define CUSTOM_STILLNESS_CHAR_UUID    0x1402


/**
 * @brief Orientation (yönelim) çıktısı için sabitler (BMI3 sensör).
 */

/** @brief Cihaz yüzü yukarı. */
#define BMI3_FACE_UP                   0x00
/** @brief Cihaz yüzü aşağı. */
#define BMI3_FACE_DOWN                 0x01

/** @brief Dikey, üst taraf sağda. */
#define BMI3_PORTRAIT_UP_RIGHT         0x00
/** @brief Yatay, sol taraf yukarıda. */
#define BMI3_LANDSCAPE_LEFT            0x01
/** @brief Dikey, üst taraf aşağıda. */
#define BMI3_PORTRAIT_UP_DOWN          0x02
/** @brief Yatay, sağ taraf yukarıda. */
#define BMI3_LANDSCAPE_RIGHT           0x03


/**
 * @brief MTM_EEG_0x02 cihazı için servis ve karakteristik UUID tanımları.
 */

/** @brief Cihaz adı. */
#define MTM_EEG_DEVICE_NAME "MTM_EEG_0x02"

/** @brief EEG servis UUID tabanı (128-bit, LSB format). */
#define EEG_SERVICE_UUID_BASE            {0xD3, 0x39, 0xD1, 0x5D, 0x7D, 0x0C, 0xDF, 0x9C, 0x17, 0x43, 0x25, 0x86, 0x53, 0x14, 0x86, 0xAF}

/** @brief EEG ana servis kısa UUID'si. */
#define EEG_SERVICE_UUID                 0x1453

/** @brief EEG kontrol karakteristiği UUID'si (Write). */
#define EEG_CONTROL_CHAR_UUID            0x1454

/** @brief EEG veri karakteristiği UUID'si (Notify). */
#define EEG_DATA_CHAR_UUID               0x1455


/**
 * @brief Bağlantı ve veri aktarım parametreleri.
 */

/** @brief Aynı anda bağlanabilecek maksimum cihaz sayısı. Config dosyasında tanımlanılabilir durumdadır.*/
#define MAX_CONNECTED_DEVICES NRF_SDH_BLE_CENTRAL_LINK_COUNT

/** @brief İstenen MTU (Maximum Transmission Unit) boyutu. Bu kartlara bağlı olarak artırılabilir veri kalitesi artar */
#define DESIRED_MTU_SIZE 247


/**
 * @brief EEG cihazı durumlarını tanımlayan enum.
 */
typedef enum {
    STATE_IDLE,         /**< Cihaz bağlı, servis keşfi tamamlandı, komut bekleniyor. */
    STATE_RAW_SENT,     /**< RAW komutu gönderildi, cevap bekleniyor. */
    STATE_START_SENT,   /**< START komutu gönderildi, cevap/veri bekleniyor. */
    STATE_STREAMING,    /**< Cihaz veri gönderiyor. */
    STATE_STOP_SENT     /**< STOP komutu gönderildi, işlem tamamlanıyor. */
} eeg_state_t;


/**
 * @brief MTM_EEG cihazının 54 numaralı karakteristiğine gönderilen komut sabitleri.
 *
 * @note Komutlar ASCII karakter dizisi olarak tanımlıdır.
 */
static const uint8_t CMD_RAW[]   = {'R', 'A', 'W'};   /**< Ham veri talep komutu. */
static const uint8_t CMD_START[] = {'S', 'T', 'R'};   /**< Veri akışını başlatma komutu. */
static const uint8_t CMD_STOP[]  = {'S', 'T', 'O', 'P'}; /**< Veri akışını durdurma komutu. */


/**
 * @brief BLE modülleri için gerekli örnek tanımları.
 *
 * @details Bu makrolar, BLE servis keşfi, tarama, GATT ve GATT kuyruğu için
 * gerekli global yapıları oluşturur. Çoklu cihaz desteği için diziler tanımlanır.
 */

/** @brief Çoklu cihaz için servis keşif modülü örnek dizisi. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, MAX_CONNECTED_DEVICES); 

/** @brief BLE tarama modülü örneği. */
NRF_BLE_SCAN_DEF(m_scan);

/** @brief BLE GATT modülü örneği. */
NRF_BLE_GATT_DEF(m_gatt);

/** @brief BLE GATT kuyruğu örneği (çoklu cihaz desteği ile). */
NRF_BLE_GQ_DEF(m_ble_gatt_queue, MAX_CONNECTED_DEVICES, NRF_BLE_GQ_QUEUE_SIZE);



/**
 * @brief Kodun ilerleyen kısımlarında tanımlanacak olan yardımcı fonksiyonların prototipleri (bildirimleri).
 * Bu, C derleyicisinin bu fonksiyonları tanımlanmadan önce tanıyabilmesini sağlar.
 */
uint64_t get_uptime_ms(void);
void format_uptime_string(uint64_t total_ms, char *buffer, size_t buffer_len);
void enable_hvx_for_handle(uint16_t conn_handle, uint16_t cccd_handle, uint8_t hvx_type);
/**
 * @brief BLE tarama işlemi için kullanılacak parametreleri tutan yapı.
 * Bu ayarlar, cihazın çevresini gözlemleyeceğini belirler.
 */
static ble_gap_scan_params_t m_scan_params = {
    .active = 1, .interval = SCAN_INTERVAL, .window = SCAN_WINDOW, .timeout = 0, .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL
};
/**
 * @brief Bir cihaza bağlanırken teklif edilecek bağlantı parametrelerini tutan yapı.
 * Bu ayarlar, bağlantının hızını, gecikmesini ve kararlılığını etkiler.
 */
static ble_gap_conn_params_t m_connection_params = {
    .min_conn_interval = MSEC_TO_UNITS(20, UNIT_1_25_MS),  // 30ms
    .max_conn_interval = MSEC_TO_UNITS(30, UNIT_1_25_MS),  // 50ms
    .slave_latency = SLAVE_LATENCY, 
    .conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS) // 4 saniye
};


//İki cihaz için uuıd ayarları
static ble_uuid_t m_nordic_adv_uuids[] = {{CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}};
static ble_uuid_t m_eeg_service_uuids[] = {{EEG_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}};


// 1. Sadece taranan cihaz bilgilerini tutan yapı
typedef struct {
    ble_gap_addr_t address;
    char name[DEVICE_NAME_MAX_LEN];
    int8_t rssi;
} ble_device_t;

typedef struct {
    uint16_t last_packet_no;
    uint32_t total_packet_count;
    uint32_t error_count;
} eeg_chan_state_t;

/**
 * @brief Her bir bağlı cihazın bilgilerini (handle, isim, tip, karakteristikler) tutan yapı.
 * Bu yapı, birden fazla cihaza aynı anda bağlanmayı ve yönetmeyi sağlar.
 */
typedef struct {
    uint16_t conn_handle;
    char     name[DEVICE_NAME_MAX_LEN];
    bool     is_nordic_device;
    ble_gap_addr_t peer_addr;
    // Nordic Handles
    uint16_t nordic_accel_handle;
    uint16_t nordic_accel_cccd_handle;
    uint16_t nordic_gyro_handle;
    uint16_t nordic_gyro_cccd_handle;
    uint16_t nordic_orientation_handle;
    uint16_t nordic_orientation_cccd_handle;
    uint16_t nordic_stillness_handle;
    uint16_t nordic_stillness_cccd_handle;

    // MTM EEG Handles
    bool     eeg_discovery_complete;      // Bu EEG cihazının servis keşfi tamamlandı mı?
    eeg_state_t eeg_command_state;      // Bu EEG cihazının komut durumu bu önemli çünkü 2 y gönderildi 3 e gödnerilmedi durumunun önüne geçmek istedim (0:beklemede, 1:RAW gönderildi, vs.)
    bool     eeg_stop_command_sent;
    uint16_t eeg_control_char_handle;
    uint16_t eeg_control_char_cccd_handle;
    uint16_t eeg_data_char_handle;
    uint16_t eeg_data_char_cccd_handle; 
} connected_device_info_t;

/**
 * @brief Bağlantı bekleyen cihaz bilgilerini tutan yapı.
 */
typedef struct {
    ble_gap_addr_t  peer_addr;                  /**< Cihazın BLE adresi. */
    char            dev_name[DEVICE_NAME_MAX_LEN]; /**< Cihaz adı. */
    bool            is_nordic_type_candidate;   /**< Cihazın Nordic tabanlı olup olmadığı adayı. */
    bool            is_valid;                   /**< Bilgilerin geçerli olup olmadığını gösterir. */
} pending_connection_info_t;


/**
 * @brief Uygulamanın cihaz listesi ve bağlantı durumu ile ilgili global değişkenleri.
 */

/** @brief Tarama sırasında bulunan cihazların bilgilerini tutar. */
static ble_device_t m_devices[MAX_DEVICES];

/** @brief Şu ana kadar bulunan cihaz sayısı. */
static uint8_t m_device_count = 0;

/** @brief Şu anda tarama yapılıp yapılmadığını gösterir. */
static bool m_scanning = false;

/** @brief Bağlı cihazların bilgilerini tutar. */
static connected_device_info_t m_connected_devices[MAX_CONNECTED_DEVICES];

/** @brief Aktif bağlantı sayısı. */
static uint8_t m_current_connections = 0;

/** @brief Son bulunan hedef cihazın adı. */
static char s_last_found_target_name[DEVICE_NAME_MAX_LEN] = {0};

/** @brief Tüm cihazlar için global stop zamanlayıcısının başlatılıp başlatılmadığını gösterir. */
static bool m_stop_timer_started_globally = false;

/** @brief Bağlantı bekleyen tek cihazın bilgileri. */
static pending_connection_info_t m_pending_connection = {0};

/** @brief Her bağlı cihaz için EEG paket sayacı. */
static uint16_t eeg_packet_counters[MAX_CONNECTED_DEVICES] = {0};


// Fonksiyon prototipleri
static void uart_event_handler(app_uart_evt_t * p_event);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void db_disc_handler(ble_db_discovery_evt_t * p_evt);
static void start_scan(void);
static bool is_name_valid(const char *name);
static int find_device_index(uint16_t conn_handle);
void uart_send_string(const char *str);
static void get_device_name_from_advdata(ble_gap_evt_adv_report_t const * p_adv_report, char * name_buffer, size_t buffer_len);
void write_characteristic_value(uint16_t conn_handle, uint16_t char_handle, const uint8_t *data, uint16_t len);

APP_TIMER_DEF(m_eeg_manager_timer_id);  
APP_TIMER_DEF(m_eeg_stop_timer_id);    

/**
 * @brief BLE taramasını başlatır.
 */

static void start_scan(void)
{
    ret_code_t err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Scanning started.");
}

/**
 * @brief Belirtilen bir karakteristik için bildirim (notification) açma isteğini GATT kuyruğuna ekler.
 * 
 * GATT (Generic Attribute Profile), BLE cihazlarının servis–karakteristik–descriptor yapısında veri
 * alışverişi yapmasını tanımlar. Bir karakteristiğin bildirim gönderebilmesi için, o karakteristiğe
 * ait CCCD (Client Characteristic Configuration Descriptor, UUID 0x2902) değerinin yazılarak
 * aktif edilmesi gerekir.
 * 
 * CCCD’ye yazılacak değerler:
 *   0x0001 → Notification aç
 *   0x0002 → Indication aç
 *   0x0000 → Kapat
 * 
 * Bu fonksiyon, doğrudan yazma işlemi yapmaz; işlemi nrf_ble_gq (GATT Queue) kuyruğuna ekler.
 * Böylece BLE’de aynı anda sadece tek GATT işlemi yapılabilmesi kuralına uyarak çakışma hatalarını engeller.
 */
void enable_notification_for_handle(uint16_t conn_handle, uint16_t cccd_handle)
{
    // CCDD handle 0 ise bildirim açmaya çalışma (bazı karakteristiklerin CCCD'si olmaz)
    if (cccd_handle == 0) {
        NRF_LOG_WARNING("CCCD handle is 0, cannot enable notification for conn_handle %d", conn_handle);
        return;
    }

    static uint8_t cccd_value[2] = {BLE_GATT_HVX_NOTIFICATION, 0x00};  // Enable notifications

    ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = 0, // BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE genellikle tekli yazma için kullanılmaz
        .handle   = cccd_handle,
        .offset   = 0,
        .len      = sizeof(cccd_value),
        .p_value  = cccd_value
    };

    nrf_ble_gq_req_t req = {
        .type          = NRF_BLE_GQ_REQ_GATTC_WRITE,
        .error_handler = NULL, // Hata durumunda özel bir handler gerekirse eklenebilir
        .params.gattc_write = write_params
    };

    ret_code_t err_code = nrf_ble_gq_item_add(&m_ble_gatt_queue, &req, conn_handle);
    APP_ERROR_CHECK(err_code);
   // NRF_LOG_INFO("Notification scheduled for handle: 0x%X on conn_handle %d", cccd_handle, conn_handle);
    if (err_code != NRF_SUCCESS) {
         NRF_LOG_WARNING("GQ queue full! Error: 0x%X", err_code);
               }
}
/**
 * @brief Belirtilen bir karakteristiğe veri yazma isteğini GATT kuyruğuna ekler.
 * 
 * GATT (Generic Attribute Profile), BLE cihazlarının servis–karakteristik–descriptor yapısında
 * veri okuma, yazma ve bildirim alma işlemlerini tanımlar.
 * 
 * Bu fonksiyon, doğrudan yazma işlemi yapmaz; yazma isteğini nrf_ble_gq (GATT Queue) üzerinden
 * kuyruğa ekler. Bu yöntem, BLE protokolünde aynı anda yalnızca tek bir GATT işlemi yapılabilmesi
 * kuralına uyarak veri çakışmalarını önler.
 * 
 * Parametreler:
 *   conn_handle  → Bağlantı ID’si (hangi cihaza bağlı olduğumuzu belirtir)
 *   char_handle  → Yazma yapılacak karakteristiğin handle değeri
 *   data         → Yazılacak veri dizisinin adresi
 *   len          → Yazılacak verinin uzunluğu (byte cinsinden)
 * 
 * Not:
 *   Eğer char_handle 0 ise, bu karakteristik bulunamamış demektir ve yazma işlemi yapılmaz.
 */

void write_characteristic_value(uint16_t conn_handle, uint16_t char_handle, const uint8_t *data, uint16_t len) 
{
    // Karakteristik handle 0 ise, geçersiz karakteristik; yazma işlemi iptal edilir
    if (char_handle == 0) {
        NRF_LOG_WARNING("Characteristic handle is 0, cannot write value for conn_handle %d", conn_handle);
        return;
    }

    // GATT yazma parametreleri hazırlanıyor
    ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_REQ, // Normal yazma isteği
        .flags    = 0,                     // Tek seferlik yazma (prepared write değil)
        .handle   = char_handle,           // Yazılacak karakteristiğin handle değeri
        .offset   = 0,                     // Verinin baştan yazılması
        .len      = len,                   // Yazılacak veri uzunluğu
        .p_value  = data                   // Yazılacak verinin adresi
    };

    // GATT Kuyruk isteği oluşturuluyor
    nrf_ble_gq_req_t req = {
        .type          = NRF_BLE_GQ_REQ_GATTC_WRITE, // Tür: GATT Client write
        .error_handler = NULL,                       // Özel hata işleyici yok
        .params.gattc_write = write_params           // Yazma parametreleri ekleniyor
    };

    // İsteği kuyruğa ekle
    ret_code_t err_code = nrf_ble_gq_item_add(&m_ble_gatt_queue, &req, conn_handle);
    APP_ERROR_CHECK(err_code); // Hata varsa sistem hatası ver

    // İşlemin kuyruğa eklendiğini logla
    NRF_LOG_INFO("Write scheduled for handle 0x%X (conn_handle %d)", char_handle, conn_handle);
}


/**
 * @brief Verilen toplam çalışma süresini (ms cinsinden) "HH:MM:SS.mmm" formatında stringe çevirir.
 * 
 * Bu fonksiyon, sistemin çalışma süresini milisaniye cinsinden alır ve okunabilir bir zaman formatına dönüştürür.
 * 
 * Örneğin:
 *   total_ms = 3.726.451 ms  →  "01:02:06.451"
 * 
 * Parametreler:
 *   total_ms   → Toplam süre (milisaniye cinsinden, uint64_t)
 *   buffer     → Oluşan formatlı stringin yazılacağı char dizisi
 *   buffer_len → buffer’ın uzunluğu (taşmayı önlemek için)
 * 
 * Notlar:
 *   - Eğer buffer NULL veya uzunluğu 0 ise fonksiyon işlem yapmaz.
 *   - snprintf kullanıldığı için buffer taşması önlenir.
 */
void format_uptime_string(uint64_t total_ms, char *buffer, size_t buffer_len)
{
    // Geçersiz buffer kontrolü (NULL veya uzunluk 0)
    if (buffer == NULL || buffer_len == 0) return;

    uint32_t hours, minutes, seconds, milliseconds;

    // Milisaniye kısmını hesapla
    milliseconds = total_ms % 1000;

    // Saniyeyi hesapla (0–59 arası)
    seconds      = (total_ms / 1000) % 60;

    // Dakikayı hesapla (0–59 arası)
    minutes      = (total_ms / (1000 * 60)) % 60;

    // Saat kısmını hesapla (0’dan büyük olabilir, sınırsız artar)
    hours        = (total_ms / (1000 * 60 * 60));

    // Formatlı stringi oluştur (örnek: "02:15:30.123")
    snprintf(buffer, buffer_len, "%02u:%02u:%02u.%03u",
             hours,
             minutes,
             seconds,
             milliseconds);
}



/**
 * @brief Sistemin çalışma süresini (uptime) milisaniye cinsinden döndürür.
 *
 * @details `app_timer_cnt_get()` fonksiyonundan elde edilen zamanlayıcı
 * sayaç değerini milisaniyeye çevirir. Dönüş değeri 64-bit olduğu için
 * uzun çalışma sürelerinde taşma (overflow) yaşanmaz.
 *
 * @return Çalışma süresi [ms] cinsinden.
 *
 * @note APP_TIMER_CLOCK_FREQ değeri, zamanlayıcı frekansını (Hz) temsil eder.
 */
uint64_t get_uptime_ms(void)
{
    uint32_t ticks = app_timer_cnt_get();
    return ((uint64_t)ticks * 1000) / APP_TIMER_CLOCK_FREQ;
}

/**
 * @brief Verilen bağlantı handle’ına sahip cihazın bağlı cihazlar dizisindeki indeksini bulur.
 * 
 * Bu fonksiyon, BLE bağlantıları için tutulan `m_connected_devices` dizisinde
 * parametre olarak verilen `conn_handle` değerini arar.
 * 
 * Eğer bağlantı bulunursa, dizideki indeks numarası döndürülür.
 * Bulunamazsa `-1` döndürülür.
 * 
 * Parametre:
 *   conn_handle → Bağlantı ID’si (nRF52’nin BLE stack tarafından atadığı handle)
 * 
 * Geri dönüş:
 *   0..MAX_CONNECTED_DEVICES-1 → Bağlantının bulunduğu dizin
 *   -1                         → Bağlantı bulunamadı
 */
static int find_device_index(uint16_t conn_handle)
{
    // Bağlı cihazlar dizisinde dolaş
    for (int i = 0; i < MAX_CONNECTED_DEVICES; i++)
    {
        // Eğer bu indeksteki cihazın conn_handle’ı aranan ile eşleşiyorsa
        if (m_connected_devices[i].conn_handle == conn_handle)
        {
            return i; // İndeksi döndür
        }
    }

    // Hiç eşleşme bulunamazsa -1 döndür
    return -1;
}

/**
 * @brief EEG cihazlarına sıralı olarak STOP komutu göndermek için kullanılan zamanlayıcı işleyicisi.
 * 
 * Bu fonksiyon bir durum makinesi (state machine) mantığında çalışır. 30 saniyelik veri toplama
 * süresi dolduğunda tetiklenir ve bağlı tüm EEG cihazlarına STOP komutu gönderir.
 * 
 * İşleyiş:
 *   - m_current_connections kadar bağlı cihaz döngüye alınır.
 *   - Her cihaz için:
 *       - Eğer cihaz EEG cihazıysa (is_nordic_device == false) ve henüz STOP komutu gönderilmemişse
 *         (eeg_stop_command_sent == false), 
 *           - write_characteristic_value fonksiyonu ile STOP komutu karakteristiğine yazılır.
 *           - eeg_stop_command_sent true olarak işaretlenir.
 * 
 * Bu sayede:
 *   - STOP komutu her EEG cihazına yalnızca bir kez gönderilmiş olur.
 *   - Zamanlayıcı tetiklendikten sonra veri toplama güvenli bir şekilde sonlandırılır.
 * 
 * Parametre:
 *   p_context → Timer tarafından geçirilen bağlam pointer’ı (bu örnekte kullanılmıyor)
 */
static void eeg_stop_timer_handler(void * p_context)
{
    // Bilgilendirme logu
    NRF_LOG_INFO("30-second data collection period is over. Sending STOP to all EEG devices.");

    // Tüm bağlı cihazları dolaş
    for (int idx = 0; idx < m_current_connections; idx++)
    {
        // Cihaz bir EEG ise ve STOP komutu daha önce gönderilmediyse
        if (!m_connected_devices[idx].is_nordic_device && !m_connected_devices[idx].eeg_stop_command_sent)
        {
            // STOP komutunu yaz karakteristiğine gönder
            write_characteristic_value(m_connected_devices[idx].conn_handle,
                                       m_connected_devices[idx].eeg_control_char_handle,
                                       CMD_STOP, sizeof(CMD_STOP));

            // Komut gönderildi olarak işaretle, tekrar gönderilmesin
            m_connected_devices[idx].eeg_stop_command_sent = true;
        }
    }
}

/**
 * @brief Cihaz adının geçerli olup olmadığını kontrol eder.
 *
 * @details
 * Aşağıdaki isim formatları geçerli kabul edilir:
 * - Tam olarak `"Nord"` olan cihaz isimleri.
 * - `"MTM_EEG_"` ile başlayan ve devamında yalnızca rakam içeren isimler.
 * - `"MTM_ADS_"` ile başlayan ve devamında yalnızca rakam içeren isimler.
 *
 * Diğer tüm isimler geçersiz sayılır.
 *
 * @param[in] name Kontrol edilecek cihaz adı (null-terminated C string).
 *
 * @return
 * - `true`  : İsim geçerli.
 * - `false` : İsim geçersiz.
 *
 * @note Bu kontrol, cihaz keşif (scan) sırasında filtreleme için kullanılır.
 */
static bool is_name_valid(const char *name)
{
    // Adı "Nord" olan cihazları kabul et
    if (strcmp(name, "Nord") == 0)
    {
        return true;
    }

    // Adı "MTM_EEG_" ile başlayan cihazları kontrol et
    if (strstr(name, "MTM_EEG_") == name) // İsim "MTM_EEG_" ile başlıyorsa
    {
        // "MTM_EEG_" kısmından sonraki karakterlerin sadece rakamlardan oluştuğunu doğrula.
        for (int i = strlen("MTM_EEG_"); i < strlen(name); i++)
        {
            if (name[i] < '0' || name[i] > '9') // Karakter bir rakam değilse
            {
                return false; // Geçersiz isim
            }
        }
        return true; // Tüm kontrolleri geçti, isim geçerli
    }

    // Adı "MTM_ADS_" ile başlayan cihazları kontrol et
    if (strstr(name, "MTM_ADS_") == name)
    {
        for (int i = strlen("MTM_ADS_"); i < strlen(name); i++)
        {
            if (name[i] < '0' || name[i] > '9')
            {
                return false;
            }
        }
        return true;
    }

    // "Nord", "MTM_EEG_XX" veya "MTM_ADS_XX" formatında olmayan tüm isimleri reddet
    return false;
}





/**
 * @brief EEG cihaz yönetim zamanlayıcısı callback fonksiyonu.
 *
 * @details
 * Bu fonksiyon, bağlı cihazların EEG komut akışını yönetir.
 * Her zamanlayıcı tetiklenmesinde (periodik olarak) aşağıdaki adımlar uygulanır:
 * - Nordic olmayan ve EEG servis keşfi tamamlanmış cihazlar kontrol edilir.
 * - Eğer cihazın durumu `STATE_IDLE` ise:
 *   - RAW komutu (`CMD_RAW`) gönderilir.
 *   - Durum `STATE_RAW_SENT` olarak güncellenir.
 * - Eğer cihazın durumu `STATE_RAW_SENT` ise:
 *   - START komutu (`CMD_START` veya "STR") gönderilir.
 *   - Durum `STATE_STREAMING` olarak güncellenir.
 *   - Global STOP zamanlayıcısı başlatılmamışsa, 30 saniyelik STOP zamanlayıcısı başlatılır
 *     ve `m_stop_timer_started_globally` true yapılır.
 *
 * @param[in] p_context  Zamanlayıcı callback parametresi (kullanılmıyor, NULL).
 *
 * @note
 * - Global değişken `m_stop_timer_started_globally`, STOP zamanlayıcısının sadece bir kez
 *   başlatılmasını sağlar.
 * - Zamanlayıcı süresi `APP_TIMER_TICKS(30000)` ile 30 saniye olarak ayarlanmıştır.
 * - Komutlar `write_characteristic_value()` ile BLE üzerinden gönderilir.
 */
static void eeg_manager_timer_handler(void * p_context)
{
    for (int idx = 0; idx < m_current_connections; idx++)
    {
        if (!m_connected_devices[idx].is_nordic_device && m_connected_devices[idx].eeg_discovery_complete)
        {
            // Cihaz bekleme durumundaysa RAW komutu gönder
            if (m_connected_devices[idx].eeg_command_state == STATE_IDLE)
            {
                NRF_LOG_INFO("Manager: Sending RAW to %s.", m_connected_devices[idx].name);
                write_characteristic_value(m_connected_devices[idx].conn_handle,
                                           m_connected_devices[idx].eeg_control_char_handle,
                                           CMD_RAW, sizeof(CMD_RAW));
                m_connected_devices[idx].eeg_command_state = STATE_RAW_SENT;
            }
            // RAW gönderildiyse ve hala STATE_RAW_SENT durumundaysa STR komutu gönder
            else if (m_connected_devices[idx].eeg_command_state == STATE_RAW_SENT)
            {
                NRF_LOG_INFO("Manager: Sending STR to %s.", m_connected_devices[idx].name);
                write_characteristic_value(m_connected_devices[idx].conn_handle,
                                           m_connected_devices[idx].eeg_control_char_handle,
                                           CMD_START, sizeof(CMD_START));
                m_connected_devices[idx].eeg_command_state = STATE_STREAMING;
                NRF_LOG_INFO("State for %s set to STREAMING. Now waiting for data.", m_connected_devices[idx].name);

                // Global STOP zamanlayıcısını yalnızca bir kez başlat
                if (!m_stop_timer_started_globally)
                {
                    ret_code_t err_code = app_timer_start(m_eeg_stop_timer_id, APP_TIMER_TICKS(30000), NULL);
                    APP_ERROR_CHECK(err_code);
                    m_stop_timer_started_globally = true;
                    NRF_LOG_INFO("30-second STOP timer has been started.");
                }
            }
        }
    }
}

//void send_raw_data_over_uart(uint8_t device_id, const uint8_t *data, uint16_t len)
//{
//    uint8_t crc = 0;
//    for (uint16_t i = 0; i < len; i++) {
//        crc ^= data[i];
//    }

//    // Paket Başlatıcı
//    if (app_uart_put(0xA5) != NRF_SUCCESS) NRF_LOG_WARNING("UART Overflow at 0xA5");
//    if (app_uart_put(device_id) != NRF_SUCCESS) NRF_LOG_WARNING("UART Overflow at device_id");
//    if (app_uart_put(len) != NRF_SUCCESS) NRF_LOG_WARNING("UART Overflow at length");

//    // Veri gönderimi
//    for (uint16_t i = 0; i < len; i++) {
//        ret_code_t uart_ret = app_uart_put(data[i]);
//        if (uart_ret != NRF_SUCCESS) {
//            NRF_LOG_WARNING("UART Overflow at byte %d", i);
//            // Alternatif: nrf_delay_us(50); retry gibi mekanizma koyulabilir
//        }
//    }

//    // CRC ve Sonlandırıcı
//    if (app_uart_put(crc) != NRF_SUCCESS) NRF_LOG_WARNING("UART Overflow at CRC");
//    if (app_uart_put(0x5A) != NRF_SUCCESS) NRF_LOG_WARNING("UART Overflow at 0x5A");
//}

/**
 * @brief EEG veya sensör verilerini UART üzerinden güvenli şekilde gönderir.
 *
 * @details
 * Bu fonksiyon, her veri paketi için başlatıcı, cihaz ID'si, örnek sayısı,
 * veri, CRC ve paket sonlandırıcıyı gönderir. 
 * CRC basit XOR ile hesaplanır ve veri bütünlüğünü kontrol etmek için kullanılır.
 *
 * Paket formatı:
 * | Başlatıcı (0xA5) | Device ID | Sample Count LSB | Sample Count MSB |
 * @param[in] device_id     Gönderilen verinin cihaz ID'si.
 * @param[in] data          Gönderilecek 16-bit veri dizisi.
 * @param[in] sample_count  Dizideki örnek sayısı.
 *
 * @note `UART_PUT_BYTE` makrosu ile her byte bloklayıcı olarak gönderilir
 * ve tampon taşması durumunda maksimum 10 deneme yapılır.
 */
void send_raw_data_over_uart_safe(uint8_t device_id, const int16_t *data, uint16_t sample_count)
{
    uint8_t crc = 0;

    // Paket başlatıcı
    UART_PUT_BYTE(0xA5);
    UART_PUT_BYTE(device_id);
    UART_PUT_BYTE(sample_count & 0xFF);        // Sample count LSB
    UART_PUT_BYTE((sample_count >> 8) & 0xFF); // Sample count MSB

    // Veri gönderimi
    for (uint16_t i = 0; i < sample_count; i++) {
        uint8_t lsb = data[i] & 0xFF;
        uint8_t msb = (data[i] >> 8) & 0xFF;

        UART_PUT_BYTE(lsb);
        UART_PUT_BYTE(msb);

        crc ^= lsb;
        crc ^= msb;
    }

    // CRC ve paket sonlandırıcı
    UART_PUT_BYTE(crc);
    UART_PUT_BYTE(0x5A);
}


/**
 * @brief Servis keşfi tamamlandığında çalışan işleyici (handler).
 * 
 * Bu fonksiyon, BLE cihazına bağlandıktan sonra GATT servis keşfi tamamlandığında tetiklenir.
 * 
 * İşleyiş:
 *   - Eğer olay türü BLE_DB_DISCOVERY_COMPLETE değilse, fonksiyon geri döner.
 *   - Bağlantı handle’ına göre cihazın bağlı cihazlar dizisindeki indeksini bulur.
 *   - Bulunamazsa fonksiyon geri döner.
 *   - Bulunan servis türüne göre cihaz ya Nordic ya da EEG olarak işaretlenir ve ilgili
 *     karakteristik handle’ları dizide saklanır.
 *   - Nordic cihazlarda tüm sensörler için bildirimler açılır.
 *   - EEG cihazlarında veri karakteristiği için bildirimler açılır.
 * 
 * Parametre:
 *   p_evt → BLE servis keşif olay verisi (ble_db_discovery_evt_t)
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    // Eğer servis keşfi tamamlanmadıysa fonksiyondan çık
    if (p_evt->evt_type != BLE_DB_DISCOVERY_COMPLETE) return;

    // Cihazın bağlı cihazlar dizisindeki indeksini bul
    int idx = find_device_index(p_evt->conn_handle);
    if (idx == -1) return; // Bulunamazsa çık

    // Keşfedilen servisi al
    ble_gatt_db_srv_t const * p_service = &p_evt->params.discovered_db;

    // --------------- NORDIC CİHAZI İSE ----------------
    if (p_service->srv_uuid.uuid == CUSTOM_SERVICE_UUID)
    {
        // Cihazı Nordic olarak işaretle
        m_connected_devices[idx].is_nordic_device = true;

        // Logla
        NRF_LOG_INFO("Service discovery complete for %s (handle: %d), found service with UUID 0x%X", 
                     m_connected_devices[idx].name, p_evt->conn_handle, p_service->srv_uuid.uuid);

        // Karakteristik handle’larını ata
        for (uint32_t j = 0; j < p_service->char_count; j++)
        {
            switch (p_service->charateristics[j].characteristic.uuid.uuid)
            {
                case CUSTOM_ACCEL_CHAR_UUID:
                    m_connected_devices[idx].nordic_accel_handle = p_service->charateristics[j].characteristic.handle_value;
                    m_connected_devices[idx].nordic_accel_cccd_handle = p_service->charateristics[j].cccd_handle;
                    break;
                case CUSTOM_GYRO_CHAR_UUID:
                    m_connected_devices[idx].nordic_gyro_handle = p_service->charateristics[j].characteristic.handle_value;
                    m_connected_devices[idx].nordic_gyro_cccd_handle = p_service->charateristics[j].cccd_handle;
                    break;
                case CUSTOM_ORIENTATION_CHAR_UUID:
                    m_connected_devices[idx].nordic_orientation_handle = p_service->charateristics[j].characteristic.handle_value;
                    m_connected_devices[idx].nordic_orientation_cccd_handle = p_service->charateristics[j].cccd_handle;
                    break;
                case CUSTOM_STILLNESS_CHAR_UUID:
                    m_connected_devices[idx].nordic_stillness_handle = p_service->charateristics[j].characteristic.handle_value;
                    m_connected_devices[idx].nordic_stillness_cccd_handle = p_service->charateristics[j].cccd_handle;
                    break;
            }
        }

        // Nordic karakteristikler için bildirimleri etkinleştir
        enable_notification_for_handle(p_evt->conn_handle, m_connected_devices[idx].nordic_accel_cccd_handle);
        enable_notification_for_handle(p_evt->conn_handle, m_connected_devices[idx].nordic_gyro_cccd_handle);
        enable_notification_for_handle(p_evt->conn_handle, m_connected_devices[idx].nordic_orientation_cccd_handle);
        enable_notification_for_handle(p_evt->conn_handle, m_connected_devices[idx].nordic_stillness_cccd_handle);
    }
    // ---------------- EEG CİHAZI İSE -----------------
    else if (p_service->srv_uuid.uuid == EEG_SERVICE_UUID)
    {
        m_connected_devices[idx].is_nordic_device = false;

        // Karakteristik handle’larını ata
        for (uint32_t j = 0; j < p_service->char_count; j++)
        {
            switch (p_service->charateristics[j].characteristic.uuid.uuid)
            {
                case EEG_CONTROL_CHAR_UUID:
                    m_connected_devices[idx].eeg_control_char_handle = p_service->charateristics[j].characteristic.handle_value;
                    m_connected_devices[idx].eeg_control_char_cccd_handle = p_service->charateristics[j].cccd_handle;
                    break;
                case EEG_DATA_CHAR_UUID:
                    m_connected_devices[idx].eeg_data_char_handle = p_service->charateristics[j].characteristic.handle_value;
                    m_connected_devices[idx].eeg_data_char_cccd_handle = p_service->charateristics[j].cccd_handle;
                    break;
            }
        }

        // EEG veri karakteristiği için bildirimleri aç
        enable_hvx_for_handle(p_evt->conn_handle,
                              m_connected_devices[idx].eeg_data_char_cccd_handle,
                              BLE_GATT_HVX_NOTIFICATION);

        // EEG keşfinin tamamlandığını işaretle
        m_connected_devices[idx].eeg_discovery_complete = true;

        // Logla
        NRF_LOG_INFO("Discovery complete for %s. Data channel notifications enabled.", m_connected_devices[idx].name);
    }
}


//BU KODDA KULANILMAYAN TEMEL ALINAN ÖRNEK KOD 
// NOT: BU KOD SD KART İÇİNDİR.
static void fatfs_example()
{
    static FATFS fs;
    static DIR dir;
    static FILINFO fno;
    static FIL file;
    uint16_t percentage;

    uint32_t bytes_written;
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
        NRF_LOG_INFO("Disk init state: %d", disk_state);

    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.");
        return;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
        return;
    }

    NRF_LOG_INFO("\r\n Listing directory: /");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!");
        return;
    }

    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return;
        }

        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)fno.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
    NRF_LOG_RAW_INFO("");

    NRF_LOG_INFO("Writing to file " FILE_NAME "...");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
        return;
    }
    
    ff_result = f_write(&file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);

    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
        NRF_LOG_INFO("%d bytes written.", bytes_written);
    }

    (void) f_close(&file);
    return;
}

/**
 * @brief SD kartı başlatır ve log dosyasını açar.
 * 
 * Bu fonksiyon, sistem başlatıldığında SD kartın kullanılabilir hale gelmesini sağlar
 * ve FATFS dosya sistemi üzerinden log dosyasını açar.
 * 
 * İşleyiş adımları:
 *   1. SD kart blok cihazını kaydeder ve disk sürücüsünü başlatır.
 *   2. Diskin başlatılması 3 deneme ile yapılır; başarısız olursa hata logu verir.
 *   3. Dosya sistemini (FATFS) mount eder.
 *   4. Log dosyasını açar (varsa ekleme modunda), başarılıysa `log_file_opened = true`.
 * 
 * Geri dönüş:
 *   - true  : SD kart başarıyla başlatıldı ve log dosyası açıldı.
 *   - false : Başlatma veya dosya açma işlemi başarısız oldu.
 */
static FATFS fs;                 // FATFS yapı nesnesi
static FIL log_file;              // Log dosyası nesnesi
static bool log_file_opened = false; // Log dosyası açıldı mı?

bool sd_card_init(void)
{
    // SD kart blok cihazı yapılandırması
    static diskio_blkdev_t drives[] =
    {
        DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    // Blok cihazı kaydet
    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");

    // Disk başlatma durumunu tutacak değişken
    DSTATUS disk_state = STA_NOINIT;

    // Diski maksimum 3 kez başlatmayı dene
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }

    // Disk başlatma başarısızsa hata logu ve false döndür
    if (disk_state)
    {
        NRF_LOG_ERROR("Disk initialization failed: %d", disk_state);
        return false;
    }

    // Dosya sistemini mount et
    NRF_LOG_INFO("Mounting volume...");
    FRESULT ff_result = f_mount(&fs, "", 1);
    if (ff_result != FR_OK)
    {
        NRF_LOG_ERROR("Mount failed: %d", ff_result);
        return false;
    }

    // Log dosyasını aç (yoksa oluştur, varsa ekle)
    FRESULT res = f_open(&log_file, FILE_NAME, FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) 
    {
        NRF_LOG_ERROR("Log dosyası açılamadı: %d", res);
        return false;
    }

    // Başarıyla açıldı
    log_file_opened = true;

    return true;
}

/**
 * @brief Log verisini SD karta yazar.
 *
 * Bu fonksiyon, daha önce açılmış olan log dosyasına veri ekler.
 * 
 * İşleyiş:
 *   - Eğer log dosyası açılmamışsa (`log_file_opened == false`), fonksiyon hiçbir şey yapmaz.
 *   - `f_write()` ile verilen string dosyaya yazılır.
 *   - Yazılan byte sayısı beklenen uzunlukla eşleşmezse hata logu basılır.
 *   - Her 50 yazmadan sonra `f_sync()` çağrılarak dosya senkronize edilir ve veri güvenliği sağlanır.
 * 
 * Parametre:
 *   log_str → Yazılacak null-terminated log stringi
 */
void fatfs_write_log(const char *log_str)
{
    if (!log_file_opened) return; // Dosya açık değilse çık

    UINT bw; // Yazılan byte sayısı
    FRESULT res = f_write(&log_file, log_str, strlen(log_str), &bw); // Dosyaya yaz

    // Yazma hatası veya eksik byte yazıldıysa hata logu
    if (res != FR_OK || bw != strlen(log_str)) {
        NRF_LOG_ERROR("Dosya yazma hatası: %d", res);
    }

    // Dosya senkronizasyonu için sayaç
    static uint16_t sync_counter = 0;
    if (++sync_counter >= 50) {  // Her 50 yazmadan sonra
        f_sync(&log_file);        // Dosyayı diske yaz
        sync_counter = 0;         // Sayaçı sıfırla
    }
}

/**
 * @brief Açık olan log dosyasını kapatır.
 *
 * Bu fonksiyon, sistem kapanırken veya loglamayı durdururken çağrılır.
 * 
 * İşleyiş:
 *   - Eğer log dosyası açıksa, `f_close()` ile kapatılır.
 *   - `log_file_opened` false yapılarak durum güncellenir.
 */
void fatfs_close_log_file(void)
{
    if (log_file_opened) {
        f_close(&log_file);        // Dosyayı kapat
        log_file_opened = false;   // Durumu güncelle
    }
}



/**
 * @brief Log verilerini geçici olarak saklamak için kullanılan çevrimsel (ring) buffer.
 *
 * @details
 * Bu yapı, özellikle SD karta veya UART’a yazmadan önce log verilerini
 * geçici olarak tutmak için tasarlanmıştır. Tek yazar / tek okuyucu
 * senaryosu için uygundur.
 */

#define LOG_BUFFER_SIZE 65536 /**< Log buffer boyutu (bayt). */

static char log_buffer[LOG_BUFFER_SIZE]; /**< Log verilerini tutan dizi. */
static volatile uint32_t log_buf_head = 0; /**< Buffer başı (yazma konumu). */
static volatile uint32_t log_buf_tail = 0; /**< Buffer sonu (okuma konumu). */

/**
 * @brief Log buffer'ın boş olup olmadığını kontrol eder.
 * @return true: buffer boş, false: buffer dolu veya kısmen dolu.
 */
bool log_buffer_is_empty(void) {
    return log_buf_head == log_buf_tail;
}

/**
 * @brief Log buffer'ın dolu olup olmadığını kontrol eder.
 * @return true: buffer dolu, false: buffer boş veya kısmen dolu.
 */
bool log_buffer_is_full(void) {
    return ((log_buf_head + 1) % LOG_BUFFER_SIZE) == log_buf_tail;
}

/**
 * @brief Buffer'a tek bir karakter ekler.
 * @param[in] c Eklenecek karakter.
 * @return true: karakter başarıyla eklendi, false: buffer dolu.
 */
bool log_buffer_push(char c) {
    if (log_buffer_is_full()) return false;
    log_buffer[log_buf_head] = c;
    log_buf_head = (log_buf_head + 1) % LOG_BUFFER_SIZE;
    return true;
}

/**
 * @brief Buffer'a null-terminated string ekler.
 * @param[in] str Eklenecek string.
 * @return true: string başarıyla eklendi, false: buffer dolu.
 */
bool log_buffer_push_string(const char* str) {
    while (*str) {
        if (!log_buffer_push(*str++)) return false;
    }
    return true;
}

/**
 * @brief Buffer'dan bir karakter okur.
 * @param[out] c Okunan karakterin adresi.
 * @return true: karakter başarıyla okundu, false: buffer boş.
 */
bool log_buffer_pop(char *c) {
    if (log_buffer_is_empty()) return false;
    *c = log_buffer[log_buf_tail];
    log_buf_tail = (log_buf_tail + 1) % LOG_BUFFER_SIZE;
    return true;
}
/**
 * @brief Temel BLE bağlantı ve veri alma işlemleri burada yapılır.
 *
 * Bu fonksiyon, BLE stack'ten gelen tüm olayları (event) işler:
 *   - Cihaz tarama (advertising) raporları
 *   - Bağlantı ve bağlantı kesilme olayları
 *   - MTU değişim yanıtları
 *   - HVX (notification/indication) olayları
 *
 * @param[in] p_ble_evt BLE olay verisi
 * @param[in] p_context Kullanıcı bağlamı (context), burada kullanılmıyor
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    char uart_buf[256];

    switch (p_ble_evt->header.evt_id)
    {
        // ---------------- BLE_ADV_REPORT ----------------
        case BLE_GAP_EVT_ADV_REPORT:
        {
            // Eğer maksimum bağlantı sayısına ulaşıldıysa taramayı durdur
            if (m_current_connections >= MAX_CONNECTED_DEVICES) {
                nrf_ble_scan_stop();
                return;
            }

            ble_gap_evt_adv_report_t const * p_adv_report = &p_ble_evt->evt.gap_evt.params.adv_report;
            char dev_name[DEVICE_NAME_MAX_LEN];
            get_device_name_from_advdata(p_adv_report, dev_name, sizeof(dev_name));

            // Geçerli cihaz isimlerini filtrele
            if ((strcmp(dev_name, "Nord") == 0) || 
                (strstr(dev_name, "MTM_EEG_") == dev_name) || 
                (strstr(dev_name, "MTM_ADS_") == dev_name))
            {
                // Cihaza zaten bağlı mıyız kontrolü
                for (int i = 0; i < m_current_connections; i++) {
                    if (m_connected_devices[i].conn_handle != BLE_CONN_HANDLE_INVALID && 
                        strcmp(m_connected_devices[i].name, dev_name) == 0) {
                        return; // Zaten bağlı
                    }
                }
                
                NRF_LOG_INFO(">>> Found VALID TARGET: %s. Attempting to connect...", dev_name);
                strcpy(s_last_found_target_name, dev_name);
                
                // Bağlantı isteği gönder
                err_code = sd_ble_gap_connect(&p_adv_report->peer_addr, &m_scan_params, &m_connection_params, APP_BLE_CONN_CFG_TAG);
                if (err_code != NRF_SUCCESS) {
                    NRF_LOG_ERROR("Connection attempt failed for %s, error 0x%X", dev_name, err_code);
                }
            }
            // İsim geçerli değilse hiçbir şey yapma
            break;
        }

        // ---------------- BLE_CONNECTED ----------------
        case BLE_GAP_EVT_CONNECTED:
        {
            // MTU değişim isteği gönder
            sd_ble_gattc_exchange_mtu_request(conn_handle, DESIRED_MTU_SIZE);
            NRF_LOG_INFO("Connected, handle: %d.", conn_handle);

            int idx = m_current_connections;
            if (idx < MAX_CONNECTED_DEVICES) {
                // Cihaz bilgilerini sıfırla ve doldur
                memset(&m_connected_devices[idx], 0, sizeof(connected_device_info_t));
                m_connected_devices[idx].conn_handle = conn_handle;
                strcpy(m_connected_devices[idx].name, s_last_found_target_name);

                // Servis keşfini başlat
                err_code = ble_db_discovery_start(&m_db_disc[idx], conn_handle);
                APP_ERROR_CHECK(err_code);

                m_current_connections++;
            }

            // Hala boş slot varsa taramaya devam et
            if (m_current_connections < MAX_CONNECTED_DEVICES) { start_scan(); }
            break;
        }

        // ---------------- BLE_DISCONNECTED ----------------
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected, handle: %d, reason: 0x%X.", conn_handle, p_ble_evt->evt.gap_evt.params.disconnected.reason);

            int idx = find_device_index(conn_handle);
            if (idx != -1) {
                // Bağlantıyı listeden kaldır ve cihaz dizisini kaydır
                m_current_connections--;
                for (int i = idx; i < m_current_connections; i++) {
                    m_connected_devices[i] = m_connected_devices[i + 1];
                }
                memset(&m_connected_devices[m_current_connections], 0, sizeof(connected_device_info_t));
                m_connected_devices[m_current_connections].conn_handle = BLE_CONN_HANDLE_INVALID;
            }
            start_scan(); // Taramayı tekrar başlat
            break;
        }

        // ---------------- BLE_GATTC_EVT_EXCHANGE_MTU_RSP ----------------
        case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
        {
            uint16_t mtu = p_ble_evt->evt.gattc_evt.params.exchange_mtu_rsp.server_rx_mtu; 
            // MTU değişimi yanıtı alındı, burada loglanabilir
            break;
        }

        // ---------------- BLE_GATTC_EVT_HVX (Notification / Indication) ----------------
        case BLE_GATTC_EVT_HVX: 
        {
            int idx = find_device_index(conn_handle);
            if (idx == -1) break; // Bağlı cihaz bulunamadı

            char uart_buf[256];
            char time_str[24] = {0};
            format_uptime_string(get_uptime_ms(), time_str, sizeof(time_str));

            uint16_t handle = p_ble_evt->evt.gattc_evt.params.hvx.handle;
            const uint8_t *p_data = p_ble_evt->evt.gattc_evt.params.hvx.data;
            uint16_t length = p_ble_evt->evt.gattc_evt.params.hvx.len;          

            if (m_connected_devices[idx].is_nordic_device)
            {
                // ---------------- NORDIC CİHAZI VERİLERİ ----------------
                if (handle == m_connected_devices[idx].nordic_accel_handle && length == 6)
                {
                    int16_t x, y, z;
                    memcpy(&x, &p_data[0], 2);
                    memcpy(&y, &p_data[2], 2);
                    memcpy(&z, &p_data[4], 2);

                    NRF_LOG_INFO("Timestamp: %s | Nordic Accelerometer: x=%d, y=%d, z=%d", time_str, x, y, z);
                    snprintf(uart_buf, sizeof(uart_buf), "NA,%s,%d,%d,%d\r\n", time_str, x, y, z);
                    fatfs_write_log(uart_buf);
                }
                else if (handle == m_connected_devices[idx].nordic_gyro_handle && length == 6)
                {
                    int16_t gx, gy, gz;
                    memcpy(&gx, &p_data[0], 2);
                    memcpy(&gy, &p_data[2], 2);
                    memcpy(&gz, &p_data[4], 2);

                    NRF_LOG_INFO("Timestamp: %s | Nordic Gyroscope: x=%d, y=%d, z=%d", time_str, gx, gy, gz);
                    snprintf(uart_buf, sizeof(uart_buf), "NG,%s,%d,%d,%d\r\n", time_str, gx, gy, gz);
                }
                else if (handle == m_connected_devices[idx].nordic_orientation_handle && length >= 1)
                {
                    uint8_t orientation_val = p_data[0];
                    char orientation_str[20], facing_str[20];

                    // Orientation decoding
                    if (orientation_val == BMI3_PORTRAIT_UP_RIGHT) strcpy(orientation_str, "PORTRAIT_UP_RIGHT");
                    else if (orientation_val == BMI3_LANDSCAPE_LEFT) strcpy(orientation_str, "LANDSCAPE_LEFT");
                    else if (orientation_val == BMI3_PORTRAIT_UP_DOWN) strcpy(orientation_str, "PORTRAIT_UP_DOWN");
                    else if (orientation_val == BMI3_LANDSCAPE_RIGHT) strcpy(orientation_str, "LANDSCAPE_RIGHT");
                    else strcpy(orientation_str, "UNKNOWN");

                    // Facing decoding
                    if (length >= 2) {
                        uint8_t facing_val = p_data[1];
                        if (facing_val == BMI3_FACE_UP) strcpy(facing_str, "FACE_UP");
                        else if (facing_val == BMI3_FACE_DOWN) strcpy(facing_str, "FACE_DOWN");
                        else snprintf(facing_str, sizeof(facing_str), "UNKNOWN(0x%02X)", facing_val);
                    } else {
                        strcpy(facing_str, "N/A");
                    }

                    NRF_LOG_INFO("Timestamp: %s | Nordic Orientation: %s, Facing: %s", time_str, orientation_str, facing_str);
                    snprintf(uart_buf, sizeof(uart_buf), "NO,%s,%s,%s\r\n", time_str, orientation_str, facing_str);
                }
                else if (handle == m_connected_devices[idx].nordic_stillness_handle && length == 1)
                {
                    uint8_t still = p_data[0];
                    NRF_LOG_INFO("Timestamp: %s | Nordic Stillness: %d", time_str, still);
                    snprintf(uart_buf, sizeof(uart_buf), "NS,%s,%d\r\n", time_str, still);
                }
            }
            else
            {
                // ---------------- MTM EEG CİHAZI VERİLERİ ----------------
                if (handle == m_connected_devices[idx].eeg_data_char_handle && length == 8) 
                {
                    int16_t ch1, ch2, ch3, ch4;
                    memcpy(&ch1, &p_data[0], 2);
                    memcpy(&ch2, &p_data[2], 2);
                    memcpy(&ch3, &p_data[4], 2);
                    memcpy(&ch4, &p_data[6], 2);

                    char time_str[24];
                    format_uptime_string(get_uptime_ms(), time_str, sizeof(time_str));

                    char log_line[64];
                    snprintf(log_line, sizeof(log_line),
                             "%s,0x%04X,0x%04X,0x%04X,0x%04X\r\n",
                             time_str,
                             (uint16_t)ch1,
                             (uint16_t)ch2,
                             (uint16_t)ch3,
                             (uint16_t)ch4);

                    // Buffer’a ekle, dosyaya yazmayı yönet
                    log_buffer_push_string(log_line);
                }
            }
            break; // HVX case sonu
        }

        default:
            break;
    }
}

/**
 * @brief Main döngüsünde log buffer'daki verileri SD karta yazar.
 *
 * Bu fonksiyon, log buffer'da biriken verileri alır ve SD karta toplu şekilde yazar.
 * Bu sayede çok sayıda küçük yazma işlemi yerine daha verimli toplu yazma yapılır.
 *
 * İşleyiş:
 *   - `log_buffer` boş değilse karakter karakter okur (`log_buffer_pop`).
 *   - Okunan karakterler geçici `write_buf` buffer'ına eklenir.
 *   - Eğer `write_buf` dolarsa veya bir satır sonu (`\n`) gelirse:
 *       - Buffer sonlandırılır (`\0` eklenir)
 *       - `fatfs_write_log()` ile SD karta yazılır
 *       - `write_len` sıfırlanır ve buffer tekrar doldurulur
 *
 * @note Bu fonksiyon tipik olarak ana döngüde veya periyodik task içinde çağrılır.
 *       SD karta yazma verimliliğini artırmak için küçük logları birleştirir.
 */
void main_loop_process(void)
{
    static char ch;                    // Log buffer'dan alınan karakter
    static char write_buf[1024];       // SD karta yazılacak geçici buffer
    static uint32_t write_len = 0;     // Geçici buffer doluluk miktarı

    // Log buffer boş değilse karakter karakter işleme
    while (!log_buffer_is_empty()) {
        log_buffer_pop(&ch);           // Bir karakter al
        write_buf[write_len++] = ch;   // Buffer'a ekle

        // Buffer dolduysa veya satır sonu gelmişse yaz
        if (write_len >= sizeof(write_buf) - 1 || ch == '\n') {
            write_buf[write_len] = '\0';      // String sonlandır
            fatfs_write_log(write_buf);       // SD karta yaz
            write_len = 0;                     // Buffer sıfırla
        }
    }
}


/**
 * @brief Bir karakteristik için bildirimleri (notifications) etkinleştirmeyi GATT kuyruğuna ekler.
 *
 * BLE cihazlarında bir karakteristiğin değeri değiştiğinde, cihazın bu değeri otomatik olarak
 * client'a göndermesini sağlamak için CCCD (Client Characteristic Configuration Descriptor)
 * değerine 0x0001 yazılır. Bu fonksiyon, yazma işlemini güvenli bir şekilde GATT kuyruğuna ekler.
 *
 * İşleyiş:
 * 1. CCCD handle geçerli değilse uyarı loglanır ve işlem yapılmaz.
 * 2. Notification için gerekli değer (0x0001) hazırlanır.
 * 3. `ble_gattc_write_params_t` ile GATT yazma parametreleri oluşturulur.
 * 4. `nrf_ble_gq_req_t` ile GATT kuyruğu (GATT Queue) üzerinden yazma isteği sıraya eklenir.
 * 5. Hata varsa `APP_ERROR_CHECK` ile kontrol edilir.
 * 6. Başarıyla sıraya eklenirse bilgi logu yazılır.
 *
 * @param[in] conn_handle Bağlantı tutamacı (hangi cihaza yazılacağını belirtir)
 * @param[in] cccd_handle Bildirimleri etkinleştirecek CCCD karakteristik tutamacı
 */
void enable_notifications(uint16_t conn_handle, uint16_t cccd_handle)
{
    // CCCD handle geçersizse işlem yapma
    if (cccd_handle == 0) {
        NRF_LOG_WARNING("CCCD handle is 0, cannot enable notification for conn_handle %d", conn_handle);
        return;
    }

    // Notification etkinleştirme değeri: 0x0001 (LSB first)
    static uint8_t cccd_value[2] = {BLE_GATT_HVX_NOTIFICATION, 0x00};

    // GATT Write parametreleri hazırlanıyor
    ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_REQ,  // Yazma isteği
        .flags    = 0,                      // Özel bayrak yok
        .handle   = cccd_handle,            // Hedef CCCD handle
        .offset   = 0,                      // Başlangıç offset
        .len      = sizeof(cccd_value),     // Yazılacak veri uzunluğu (2 byte)
        .p_value  = cccd_value              // Yazılacak veri
    };

    // GATT Kuyruğu üzerinden yazma isteği oluşturuluyor
    nrf_ble_gq_req_t req = {
        .type          = NRF_BLE_GQ_REQ_GATTC_WRITE,  // GATT client write request
        .error_handler = NULL,                        // Hata yönetimi opsiyonel
        .params.gattc_write = write_params           // Yazma parametreleri
    };

    // İsteği GATT kuyruğuna ekle
    ret_code_t err_code = nrf_ble_gq_item_add(&m_ble_gatt_queue, &req, conn_handle);
    APP_ERROR_CHECK(err_code); // Hata varsa uygulamayı durdur veya logla

    // Başarıyla sıraya eklendiği bilgisi loglanır
    NRF_LOG_INFO("Notification scheduled for cccd_handle: 0x%X on conn_handle %d", cccd_handle, conn_handle);
}

/**
 * @brief Verilen bir metni (string) UART üzerinden karakter karakter gönderir.
 *
 * Bu fonksiyon, UART üzerinden veri gönderirken tampon (FIFO) doluysa bekler
 * ve bu sırada işlemciyi düşük güç modunda çalıştırır.
 *
 * İşleyiş:
 * 1. `str` NULL ise fonksiyon hiçbir işlem yapmaz.
 * 2. Her karakter `app_uart_put()` ile gönderilir.
 * 3. Eğer UART tamponu doluysa, `app_uart_put()` başarısız olur ve
 *    while döngüsü ile beklenir.
 * 4. Bekleme sırasında `nrf_pwr_mgmt_run()` çağrılarak işlemci düşük güç modunda çalışır,
 *    böylece güç tasarrufu sağlanır.
 *
 * @param[in] str Gönderilecek null-terminated string.
 */
void uart_send_string(const char *str)
{
    if (str == NULL) return; // Null kontrolü

    // Her karakteri sırayla gönder
    for (size_t i = 0; i < strlen(str); i++) {
        // Eğer UART tamponu doluysa bekle ve güç yönetimini çalıştır
        while (app_uart_put(str[i]) != NRF_SUCCESS) {
            nrf_pwr_mgmt_run(); // Düşük güç modu
        }
    }
}
/**
 * @brief Gelen bir BLE anons (advertising) raporundan cihazın ismini ayıklar.
 *
 * BLE anons paketleri, 'length-type-value' (uzunluk-tip-veri) formatında alanlardan oluşur.
 * Bu fonksiyon, her alanı tek tek kontrol eder ve tipi 'Complete Local Name' veya
 * 'Short Local Name' olan alanı bulursa, içerisindeki cihaz ismini verilen tampona kopyalar.
 *
 * İşleyiş:
 * 1. Parametreler geçersizse (NULL veya uzunluk sıfırsa), tampon "NULL_REPORT" ile doldurulur.
 * 2. Veri alanları boyunca döngü çalışır:
 *    - Her alanın uzunluğu ve tipi okunur.
 *    - Tip uygun ise isim uzunluğu belirlenir ve tampon boyutu aşılmamasına dikkat edilerek kopyalanır.
 *    - String sonlandırıcı ('\0') eklenir.
 * 3. Eğer isim bulunamazsa tampon "Unknown" ile doldurulur.
 *
 * @param[in] p_adv_report BLE advertising raporu
 * @param[out] name_buffer Bulunan cihaz isminin kopyalanacağı tampon
 * @param[in] buffer_len Tampon uzunluğu
 */
static void get_device_name_from_advdata(ble_gap_evt_adv_report_t const * p_adv_report,
                                         char * name_buffer,
                                         size_t buffer_len)
{
    // Parametre kontrolü
    if (buffer_len == 0 || name_buffer == NULL || p_adv_report == NULL || p_adv_report->data.p_data == NULL) {
        if (name_buffer) strncpy(name_buffer, "NULL_REPORT", buffer_len - 1);
        return;
    }

    name_buffer[0] = '\0'; // Önce boş string yap
    uint8_t * p_data = p_adv_report->data.p_data;
    uint8_t data_len = p_adv_report->data.len;

    // Tüm field'ları dolaş
    for (uint8_t index = 0; index < data_len;)
    {
        uint8_t field_len = p_data[index]; // Field uzunluğu
        if (field_len == 0 || (index + field_len) > data_len) break;
        uint8_t field_type = p_data[index + 1]; // Field tipi

        // Eğer field tipi Complete veya Short Local Name ise
        if (field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME ||
            field_type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME) {
            
            uint8_t name_len = field_len - 1; // Tip baytını çıkar
            if (name_len > buffer_len - 1) name_len = buffer_len - 1; // Tampon sınırı
            memcpy(name_buffer, &p_data[index + 2], name_len); // İsmi kopyala
            name_buffer[name_len] = '\0'; // Sonlandırıcı ekle
            return;
        }
        index += field_len + 1; // Bir sonraki field'a geç
    }

    // Eğer isim bulunamazsa
    strncpy(name_buffer, "Unknown", buffer_len - 1);
    name_buffer[buffer_len - 1] = '\0';
}
/**
 * @brief UART donanımını loglama ve seri haberleşme için yapılandırır ve başlatır.
 *
 * İşleyiş:
 * 1. RX, TX, RTS, CTS pinleri belirlenir (nRF GPIO numaraları).
 * 2. FIFO tampon boyutları ve baud hızı (115200) ayarlanır.
 * 3. UART olayları için `uart_event_handler` callback fonksiyonu kaydedilir.
 * 4. APP_UART_FIFO_INIT ile UART başlatılır ve hata kontrolü yapılır.
 *
 * @note RTS/CTS pinleri, donanımsal akış kontrolü sağlar ve veri kaybını önler.
 */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params = {
        .rx_pin_no   = RX_PIN_NUMBER,
        .tx_pin_no   = TX_PIN_NUMBER,
        .rts_pin_no  = RTS_PIN_NUMBER,
        .cts_pin_no  = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_ENABLED, // Donanımsal akış kontrolü
        .use_parity  = false,
        .baud_rate   = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handler,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code); // Başarısızsa hata verir
}

/**
 * @brief GATT (Generic Attribute Profile) modülünü başlatır ve ayarlar.
 *
 * İşleyiş:
 * 1. `nrf_ble_gatt_init` ile GATT modülü başlatılır.
 * 2. MTU boyutu artırılır (`nrf_ble_gatt_att_mtu_central_set`) böylece tek pakette
 *    daha fazla veri gönderilebilir.
 * 3. Hatalar APP_ERROR_CHECK ile kontrol edilir.
 *
 * @note MTU boyutunun artırılması, sensör verisi veya yüksek hacimli BLE veri
 *       akışlarında performansı artırır.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, 247); // Maksimum MTU boyutu
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief BLE Servis Keşfi (Database Discovery) modülünü başlatır.
 *
 * İşleyiş:
 * 1. `ble_db_discovery_init_t` yapısı sıfırlanır ve olay işleyici (`db_disc_handler`)
 *    ve GATT kuyruğu (`m_ble_gatt_queue`) atanır.
 * 2. `ble_db_discovery_init()` ile servis keşfi modülü başlatılır.
 * 3. Keşfedilecek servis UUID'leri kaydedilir:
 *    - Nordic sensör servisi (CUSTOM_SERVICE_UUID)
 *    - MTM EEG sensör servisi (EEG_SERVICE_UUID)
 * 4. Her UUID için `ble_db_discovery_evt_register()` çağrılır ve hata kontrolü yapılır.
 *
 * @note Bu fonksiyon, cihaza bağlandıktan sonra servis ve karakteristikleri otomatik
 *       olarak bulmak ve kullanmak için gereklidir.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code;
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));
    db_init.evt_handler  = db_disc_handler;   // Servis keşfi tamamlandığında çağrılacak handler
    db_init.p_gatt_queue = &m_ble_gatt_queue; // GATT istekleri için kuyruk

    err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code); // Başarısızsa hata verir
    
    // Nordic sensör servisi için UUID kaydı
    ble_uuid_t nordic_db_disc_uuid = {
        .uuid = CUSTOM_SERVICE_UUID,
        .type = m_nordic_adv_uuids[0].type
    };
    err_code = ble_db_discovery_evt_register(&nordic_db_disc_uuid);
    APP_ERROR_CHECK(err_code);

    // MTM EEG sensör servisi için UUID kaydı
    ble_uuid_t eeg_db_disc_uuid = {
        .uuid = EEG_SERVICE_UUID,
        .type = m_eeg_service_uuids[0].type
    };
    err_code = ble_db_discovery_evt_register(&eeg_db_disc_uuid);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief BLE Tarama (Scanning) modülünü başlatır.
 *
 * İşleyiş:
 * 1. `nrf_ble_scan_init()` ile tarama (scan) modülü başlatılır.
 *    - `m_scan` yapısı tarama parametrelerini tutar.
 *    - İkinci ve üçüncü parametreler NULL bırakılarak varsayılan event handler ve context kullanılır.
 * 2. `nrf_ble_scan_filters_disable()` ile tüm filtreler devre dışı bırakılır.
 *    - Yani tarama sırasında tüm anons paketleri alınır, filtre uygulanmaz.
 * 3. Her iki fonksiyon için hata kontrolü (`APP_ERROR_CHECK`) yapılır.
 *
 * @note Bu fonksiyon, çevredeki BLE cihazlarını bulmak için ana döngü veya başlangıçta çağrılır.
 */
static void scan_init(void)
{
    ret_code_t err_code;

    // Tarama modülünü başlat
    err_code = nrf_ble_scan_init(&m_scan, NULL, NULL); 
    APP_ERROR_CHECK(err_code);

    // Tüm filtreleri devre dışı bırak (her anonsu al)
    err_code = nrf_ble_scan_filters_disable(&m_scan);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Özel 128-bit base UUID'leri SoftDevice'e (BLE yığınına) kaydeder.
 *
 * İşleyiş:
 * 1. BLE yığını, standart olmayan (vendor-specific) UUID'leri tanıyabilmek için
 *    bu UUID'leri önceden kaydetmemizi ister.
 * 2. `static bool` bayrakları ile Nordic ve EEG UUID'lerinin sadece bir kez kaydedilmesi sağlanır.
 * 3. Nordic sensör UUID'si (`CUSTOM_SERVICE_UUID_BASE`) `sd_ble_uuid_vs_add()` ile SoftDevice'e eklenir.
 *    - Bu işlem sonrası yığına atanmış kısa bir `type` numarası alınır.
 *    - `m_nordic_adv_uuids[0].type` bu type numarası ile güncellenir.
 * 4. MTM EEG sensör UUID'si (`EEG_SERVICE_UUID_BASE`) aynı şekilde eklenir ve `m_eeg_service_uuids[0].type` güncellenir.
 * 5. Her iki UUID kaydı için başarılı işlem loglanır.
 *
 * @note Bu fonksiyon uygulama başlangıcında bir kez çağrılır.
 * @note SoftDevice’e eklenmiş olan type numarası, daha sonra UUID kullanılarak servis keşfi
 *       ve bağlanma işlemlerinde hızlı erişim sağlar.
 */
static void init_custom_uuid(void)
{
    static bool nordic_uuid_registered = false;
    static bool eeg_uuid_registered = false;

    // Nordic sensör UUID'sini ekle
    if (!nordic_uuid_registered)
    {
        uint8_t uuid_type_nordic;
        ble_uuid128_t base_uuid_nordic = { .uuid128 = CUSTOM_SERVICE_UUID_BASE };
        ret_code_t err_code = sd_ble_uuid_vs_add(&base_uuid_nordic, &uuid_type_nordic);
        APP_ERROR_CHECK(err_code);
        m_nordic_adv_uuids[0].type = uuid_type_nordic;
        nordic_uuid_registered = true;
        NRF_LOG_INFO("Nordic Base UUID registered. Type: %d", uuid_type_nordic);
    }

    // MTM EEG sensör UUID'sini ekle
    if (!eeg_uuid_registered)
    {
        uint8_t uuid_type_eeg;
        ble_uuid128_t base_uuid_eeg = { .uuid128 = EEG_SERVICE_UUID_BASE };
        ret_code_t err_code = sd_ble_uuid_vs_add(&base_uuid_eeg, &uuid_type_eeg);
        APP_ERROR_CHECK(err_code);
        m_eeg_service_uuids[0].type = uuid_type_eeg;
        eeg_uuid_registered = true;
        NRF_LOG_INFO("MTM EEG Base UUID registered. Type: %d", uuid_type_eeg);
    }
}

/**
 * @brief Nordic SoftDevice'i (BLE yığınını) başlatır ve konfigüre eder.
 *
 * İşleyiş:
 * 1. `nrf_sdh_enable_request()` ile SoftDevice etkinleştirilir.
 *    - SoftDevice, Nordic çipte BLE ve diğer sistem fonksiyonlarını yöneten
 *      düşük seviyeli yazılımdır.
 * 2. `nrf_sdh_ble_default_cfg_set()` çağrısı ile BLE için gerekli RAM miktarı
 *    ve diğer temel konfigürasyonlar ayarlanır.
 * 3. `nrf_sdh_ble_enable()` ile BLE fonksiyonları başlatılır.
 * 4. `NRF_SDH_BLE_OBSERVER` makrosu ile yazdığımız `ble_evt_handler` fonksiyonu
 *    BLE olaylarını gözlemleyecek şekilde kaydedilir.
 *    - Bu sayede bağlantı, veri gelmesi, anons paketleri gibi tüm BLE olayları
 *      otomatik olarak bizim handler fonksiyonumuza yönlendirilir.
 *
 * @note Bu fonksiyon uygulama başlatıldığında çağrılmalıdır.
 * @note `APP_BLE_CONN_CFG_TAG` ve `APP_BLE_OBSERVER_PRIO` projede tanımlanmış
 *       önceden belirlenmiş değerlerdir.
 */
static void ble_stack_init(void)
{
    // SoftDevice etkinleştiriliyor
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // BLE için gerekli RAM başlangıç adresi
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // BLE yığını etkinleştiriliyor
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // BLE olaylarını gözlemek için observer kaydı
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
/**
 * @brief Belirtilen CCCD handle için HVX (Notification/Indication) özelliğini etkinleştirir.
 *
 * @details
 * - BLE cihazındaki karakteristiğin CCCD'sine yazma yaparak
 *   client-side notification veya indication başlatır.
 * - CCCD handle geçersizse işlem yapılmaz ve uyarı loglanır.
 * - `hvx_type` ile notification (BLE_GATT_HVX_NOTIFICATION) veya 
 *   indication (BLE_GATT_HVX_INDICATION) seçilir.
 * - Yazma işlemi doğrudan değil, GATT Queue üzerinden güvenli bir şekilde sıraya alınır.
 *
 * @param[in] conn_handle  Bağlantı handle’ı.
 * @param[in] cccd_handle  Etkinleştirilecek CCCD handle’ı.
 * @param[in] hvx_type     HVX türü (notification veya indication).
 *
 * @note Bu fonksiyon, bir characteristic için client-side HVX başlatmak için kullanılır.
 * @warning CCCD handle geçersiz ise fonksiyon hiçbir işlem yapmaz.
 */
void enable_hvx_for_handle(uint16_t conn_handle, uint16_t cccd_handle, uint8_t hvx_type)
{
    if (cccd_handle == BLE_GATT_HANDLE_INVALID) {
        NRF_LOG_WARNING("CCCD handle is invalid, cannot enable HVX for conn_handle %d", conn_handle);
        return;
    }

    uint8_t cccd_value[2] = {hvx_type, 0x00}; // CCCD değerini ayarla

    ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = 0,
        .handle   = cccd_handle,
        .offset   = 0,
        .len      = sizeof(cccd_value),
        .p_value  = cccd_value
    };

    // GATT kuyruğuna yazma isteğini ekle
    nrf_ble_gq_req_t req = {
        .type          = NRF_BLE_GQ_REQ_GATTC_WRITE,
        .error_handler = NULL,
        .params.gattc_write = write_params
    };

    ret_code_t err_code = nrf_ble_gq_item_add(&m_ble_gatt_queue, &req, conn_handle);
    APP_ERROR_CHECK(err_code);

    if (hvx_type == BLE_GATT_HVX_NOTIFICATION) {
        NRF_LOG_INFO("Notification scheduled for handle: 0x%X on conn_handle %d", cccd_handle, conn_handle);
    } 
    else {
        NRF_LOG_INFO("Indication scheduled for handle: 0x%X on conn_handle %d", cccd_handle, conn_handle);
    }
}


/*
 * @brief Güç yönetimi kütüphanesini başlatır.
 *
 * Bu fonksiyon, `nrf_pwr_mgmt_run()` fonksiyonunun kullanılabilmesi için gerekli
 * altyapıyı kurar. `nrf_pwr_mgmt_run()`, ana döngüde çağrılarak, işlemciyi
 * bir sonraki olaya kadar uyku moduna sokar ve pil ömrünü önemli ölçüde uzatır.
 */
static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Uygulamanın ana döngüsünde (while(1)) çağrılan güç yönetimi ve arka plan işlemcisi.
 *
 * Bu fonksiyonun iki temel görevi vardır:
 * 1. NRF_LOG_PROCESS(): Arka planda işlenmeyi bekleyen log mesajları varsa bunları işler (örn: UART'a gönderir).
 * 2. nrf_pwr_mgmt_run(): Eğer yapılacak bir iş (loglama gibi) yoksa, bir sonraki olaya (interrupt)
 *    kadar işlemciyi uyku moduna alarak güç tüketimini minimuma indirir.
 */

static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**
 * @brief UART olaylarını işleyen callback fonksiyonu.
 *
 * @details
 * - Bu fonksiyon `app_uart` kütüphanesi tarafından çağrılır.
 * - UART iletişim hatası (`APP_UART_COMMUNICATION_ERROR`) veya
 *   FIFO taşma hatası (`APP_UART_FIFO_ERROR`) durumunda,
 *   `APP_ERROR_HANDLER` çağrılarak hata yönetimi yapılır.
 * - Diğer event tipleri bu fonksiyonda işlenmez.
 *
 * @param[in] p_event UART event bilgilerini içeren yapı.
 *
 * @note Bu callback, UART üzerinden veri iletişimi sırasında
 *       kritik hataları yakalamak ve sistemin uygun şekilde
 *       tepki vermesini sağlamak için kullanılır.
 */
static void uart_event_handler(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**
 * @brief Uygulamanın ana giriş noktası.
 */
int main(void)
{
    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    ret_code_t err_code = app_timer_init(); 
    APP_ERROR_CHECK(err_code);

    power_management_init();
    uart_init();
    
    // 4. BLE yığınını başlat
    ble_stack_init();
    
    sd_card_init();
    // 5. Diğer BLE modüllerini başlat
    gatt_init();
    init_custom_uuid();   
    db_discovery_init();
    sd_card_init();
    scan_init();

    // 6. MTM EEG için özel komut zamanlayıcısını oluştur
      err_code = app_timer_create(&m_eeg_manager_timer_id, 
                                APP_TIMER_MODE_REPEATED, // Tekrarlı mod
                                eeg_manager_timer_handler);
    APP_ERROR_CHECK(err_code);

    // 7. EEG cihazları için DURDURMA zamanlayıcısını oluştur. Bu TEK SEFERLİK çalışacak.
    err_code = app_timer_create(&m_eeg_stop_timer_id, 
                                APP_TIMER_MODE_SINGLE_SHOT, 
                                eeg_stop_timer_handler);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Application started.");

    // 8. Taramayı BAŞLAT.
    start_scan();

    err_code = app_timer_start(m_eeg_manager_timer_id, APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(err_code);

    while(1)
    {           
          
        // Log buffer yazma işlemi
        main_loop_process();
        idle_state_handle();
        NRF_LOG_FLUSH();
          // Power management
        nrf_pwr_mgmt_run();
    }  
}
