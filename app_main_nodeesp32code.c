#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "esp_timer.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "sdkconfig.h"

#include <ets_sys.h>
#include <esp_idf_lib_helpers.h>
#include "dht.h"

#include "esp_err.h"
#include "driver/i2c.h"

#include "font8x8_basic.h"
#include "ssd1306.h"
//OLED
static const char *TAG_OLED = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
#define ESP_SLAVE_ADDR 0x3C /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

SemaphoreHandle_t print_mux = NULL;

static esp_err_t i2c_master_init(void)
{
	int i2c_master_port = 0;
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = 5,
		.scl_io_num = 4,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 100000
	};
	i2c_param_config(i2c_master_port, &conf);
	return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}
//init OLED
void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(TAG_OLED, "OLED configured successfully");
	} else {
		ESP_LOGE(TAG_OLED, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}
//hien thi OLED 
void task_ssd1306_display_text(const void *arg_text) {
	char *text = (char*)arg_text;
	uint8_t text_len = strlen(text);

	i2c_cmd_handle_t cmd;

	uint8_t cur_page = 0;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00, true); // reset column - choose column --> 0
	i2c_master_write_byte(cmd, 0x10, true); // reset line - choose line --> 0
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	for (uint8_t i = 0; i < text_len; i++) {
		if (text[i] == '\n') {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, 0x00, true); // reset column
			i2c_master_write_byte(cmd, 0x10, true);
			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		} else {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		}
	}

}
//xóa màn hình 
void task_ssd1306_display_clear(void *ignore) {
	i2c_cmd_handle_t cmd;

	uint8_t clear[128];
	for (uint8_t i = 0; i < 128; i++) {
		clear[i] = 0;
	}
	for (uint8_t i = 0; i < 8; i++) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, 0xB0 | i, true);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
		i2c_master_write(cmd, clear, 128, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}
    //vTaskDelete(NULL);
}
//MQTT
static const char *TAG = "MQTTWS_EXAMPLE";

char v[128] = "abcdjfjfjhadadad";
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            float humidity=0.0;
            float temperature=0.0;
            while(1)
            {
                    esp_err_t res = dht_read_float_data(DHT_TYPE_DHT11, GPIO_NUM_4, &humidity, &temperature);
                    if (res == ESP_OK)
                    {
                        char tem[127];
                        char hum[127];
                        char sum[255];
                        char oled[300];
                        printf("Humidity: %.2f%%\n", humidity);
                        printf("Temperature: %.2f°C\n", temperature);
                        sprintf(tem, "%.2f", temperature);
                        sprintf(hum, "%.2f", humidity);
                        memset(sum, 0, sizeof(sum));
                        strcat(sum, tem);
                        strcat(sum, hum);
                        esp_mqtt_client_publish(client, "topic1",sum, 0, 0, 0);
                        memset(oled, 0, sizeof(oled));
                        strcat(oled, "\nTemperature:\n");
                        strcat(oled, tem);
                        strcat(oled, "*C\n");
                        strcat(oled, "\nHumidity:");
                        strcat(oled, hum);
                        strcat(oled, "%");
                        task_ssd1306_display_clear(oled);
                        task_ssd1306_display_text(oled);
                    }
                vTaskDelay(10000 / portTICK_PERIOD_MS);
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            //printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            //printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}
static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .host = "mqtt.flespi.io",
        .port = 1883,
        .username = "moJruoiv8YC1nbgzW2ffeFNi4cnCwNyIZ7Xjj6ChS2834esO7yU91xhVxvnT8JpV",
        .password = "",
        .client_id = "VuongESP"
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}
// DHT11
#define DHT_TIMER_INTERVAL 2 //khoảng thời gian giữa các lần đọc trạng thái của chân GPIO
#define DHT_DATA_BITS 40 //số lượng bit truyền đi
#define DHT_DATA_BYTES (DHT_DATA_BITS / 8)//số lượng bytes truyền đi bằng bit chia 8

/*
 *  Note:
 *  A suitable pull-up resistor should be connected to the selected GPIO line
 *
 *  __           ______          _______                              ___________________________
 *    \    A    /      \   C    /       \   DHT duration_data_low    /                           \
 *     \_______/   B    \______/    D    \__________________________/   DHT duration_data_high    \__
 *
 *
 *  Initializing communications with the DHT requires four 'phases' as follows:
 *
 *  Phase A - MCU pulls signal low for at least 18000 us
 *  Phase B - MCU allows signal to float back up and waits 20-40us for DHT to pull it low
 *  Phase C - DHT pulls signal low for ~80us
 *  Phase D - DHT lets signal float back up for ~80us
 *
 *  After this, the DHT transmits its first bit by holding the signal low for 50us
 *  and then letting it float back high for a period of time that depends on the data bit.
 *  duration_data_high is shorter than 50us for a logic '0' and longer than 50us for logic '1'.
 *
 *  There are a total of 40 data bits transmitted sequentially. These bits are read into a byte array
 *  of length 5.  The first and third bytes are humidity (%) and temperature (C), respectively.  Bytes 2 and 4
 *  are zero-filled and the fifth is a checksum such that:
 *
 *  byte_5 == (byte_1 + byte_2 + byte_3 + byte_4) & 0xFF
 *
 */

static const char *TANG = "dht";

#if HELPER_TARGET_IS_ESP32
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
//các macro dùng để bắt đầu và kết thúc mã bảo vệ vùng nhớ bằng mux
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL(&mux)

#elif HELPER_TARGET_IS_ESP8266
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL()
#endif

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)// macro kiểm tra tham số truyền vào
//macro kiểm tra lỗi 
#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            PORT_EXIT_CRITICAL(); \
            ESP_LOGE(TANG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

//hàm dht_await_pin_state() là hàm xác định thời gian chờ cho đến khi GPIO chuyển sang trạng thái cần thiết hoặc hết thời gian chờ mà không chuyển sang được trạng thái cần thiết sẽ là trả về flase
static esp_err_t dht_await_pin_state(gpio_num_t pin, uint32_t timeout,
       int expected_pin_state, uint32_t *duration)
{
    //thiết lập chân gpio thành input dể đọc dữ liệu 
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    //tạo vòng lặp để kiểm tra trạng thái của chân gpio với thực hiện trong khoảng thời gian là timeout và tăng lên DHT_TIMER_INTERVAL để đảm bảo rằng khoảng thời gian đọc giữa các chân GPIO là đúng.
    for (uint32_t i = 0; i < timeout; i += DHT_TIMER_INTERVAL)
    {
        // cần 1 khoảng thời gian bằng với khoảng thời gian gữa các lần đọc
        ets_delay_us(DHT_TIMER_INTERVAL);
        if (gpio_get_level(pin) == expected_pin_state) // sử dụng gpio_get_level() để kiểm tra trạng thái của chân GPIO có bằng với trạng thái mong muốn hay không 
        {
            // và lúc đó thì biến duration sẽ được gán giá trị là i chính là thời gian đã trôi qua
            if (duration)
                *duration = i;
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
    // hàm trả về ESP_OK nếu như chân GPIO đã chuyển sang trạng thái mong muốn
}

// hàm dht_fetch_data() dùng để gửi yêu cầu đọc dữ liệu từ cảm biến và đọc các bit từ dữ liệu gửi về
static inline esp_err_t dht_fetch_data(dht_sensor_type_t sensor_type, gpio_num_t pin, uint8_t data[DHT_DATA_BYTES])
{
    //khai báo các biến để lưu trữ thời gian đạt được trong quá trình đọc dữ liệu
    uint32_t low_duration;
    uint32_t high_duration;
    //Để đảm bảo việc giao tiếp và đồng bộ dữ liệu được chính xác từ cảm biến thì có 4 giai đoạn
    //Giai đoạn A là để gửi yêu cầu đọc dữ liệu lên cảm biến
    //Để gửi yêu cầu thì chúng ta chuyển chân GPIO về mode OUTPUT mức thấp và delay trong 1 khoảng thời gian trước khi đưa chân GPIO lên lại mức cao 
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 0);
    ets_delay_us(sensor_type == DHT_TYPE_SI7021 ? 500 : 20000);
    gpio_set_level(pin, 1);

    // Giai đoạn B nhằm chủ yếu để đồng bộ hóa dữ liệu khi đọc vì cảm biến sẽ truyền dữ liệu bằng cách lấy từ logic thấp -> cao
    // và cho thời gian chờ là 40micro tạo sự chênh lệch giữa quá trình chuyển trạng thái cao -> thấp giữa GPIO và DHT
    esp_err_t err = dht_await_pin_state(pin, 40, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initialization error, problem in phase 'B'");
        return err;
    }

    // Giai đoạn C là quan trọng vì là khi cảm biến bắt đầu truyền dữ liệu
    // và thời gian chờ là 80micro để chuyển từ thấp -> cao giữa GPIO và DHT
    err = dht_await_pin_state(pin, 88, 1, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initialization error, problem in phase 'C'");
        return err;
    }

    // Giai đoạn D với thời gian chờ là 88micro để chuyển trạng thái từ cao -> thấp để cảm biến truyền dữ liệu chính thức
    err = dht_await_pin_state(pin, 88, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initialization error, problem in phase 'D'");
        return err;
    }

    //đọc lần lượt từng bit dữ liệu (40bits)
    for (int i = 0; i < DHT_DATA_BITS; i++)
    {
        //sử dụng hàm dht_await_pin_state()để chờ trạng thái logic cao và thấp của GPIO trong mỗi bit
        CHECK_LOGE(dht_await_pin_state(pin, 65, 1, &low_duration),
                "LOW bit timeout");
        CHECK_LOGE(dht_await_pin_state(pin, 75, 0, &high_duration),
                "HIGH bit timeout");
        //biến "b" và "m" lần lượt là byte và bit trong mảng data
        uint8_t b = i / 8;
        uint8_t m = i % 8;
        // và nếu như bit đó là 0 thì sẽ gán lại cho vị trí thứ b đó là bit 0
        if (!m)
            data[b] = 0;
        //nếu như mức logic cao > thấp thì bit đó sẽ là 1 và ngược lại bit đó là 0.
        //gán giá trị bit 1 hoặc 0 vào vị trí bit cần được gán (7-m). 
        data[b] |= (high_duration > low_duration) << (7 - m);
    }

    return ESP_OK;
}

// vì dữ liệu trả về từ cảm biến là 40bit với 16 bit đầu là nhiệt độ, 16 bit tiếp theo là độ ẩm,8 bit còn lại không sử dụng
// và với mỗi 16bit đó thì sẽ chia thành 8bit số nguyên và 8 bit số thập phân
// vì thế hàm dht_convert_data() sẽ dùng để chuyển 2 byte (16 bit) thành số nguyên và xử lý bit dấu
static inline int16_t dht_convert_data(dht_sensor_type_t sensor_type, uint8_t msb, uint8_t lsb)
{
    int16_t data;
    //trong trường hợp là DHT11 thì ta chỉ cần lấy msb(byte thứ nhất chứa các bit cao) nhân với 10 vì DHT11 chỉ có thể lấy được phần nguyên
    if (sensor_type == DHT_TYPE_DHT11)
    {
        data = msb * 10;
    }
    else //đối với DHT22 thì khác
    {
        data = msb & 0x7F; // biến data lưu giữ giá trị byte thứ nhất là msb và 7 bit cuối(0x7F)
        data <<= 8;// và data sẽ được dịch trái 8 bit để chừa chỗ cho lsb ở vị trí thấp hơn
        data |= lsb;// lúc này lsb sẽ được gán vào vị trí tương ứng trong data
        if (msb & BIT(7))//kiểm tra xem số đó có phải là số âm hay không
            data = -data;       // chuyển về số dương
    }
    return data;
}
//Hàm này dùng để đọc dữ liệu từ cảm biến và chuyển nó về dạng số nguyên
esp_err_t dht_read_data(dht_sensor_type_t sensor_type, gpio_num_t pin,
        int16_t *humidity, int16_t *temperature)
{
    CHECK_ARG(humidity || temperature);// kiểm tra xem con trỏ có mang giá trị NULL hay không

    uint8_t data[DHT_DATA_BYTES] = { 0 };// khởi tạo mảng với kích thước là 4byte để lưu giá trị từ cảm biến
    //cấu hình cho chân GPIO ở dạng OUTPUT mức cao (push-pull) để đọc dữ liệu từ cảm biến
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 1);
    //sau khi đọc xong thì giá trị sẽ được lưu vào data và kết quả việc đọc sẽ lưu vào result bằng cách dùng hàm dht_fetch_data()
    PORT_ENTER_CRITICAL();
    esp_err_t result = dht_fetch_data(sensor_type, pin, data);
    if (result == ESP_OK)
        PORT_EXIT_CRITICAL();
    //sau khi đọc xong thì sẽ cấu hình lại chân GPIO như cũ
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 1);

    if (result != ESP_OK)
        return result;
    // chúng ta tính sum của nó bằng cách cộng tất cả byte lại và chỉ lưu các giá trị mức thấp bằng cách and với 0xFF
    // và ta sẽ kiểm tra data[4]==sum để xác định dữ liệu có đúng hoàn toàn hay không
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
    {
        ESP_LOGE(TANG, "Checksum failed, invalid data received from sensor");
        return ESP_ERR_INVALID_CRC;
    }
    //cuối cùng ta kiểm tra xem 2 con trỏ, nếu khác NULL thì nó sẽ chuyển về dạng số nguyên với hàm dht_convert_data()
    if (humidity)
        *humidity = dht_convert_data(sensor_type, data[0], data[1]);
    if (temperature)
        *temperature = dht_convert_data(sensor_type, data[2], data[3]);

    ESP_LOGD(TANG, "Sensor data: humidity=%d, temp=%d", *humidity, *temperature);

    return ESP_OK;
}
//đọc dữ liệu từ cảm biến và sẽ chuyển nó về dạng float
esp_err_t dht_read_float_data(dht_sensor_type_t sensor_type, gpio_num_t pin,
        float *humidity, float *temperature)
{
    CHECK_ARG(humidity || temperature);

    int16_t i_humidity, i_temp;

    esp_err_t res = dht_read_data(sensor_type, pin, humidity ? &i_humidity : NULL, temperature ? &i_temp : NULL);
    if (res != ESP_OK)
        return res;

    if (humidity)
        *humidity = i_humidity / 10.0;
    if (temperature)
        *temperature = i_temp / 10.0;

    return ESP_OK;
}



void app_main(void)
{
    //OLED
    ESP_ERROR_CHECK(i2c_master_init());
    ssd1306_init();
    //MQTT
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_WS", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    mqtt_app_start();
}
