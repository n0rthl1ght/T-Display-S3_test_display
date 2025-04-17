#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lvgl.h"

// Макросы для удобной работы с минимальным и максимальным значениями
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

// Конфигурация пинов и параметров дисплея T-Display-S3
#define LCD_PIXEL_CLOCK_HZ  (2 * 1000 * 1000) // Частота пиксельного тактирования: 2 МГц
                                              // Влияние: слишком высокая частота (>20 МГц) может вызвать артефакты,
                                              // слишком низкая (<1 МГц) замедлит рендеринг.
#define LCD_BK_LIGHT_ON_LEVEL 1               // Уровень для включения подсветки (1 = включено)
#define LCD_PIN_BK_LIGHT    38                // Пин подсветки
#define LCD_PIN_CS          6                 // Пин Chip Select (CS) для выбора дисплея
#define LCD_PIN_DC          7                 // Пин Data/Command для переключения между данными и командами
#define LCD_PIN_RST         5                 // Пин сброса дисплея
#define LCD_PIN_WR          8                 // Пин записи (WR) для синхронизации данных
#define LCD_PIN_RD          9                 // Пин чтения (не используется, но должен быть настроен)
#define LCD_PIN_DATA0       39                // Пины данных для 8-битной шины i80
#define LCD_PIN_DATA1       40
#define LCD_PIN_DATA2       41
#define LCD_PIN_DATA3       42
#define LCD_PIN_DATA4       45
#define LCD_PIN_DATA5       46
#define LCD_PIN_DATA6       47
#define LCD_PIN_DATA7       48
#define LCD_H_RES           170               // Физическое горизонтальное разрешение дисплея (170 пикселей)
#define LCD_V_RES           320               // Физическое вертикальное разрешение дисплея (320 пикселей)
#define LCD_CMD_BITS        8                 // Количество бит для команд
#define LCD_PARAM_BITS      8                 // Количество бит для параметров
#define LCD_X_OFFSET        0                 // Смещение области отображения по X (логическое)
#define LCD_Y_OFFSET        0                 // Смещение области отображения по Y (логическое)

// Конфигурация буфера LVGL для рендеринга
#define LVGL_BUFFER_LINES   40                // Количество строк в буфере LVGL
                                              // Влияние: меньшее значение (например, 10) снижает потребление памяти,
                                              // но увеличивает количество операций рендеринга, что может замедлить вывод.
                                              // Большое значение (например, 170) увеличивает память, но ускоряет рендеринг.
#define LVGL_BUFFER_SIZE    (LCD_H_RES * LVGL_BUFFER_LINES * sizeof(lv_color_t)) // Размер буфера в байтах

// Перечисление для режимов ориентации дисплея
typedef enum {
    DISPLAY_ORIENTATION_0,   // 0°: физический x=логический x, y=логический y
    DISPLAY_ORIENTATION_90,  // 90°: физический x=логический y, y=логический x
    DISPLAY_ORIENTATION_180, // 180°: физический x=инверсия логического x, y=инверсия логического y
    DISPLAY_ORIENTATION_270  // 270°: физический x=инверсия логического y, y=инверсия логического x
} display_orientation_t;

// Глобальные переменные
static const char *TAG = "example";           // Тег для логирования
static esp_lcd_panel_handle_t panel_handle = NULL; // Дескриптор панели дисплея
static esp_lcd_panel_io_handle_t io_handle = NULL; // Дескриптор интерфейса i80
static lv_disp_t *lvgl_disp = NULL;           // Дескриптор дисплея LVGL
static display_orientation_t current_orientation = DISPLAY_ORIENTATION_90; // Текущая ориентация (по умолчанию 90°)

// Структура для инициализационных команд ST7789
typedef struct {
    uint8_t addr;          // Адрес команды (например, 0x11 для Sleep Out)
    uint8_t param[16];     // Параметры команды (до 16 байт)
    uint8_t len;           // Длина параметров и флаг задержки (бит 7: 1 = задержка 120 мс)
} lcd_cmd_t;

// Список команд инициализации ST7789
// Команда 0x36 (MADCTL) исключена, так как она задаётся в set_display_orientation
static lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},                             // Sleep Out: выход из спящего режима, задержка 120 мс
    {0x21, {0}, 0},                                    // INVON: включение инверсии цветов
                                                        // Влияние: без INVON цвета могут быть инвертированы (например, белый станет чёрным).
    {0x35, {0x00}, 1},                                 // TEON: включение tearing effect для синхронизации
    {0x3A, {0x55}, 1},                                 // Pixel Format: RGB565 (16 бит на пиксель)
                                                        // Влияние: установка 0x66 (RGB666) увеличит размер данных, что не поддерживается шиной i80 в данном коде.
    {0xB2, {0x0C, 0x0C, 0x00, 0x33, 0x33}, 5},        // Porch Setting: настройка временных интервалов
    {0xB7, {0x35}, 1},                                 // Gate Control: управление затвором
    {0xBB, {0x19}, 1},                                 // VCOM Setting: настройка напряжения
    {0xC0, {0x2C}, 1},                                 // LCM Control: управление модулем
    {0xC2, {0x01}, 1},                                 // VDV/VRH Enable: включение VDV/VRH
    {0xC3, {0x12}, 1},                                 // VRH Set: установка VRH
    {0xC4, {0x20}, 1},                                 // VDV Set: установка VDV
    {0xC6, {0x0F}, 1},                                 // Frame Rate Control: частота обновления 60 Гц
                                                        // Влияние: установка 0x05 (120 Гц) может вызвать мерцание на некоторых дисплеях.
    {0xD0, {0xA4, 0xA1}, 2},                           // Power Control: управление питанием
    {0xE0, {0xD0, 0x08, 0x11, 0x08, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34}, 14}, // Positive Gamma
    {0xE1, {0xD0, 0x08, 0x11, 0x08, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34}, 14}, // Negative Gamma
    {0x2A, {0x00, 0x00, 0x01, 0x3F}, 4},              // CASET: задание столбцов 0-319 (320 пикселей по физическому Y)
    {0x2B, {0x00, 0x00, 0x00, 0xA9}, 4},              // RASET: задание строк 0-169 (170 пикселей по физическому X)
                                                        // Влияние: неправильные значения (например, 0x00, 0xA9 для CASET) обрежут изображение.
    {0x29, {0}, 0 | 0x80},                             // Display On: включение дисплея, задержка 120 мс
    {0, {0}, 0xff},                                    // Конец списка команд
};

// Прототип функции clear_screen для устранения ошибок компиляции
static esp_err_t clear_screen(uint16_t color);

/**
 * Устанавливает ориентацию дисплея (0°, 90°, 180°, 270°).
 * Обновляет параметр MADCTL, разрешение LVGL, смещения (x_gap, y_gap) и очищает экран.
 * @param orientation Режим ориентации (DISPLAY_ORIENTATION_0, 90, 180, 270)
 * @return ESP_OK при успехе, иначе код ошибки
 */
static esp_err_t set_display_orientation(display_orientation_t orientation) {
    ESP_LOGI(TAG, "Setting display orientation: %d", orientation);

    // Определение параметров MADCTL, разрешения и смещений
    uint8_t madctl; // Регистр MADCTL управляет ориентацией и порядком сканирования
                    // MY: инверсия строк, MX: инверсия столбцов, MV: обмен осей, BGR: порядок цветов
    int hor_res, ver_res; // Логическое разрешение (зависит от ориентации)
    int x_gap, y_gap;     // Смещения области отображения для компенсации физического расположения пикселей
    switch (orientation) {
        case DISPLAY_ORIENTATION_0:
            madctl = 0x08; // MY=0, MX=0, MV=0, BGR=1
                           // Логический X = физический X, Y = физический Y
            hor_res = LCD_H_RES; // 170 пикселей
            ver_res = LCD_V_RES; // 320 пикселей
            x_gap = 35;          // Смещение по X для выравнивания
            y_gap = 0;           // Без смещения по Y
            break;
        case DISPLAY_ORIENTATION_90:
            madctl = 0x68; // MY=0, MX=1, MV=1, BGR=1
                           // Логический X = физический Y, Y = физический X
            hor_res = LCD_V_RES; // 320 пикселей
            ver_res = LCD_H_RES; // 170 пикселей
            x_gap = 0;           // Без смещения по X
            y_gap = 35;          // Смещение по Y для выравнивания
            break;
        case DISPLAY_ORIENTATION_180:
            madctl = 0xC8; // MY=1, MX=1, MV=0, BGR=1
                           // Логический X = инверсия физического X, Y = инверсия физического Y
            hor_res = LCD_H_RES; // 170 пикселей
            ver_res = LCD_V_RES; // 320 пикселей
            x_gap = 35;          // Смещение по X
            y_gap = 0;           // Без смещения по Y
            break;
        case DISPLAY_ORIENTATION_270:
            madctl = 0xA8; // MY=1, MX=0, MV=1, BGR=1
                           // Логический X = инверсия физического Y, Y = инверсия физического X
            hor_res = LCD_V_RES; // 320 пикселей
            ver_res = LCD_H_RES; // 170 пикселей
            x_gap = 0;           // Без смещения по X
            y_gap = 35;          // Смещение по Y
            break;
        default:
            ESP_LOGE(TAG, "Invalid orientation: %d", orientation);
            return ESP_ERR_INVALID_ARG;
    }

    // Пример влияния: если установить madctl=0x00 (BGR=0), цвета будут в формате RGB, что может
    // привести к неправильному отображению (например, красный станет синим).
    // Неправильные x_gap/y_gap (например, x_gap=0 для 0°) сместят изображение влево или обрежут его.

    // Отправка команды MADCTL для установки ориентации
    esp_err_t ret = esp_lcd_panel_io_tx_param(io_handle, 0x36, &madctl, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MADCTL: %s", esp_err_to_name(ret));
        return ret;
    }

    // Установка смещений x_gap и y_gap
    ret = esp_lcd_panel_set_gap(panel_handle, x_gap, y_gap);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gap (x_gap=%d, y_gap=%d): %s", x_gap, y_gap, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set display gap: x_gap=%d, y_gap=%d", x_gap, y_gap);

    // Обновление текущей ориентации
    current_orientation = orientation;

    // Обновление разрешения в драйвере LVGL
    if (lvgl_disp) {
        lv_disp_drv_t *disp_drv = lvgl_disp->driver;
        disp_drv->hor_res = hor_res;
        disp_drv->ver_res = ver_res;
        lv_disp_set_rotation(lvgl_disp, orientation * 90); // Уведомление LVGL о повороте
                                                           // Влияние: без этого текст LVGL может быть повёрнут неправильно.
        ESP_LOGI(TAG, "Updated LVGL resolution: %dx%d", hor_res, ver_res);
    }

    // Очистка экрана для устранения артефактов от предыдущей ориентации
    ret = clear_screen(0x0000); // Чёрный фон
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear screen after orientation change: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * Устанавливает область рисования на дисплее ST7789.
 * Преобразует логические координаты в физические с учётом текущей ориентации.
 * @param x_start Начальная координата X (логическая)
 * @param x_end Конечная координата X (логическая)
 * @param y_start Начальная координата Y (логическая)
 * @param y_end Конечная координата Y (логическая)
 * @return ESP_OK при успехе, иначе код ошибки
 */
static esp_err_t set_draw_area(int x_start, int x_end, int y_start, int y_end) {
    ESP_LOGD(TAG, "Setting draw area: x=%d-%d, y=%d-%d (before offset, orientation=%d)", 
             x_start, x_end, y_start, y_end, current_orientation);

    // Применение логических смещений (LCD_X_OFFSET, LCD_Y_OFFSET)
    x_start += LCD_X_OFFSET;
    x_end += LCD_X_OFFSET;
    y_start += LCD_Y_OFFSET;
    y_end += LCD_Y_OFFSET;

    // Ограничение координат в зависимости от текущей ориентации
    int max_x = (current_orientation == DISPLAY_ORIENTATION_0 || current_orientation == DISPLAY_ORIENTATION_180) ? LCD_H_RES - 1 : LCD_V_RES - 1;
    int max_y = (current_orientation == DISPLAY_ORIENTATION_0 || current_orientation == DISPLAY_ORIENTATION_180) ? LCD_V_RES - 1 : LCD_H_RES - 1;
    x_start = MAX(x_start, 0);
    x_end = MIN(x_end, max_x);
    y_start = MAX(y_start, 0);
    y_end = MIN(y_end, max_y);

    // Преобразование логических координат в физические
    uint16_t col_start, col_end, row_start, row_end;
    switch (current_orientation) {
        case DISPLAY_ORIENTATION_0:
            col_start = x_start; col_end = x_end; // Физический X
            row_start = y_start; row_end = y_end; // Физический Y
            break;
        case DISPLAY_ORIENTATION_90:
            col_start = y_start; col_end = y_end; // Физический Y
            row_start = x_start; row_end = x_end; // Физический X
            break;
        case DISPLAY_ORIENTATION_180:
            col_start = LCD_H_RES - 1 - x_end; col_end = LCD_H_RES - 1 - x_start; // Инверсия X
            row_start = LCD_V_RES - 1 - y_end; row_end = LCD_V_RES - 1 - y_start; // Инверсия Y
            break;
        case DISPLAY_ORIENTATION_270:
            col_start = LCD_H_RES - 1 - y_end; col_end = LCD_H_RES - 1 - y_start; // Инверсия Y
            row_start = LCD_V_RES - 1 - x_end; row_end = LCD_V_RES - 1 - x_start; // Инверсия X
            break;
        default:
            ESP_LOGE(TAG, "Invalid orientation state: %d", current_orientation);
            return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Physical draw area: cols=%d-%d, rows=%d-%d", col_start, col_end, row_start, row_end);

    // Пример влияния: если не учесть инверсию в 180° или 270°, изображение будет перевёрнуто.
    // Например, установка col_start=x_start для 180° сдвинет изображение в противоположную сторону.

    uint8_t params[4];
    esp_err_t ret;

    // Установка CASET (столбцы, физический Y)
    params[0] = (col_start >> 8) & 0xFF;
    params[1] = col_start & 0xFF;
    params[2] = (col_end >> 8) & 0xFF;
    params[3] = col_end & 0xFF;
    ret = esp_lcd_panel_io_tx_param(io_handle, 0x2A, params, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CASET failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Установка RASET (строки, физический X)
    params[0] = (row_start >> 8) & 0xFF;
    params[1] = row_start & 0xFF;
    params[2] = (row_end >> 8) & 0xFF;
    params[3] = row_end & 0xFF;
    ret = esp_lcd_panel_io_tx_param(io_handle, 0x2B, params, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RASET failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Начало записи в память (RAMWR)
    ret = esp_lcd_panel_io_tx_param(io_handle, 0x2C, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RAMWR failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * Очищает экран, заполняя его указанным цветом в формате RGB565.
 * Учитывает текущую ориентацию для корректной установки области.
 * @param color Цвет в формате RGB565 (0x0000 = чёрный, 0xFFFF = белый)
 * @return ESP_OK при успехе, иначе код ошибки
 */
static esp_err_t clear_screen(uint16_t color) {
    ESP_LOGI(TAG, "Clearing screen with color 0x%04X, free heap: %" PRIu32, color, esp_get_free_heap_size());

    // Определение размеров области в зависимости от ориентации
    int hor_res = (current_orientation == DISPLAY_ORIENTATION_0 || current_orientation == DISPLAY_ORIENTATION_180) ? LCD_H_RES : LCD_V_RES;
    int ver_res = (current_orientation == DISPLAY_ORIENTATION_0 || current_orientation == DISPLAY_ORIENTATION_180) ? LCD_V_RES : LCD_H_RES;

    // Выделение буфера для заливки
    uint16_t *buffer = heap_caps_malloc(hor_res * ver_res * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate clear buffer");
        return ESP_ERR_NO_MEM;
    }

    // Заполнение буфера указанным цветом
    for (int i = 0; i < hor_res * ver_res; i++) {
        buffer[i] = color;
    }

    // Установка области рисования для всего экрана
    esp_err_t ret = set_draw_area(0, hor_res - 1, 0, ver_res - 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set draw area failed: %s", esp_err_to_name(ret));
        free(buffer);
        return ret;
    }

    // Отрисовка буфера на дисплее
    ESP_LOGI(TAG, "Drawing bitmap: x=0-%d, y=0-%d", hor_res - 1, ver_res - 1);
    ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, hor_res, ver_res, buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Draw bitmap failed: %s", esp_err_to_name(ret));
    }

    // Завершение передачи данных
    esp_lcd_panel_io_tx_color(io_handle, -1, NULL, 0);
    free(buffer);

    // Пример влияния: если не вызвать esp_lcd_panel_io_tx_color, данные могут остаться в буфере,
    // что приведёт к частичному обновлению экрана или артефактам.

    return ret;
}

/**
 * Тестирует дисплей, заливая его разными цветами и отображая цветные полосы по краям.
 * Учитывает текущую ориентацию для корректного отображения.
 */
static void test_fill_screen(void) {
    ESP_LOGI(TAG, "Starting color test...");
    esp_err_t ret;

    // Массив цветов для теста заливки
    const struct {
        uint16_t color; // Цвет в формате RGB565
        const char *name; // Название цвета для логов
    } colors[] = {
        {0xF800, "Red"},    // Красный (R=11111, G=000000, B=00000)
        {0x001F, "Blue"},   // Синий (R=00000, G=000000, B=11111)
        {0x07E0, "Green"},  // Зелёный (R=00000, G=111111, B=00000)
        {0x0000, "Black"},  // Чёрный
        {0xFFFF, "White"}   // Белый
    };

    // Заливка экрана каждым цветом с задержкой 2 секунды
    for (size_t i = 0; i < sizeof(colors) / sizeof(colors[0]); i++) {
        ret = clear_screen(colors[i].color);
        ESP_LOGI(TAG, "%s clear (0x%04X) returned: %s", colors[i].name, colors[i].color, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Тест цветных полос по краям экрана
    ESP_LOGI(TAG, "Testing edges with colored strips...");
    int hor_res = (current_orientation == DISPLAY_ORIENTATION_0 || current_orientation == DISPLAY_ORIENTATION_180) ? LCD_H_RES : LCD_V_RES;
    int ver_res = (current_orientation == DISPLAY_ORIENTATION_0 || current_orientation == DISPLAY_ORIENTATION_180) ? LCD_V_RES : LCD_H_RES;

    // Выделение буфера для полос
    uint16_t *buffer = heap_caps_malloc(hor_res * ver_res * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer for edge test");
        return;
    }

    // Заполнение буфера для отображения полос
    for (int y = 0; y < ver_res; y++) {
        for (int x = 0; x < hor_res; x++) {
            int idx = y * hor_res + x;
            buffer[idx] = 0x0000; // Чёрный фон
            if (y < 30) buffer[idx] = 0xF800; // Красная полоса сверху
            else if (y >= ver_res - 30) buffer[idx] = 0x001F; // Синяя полоса снизу
            else if (x < 30) buffer[idx] = 0x07E0; // Зелёная полоса слева
            else if (x >= hor_res - 30) buffer[idx] = 0xFFFF; // Белая полоса справа
        }
    }

    // Установка области рисования и отрисовка полос
    ret = set_draw_area(0, hor_res - 1, 0, ver_res - 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set draw area failed: %s", esp_err_to_name(ret));
        free(buffer);
        return;
    }

    ESP_LOGI(TAG, "Drawing edge test: x=0-%d, y=0-%d", hor_res - 1, ver_res - 1);
    ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, hor_res, ver_res, buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Edge test draw failed: %s", esp_err_to_name(ret));
    }

    esp_lcd_panel_io_tx_color(io_handle, -1, NULL, 0);
    free(buffer);
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Пример влияния: если задать неправильные границы (например, x=0-100 вместо 0-319 для 90°),
    // полосы будут обрезаны, и только часть экрана обновится.
}

/**
 * Callback-функция для рендеринга LVGL на дисплей ST7789.
 * Передаёт пиксельные данные в дисплей с учётом текущей ориентации.
 * @param disp_drv Драйвер дисплея LVGL
 * @param area Область для рендеринга (координаты x1, x2, y1, y2)
 * @param color_p Буфер с данными цвета (RGB565)
 */
static void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    int x_start = area->x1;
    int x_end = area->x2;
    int y_start = area->y1;
    int y_end = area->y2;

    ESP_LOGD(TAG, "LVGL flush: x=%d-%d, y=%d-%d", x_start, x_end, y_start, y_end);

    // Установка области рисования
    esp_err_t ret = set_draw_area(x_start, x_end, y_start, y_end);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LVGL set draw area failed: %s", esp_err_to_name(ret));
        return; // Не вызывать lv_disp_flush_ready при ошибке
    }

    // Отрисовка пиксельных данных
    ret = esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end + 1, y_end + 1, color_p);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LVGL draw bitmap failed: %s", esp_err_to_name(ret));
        return;
    }

    // Завершение передачи данных
    esp_lcd_panel_io_tx_color(io_handle, -1, NULL, 0);

    // Уведомление LVGL о завершении рендеринга
    lv_disp_flush_ready(disp_drv);

    // Пример влияния: если не вызвать lv_disp_flush_ready, LVGL будет считать, что рендеринг не завершён,
    // что приведёт к задержкам или пропуску кадров.
}

/**
 * Инициализирует библиотеку LVGL и регистрирует дисплейный драйвер.
 * Настраивает буферы рендеринга и фон экрана.
 */
static void init_lvgl(void) {
    ESP_LOGI(TAG, "Initializing LVGL...");

    // Инициализация LVGL
    lv_init();

    // Выделение двух буферов для рендеринга
    static lv_color_t lvgl_buf1[LVGL_BUFFER_SIZE / sizeof(lv_color_t)];
    static lv_color_t lvgl_buf2[LVGL_BUFFER_SIZE / sizeof(lv_color_t)];
    static lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, lvgl_buf1, lvgl_buf2, LVGL_BUFFER_SIZE / sizeof(lv_color_t));

    // Пример влияния: использование одного буфера (lvgl_buf2=NULL) может вызвать мерцание,
    // так как LVGL будет рендерить новый кадр, пока старый ещё передаётся на дисплей.

    // Настройка драйвера дисплея LVGL
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_V_RES; // Изначально 320 (будет обновлено в set_display_orientation)
    disp_drv.ver_res = LCD_H_RES; // Изначально 170
    disp_drv.flush_cb = lvgl_flush_cb; // Callback для рендеринга
    disp_drv.draw_buf = &disp_buf;     // Буфер рендеринга
    disp_drv.full_refresh = 0;         // Отключение полного обновления для оптимизации
    lvgl_disp = lv_disp_drv_register(&disp_drv);

    // Установка чёрного фона для активного экрана
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

    ESP_LOGI(TAG, "LVGL initialized, display registered with black background");

    // Пример влияния: установка белого фона (lv_color_white()) сделает текст "Hello World" невидимым,
    // так как он белый по умолчанию.
}

/**
 * Создаёт и настраивает виджет с текстом "Hello World" через LVGL.
 * Позиционирует метку в центре с указанным размером шрифта.
 * @param font_size Размер шрифта (16 или 28 для Montserrat)
 */
static void create_hello_world_label(int font_size) {
    ESP_LOGI(TAG, "Creating Hello World label with font size %d...", font_size);

    // Очистка текущего экрана от предыдущих объектов
    lv_obj_clean(lv_scr_act());
    // Влияние: без lv_obj_clean предыдущие метки останутся на экране, вызывая наложение.

    // Создание метки
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello World");

    // Настройка стиля
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_text_color(&style, lv_color_white()); // Белый текст
    lv_style_set_bg_opa(&style, LV_OPA_COVER);        // Непрозрачный фон
    lv_style_set_bg_color(&style, lv_color_black());  // Чёрный фон

    // Выбор шрифта
    const lv_font_t *font = NULL;
    if (font_size == 16) {
        font = &lv_font_montserrat_16;
        if (font == NULL) {
            font = lv_font_default();
            ESP_LOGW(TAG, "Montserrat 16 not available, using default font");
        }
    } else if (font_size == 28) {
        font = &lv_font_montserrat_28;
        if (font == NULL) {
            font = lv_font_default();
            ESP_LOGW(TAG, "Montserrat 28 not available, using default font");
        }
    } else {
        font = lv_font_default();
        ESP_LOGW(TAG, "Invalid font size %d, using default font", font_size);
    }
    lv_style_set_text_font(&style, font);
    lv_obj_add_style(label, &style, 0);

    // Позиционирование метки в центре
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    ESP_LOGI(TAG, "Hello World label created and styled, font size=%d, position: x=%d, y=%d", 
             font_size, lv_obj_get_x(label), lv_obj_get_y(label));

    // Пример влияния: если установить lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0),
    // текст сместится в верхний левый угол, что может быть нежелательно при смене ориентации.
}

/**
 * Инициализирует дисплей ST7789 с использованием шины i80.
 * Настраивает пины, шину, интерфейс и отправляет команды инициализации.
 */
static void init_display(void) {
    ESP_LOGI(TAG, "Setting up parallel interface...");

    // Конфигурация пина RD (чтение, не используется, но должен быть в высоком состоянии)
    ESP_LOGI(TAG, "Configuring RD pin...");
    gpio_config_t rd_gpio_config = {
        .pin_bit_mask = 1ULL << LCD_PIN_RD,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rd_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level(LCD_PIN_RD, 1));

    // Конфигурация пина подсветки
    ESP_LOGI(TAG, "Configuring backlight...");
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << LCD_PIN_BK_LIGHT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level(LCD_PIN_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL));
    vTaskDelay(pdMS_TO_TICKS(100)); // Задержка для стабилизации подсветки
    ESP_LOGI(TAG, "Backlight set to %d", LCD_BK_LIGHT_ON_LEVEL);

    // Пример влияния: если не включить подсветку (LCD_BK_LIGHT_ON_LEVEL=0),
    // экран останется тёмным, и ничего не будет видно.

    // Инициализация шины i80
    ESP_LOGI(TAG, "Initializing i80 bus...");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT, // Источник тактирования (по умолчанию PLL)
        .dc_gpio_num = LCD_PIN_DC,      // Пин для Data/Command
        .wr_gpio_num = LCD_PIN_WR,      // Пин для записи
        .data_gpio_nums = {
            LCD_PIN_DATA0, LCD_PIN_DATA1, LCD_PIN_DATA2, LCD_PIN_DATA3,
            LCD_PIN_DATA4, LCD_PIN_DATA5, LCD_PIN_DATA6, LCD_PIN_DATA7,
        },
        .bus_width = 8, // 8-битная шина
        .max_transfer_bytes = LCD_H_RES * LCD_V_RES * sizeof(uint16_t), // Максимальный размер передачи
        .psram_trans_align = 64, // Выравнивание для PSRAM
        .sram_trans_align = 4,   // Выравнивание для SRAM
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    // Инициализация интерфейса i80
    ESP_LOGI(TAG, "Initializing i80 interface...");
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = LCD_PIN_CS, // Пин Chip Select
        .pclk_hz = LCD_PIXEL_CLOCK_HZ, // Частота тактирования
        .trans_queue_depth = 10, // Глубина очереди передачи
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,  // Уровень для команд
            .dc_dummy_level = 0,
            .dc_data_level = 1, // Уровень для данных
        },
        .flags = {
            .cs_active_high = 0, // CS активен на низком уровне
            .reverse_color_bits = 0, // Без инверсии порядка бит
            .swap_color_bytes = 1,   // Смена байтов для RGB565 (Big Endian)
            .pclk_active_neg = 0,    // Тактирование на положительном фронте
        },
        .lcd_cmd_bits = LCD_CMD_BITS,   // 8 бит для команд
        .lcd_param_bits = LCD_PARAM_BITS // 8 бит для параметров
    };
    ESP_LOGI(TAG, "i80 config: swap_color_bytes=%d, reverse_color_bits=%d", io_config.flags.swap_color_bytes, io_config.flags.reverse_color_bits);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    // Пример влияния: если установить swap_color_bytes=0, цвета будут искажены
    // (например, красный станет синим из-за неправильного порядка байтов).

    // Инициализация панели ST7789
    ESP_LOGI(TAG, "Initializing ST7789 panel...");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_RST, // Пин сброса
        .color_space = ESP_LCD_COLOR_SPACE_RGB, // Цветовое пространство RGB
        .bits_per_pixel = 16, // 16 бит на пиксель (RGB565)
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // Сброс панели
    ESP_LOGI(TAG, "Resetting panel...");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(100));

    // Отправка инициализационных команд
    ESP_LOGI(TAG, "Sending ST7789 init commands...");
    for (uint8_t i = 0; lcd_st7789v[i].addr != 0 || lcd_st7789v[i].len != 0xff; i++) {
        ESP_LOGI(TAG, "Sending cmd 0x%02X, len=%d", lcd_st7789v[i].addr, lcd_st7789v[i].len & 0x7f);
        esp_err_t ret = esp_lcd_panel_io_tx_param(io_handle, lcd_st7789v[i].addr,
                                                  lcd_st7789v[i].param,
                                                  lcd_st7789v[i].len & 0x7f);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cmd 0x%02X failed: %s", lcd_st7789v[i].addr, esp_err_to_name(ret));
        }
        if (lcd_st7789v[i].len & 0x80) {
            vTaskDelay(pdMS_TO_TICKS(120)); // Задержка для команд, требующих времени
        }
    }

    // Включение дисплея
    ESP_LOGI(TAG, "Configuring panel...");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Установка начальной ориентации
    ESP_LOGI(TAG, "Setting initial orientation");
    esp_err_t ret = set_display_orientation(DISPLAY_ORIENTATION_90);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial orientation: %s", esp_err_to_name(ret));
    }
}

/**
 * Задача для периодического вызова lv_tick_inc().
 * Обеспечивает корректное отсчёты времени для LVGL.
 */
static void lvgl_tick_task(void *arg) {
    while (1) {
        lv_tick_inc(10); // Увеличение счётчика времени LVGL на 10 мс
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Главная функция приложения.
 * Инициализирует дисплей, LVGL, выводит текст и тестирует ориентации.
 */
void app_main(void) {
    ESP_LOGI(TAG, "Starting application...");
    ESP_LOGI(TAG, "Stack watermark: %u", uxTaskGetStackHighWaterMark(NULL));

    // Инициализация дисплея
    init_display();

    // Инициализация LVGL
    init_lvgl();

    // Запуск задачи для LVGL tick
    xTaskCreate(lvgl_tick_task, "lvgl_tick", 2048, NULL, 2, NULL);

    // Очистка экрана перед рендерингом LVGL
    ESP_LOGI(TAG, "Clearing screen before LVGL rendering...");
    esp_err_t ret = clear_screen(0x0000);
    ESP_LOGI(TAG, "Pre-LVGL clear returned: %s", esp_err_to_name(ret));

    // Создание начальной метки "Hello World" с шрифтом 28
    create_hello_world_label(28);
    for (int i = 0; i < 500; i++) { // 5 секунд (500 * 10 мс)
        lv_task_handler(); // Обработка и рендеринг LVGL
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Тест смены ориентаций
    display_orientation_t orientations[] = {
        DISPLAY_ORIENTATION_0, DISPLAY_ORIENTATION_90, DISPLAY_ORIENTATION_180, DISPLAY_ORIENTATION_270
    };
    for (int i = 0; i < 4; i++) {
        // Установка ориентации
        ret = set_display_orientation(orientations[i]);
        ESP_LOGI(TAG, "Set orientation %d returned: %s", orientations[i], esp_err_to_name(ret));

        // Выполнение теста заливки и полос
        test_fill_screen();

        // Очистка экрана перед созданием метки
        ret = clear_screen(0x0000);
        ESP_LOGI(TAG, "Cleared screen before label, returned: %s", esp_err_to_name(ret));

        // Создание метки "Hello World" с шрифтом 16
        create_hello_world_label(16);

        // Цикл для рендеринга метки в течение 5 секунд
        for (int j = 0; j < 500; j++) { // 5 секунд (500 * 10 мс)
            lv_task_handler();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    ESP_LOGI(TAG, "Entering main loop");
    while (1) {
        // Обновление LVGL
        lv_task_handler();
        ESP_LOGD(TAG, "LVGL task handler called, free heap: %" PRIu32, esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Пример влияния: если не вызывать lv_task_handler в цикле ориентаций,
    // метки не будут рендериться, и текст "Hello World" не появится.
}