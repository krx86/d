#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Arduino.h>
#include <SPI.h>

// LVGL konfigurācija
#define LV_CONF_INCLUDE_SIMPLE
#define LV_USE_MEM_MONITOR 0
#define LV_DISP_DEF_REFR_PERIOD 30
#define LV_INDEV_DEF_READ_PERIOD 30

// Pieskāriena konfigurācija
#define TOUCH_IRQ 7
#define CAL_X_MIN 200
#define CAL_X_MAX 3800
#define CAL_Y_MIN 200
#define CAL_Y_MAX 3800
#define TOUCH_THRESHOLD 100


// Bufera konfigurācija
#define BUF_HEIGHT (TFT_HEIGHT/10)
#define BUF_SIZE (TFT_WIDTH * BUF_HEIGHT)

TFT_eSPI tft = TFT_eSPI();
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[BUF_SIZE];
static lv_color_t buf2[BUF_SIZE];

// Pieskāriena dati
uint16_t last_x = 0, last_y = 0;
bool last_touched = false;

// Filtrēšanas konfigurācija
#define FILTER_SAMPLES 5
uint16_t x_history[FILTER_SAMPLES] = {0};
uint16_t y_history[FILTER_SAMPLES] = {0};
uint8_t history_index = 0;

// Displeja atjaunošanas funkcija
void mans_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors(&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

// Pārbauda, vai pieskārieni ir derīgi
bool ir_derigs_pieskariens(uint16_t x, uint16_t y) {
    return !(y > (CAL_Y_MAX + TOUCH_THRESHOLD) || (x < TOUCH_THRESHOLD));
}

// Pieskāriena lasīšanas funkcija
void mans_pieskariens_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    uint16_t touchX, touchY;
    bool touched = tft.getTouchRaw(&touchX, &touchY);
    
    last_touched = touched && ir_derigs_pieskariens(touchX, touchY);
    
    if (!last_touched) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }
    
    last_x = constrain(map(touchX, CAL_X_MIN, CAL_X_MAX, TFT_WIDTH, 0), 0, TFT_WIDTH - 1);
    last_y = constrain(map(touchY, CAL_Y_MIN, CAL_Y_MAX, TFT_HEIGHT, 0), 0, TFT_HEIGHT - 1);
    
    x_history[history_index] = last_x;
    y_history[history_index] = last_y;
    history_index = (history_index + 1) % FILTER_SAMPLES;
    
    data->point.x = last_y;
    data->point.y = last_x;
    data->state = LV_INDEV_STATE_PR;
}

// Filtra funkcija
uint16_t pielietot_filteri(uint16_t *history) {
    uint16_t min = history[0], max = history[0];
    uint32_t sum = history[0];
    
    for (uint8_t i = 1; i < FILTER_SAMPLES; i++) {
        if (history[i] < min) min = history[i];
        if (history[i] > max) max = history[i];
        sum += history[i];
    }
    
    return (sum - min - max) / (FILTER_SAMPLES - 2);
}

// Funkcija, kas atjauno pieskāriena datus
void atjauninat_pieskariena_display() {
    static uint16_t prev_x = 0, prev_y = 0;
    
    if (last_touched) {
        uint16_t filtered_x = pielietot_filteri(x_history);
        uint16_t filtered_y = pielietot_filteri(y_history);
        
        if (abs(filtered_x - prev_x) > 1 || abs(filtered_y - prev_y) > 1) {
            Serial.printf("Touch at: X: %d, Y: %d\n", filtered_y, filtered_x);
            prev_x = filtered_x;
            prev_y = filtered_y;
        }
    }
}


// Globālie stili
static lv_style_t style_indic;

// Mainīgie


lv_obj_t *blue_bar;    // Zilais stabiņš (0-25°C)
lv_obj_t *red_bar;     // Sarkanais stabiņš (25-40°C)
bool red_active = false;
int skaititajs = 65;  


// Atjaunina stabiņu stāvokli
void update_bars(int temp) {
    // Vienmēr atjaunina zilo stabiņu
    lv_bar_set_value(blue_bar, (temp > 30) ? 30 : temp, LV_ANIM_OFF);
    
    if(temp > 30) {
        // Parāda sarkano stabiņu un atjaunina tā vērtību
        lv_obj_clear_flag(red_bar, LV_OBJ_FLAG_HIDDEN);
        lv_bar_set_value(red_bar, temp, LV_ANIM_OFF);
    } else {
        // Paslēpj sarkano stabiņu
        lv_obj_add_flag(red_bar, LV_OBJ_FLAG_HIDDEN);
    }
}



void setup() {
    Serial.begin(115200);
    Serial.println("Sistēmas inicializācija...");
    
    tft.init();
    tft.setRotation(2);
    tft.fillScreen(TFT_WHITE);
    
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, BUF_SIZE);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = tft.width();
    disp_drv.ver_res = tft.height();
    disp_drv.flush_cb = mans_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = mans_pieskariens_read;
    lv_indev_drv_register(&indev_drv);

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xF3F4F3), 0);

    // Taisnstūris ar apmali
    lv_obj_t * rect = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rect, 300, 400);
    lv_obj_align(rect, LV_ALIGN_BOTTOM_MID, 0, -15);
    lv_obj_set_style_radius(rect, 20, 0);
    lv_obj_set_style_bg_color(rect, lv_color_hex(0xfcfdfd), 0);
    lv_obj_set_style_border_width(rect, 2, 0);

// Izveidojam baru
blue_bar = lv_bar_create(lv_scr_act());
lv_obj_set_size(blue_bar, 80, 300);
lv_obj_set_pos(blue_bar, 40, 100);
lv_bar_set_range(blue_bar, 5, 80);

// Fona krāsa un noapalotie augšējie stūri (10px rādiuss)
lv_obj_set_style_radius(blue_bar, 40, LV_PART_MAIN); // Noapalot augšējos stūrus
lv_obj_set_style_clip_corner(blue_bar, true, LV_PART_MAIN); // Ieslēdz noapalošanu

// Indikatora krāsa (zila) un NO noapalotie stūri (0 rādiuss)
lv_obj_set_style_bg_color(blue_bar, lv_color_hex(0x7DD0F2), LV_PART_INDICATOR);
lv_obj_set_style_radius(blue_bar, 0, LV_PART_INDICATOR); // Taisns indikators
    
    // Sarkanais stabiņš (25-40°C) ar gradientu
    red_bar = lv_bar_create(lv_scr_act());
    lv_obj_set_size(red_bar, 80, 175);
    lv_obj_align(red_bar, LV_ALIGN_LEFT_MID, 40, -20);
    lv_bar_set_range(red_bar, 30, 85);
    
    // Gradienta iestatījumi
    lv_obj_set_style_bg_opa(red_bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(red_bar, lv_color_hex(0xFFC0C0), LV_PART_INDICATOR);       // Apakšējā krāsa (zila)
    lv_obj_set_style_bg_grad_color(red_bar, lv_color_hex(0x7DD0F2), LV_PART_INDICATOR);  // Augšējā krāsa (sarkana)
    lv_obj_set_style_bg_grad_dir(red_bar, LV_GRAD_DIR_VER, LV_PART_INDICATOR);          // Vertikālais gradients
    
    lv_obj_set_style_radius(red_bar, 5, LV_PART_INDICATOR);
    lv_obj_set_style_radius(red_bar, 5, LV_PART_MAIN);

    lv_obj_set_style_bg_opa(red_bar, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(red_bar, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_add_flag(red_bar, LV_OBJ_FLAG_HIDDEN);
    

  

    // Lielais aplis
    lv_obj_t * circle = lv_obj_create(lv_scr_act());
    lv_obj_set_size(circle, 120, 120);
    lv_obj_set_pos(circle, 20, 480 - 150);
    lv_obj_set_style_radius(circle, 60, 0);
    lv_obj_set_style_bg_color(circle, lv_color_hex(0x7DD0F2), 0);
    lv_obj_set_style_border_width(circle, 0, 0); // Noņemam apmali

    Serial.println("Sistēma gatava lietošanai!");
}

void loop() {
    lv_timer_handler();
    
    static uint32_t last_update = 0;
    if (millis() - last_update >= 100) {
        atjauninat_pieskariena_display();
        last_update = millis();
    }
    

    
    // Atjaunina stabiņu ar skaitītāja vērtību
    update_bars(skaititajs);
    
    lv_tick_inc(5);
    delay(5);
}
