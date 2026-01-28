#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "lvgl.h"

// ---------------------------------------------------
// HARDWARE PINS
// ---------------------------------------------------
#define LCD_HOST       SPI2_HOST
#define PIN_NUM_MOSI   23
#define PIN_NUM_CLK    18
#define PIN_NUM_CS     5
#define PIN_NUM_DC     4
#define PIN_NUM_RST    22
#define PIN_NUM_BL     2
#define LCD_H_RES      240
#define LCD_V_RES      320

// ---------------------------------------------------
// GLOBAL STYLES & OBJECTS
// ---------------------------------------------------
static lv_disp_drv_t disp_drv; 

static lv_style_t style_screen;
static lv_style_t style_card;
static lv_style_t style_title;
static lv_style_t style_value_big;
static lv_style_t style_btn_circle;
static lv_style_t style_scada_panel;

// ---------------------------------------------------
// HARDWARE DRIVERS
// ---------------------------------------------------
void init_backlight_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_NUM_BL, .duty = 1023, .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
}

static void increase_lvgl_tick(void *arg) { lv_tick_inc(2); }

// ---------------------------------------------------
// STYLES
// ---------------------------------------------------
void init_styles() {
    // 1. Screen Base (Soft White)
    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, lv_color_hex(0xF5F7FA)); 
    lv_style_set_bg_opa(&style_screen, LV_OPA_COVER);
    lv_style_set_text_font(&style_screen, &lv_font_montserrat_14);
    lv_style_set_text_color(&style_screen, lv_color_hex(0x2D3436)); 

    // 2. White Card with Shadow
    lv_style_init(&style_card);
    lv_style_set_bg_color(&style_card, lv_color_white());
    lv_style_set_radius(&style_card, 15);
    lv_style_set_shadow_width(&style_card, 15); 
    lv_style_set_shadow_color(&style_card, lv_color_hex(0xD0D0D0));
    lv_style_set_shadow_opa(&style_card, LV_OPA_40);
    lv_style_set_shadow_ofs_y(&style_card, 4);

    // 3. Titles
    lv_style_init(&style_title);
    lv_style_set_text_font(&style_title, &lv_font_montserrat_20);
    lv_style_set_text_color(&style_title, lv_color_hex(0x000000));

    // 4. Big Values
    lv_style_init(&style_value_big);
    lv_style_set_text_font(&style_value_big, &lv_font_montserrat_28);

    // 5. Music Buttons (Circle)
    lv_style_init(&style_btn_circle);
    lv_style_set_radius(&style_btn_circle, LV_RADIUS_CIRCLE);
    lv_style_set_bg_color(&style_btn_circle, lv_color_white());
    lv_style_set_shadow_width(&style_btn_circle, 10);
    lv_style_set_shadow_color(&style_btn_circle, lv_color_hex(0xCCCCCC));
    lv_style_set_text_color(&style_btn_circle, lv_color_black());
}

// ---------------------------------------------------
// HELPER: Parameter Box
// ---------------------------------------------------
void create_param_box(lv_obj_t *parent, const char *lbl, const char *val, int x, int y) {
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_set_size(box, 60, 45);
    lv_obj_set_style_bg_color(box, lv_color_hex(0xF0F0F0), 0); 
    lv_obj_align(box, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_set_style_border_width(box, 0, 0);
    
    lv_obj_t *l = lv_label_create(box);
    lv_label_set_text(l, lbl);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0x888888), 0);
    lv_obj_align(l, LV_ALIGN_TOP_MID, 0, 5);

    lv_obj_t *v = lv_label_create(box);
    lv_label_set_text(v, val);
    lv_obj_align(v, LV_ALIGN_BOTTOM_MID, 0, -5);
}

// ---------------------------------------------------
// SCREEN 1: WEATHER (LIGHT)
// ---------------------------------------------------
void demo_weather(int tick) {
    static lv_obj_t *card = NULL;
    if (card == NULL) {
        lv_obj_clean(lv_scr_act());
        lv_obj_add_style(lv_scr_act(), &style_screen, 0);

        card = lv_obj_create(lv_scr_act());
        lv_obj_add_style(card, &style_card, 0);
        lv_obj_set_size(card, 220, 300);
        lv_obj_center(card);

        lv_obj_t *city = lv_label_create(card);
        lv_label_set_text(city, "Weather");
        lv_obj_add_style(city, &style_title, 0);
        lv_obj_align(city, LV_ALIGN_TOP_MID, 0, 0);

        lv_obj_t *status = lv_label_create(card);
        lv_label_set_text(status, "Sunny");
        lv_obj_set_style_text_color(status, lv_palette_main(LV_PALETTE_ORANGE), 0);
        lv_obj_align(status, LV_ALIGN_TOP_MID, 0, 25);

        // Sun Icon
        lv_obj_t *icon = lv_obj_create(card);
        lv_obj_set_size(icon, 50, 50);
        lv_obj_set_style_radius(icon, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(icon, lv_palette_main(LV_PALETTE_YELLOW), 0);
        lv_obj_align(icon, LV_ALIGN_TOP_LEFT, 20, 60);

        lv_obj_t *temp = lv_label_create(card);
        lv_label_set_text(temp, "28°C");
        lv_obj_add_style(temp, &style_value_big, 0);
        lv_obj_align(temp, LV_ALIGN_TOP_RIGHT, -20, 70);

        create_param_box(card, "Hum", "40%", 0, 130);
        create_param_box(card, "Wind", "8km", 65, 130);
        create_param_box(card, "UV", "High", 130, 130);
        create_param_box(card, "Pres", "1015", 0, 180);
        create_param_box(card, "Vis", "15km", 65, 180);
        create_param_box(card, "Sun", "06:15", 130, 180);
    }
    if (tick == -1) card = NULL;
}

// ---------------------------------------------------
// SCREEN 2: MUSIC (WITH ICONS)
// ---------------------------------------------------
void demo_music(int tick) {
    static lv_obj_t *bar = NULL;
    if (bar == NULL) {
        lv_obj_clean(lv_scr_act());
        lv_obj_add_style(lv_scr_act(), &style_screen, 0);

        lv_obj_t *art = lv_obj_create(lv_scr_act());
        lv_obj_set_size(art, 140, 140);
        lv_obj_align(art, LV_ALIGN_TOP_MID, 0, 20);
        lv_obj_set_style_bg_color(art, lv_color_hex(0xEEEEEE), 0); 
        lv_obj_set_style_radius(art, 8, 0);
        
        lv_obj_t *note = lv_label_create(art);
        lv_label_set_text(note, LV_SYMBOL_AUDIO); 
        lv_obj_add_style(note, &style_value_big, 0);
        lv_obj_set_style_text_color(note, lv_color_hex(0x555555), 0);
        lv_obj_center(note);

        lv_obj_t *t = lv_label_create(lv_scr_act());
        lv_label_set_text(t, "Blinding Lights");
        lv_obj_add_style(t, &style_title, 0);
        lv_obj_align(t, LV_ALIGN_CENTER, 0, 20);
        
        lv_obj_t *a = lv_label_create(lv_scr_act());
        lv_label_set_text(a, "The Weeknd");
        lv_obj_align(t, LV_ALIGN_CENTER, 0, 45);

        // Controls
        lv_obj_t *btn_play = lv_btn_create(lv_scr_act());
        lv_obj_set_size(btn_play, 50, 50);
        lv_obj_add_style(btn_play, &style_btn_circle, 0);
        lv_obj_set_style_bg_color(btn_play, lv_palette_main(LV_PALETTE_BLUE), 0); 
        lv_obj_align(btn_play, LV_ALIGN_BOTTOM_MID, 0, -50);
        
        lv_obj_t *lbl_play = lv_label_create(btn_play);
        lv_label_set_text(lbl_play, LV_SYMBOL_PLAY);
        lv_obj_set_style_text_color(lbl_play, lv_color_white(), 0);
        lv_obj_center(lbl_play);

        lv_obj_t *btn_prev = lv_btn_create(lv_scr_act());
        lv_obj_set_size(btn_prev, 40, 40);
        lv_obj_add_style(btn_prev, &style_btn_circle, 0);
        lv_obj_align(btn_prev, LV_ALIGN_BOTTOM_MID, -60, -55);
        lv_obj_t *lbl_prev = lv_label_create(btn_prev);
        lv_label_set_text(lbl_prev, LV_SYMBOL_PREV);
        lv_obj_center(lbl_prev);

        lv_obj_t *btn_next = lv_btn_create(lv_scr_act());
        lv_obj_set_size(btn_next, 40, 40);
        lv_obj_add_style(btn_next, &style_btn_circle, 0);
        lv_obj_align(btn_next, LV_ALIGN_BOTTOM_MID, 60, -55);
        lv_obj_t *lbl_next = lv_label_create(btn_next);
        lv_label_set_text(lbl_next, LV_SYMBOL_NEXT);
        lv_obj_center(lbl_next);

        bar = lv_slider_create(lv_scr_act());
        lv_obj_set_size(bar, 200, 6);
        lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -25);
        lv_obj_set_style_bg_color(bar, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
    }
    
    if (bar) {
        int progress = (tick % 100);
        lv_slider_set_value(bar, progress, LV_ANIM_ON);
    }
    if (tick == -1) bar = NULL;
}

// ---------------------------------------------------
// SCREEN 3: INDUSTRIAL (LIGHT MODE)
// ---------------------------------------------------
void demo_scada(int tick) {
    static lv_obj_t *gauge = NULL;
    static lv_meter_indicator_t *needle = NULL;
    static lv_obj_t *chart = NULL;
    static lv_chart_series_t *ser = NULL;

    if (gauge == NULL) {
        lv_obj_clean(lv_scr_act());
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xE0E0E0), 0);

        // Header
        lv_obj_t *top = lv_obj_create(lv_scr_act());
        lv_obj_set_size(top, 240, 45);
        lv_obj_align(top, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_set_style_bg_color(top, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_radius(top, 0, 0);
        
        lv_obj_t *logo = lv_label_create(top);
        lv_label_set_text(logo, "AIOTA 360 | SYSTEM");
        lv_obj_set_style_text_color(logo, lv_color_black(), 0);
        lv_obj_align(logo, LV_ALIGN_LEFT_MID, 5, 0);

        // Status LED (Green)
        lv_obj_t *led = lv_obj_create(top);
        lv_obj_set_size(led, 15, 15);
        lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(led, lv_palette_main(LV_PALETTE_GREEN), 0);
        lv_obj_align(led, LV_ALIGN_RIGHT_MID, -10, 0);

        // Light Gauge
        gauge = lv_meter_create(lv_scr_act());
        lv_obj_set_size(gauge, 150, 150);
        lv_obj_align(gauge, LV_ALIGN_TOP_MID, 0, 55);
        lv_obj_set_style_bg_color(gauge, lv_color_white(), 0);
        
        lv_meter_scale_t *s = lv_meter_add_scale(gauge);
        lv_meter_set_scale_range(gauge, s, 0, 3000, 270, 135);
        lv_meter_set_scale_ticks(gauge, s, 11, 2, 10, lv_color_black());
        
        lv_meter_add_arc(gauge, s, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
        needle = lv_meter_add_needle_line(gauge, s, 4, lv_palette_main(LV_PALETTE_RED), -10);

        // Boxes
        lv_obj_t *b1 = lv_obj_create(lv_scr_act());
        lv_obj_set_size(b1, 80, 50);
        lv_obj_set_style_bg_color(b1, lv_color_white(), 0);
        lv_obj_align(b1, LV_ALIGN_LEFT_MID, 10, 40);
        lv_obj_t *l1 = lv_label_create(b1);
        lv_label_set_text(l1, "415V");
        lv_obj_add_style(l1, &style_title, 0);
        lv_obj_center(l1);

        lv_obj_t *b2 = lv_obj_create(lv_scr_act());
        lv_obj_set_size(b2, 80, 50);
        lv_obj_set_style_bg_color(b2, lv_color_white(), 0);
        lv_obj_align(b2, LV_ALIGN_RIGHT_MID, -10, 40);
        lv_obj_t *l2 = lv_label_create(b2);
        lv_label_set_text(l2, "32A");
        lv_obj_add_style(l2, &style_title, 0);
        lv_obj_center(l2);

        // Chart
        chart = lv_chart_create(lv_scr_act());
        lv_obj_set_size(chart, 220, 60);
        lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, -10);
        lv_obj_set_style_bg_color(chart, lv_color_white(), 0);
        lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
        ser = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    }

    if (gauge && chart && needle) {
        int val = 1500 + (tick % 500);
        lv_meter_set_indicator_value(gauge, needle, val);
        lv_chart_set_next_value(chart, ser, lv_rand(40, 60));
    }
    if (tick == -1) gauge = NULL;
}

// ---------------------------------------------------
// SCREEN 4: CLOUD SYNC
// ---------------------------------------------------
void demo_iot(int tick) {
    static lv_obj_t *list = NULL;
    if (list == NULL) {
        lv_obj_clean(lv_scr_act());
        lv_obj_add_style(lv_scr_act(), &style_screen, 0);

        lv_obj_t *h = lv_label_create(lv_scr_act());
        lv_label_set_text(h, "Cloud Sync");
        lv_obj_add_style(h, &style_title, 0);
        lv_obj_align(h, LV_ALIGN_TOP_LEFT, 20, 20);

        lv_obj_t *st = lv_label_create(lv_scr_act());
        lv_label_set_text(st, "AWS IoT: " LV_SYMBOL_OK " Online");
        lv_obj_set_style_text_color(st, lv_palette_main(LV_PALETTE_GREEN), 0);
        lv_obj_align(st, LV_ALIGN_TOP_LEFT, 20, 50);

        list = lv_obj_create(lv_scr_act());
        lv_obj_set_size(list, 220, 210);
        lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, -10);
        lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(list, 0, 0);

        const char *names[] = {"Pressure", "Temp Core", "Vibration", "Flow Rate"};
        const char *vals[] = {"1020 Pa", "64 C", "0.2 G", "12 L/m"};

        for(int i=0; i<4; i++) {
            lv_obj_t *item = lv_obj_create(list);
            lv_obj_set_size(item, 180, 50);
            lv_obj_add_style(item, &style_card, 0);
            
            lv_obj_t *l = lv_label_create(item);
            lv_label_set_text(l, names[i]);
            
            lv_obj_t *v = lv_label_create(item);
            lv_label_set_text(v, vals[i]);
            lv_obj_set_style_text_font(v, &lv_font_montserrat_14, 0);
            lv_obj_align(v, LV_ALIGN_RIGHT_MID, 0, 0);
        }
    }
    if (tick == -1) list = NULL;
}

// ---------------------------------------------------
// SCREEN 5: SMART LIGHT (FIXED TEXT UPDATE)
// ---------------------------------------------------
void demo_light(int tick) {
    static lv_obj_t *bulb = NULL;
    static lv_obj_t *txt_color = NULL;

    if (bulb == NULL) {
        lv_obj_clean(lv_scr_act());
        lv_obj_add_style(lv_scr_act(), &style_screen, 0);

        lv_obj_t *h = lv_label_create(lv_scr_act());
        lv_label_set_text(h, "Studio Light");
        lv_obj_add_style(h, &style_title, 0);
        lv_obj_align(h, LV_ALIGN_TOP_MID, 0, 30);

        bulb = lv_obj_create(lv_scr_act());
        lv_obj_set_size(bulb, 120, 120);
        lv_obj_set_style_radius(bulb, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(bulb, 5, 0);
        lv_obj_set_style_border_color(bulb, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(bulb);
        lv_obj_set_style_shadow_width(bulb, 25, 0);

        txt_color = lv_label_create(lv_scr_act());
        lv_label_set_text(txt_color, "Color: Syncing...");
        lv_obj_align(txt_color, LV_ALIGN_BOTTOM_MID, 0, -50);
    }

    if (bulb) {
        int phase = (tick / 40) % 5;
        lv_color_t c;
        char *name = "";

        switch(phase) {
            case 0: c = lv_palette_main(LV_PALETTE_PURPLE); name = "Purple"; break;
            case 1: c = lv_palette_main(LV_PALETTE_ORANGE); name = "Warm"; break;
            case 2: c = lv_palette_main(LV_PALETTE_BLUE); name = "Cool"; break;
            case 3: c = lv_palette_main(LV_PALETTE_TEAL); name = "Teal"; break;
            case 4: c = lv_palette_main(LV_PALETTE_RED); name = "Red"; break;
            default: c = lv_color_white(); name = "White"; break;
        }
        lv_obj_set_style_bg_color(bulb, c, 0);
        lv_obj_set_style_shadow_color(bulb, c, 0);
        
        // FIX: Update text label
        lv_label_set_text_fmt(txt_color, "Color: %s", name);
    }
    if (tick == -1) bulb = NULL;
}

// ---------------------------------------------------
// SCREEN 6: COLOR SPECTRUM (NEW!)
// ---------------------------------------------------
void demo_colors(int tick) {
    static lv_obj_t *obj = NULL;
    if (obj == NULL) {
        lv_obj_clean(lv_scr_act());
        
        // 3 Vertical Bars
        for(int i=0; i<3; i++) {
             obj = lv_obj_create(lv_scr_act());
             lv_obj_set_size(obj, 80, 320);
             lv_obj_align(obj, LV_ALIGN_LEFT_MID, i*80, 0);
             lv_obj_set_style_border_width(obj, 0, 0);
             lv_obj_set_style_radius(obj, 0, 0);
             
             if(i==0) lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_RED), 0);
             if(i==1) lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_GREEN), 0);
             if(i==2) lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_BLUE), 0);
             
             // Add text
             lv_obj_t *l = lv_label_create(obj);
             if(i==0) lv_label_set_text(l, "R");
             if(i==1) lv_label_set_text(l, "G");
             if(i==2) lv_label_set_text(l, "B");
             lv_obj_set_style_text_font(l, &lv_font_montserrat_28, 0);
             lv_obj_set_style_text_color(l, lv_color_white(), 0);
             lv_obj_center(l);
        }
    }
    // Simple screen, no updates needed
    if (tick == -1) obj = NULL;
}

// ---------------------------------------------------
// MAIN
// ---------------------------------------------------
void app_main(void)
{
    init_backlight_pwm();
    spi_bus_config_t buscfg = { .sclk_io_num = PIN_NUM_CLK, .mosi_io_num = PIN_NUM_MOSI, .miso_io_num = -1, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t) };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = { .dc_gpio_num = PIN_NUM_DC, .cs_gpio_num = PIN_NUM_CS, .pclk_hz = 40 * 1000 * 1000, .lcd_cmd_bits = 8, .lcd_param_bits = 8, .spi_mode = 0, .trans_queue_depth = 10, .on_color_trans_done = notify_lvgl_flush_ready, .user_ctx = &disp_drv };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_HOST, &io_config, &io_handle));
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = { .reset_gpio_num = PIN_NUM_RST, .rgb_endian = LCD_RGB_ENDIAN_RGB, .bits_per_pixel = 16 };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    lv_init();
    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_H_RES * 40);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES; disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb; disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    const esp_timer_create_args_t lvgl_tick_timer_args = { .callback = &increase_lvgl_tick, .name = "lvgl_tick" };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
    esp_timer_start_periodic(lvgl_tick_timer, 2000); 

    init_styles();

    int tick = 0;
    while (1) {
        lv_timer_handler(); 
        vTaskDelay(pdMS_TO_TICKS(20));
        tick++;

        // 6 DEMOS (200 ticks each)
        if (tick < 200) { if(tick==5) demo_weather(0); }
        else if (tick == 200) demo_weather(-1);

        else if (tick < 400) { if(tick==205) demo_music(0); else demo_music(tick); }
        else if (tick == 400) demo_music(-1);

        else if (tick < 600) { if(tick==405) demo_scada(0); else demo_scada(tick); }
        else if (tick == 600) demo_scada(-1);

        else if (tick < 800) { if(tick==605) demo_iot(0); }
        else if (tick == 800) demo_iot(-1);

        else if (tick < 1000) { if(tick==805) demo_light(0); else demo_light(tick); }
        else if (tick == 1000) demo_light(-1);

        else if (tick < 1200) { if(tick==1005) demo_colors(0); }
        else if (tick == 1200) demo_colors(-1);
        
        else tick = 0;
    }
}
