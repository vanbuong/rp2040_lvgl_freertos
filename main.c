/*
 * FreeRTOS V202212.01
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/******************************************************************************
 * This project provides two demo applications.  A simple blinky style project,
 * and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting (defined in this file) is used to
 * select between the two.  The simply blinky demo is implemented and described
 * in main_blinky.c.  The more comprehensive test and demo application is
 * implemented and described in main_full.c.
 *
 * This file implements the code that is not demo specific, including the
 * hardware setup and standard FreeRTOS hook functions.
 *
 * ENSURE TO READ THE DOCUMENTATION PAGE FOR THIS PORT AND DEMO APPLICATION ON
 * THE http://www.FreeRTOS.org WEB SITE FOR FULL INFORMATION ON USING THIS DEMO
 * APPLICATION, AND ITS ASSOCIATE FreeRTOS ARCHITECTURE PORT!
 *
 */

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "portable.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"   /* Software timer related API prototypes. */

#include "main.h"
#include "hardware/spi.h"
#include "hardware/watchdog.h"
#include "hardware/rtc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "st7789.h"
#include "rfid.h"
#include "lv_conf.h"
#include "lvgl/lvgl.h"
#include "lwesp/lwesp.h"
#include "station_manager.h"
#include "sntp.h"

/* Library includes. */
#include <stdio.h>
#include "pico/stdlib.h"
/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define mainSOFTWARE_TIMER_PERIOD_MS        pdMS_TO_TICKS(250)
#define mainLVGL_TIMER_PERIOD_MS            pdMS_TO_TICKS(1)

#ifndef MY_DISP_HOR_RES
    #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen width, default value 320 is used for now.
    #define MY_DISP_HOR_RES    240
#endif

#ifndef MY_DISP_VER_RES
    #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen height, default value 240 is used for now.
    #define MY_DISP_VER_RES    135
#endif

/* Set mainCREATE_SIMPLE_BLINKY_DEMO_ONLY to one to run the simple blinky demo,
or 0 to run the more comprehensive test and demo application. */

/*-----------------------------------------------------------*/
static void lv_btnmatrix_1(void);
static void lv_btn_1(void);
void set_group_scr1(void);
void set_group_scr2(void);
static void keypad_init(void);
static void keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static uint32_t keypad_get_key(void);
/*
 * Configure the hardware as necessary to run this demo.
 */
static void prvSetupHardware( void );

/*
 * main_blinky() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 * main_full() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 0.
 */

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

static volatile bool led_state;
lv_indev_t * indev_keypad;

bool screen_state = true;
lv_obj_t * screen1;
lv_obj_t * screen2;
lv_obj_t * group;
lv_obj_t * btnm1;
lv_obj_t * btn1;
lv_obj_t * btn2;
lv_obj_t * btn3;
lv_obj_t * btn4;
lv_obj_t * slider;
lv_obj_t * slider_label;
lv_obj_t * datetime;
lv_obj_t * rfid_label;
lv_obj_t * arc;
lv_style_t style;

/*-----------------------------------------------------------*/

static void vExampleTimerCallback( TimerHandle_t xTimer )
{
    /* The timer has expired.  Count the number of times this happens.  The
    timer that calls this function is an auto re-load timer, so it will
    execute periodically. */
    led_state = !led_state;
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
}

static void vLvglTimerCallback( TimerHandle_t xTimer )
{
    lv_tick_inc(1);
}

void gui_init(void)
{
    lv_init();

    /* Example for 2) */
    static lv_disp_draw_buf_t draw_buf_dsc_2;
    static lv_color_t buf_2_1[MY_DISP_HOR_RES * 10];                        /*A buffer for 10 rows*/
    static lv_color_t buf_2_2[MY_DISP_HOR_RES * 10];                        /*An other buffer for 10 rows*/
    lv_disp_draw_buf_init(&draw_buf_dsc_2, buf_2_1, buf_2_2, MY_DISP_HOR_RES * 10);   /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = MY_DISP_HOR_RES;
    disp_drv.ver_res = MY_DISP_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = st7789_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc_2;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);

    /*------------------
     * Keypad
     * -----------------*/

    /*Initialize your keypad or keyboard if you have*/
    keypad_init();

    /*Register a keypad input device*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = keypad_read;
    indev_keypad = lv_indev_drv_register(&indev_drv);

    lv_btnmatrix_1();
    lv_btn_1();
    group = lv_group_create();

    lv_scr_load(screen1);
    set_group_scr1();
}

void gui_task(void* param)
{
    SemaphoreHandle_t xGuiSemaphore;

    xGuiSemaphore = xSemaphoreCreateMutex();

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            char str[20];
            datetime_t t;
            rtc_get_datetime(&t);
            uint8_t hour = t.hour > 12 ? t.hour - 12 : (t.hour == 0 ? 12: t.hour);
            snprintf(str, sizeof(str), "#ffffff %02d:%02d %s#", hour, t.min, t.hour < 12 ? "AM" : "PM");
            lv_label_set_text(datetime, str);
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }
}

lwespr_t
examples_common_lwesp_callback_func(lwesp_evt_t* evt) {
    switch (lwesp_evt_get_type(evt)) {
        case LWESP_EVT_AT_VERSION_NOT_SUPPORTED: {
            lwesp_sw_version_t v_min, v_curr;

            lwesp_get_min_at_fw_version(&v_min);
            lwesp_get_current_at_fw_version(&v_curr);

            printf("Current ESP[8266/32[-C3]] AT version is not supported by the library\r\n");
            printf("Minimum required AT version is: %08X\r\n", (unsigned)v_min.version);
            printf("Current AT version is: %08X\r\n", (unsigned)v_curr.version);
            break;
        }
        case LWESP_EVT_INIT_FINISH: {
            printf("Library initialized!\r\n");
            break;
        }
        case LWESP_EVT_RESET_DETECTED: {
            printf("Device reset detected!\r\n");
            break;
        }
        default: break;
    }
    return lwespOK;
}

void lwesp_init_thread(void* param)
{
    /* Initialize ESP with common callback for all examples */
    printf("Initializing LwESP\r\n");
    if (lwesp_init(examples_common_lwesp_callback_func, 1) != lwespOK) {
        printf("Cannot initialize LwESP!\r\n");
    } else {
        printf("LwESP initialized!\r\n");
    }

    station_manager_connect_to_preferred_access_point(1);

    vTaskDelete(NULL);
}
void rfid_callback(void *data)
{
    uint8_t *d = (uint8_t*) data;
    char *str = (char*)pvPortCalloc(1, 20);

    sprintf(str, "0x%02x 0x%02x 0x%02x 0x%02x", d[0], d[1], d[2], d[3]);

    lv_event_send(screen1, LV_EVENT_REFRESH, (void*)str);
}
int main( void )
{
    TimerHandle_t xExampleSoftwareTimer = NULL;
    TimerHandle_t xLvglTimer = NULL;

    /* Configure the hardware ready to run the demo. */
    prvSetupHardware();
    gui_init();
    rfid_init();

    xTaskCreate(gui_task, "gui_task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(rfid_task, "rfid_task", configMINIMAL_STACK_SIZE, &rfid_callback, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(lwesp_init_thread, "lwesp_init_task", configMINIMAL_STACK_SIZE * 4, NULL, configMAX_PRIORITIES - 1, NULL);

    /* Create the software timer as described in the comments at the top of
    this file. */
    xExampleSoftwareTimer = xTimerCreate(
            (const char *) "LEDTimer",
            mainSOFTWARE_TIMER_PERIOD_MS,
            pdTRUE,
            (void *) 0,
            vExampleTimerCallback
    );

    xLvglTimer = xTimerCreate(
            (const char *) "xLvglTimer",
            mainLVGL_TIMER_PERIOD_MS,
            pdTRUE,
            (void *) 0,
            vLvglTimerCallback
    );

    xTimerStart(xLvglTimer, 0);
    xTimerStart(xExampleSoftwareTimer, 0);

    vTaskStartScheduler();

    return 0;
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, 1);
    gpio_put(PICO_DEFAULT_LED_PIN, !PICO_DEFAULT_LED_PIN_INVERTED);

    datetime_t t;
    t.year = 23;
    t.month = 5;
    t.day = 30;
    t.dotw = 2;
    t.hour = 11;
    t.min = 59;
    t.sec = 30;

    rtc_init();
    rtc_set_datetime(&t);
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);

    //Init SPI and pin for LCD
    spi_init(SPI_CHAN, 25*1000*1000);
    //spi_set_format(SPI_CHAN, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(TFT_SCK, GPIO_FUNC_SPI);
    gpio_set_function(TFT_TX, GPIO_FUNC_SPI);

    gpio_init(TFT_CS);
    gpio_set_dir(TFT_CS, GPIO_OUT);

    gpio_init(TFT_DC);
    gpio_set_dir(TFT_DC, GPIO_OUT);

    // BACKLIGHT
    gpio_set_function(TFT_LED, GPIO_FUNC_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 5.f);
    pwm_config_set_wrap(&config, 100);
    pwm_init(pwm_gpio_to_slice_num(TFT_LED), &config, true);
    pwm_set_gpio_level(TFT_LED, 100);

    // Toggle low to reset
    gpio_put(TFT_CS, 0);
    gpio_init(TFT_RESET);
    gpio_set_dir(TFT_RESET, GPIO_OUT);
    gpio_put(TFT_RESET, 1);
    sleep_ms(50);
    gpio_put(TFT_RESET, 0);
    sleep_ms(50);
    gpio_put(TFT_RESET, 1);
    sleep_ms(50);

    st7789_init();
}

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        // uint32_t id = lv_btnmatrix_get_selected_btn(obj);
        // const char * txt = lv_btnmatrix_get_btn_text(obj, id);
        lv_obj_t * label = lv_obj_get_child(obj, 0);
        const char * txt = lv_label_get_text(label);

        if (lv_obj_get_height(obj) == 15)
            lv_obj_set_height(obj, 20);
        else
            lv_obj_set_height(obj, 15);

        //printf("Chiclet was clicked\n", txt);

    }
}

static void event_handler2(lv_event_t * e)
{
}

static void event_screen1_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    const char *str = (char*)lv_event_get_param(e);
    if (code == LV_EVENT_REFRESH) {
        lv_label_set_text(rfid_label, str);
        vPortFree(str);
        // lv_scr_load(screen2);
        // set_group_scr2();
    }
}

static void event_screen2_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_REFRESH) {
        lv_scr_load(screen1);
        set_group_scr1();
    }
}

void set_group_scr1(void)
{
    lv_group_remove_all_objs(group);
    lv_group_add_obj(group, slider);
    lv_group_add_obj(group, btn1);
    lv_group_add_obj(group, btn2);
    lv_group_add_obj(group, btn3);
    lv_group_add_obj(group, btn4);
    lv_indev_set_group(indev_keypad, group);
}

void set_group_scr2(void)
{
    lv_group_remove_all_objs(group);
    lv_group_add_obj(group, btnm1);
    lv_indev_set_group(indev_keypad, group);
}


static const char * btnm_map[] = {"1", "2", "3", "4", "5", "\n",
                                  "6", "7", "8", "9", "0", "\n",
                                  "RFID: ", ""
                                 };

void lv_btnmatrix_1(void)
{
    screen2 = lv_obj_create(NULL);
    btnm1 = lv_btnmatrix_create(screen2);
    lv_btnmatrix_set_map(btnm1, btnm_map);
    lv_btnmatrix_set_btn_width(btnm1, 10, 2);        /*Make "Action1" twice as wide as "Action2"*/
    lv_btnmatrix_set_btn_ctrl(btnm1, 10, LV_BTNMATRIX_CTRL_CHECKABLE);
    lv_obj_align(btnm1, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btnm1, event_handler2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(screen2, event_screen2_handler, LV_EVENT_REFRESH, NULL);
}

static void set_angle(void * obj, int32_t v)
{
    lv_arc_set_value(obj, v);
}

static void slider_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        printf("slider was clicked\n");
        lv_slider_set_value(slider, lv_slider_get_value(slider) + 10, LV_ANIM_ON);
    } else if (code == LV_EVENT_LONG_PRESSED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        printf("slider was long pressed %d\n", code);
        lv_slider_set_value(slider, lv_slider_get_value(slider) - 10, LV_ANIM_ON);
    }
    pwm_set_gpio_level(TFT_LED, lv_slider_get_value(slider));
    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d%%", (int)lv_slider_get_value(slider));
    lv_label_set_text(slider_label, buf);
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

void lv_btn_1(void)
{
    lv_obj_t * label;
    char str[20];
    datetime_t t;
    screen1 = lv_obj_create(NULL);

    lv_obj_set_style_bg_color(screen1, lv_color_hex(0x020202), 0);

    slider = lv_slider_create(screen1);
    lv_obj_align(slider, LV_ALIGN_CENTER, 50, 0);
    lv_obj_set_size(slider, 100, 5);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_ALL, NULL);
    lv_slider_set_value(slider, 100, LV_ANIM_ON);

    /*Create a label below the slider*/
    slider_label = lv_label_create(screen1);
    lv_label_set_text(slider_label, "100%");
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /*Create a label above the slider*/
    rfid_label = lv_label_create(screen1);
    lv_label_set_text(rfid_label, "RFID: ");
    lv_obj_set_style_text_font(rfid_label, LV_FONT_SMALL, 0);
    lv_obj_align_to(rfid_label, slider, LV_ALIGN_OUT_TOP_LEFT, 0, -10);

   
    lv_obj_t * arc = lv_arc_create(screen1);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0xff0000), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x6e6e6e), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 5, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 5, LV_PART_MAIN);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_set_size(arc, 100, 100);
    lv_arc_set_value(arc, 75);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_align(arc, LV_ALIGN_LEFT_MID, 10, 0);

    label = lv_label_create(screen1);
    lv_label_set_text(label, "#ff0000 Inprogress#");
    lv_label_set_recolor(label, true);
    lv_obj_set_style_text_font(label, LV_FONT_SMALL, 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 35, -30);

    label = lv_label_create(screen1);
    lv_label_set_text(label, "#ffffff PanL Booking#");
    lv_label_set_recolor(label, true);
    lv_obj_set_style_text_font(label, LV_FONT_TINY, 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 32, -20);

    label = lv_label_create(screen1);
    lv_label_set_text(label, "#ffffff 10:00am - 10:30am#");
    lv_label_set_recolor(label, true);
    lv_obj_set_style_text_font(label, LV_FONT_TINY, 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 25, -13);

    label = lv_label_create(screen1);
    lv_label_set_text(label, "#ffffff User-prm65#");
    lv_label_set_recolor(label, true);
    lv_obj_set_style_text_font(label, LV_FONT_TINY, 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 37, -6);

    label = lv_label_create(screen1);
    lv_label_set_text(label, "#ff0000 25#");
    lv_label_set_recolor(label, true);
    lv_obj_set_style_text_font(label, LV_FONT_LARGE, 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 43, 13);

    label = lv_label_create(screen1);
    lv_label_set_text(label, "#ff0000 min left#");
    lv_label_set_recolor(label, true);
    lv_obj_set_style_text_font(label, LV_FONT_SMALL, 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 37, 32);

    datetime = lv_label_create(screen1);
    rtc_get_datetime(&t);
    uint8_t hour = t.hour > 12 ? t.hour - 12 : (t.hour == 0 ? 12: t.hour);
    snprintf(str, sizeof(str), "#ffffff %02d:%02d %s#", hour, t.min, t.hour < 12 ? "AM" : "PM");
    lv_label_set_text(datetime, str);
    lv_label_set_recolor(datetime, true);
    lv_obj_align(datetime, LV_ALIGN_TOP_RIGHT, -5, 5);

    btn1 = lv_btn_create(screen1);
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_BOTTOM_MID, 10, -10);
    lv_obj_set_size(btn1, 30, 15);

    label = lv_label_create(btn1);
    lv_label_set_text(label, "10:00");
    lv_obj_set_style_text_font(label, LV_FONT_SMALL, 0);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, 7);

    btn2 = lv_btn_create(screen1);
    lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_BOTTOM_MID, 40, -10);
    lv_obj_set_size(btn2, 30, 15);

    label = lv_label_create(btn2);
    lv_label_set_text(label, "10:15");
    lv_obj_set_style_text_font(label, LV_FONT_SMALL, 0);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, 7);

    btn3 = lv_btn_create(screen1);
    lv_obj_add_event_cb(btn3, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn3, LV_ALIGN_BOTTOM_MID, 70, -10);
    lv_obj_set_size(btn3, 30, 15);

    label = lv_label_create(btn3);
    lv_label_set_text(label, "10:30");
    lv_obj_set_style_text_font(label, LV_FONT_SMALL, 0);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, 7);

    btn4 = lv_btn_create(screen1);
    lv_obj_add_event_cb(btn4, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn4, LV_ALIGN_BOTTOM_MID, 100, -10);
    lv_obj_set_size(btn4, 30, 15);

    label = lv_label_create(btn4);
    lv_label_set_text(label, "10:45");
    lv_obj_set_style_text_font(label, LV_FONT_SMALL, 0);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, 7);

    lv_obj_add_event_cb(screen1, event_screen1_handler, LV_EVENT_REFRESH, NULL);
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{

}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{

}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{

}

/*Initialize your keypad*/
static void keypad_init(void)
{
    /*Your code comes here*/
    gpio_init(6);
    gpio_set_dir(6, GPIO_IN);
    gpio_pull_up(6);
    gpio_init(7);
    gpio_set_dir(7, GPIO_IN);
    gpio_pull_up(7);
}

/*Will be called by the library to read the mouse*/
static void keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static uint32_t last_key = 0;

    /*Get whether the a key is pressed and save the pressed key*/
    uint32_t act_key = keypad_get_key();
    if(act_key != 0) {
        data->state = LV_INDEV_STATE_PR;
        last_key = act_key;
    }
    else {
        data->state = LV_INDEV_STATE_REL;
    }

    data->key = last_key;
}

/*Get the currently being pressed key.  0 if no key is pressed*/
static uint32_t keypad_get_key(void)
{
    /*Your code comes here*/
    if (gpio_get(6) == false)
        if (screen_state) return LV_KEY_NEXT;
        else return LV_KEY_RIGHT;
    if (gpio_get(7) == false) return LV_KEY_ENTER;

    return 0;
}

