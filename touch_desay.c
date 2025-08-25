#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <board.h>

#include <driver/gpio.h>
#include <CLI.h>
#include <driver/kunlun/touch/touch.h>
#include <driver/kunlun/touch/touch_vendor.h>

#include "msc.h"
#include "lcd.h"

#define GOODIX_EVENT_COMPLETE        0x1
#define GOODIX_EVENT_DEINIT          0x2
#define GOODIX_EVENT_DEV_RESET       0x4

/** log macro define */
#define TOUCH_DEBUG_LVL      7
#define EXTRA_FMT   "touch: "

#if TOUCH_DEBUG_LVL >= SSDK_ERR
#define TOUCH_ERR(format, ...) \
    ssdk_printf(SSDK_ERR, EXTRA_FMT format, ##__VA_ARGS__)
#else
#define TOUCH_ERR(format, ...)
#endif

#if TOUCH_DEBUG_LVL >= SSDK_WARNING
#define TOUCH_WARN(format, ...) \
    ssdk_printf(SSDK_WARNING, EXTRA_FMT format, ##__VA_ARGS__)
#else
#define TOUCH_WARN(format, ...)
#endif

#if TOUCH_DEBUG_LVL >= SSDK_INFO
#define TOUCH_INFO(format, ...) \
    ssdk_printf(SSDK_INFO, EXTRA_FMT format, ##__VA_ARGS__)
#else
#define TOUCH_INFO(format, ...)
#endif

#define LOGPRINT_ONCE(format, ...) \
    do {\
        static bool flg = false; \
        if (!flg) {\
            flg = true; \
            ssdk_printf(SSDK_EMERG, format, ##__VA_ARGS__);} \
    }while(0)


/** sdrv goodix gt928 driver structure */
typedef struct sdrv_touch_drv {
    touch_dev_t dev;
    uint16_t id;
    uint16_t version;
    uint16_t vendor;
    osEventFlagsId_t event;
    osMutexId_t mutex;
    osThreadId_t thread;
    bool is_run;
    volatile bool update_flag;

    touch_cb_chan_t cb[TOUCH_NOTICER_MAX_NUM];
}sdrv_touch_drv_t;


/* =========================== function declare session ==================== */
static int32_t goodix_init(touch_dev_t* dev);
static int32_t goodix_deinit(touch_dev_t* dev);
static int32_t goodix_online_check(touch_dev_t* dev);
static int32_t goodix_register_callback(touch_dev_t* dev
                                        , touch_event_notice cb
                                        , void* usrCtx);
static int32_t goodix_touch_control(touch_dev_t* dev
                                    , touch_ctrl_type_enum_t type
                                    , void* data
                                    , int32_t* len);

/* =========================== static value define session ================= */
static touch_ops_t s_goodix_ops = {
    .init = goodix_init,
    .deinit = goodix_deinit,
    .online_check = goodix_online_check,
    .register_callback = goodix_register_callback,
    .device_control = goodix_touch_control,
};

/* =========================== function implementation session ============= */
extern int desay_i2c_read(i2c_adap_dev_t *adap, uint8_t addr, uint8_t* buf, int len);
extern int desay_i2c_write(i2c_adap_dev_t *adap, uint8_t addr, uint8_t* buf, int len);

static uint8_t checksum(uint8_t* data, size_t len ) {
    int sum = 0;

    for (size_t i = 0; i <len; i++) {
        sum += data[i];
    }
    sum++;

    return (uint8_t)(sum & 0xFF);
}
static uint8_t tp_status[TOUCH_POINT_MAX_NUM] = {0};
static int desay_ts_read_input_report(touch_dev_t* dev, ts_coord_info_t* coordPtr){
#define DESAY_MAX_SLOTS 10


    struct desaysv_tch_point {
      uint8_t touch_id;
      uint8_t event_id;
      uint8_t x_m;
      uint8_t x_l;
      uint8_t y_m;
      uint8_t y_l;
    };

#define TCH_P_SIZE  sizeof(struct desaysv_tch_point)

    enum desaysv_event_id {
      CY_EV_NO_EVENT = 0,
      CY_EV_TOUCHDOWN = 0xC0,
      CY_EV_MOVE = 0x90,     /* significant displacement (> act dist) */
      CY_EV_LIFTOFF = 0xA0,     /* record reports last position */
    };

    struct desaysv_stand_frame first_fm = {0};
    uint8_t ext_frame_data[TOUCH_POINT_MAX_NUM*TCH_P_SIZE] = {0};
    int rc;
    int tp_num = 0;

    rc = desay_i2c_read(dev->cfg_ptr->i2c_dev, dev->cfg_ptr->i2c_addr, (uint8_t *)&first_fm, sizeof(first_fm));
    if(rc){
        return -1;
    }
#if 0
    uint8_t *data = (uint8_t *)&first_fm;
    for(uint8_t i = 0; i < sizeof(first_fm); i++){
        printf("%x ", data[i]);
    }
    printf("\n");
#endif
    uint8_t result = 0;
    if (first_fm.cmd_id != DESAYSV_CMDID_UPG_PASSWD
            && first_fm.cmd_id != DESAYSV_HOST_CMDID_UPG_MCU
                && first_fm.cmd_id != DESAYSV_CMDID_UPG) {
        result = checksum((uint8_t * ) & first_fm, sizeof(struct desaysv_stand_frame) - 1);
        if (result != first_fm.chksum) {
            TOUCH_ERR("tp i2c read checksum error %#x,%#x\n", result, first_fm.chksum);
            return -1;
        }
    }

    switch(first_fm.cmd_id) {
        case DESAYSV_CMDID_TP:
            tp_num = first_fm.data[0];
            if (tp_num < 0 || tp_num > DESAY_MAX_SLOTS){
                return -1;
            }
            if (first_fm.ext_len && tp_num > 2) {
                if (first_fm.ext_len != (tp_num -2)*TCH_P_SIZE+3){
                    return -1;
                }
                rc = desay_i2c_read(dev->cfg_ptr->i2c_dev, dev->cfg_ptr->i2c_addr, (uint8_t *)&ext_frame_data, first_fm.ext_len);
                if(rc){
                    return -1;
                }
                if (ext_frame_data[0] != DESAYSV_CMDID_TP_EXT) {
                    TOUCH_ERR("extended frams is not touch point extended frams\n");
                    return -1;
                }
                result = checksum((uint8_t *)&ext_frame_data, first_fm.ext_len-2);
                if (result != ext_frame_data[first_fm.ext_len-1]) {
                    TOUCH_ERR("tp i2c read checksum error %#x,%#x\n", result, ext_frame_data[first_fm.ext_len-1]);
                    return -1;
                }
            }
            struct desaysv_tch_point* tch_point = (struct desaysv_tch_point*)&first_fm.data[1];
            struct desaysv_tch_point* ext_tch_point = (struct desaysv_tch_point*)&ext_frame_data[1];
            memset(coordPtr, 0x00, sizeof(ts_coord_info_t));
            for (int32_t cnt = 0; cnt < tp_num; cnt++) {
                if (cnt < 2){
                    if (tch_point[cnt].touch_id > TOUCH_POINT_MAX_NUM) {
                        TOUCH_ERR("the number of touch point exceed 10\n");
                        return -1;
                    }
                    if (tch_point[cnt].event_id != CY_EV_TOUCHDOWN && tch_point[cnt].event_id != CY_EV_MOVE){
                        if (tch_point[cnt].touch_id == 0) {
                            memset(tp_status,0x00, sizeof(tp_status));
                            coordPtr->touch_num = 0;
                        }else{
                            tp_status[tch_point[cnt].touch_id -1] = 0x00;
                        }
                        //coordPtr->point[cnt].w = 0;
                    }else{
                        tp_status[tch_point[cnt].touch_id -1] = 0x01;
                        coordPtr->point[coordPtr->touch_num].id = tch_point[cnt].touch_id;
                        coordPtr->point[coordPtr->touch_num].x = tch_point[cnt].x_m << 8 | tch_point[cnt].x_l;
                        coordPtr->point[coordPtr->touch_num].y = tch_point[cnt].y_m << 8 | tch_point[cnt].y_l;
                        //coordPtr->point[cnt].w = 18;
                        coordPtr->touch_num++;
                    }
                    if (tch_point[cnt].event_id == CY_EV_TOUCHDOWN || tch_point[cnt].event_id == CY_EV_LIFTOFF) {
                        uint8_t ts_data[6];
                        ts_data[0] = tch_point[cnt].touch_id;
                        ts_data[1] = tch_point[cnt].event_id;
                        ts_data[2] = tch_point[cnt].x_m;
                        ts_data[3] = tch_point[cnt].x_l;
                        ts_data[4] = tch_point[cnt].y_m;
                        ts_data[5] = tch_point[cnt].y_l;
                        msc_publish(MSC_TOPIC_LCD_RESPONSE, "touch_status", ts_data, sizeof(ts_data), true);
                    }
                }else{
                    if (ext_tch_point[cnt-2].touch_id > TOUCH_POINT_MAX_NUM) {
                        TOUCH_ERR("the number of touch point exceed 10\n");
                        return -1;
                    }
                    if (ext_tch_point[cnt-2].event_id != CY_EV_TOUCHDOWN && ext_tch_point[cnt-2].event_id != CY_EV_MOVE){
                        if (ext_tch_point[cnt-2].touch_id == 0) {
                            memset(tp_status,0x00, sizeof(tp_status));
                            coordPtr->touch_num = 0;
                        }else{
                            tp_status[ext_tch_point[cnt-2].touch_id -1] = 0x00;
                        }
                        //coordPtr->point[cnt].w = 0;
                    }else{
                        tp_status[ext_tch_point[cnt-2].touch_id -1] = 0x01;
                        coordPtr->point[coordPtr->touch_num].id = ext_tch_point[cnt-2].touch_id;
                        coordPtr->point[coordPtr->touch_num].x = ext_tch_point[cnt-2].x_m << 8 | ext_tch_point[cnt-2].x_l;
                        coordPtr->point[coordPtr->touch_num].y = ext_tch_point[cnt-2].y_m << 8 | ext_tch_point[cnt-2].y_l;
                        //coordPtr->point[coordPtr->touch_num].w = 18;
                        coordPtr->touch_num++;
                    }
                }
            }
            // for (int i = 0; i < sizeof(tp_status); ++i) {
            //     if (tp_status[i] == 0x01){
            //         coordPtr->touch_num ++;
            //     }
            // }
			return 0;
#ifdef X9SP_MS_A12_Q_SSDK_T1EJ
        default:on_desay_lcd_callback(NULL,first_fm);
#else
        default:on_desay_lcd_callback(first_fm);
#endif
            break;
    }
    return -1;
}

static void goodix_irq_handler(struct gpio_dev *dev, int pin, void *arg)
{
    touch_dev_t* touch_dev = arg;
    sdrv_touch_drv_t* touch_drv = touch_dev->priv;

    gpio_int_enable(dev, pin, false);
    osEventFlagsSet(touch_drv->event, GOODIX_EVENT_COMPLETE);
}


static int32_t goodix_reset_device(touch_dev_t* dev)
{
    serdes_gpio_ctrl_t gpio_ctrl;
    uint8_t len = sizeof(gpio_ctrl);

    ssdk_printf(SSDK_EMERG, "touch: %s reset device start! \n", dev->name);
    if (dev->cfg_ptr->serdes_config.enable) {

        gpio_ctrl.io_num = dev->cfg_ptr->serdes_config.reset_pin;
        gpio_ctrl.dir = 0;
        gpio_ctrl.val = 0;
        serdes_control(*dev->cfg_ptr->serdes_config.serdes
                            , DES_CONTROL_GPIO_CTRL
                            , &gpio_ctrl
                            , &len);

        gpio_ctrl.io_num = dev->cfg_ptr->serdes_config.irq_pin;
        gpio_ctrl.dir = 0;
        gpio_ctrl.val = 0;
        serdes_control(*dev->cfg_ptr->serdes_config.serdes
                            , DES_CONTROL_GPIO_CTRL
                            , &gpio_ctrl
                            , &len);

        gpio_ctrl.io_num = dev->cfg_ptr->serdes_config.irq_pin;
        gpio_ctrl.dir = 0;
        gpio_ctrl.val = (dev->cfg_ptr->i2c_addr == 0x14);
        serdes_control(*dev->cfg_ptr->serdes_config.serdes
                            , DES_CONTROL_GPIO_CTRL
                            , &gpio_ctrl
                            , &len);
        osDelay(1);

        gpio_ctrl.io_num = dev->cfg_ptr->serdes_config.reset_pin;
        gpio_ctrl.dir = 0;
        gpio_ctrl.val = 1;
        serdes_control(*dev->cfg_ptr->serdes_config.serdes
                            , DES_CONTROL_GPIO_CTRL
                            , &gpio_ctrl
                            , &len);
        osDelay(5);

        gpio_ctrl.io_num = dev->cfg_ptr->serdes_config.irq_pin;
        gpio_ctrl.dir = 0;
        gpio_ctrl.val = 0;
        serdes_control(*dev->cfg_ptr->serdes_config.serdes
                            , DES_CONTROL_GPIO_CTRL
                            , &gpio_ctrl
                            , &len);
        osDelay(50);

        gpio_ctrl.io_num = dev->cfg_ptr->serdes_config.irq_pin;
        gpio_ctrl.dir = 1;
        serdes_control(*dev->cfg_ptr->serdes_config.serdes
                            , DES_CONTROL_GPIO_CTRL
                            , &gpio_ctrl
                            , &len);

        gpio_ctrl.io_num = dev->cfg_ptr->serdes_config.irq_pin;
        gpio_ctrl.dir = 1;
        serdes_control(*dev->cfg_ptr->serdes_config.serdes
                            , SER_CONTROL_GPIO_CTRL
                            , &gpio_ctrl
                            , &len);

        serdes_port_passthrough_ctl_t port_ctl;
        port_ctl.dir = 1;   /**< des -> ser */
        port_ctl.d_port = dev->cfg_ptr->serdes_config.irq_pin;
        port_ctl.s_port = dev->cfg_ptr->serdes_config.sirq_pin;
        uint8_t ctl_len = sizeof(port_ctl);
        serdes_control(*dev->cfg_ptr->serdes_config.serdes
                        , SERDES_PORT_PASSTHROUGH_CTL
                        , &port_ctl
                        , &ctl_len);
    }

    ssdk_printf(SSDK_EMERG, "touch: %s reset device end! \n", dev->name);
    return 0;
}

void touch_goodix_send_flg(touch_dev_t* dev)
{
    sdrv_touch_drv_t* sdrv_touch = (sdrv_touch_drv_t*)dev->priv;
    osEventFlagsSet(sdrv_touch->event, GOODIX_EVENT_COMPLETE);
}

static void goodix_ts_work_func(void *arg)
{
    ts_coord_info_t report_data;
    touch_dev_t* dev = arg;
    sdrv_touch_drv_t* sdrv_touch = dev->priv;
    touch_notice_t notice_data;
    uint32_t event_flg = 0;
    sdrv_touch->update_flag = false;
    bool i2c_status = false;

    TOUCH_ERR("goodix work function entry\n");

    gpio_config(dev->cfg_ptr->irq_pin.pin_dev
                , dev->cfg_ptr->irq_pin.pin_num
                , GPIO_INTR_FALLING_EDGE);

    gpio_attach_irq(dev->cfg_ptr->irq_pin.pin_dev
                , dev->cfg_ptr->irq_pin.pin_num
                , goodix_irq_handler
                , dev);
    sdrv_touch->is_run = true;
    dev->is_init = true;
    while (sdrv_touch->is_run) {
        event_flg = osEventFlagsWait(sdrv_touch->event
                                     , GOODIX_EVENT_COMPLETE | GOODIX_EVENT_DEINIT | GOODIX_EVENT_DEV_RESET
                                     , osFlagsWaitAny
                                     , osWaitForever);

        osMutexAcquire(sdrv_touch->mutex, osWaitForever);

        if ((event_flg & GOODIX_EVENT_COMPLETE)) {
            if (desay_ts_read_input_report(dev, &report_data) == 0) {
                TOUCH_INFO("%s: [%d] 0x%x, (%d, %d, %d, %d)\n", __func__, dev->cfg_ptr->screen_id
                            , report_data.touch_num
                            , report_data.point[0].id
                            , report_data.point[0].x
                            , report_data.point[0].y
                            , report_data.point[0].w);
                notice_data.evt_type = TS_NOTICE_EVT_COORD;
                memcpy(&notice_data.coord, &report_data, sizeof(report_data));
                for (int32_t i = 0; i < TOUCH_NOTICER_MAX_NUM; i++) {
                    if (!sdrv_touch->cb[i].is_valid &&
                        sdrv_touch->cb[i].cb != NULL) {
                        sdrv_touch->cb[i].cb(dev->cfg_ptr->screen_id, &notice_data, sdrv_touch->cb[i].usrCtx);
                    }
                }
                i2c_status = true;
            }
            else {
                i2c_status = false;
            }
        }

        if (event_flg & GOODIX_EVENT_DEV_RESET) {
            goodix_reset_device(dev);
        }

        if (event_flg & GOODIX_EVENT_DEINIT) {
            sdrv_touch->is_run = false;
            gpio_int_enable(dev->cfg_ptr->irq_pin.pin_dev, dev->cfg_ptr->irq_pin.pin_num, false);
            break;
        }

        if (i2c_status && (!sdrv_touch->update_flag) && gpio_get(dev->cfg_ptr->irq_pin.pin_dev, dev->cfg_ptr->irq_pin.pin_num) == 0){
            TOUCH_ERR("irq is too fast,reread iic\n");
            osEventFlagsSet(sdrv_touch->event, GOODIX_EVENT_COMPLETE);
        }else{
            gpio_int_enable(dev->cfg_ptr->irq_pin.pin_dev, dev->cfg_ptr->irq_pin.pin_num, true);
        }
        osMutexRelease(sdrv_touch->mutex);

    }

    TOUCH_ERR("touch task exit! \n");

exit:
    dev->is_init = false;
    osEventFlagsDelete(sdrv_touch->event);
    osMutexDelete(sdrv_touch->mutex);
    osThreadExit();

    return ;
}

static int goodix_config_device(touch_dev_t* dev)
{
    sdrv_touch_drv_t* sdrv_touch = dev->priv;

    sdrv_touch->event = osEventFlagsNew(NULL);
    sdrv_touch->mutex = osMutexNew(NULL);

    for (int32_t i = 0; i < TOUCH_NOTICER_MAX_NUM; i++) {
        sdrv_touch->cb[i].is_valid = true;
    }
    sdrv_touch->is_run = false;

    osThreadAttr_t attr = {
        .name = dev->cfg_ptr->name,
        .priority = osPriorityRealtime,
        .stack_size = 2048,
    };
    sdrv_touch->thread = osThreadNew(goodix_ts_work_func, dev, &attr);
    return 0;
}

static int32_t goodix_init(touch_dev_t* dev)
{
    if (dev == NULL || dev->priv == NULL) {
        TOUCH_ERR("invalid parameter for %s \n", __func__);
        return -1;
    }

    // TOUCH_INFO("[debug]goodix touch device init \n");
    // serdes config
    if (dev->cfg_ptr->serdes_config.enable) {
        // check link
        if (serdes_check_serdes_link(*dev->cfg_ptr->serdes_config.serdes) != 0) {
            return -1;
        }

        // enable port
        if (serdes_control(*dev->cfg_ptr->serdes_config.serdes
                                , SER_CONTROL_PORT_ENABLE, NULL, 0) != 0) {
            return -1;
        }

        // enable passthrough
        if (serdes_control(*dev->cfg_ptr->serdes_config.serdes
                                , SER_CONTROL_ENABLE_I2C_PASSTROUGH, NULL, 0) != 0) {
            return -1;
        }
    }

    // os config
    return goodix_config_device(dev);

}

static int32_t goodix_deinit(touch_dev_t* dev)
{
    // set event for destroy thread
    if (dev == NULL) {
        TOUCH_ERR("invalid parameter for %s \n", __func__);
        return -1;
    }

    sdrv_touch_drv_t * sdrv_touch = (sdrv_touch_drv_t *)dev->priv;
    osEventFlagsSet(sdrv_touch->event, GOODIX_EVENT_DEINIT);
    return 0;
}

static int32_t goodix_online_check(touch_dev_t* dev)
{
    int32_t ret = -1;
    if (dev == NULL) {
        TOUCH_ERR("invalid parameter for %s.\n", __func__);
        return -1;
    }

    // sdrv_touch_drv_t* sdrv_touch = (sdrv_touch_drv_t *)dev->priv;

    if (dev->cfg_ptr->serdes_config.enable) {
        if (serdes_check_serdes_link(*dev->cfg_ptr->serdes_config.serdes) == 0) {
            ret = 0;
        }
        else {
            TOUCH_INFO("%s touch device offline, please check serdes link status or touch link status\n", dev->name);
            return -1;
        }
    }

    sdrv_touch_drv_t * sdrv_touch = (sdrv_touch_drv_t *)dev->priv;
    if (sdrv_touch->is_run) {
        ret = 0;
    }
    else {
        TOUCH_ERR("%s touch device offline, please check touch work task status.\n", dev->name);
        return -1;
    }

    return ret;
}

static int32_t goodix_register_callback(touch_dev_t* dev, touch_event_notice cb, void* usrCtx)
{
    if (dev == NULL || cb == NULL) {
        TOUCH_ERR("invalid parameter for %s.\n", __func__);
        return -1;
    }

    sdrv_touch_drv_t* sdrv_touch = (sdrv_touch_drv_t *)dev->priv;

    int32_t ret = -1;

    osMutexAcquire(sdrv_touch->mutex, osWaitForever);
    for (int32_t i = 0; i < TOUCH_NOTICER_MAX_NUM; i++) {
        if (sdrv_touch->cb[i].is_valid) {
            sdrv_touch->cb[i].cb = cb;
            sdrv_touch->cb[i].usrCtx = usrCtx;
            sdrv_touch->cb[i].is_valid = false;
            ret = 0;
            TOUCH_INFO("[debug]register touch data callback with chan: %d\n", i);
            break;
        }
    }

    if (ret != 0) {
        TOUCH_WARN("there is no invalid callback channel for %s \n", __func__);
    }
    osMutexRelease(sdrv_touch->mutex);

    return ret;
}

static int32_t goodix_touch_control(touch_dev_t* dev, touch_ctrl_type_enum_t type, void* data, int32_t* len)
{
    if (dev == NULL) {
        TOUCH_ERR("parameter error for %s \n", __func__);
        return -1;
    }
    serdesCtl_i2cAddr_t i2c_addr;

    sdrv_touch_drv_t* sdrv_touch = (sdrv_touch_drv_t *)dev->priv;
    // control function not implement
    switch (type) {
        case TS_CONTROL_CHANGE_MODE:
            //@TODO change sensor mode
            break;
        case TS_CONTROL_UPDATE_FIRMWARE:
            if (data != NULL && *len >= 1) {
                sdrv_touch->update_flag = ((bool *) data)[0];
                ssdk_printf(SSDK_EMERG, "touch:update_flag %d \n", sdrv_touch->update_flag);
            }
            break;
        case TS_CONTROL_RESET_DEV:
            ssdk_printf(SSDK_EMERG, "touch: %s reset device %s \n", __func__, dev->name);
            osEventFlagsSet(sdrv_touch->event, GOODIX_EVENT_DEV_RESET);
            break;
        case TS_CONTROL_GET_COORD_CFG:
            if (data != NULL) {
                ((tsCtrl_configInfo_t *)(data))->inverted_x = dev->cfg_ptr->coord_config.inverted_x;
                ((tsCtrl_configInfo_t *)(data))->inverted_y = dev->cfg_ptr->coord_config.inverted_y;
                ((tsCtrl_configInfo_t *)(data))->max_touch_num = dev->cfg_ptr->coord_config.max_touch_num;
                ((tsCtrl_configInfo_t *)(data))->swapped_x_y = dev->cfg_ptr->coord_config.swapped_x_y;
                if (len != NULL)
                    *len = sizeof(tsCtrl_configInfo_t);
            }
            else {
                return -1;
            }
            break;
        case TS_CONTROL_GET_SCREEN_ID:
            *((int32_t *)data) = dev->cfg_ptr->screen_id;
            *len = sizeof(int32_t);
            break;
        case TS_CONTROL_GET_VENDOR_INFO:
            if (data != NULL) {
                ((tsCtrl_vendorInfo_t *)(data))->name = dev->cfg_ptr->name;
                ((tsCtrl_vendorInfo_t *)(data))->id = sdrv_touch->id;
                ((tsCtrl_vendorInfo_t *)(data))->vendor = sdrv_touch->vendor;
                ((tsCtrl_vendorInfo_t *)(data))->version = sdrv_touch->version;
                ((tsCtrl_vendorInfo_t *)(data))->ts_addr = dev->cfg_ptr->i2c_addr;
                if (dev->cfg_ptr->serdes_config.serdes != NULL) {
                    memset(&i2c_addr, 0x00, sizeof(i2c_addr));
                    serdes_control(*dev->cfg_ptr->serdes_config.serdes, SERDES_CONTROL_GET_I2C_ADDR, &i2c_addr, NULL);
                    ((tsCtrl_vendorInfo_t *)(data))->ser_addr = i2c_addr.ser_addr;
                    ((tsCtrl_vendorInfo_t *)(data))->des_addr = i2c_addr.des_addr;
                }

                if (len != NULL)
                    *len = sizeof(tsCtrl_vendorInfo_t);
            }
            break;
        case TS_CONTROL_SEND_FLG:
            touch_goodix_send_flg(dev);
            break;
        default:
            break;
    }

    return 0;
}

touch_dev_t* goodix_touch_probe(sdrv_touch_handle* touchDrv, const touch_config_t* cfgPtr)
{
    touch_dev_t* dev = NULL;
    if (cfgPtr == NULL) {
        TOUCH_ERR("invalid parameter for %s \n", __func__);
        return dev;
    }

    if (*touchDrv != NULL) {
        TOUCH_INFO("already probed touch device for %s.\n", cfgPtr->name);
        return &((sdrv_touch_drv_t *)(*touchDrv))->dev;
    }

    *touchDrv = malloc(sizeof(sdrv_touch_drv_t));
    if (*touchDrv == NULL) {
        TOUCH_ERR("malloc memory for %s device error\n", cfgPtr->name);
        return NULL;
    }
    memset(*touchDrv, 0x00, sizeof(sdrv_touch_drv_t));

    sdrv_touch_drv_t* touch_drv = (sdrv_touch_drv_t *)(*touchDrv);
    touch_drv->dev.cfg_ptr = cfgPtr;
    touch_drv->dev.name = cfgPtr->name;
    touch_drv->dev.ops = &s_goodix_ops;
    touch_drv->dev.priv = touch_drv;
    dev = &touch_drv->dev;

    if (touch_init(dev) != 0) {
        TOUCH_ERR("touch device init error\n");
    }
    lcd_init(dev);
    return dev;
}

int touch_dump_cmd(int argc, char *argv[]) {
    for (int i = 0; i < sizeof(tp_status); ++i) {
        TOUCH_ERR("id %d status %d\n", i, tp_status[i]);
    }
    return 0;
}

CLI_CMD("touch", "dump touch status\r\n\ttouch dump", touch_dump_cmd);