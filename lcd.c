//
// Created by citos on 2024/1/10.
//

#include <stdint.h>
#define LOG_TAG "lcd"
#include <stddef.h>
#include <string.h>
#include <configuration.h>
#include "lcd.h"
#include "citos_log.h"
#include "msc.h"

// 小灯状态，亮度模式
#define ILL_OFF                     0x00
#define ILL_ON                      0x01
#define LCD_MODE_AUTO               0x00
#define LCD_MODE_DAY                0x01
#define LCD_MODE_NIGHT              0x02

// 设置亮度模式
#define SET_MODE_DAY                0x00
#define SET_MODE_NIGHT              0x01

#define  TOUCH_MODE_NORMARL         0
#define  TOUCH_MODE_UPDATING        1
#define  TOUCH_MODE_REQUEST_UPDATE  2
#define  TOUCH_MODE_NONE            3
#define  TOUCH_UPDATE_CMD_SUCCESS   0
#define  TOUCH_UPDATE_CMD_FAIL      1
#define  TOUCH_UPDATE_CMD_NONE      2

struct {
  touch_dev_t *dev;
  osThreadId_t thread;
  bool isConected;
  bool tpI2cState;
  bool desI2cState;
  struct {
    uint8_t tpUpgradeStatus;//触摸升级状态
    uint8_t tpUpgradeResult;//触摸升级结果
#define LCD_MODE_APP 0x01
#define LCD_MODE_BOOT 0x02
#define LCD_MODE_SELFTEST 0x03
    uint8_t lcdMode;//显示屏工作模式
#define LCD_LOG_OFF 0x00
#define LCD_LOG_ON 0x01
    uint8_t logMode;//log传输模式
    uint8_t lcdReposeTimeout;//显示屏响应主机超时时间
#define LCD_TEMP_NOMAL 0x00
#define LCD_TEMP_ABNOMAL_DOWN 0x01
#define LCD_TEMP_ABNORMAL_OFF 0x02
    uint8_t lcdTemp;//显示屏背光温度状态
#define LCD_POWER_STANDBY 0x00
#define LCD_POWER_POWEROFF 0x01
#define LCD_POWER_POWERON 0x02
    uint8_t lcdPowerStatus;//显示屏状态
    uint16_t lcdBrightness;//显示屏背光亮度
#define LCD_BL_OFF 0x00
#define LCD_BL_ON 0x01
    uint8_t lcdBlOnOff;//背光开关状态
    uint8_t lcdSupplier;//显示屏供应商
#define LCD_SIZE_1025 0x01
#define LCD_SIZE_1230 0x02
#define LCD_SIZE_1320 0x05
    uint8_t lcdSize;//显示屏尺寸
#define LCD_TYPE_INFO 0x01
#define LCD_TYPE_CLUSTER 0x02
#define LCD_TYPE_HUD 0x03
    uint8_t lcdType;//显示屏类型
    uint8_t lcdHardware;//显示屏硬件版本
    uint8_t lcdVersion[10];
  } resp;
  struct {
    uint8_t lcdWorkStatus;
    uint8_t displayWorkStatus;
    uint8_t tpWorkStatus;
    uint8_t blWorkStatus;
    uint8_t videoWorkStatus;
    uint8_t notReceiveBl;
    uint8_t overVoltage;
    uint8_t underVoltage;
  } abNomal;
  uint8_t bootVersion[4];
  uint8_t fingerprint[9];//指纹信息
  uint8_t serialNumber[12];//序列号
  uint8_t partNum[12];//零件号
  uint8_t softPartNum;//软件零件号
  uint8_t passwordStatus;
  uint8_t tkVersion[12];
  uint8_t lcdStatus[13];  //显示屏状态，周期性
  bool isVoltageNormal;
  uint8_t lcdIll;
  struct {
    uint16_t brightness;
    uint8_t logMode;//log传输模式
    bool blOnOff;
    bool pwrOnOff;
  } setting;
} lcdserver = {.isConected = true, .setting.blOnOff = false, .setting.brightness = 28835, .setting.pwrOnOff = true, .desI2cState = true, .tpI2cState = true};

int desay_i2c_read(i2c_adap_dev_t *adap, uint8_t addr, uint8_t* buf, int len)
{
    int ret = 0;
    uint8_t wbuf = 0xFE;
    int wlen = 1;

    ret = i2c_write_read(adap, addr, &wbuf, wlen, buf, len);

    if (ret < 0) {
        CITOSPRINT_ONCE("%s: read error, ret=%d\n", __func__, ret);
        lcdserver.tpI2cState = false;
        return ret;
    }else{
        lcdserver.tpI2cState = true;
    }

    return 0;
}

bool desay_check_deser_reg(i2c_adap_dev_t *adap, uint8_t addr) {
    int ret = 0;
    uint8_t buf = 0;

    ret = i2c_read_reg16(adap, addr, 0, &buf, sizeof(buf));
    if (ret < 0) {
        CITOS_ERR("read reg0 error, ret=%d.\n", ret);
        return false;
    }
    if (buf != 0x90) {
        CITOS_ERR("reg0 expect 0x90, but get 0x%x.\n", buf);
        return false;
    }

    return true;
}

void desay_check_serializer_reg(i2c_adap_dev_t *adap, uint8_t addr) {
    int ret = 0;
    uint8_t buf = 0;

    ret = i2c_read_reg16(adap, addr, 0x700, &buf, sizeof(buf));
    if (ret < 0) {
        CITOS_ERR("read reg0x700 error, ret=%d.\n", ret);
    }
    if (buf != 0x8b) {
        CITOS_ERR("check serializer reg failed, reg0x700 expect 0x8b, but get 0x%x.\n", buf);
        extern void lvds_desay_serdes_1440x1920_lcd_panel_pre_power_on();
        lvds_desay_serdes_1440x1920_lcd_panel_pre_power_on();
        return;
    }

}

int desay_i2c_write(i2c_adap_dev_t *adap, uint8_t addr, uint8_t* buf, int len)
{
    int ret = 0;
    ret = i2c_write(adap, addr, buf, len);
    if (ret < 0) {
        CITOSPRINT_ONCE("%s: write error, ret=%d.\n", __func__, ret);
        lcdserver.tpI2cState = false;
        return ret;
    }else{
        lcdserver.tpI2cState = true;
    }

    return ret;
}

bool desay_check_link_state(i2c_adap_dev_t *adap, uint8_t addr) {
    int ret = 0;
    uint8_t buf = 0;
    static uint8_t val = 0;
    ret = i2c_read_reg16(adap, addr, 0x13, &buf, sizeof(buf));
    if (ret < 0) {
        CITOS_ERR("read reg13 error, ret=%d.\n", ret);
        return false;
    }
    if(val != buf){
    	CITOS_ERR("read reg13 changed, old = 0x%x,new = 0x%x\n", val,buf);
        val = buf;
    }
    if ((buf & 0x8) != 0x8) {
        CITOS_ERR("link failed, buf expect bit3=1, but get 0x%x.\n", buf);
        return false;
    }
    return true;
}

int desay_ts_send_i2c_data(uint8_t *data, size_t len) {
    if (lcdserver.dev == NULL) {
        return -1;
    }
    return desay_i2c_write(lcdserver.dev->cfg_ptr->i2c_dev, lcdserver.dev->cfg_ptr->i2c_addr, data, len);
}

int desay_send_tx_msg(desay_tx_msg msg) {
    if (!lcdserver.isConected) {
        CITOS_ERR("lcd is disconnect,can't send data");
        return 0;
    }
    msg.chk_sum = msg.cmd + msg.data[0] + msg.data[1] + msg.data[2] + msg.data[3] + msg.ext_len + 1;
    return desay_ts_send_i2c_data((uint8_t *) &msg, sizeof(msg));
}

int desay_send_tx_ext_msg(unsigned char cmd, const unsigned char *data, int len) {
    if (!lcdserver.isConected) {
        CITOS_ERR("lcd is disconnect,can't send data");
        return 0;
    }
    unsigned char send_buf[270] = {0};
    send_buf[0] = cmd;
    if (data == NULL || len <= 0) {
        CITOS_ERR("error data or len");
        return -1;
    }
    if (len < sizeof(send_buf) - 3) {
        if (!memcpy(&send_buf[1], data, len)) {
            CITOS_ERR("memcpy failed");
            return -1;
        }
    } else {
        CITOS_ERR("too long data");
        return -1;
    }
    send_buf[len + 1] = 0;
    send_buf[len + 2] = cmd + 0x01;
    for (int count = 0; count < len; count++) {
        send_buf[len + 2] += data[count];
    }
    return desay_ts_send_i2c_data((uint8_t *) &send_buf, len + 3);
}

int desay_startup() {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x80;
    txMsg.data[0] = 0x01;
    return desay_send_tx_msg(txMsg);
}

int desay_query_info() {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x81;
    return desay_send_tx_msg(txMsg);
}

int desay_set_timeout(uint8_t timeout) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x82;
    txMsg.data[0] = timeout;
    return desay_send_tx_msg(txMsg);
}

int desay_set_status(uint8_t powerStatus, uint16_t brightness, uint8_t blOnOff) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x83;
    txMsg.data[0] = powerStatus;
    txMsg.data[1] = brightness & 0xff;
    txMsg.data[2] = (brightness >> 8 & 0xff);
    txMsg.data[3] = blOnOff;
    return desay_send_tx_msg(txMsg);
}

int desay_set_lightperception(uint8_t luxOnOff) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x8C;
    txMsg.data[0] = luxOnOff;
    return desay_send_tx_msg(txMsg);
}

int desay_set_ill(uint8_t ill) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x85;
    txMsg.data[0] = ill;
    return desay_send_tx_msg(txMsg);
}

int desay_set_image_quality(uint8_t hue, uint8_t Saturation, uint8_t Brightness, uint8_t Sharpness) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x8D;
    txMsg.data[0] = hue;
    txMsg.data[1] = Saturation;
    txMsg.data[2] = Brightness;
    txMsg.data[3] = Sharpness;
    return desay_send_tx_msg(txMsg);
}

int desay_set_env_light(uint8_t undersun, uint8_t Contrast, uint8_t color, uint8_t eye) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x8E;
    txMsg.data[0] = undersun;
    txMsg.data[1] = Contrast;
    txMsg.data[2] = color;
    txMsg.data[3] = eye;
    return desay_send_tx_msg(txMsg);
}

int desay_set_FPGA(uint8_t fpgamode, uint8_t dayornight) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x8B;
    txMsg.data[0] = fpgamode;
    txMsg.data[1] = dayornight;
    return desay_send_tx_msg(txMsg);
}

int desay_set_password(uint8_t pf, uint8_t ps, uint8_t pt) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x87;
    txMsg.data[0] = pf;
    txMsg.data[1] = ps;
    txMsg.data[2] = pt;
    return desay_send_tx_msg(txMsg);
}

int desay_set_timestamp(const unsigned char *data) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x90;
    memcpy(txMsg.data, data, 4);
    return desay_send_tx_msg(txMsg);
}

int desay_set_log(uint8_t logOnOff){
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x94;
    txMsg.data[0] = logOnOff;
    return desay_send_tx_msg(txMsg);
}

int desay_set_eol_data(uint8_t eol_type, uint8_t data[4]) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = 0x91;
    memcpy(txMsg.data, data, sizeof(txMsg.data));
    return desay_send_tx_msg(txMsg);
}

int desay_set_fingerprint(uint8_t data[9]) {
    //TODO::数据长度为9?
    return 0;
}

int desay_send_tp_upgrade_ext(const unsigned char *data, unsigned char length) {
    return desay_send_tx_ext_msg(DESAYSV_HOST_CMDID_UPG_EXT, data, length);
}

int desay_send_tp_upgrade(const unsigned char *data, int len, unsigned char ext_len) {
    desay_tx_msg txMsg = {0};
    txMsg.cmd = DESAYSV_HOST_CMDID_UPG;
    if (len > sizeof(txMsg.data)) {
        return -1;
    }
    memcpy(txMsg.data, data, len);
    txMsg.ext_len = ext_len;
    return desay_send_tx_msg(txMsg);
}

void on_desay_lcd_callback(struct desaysv_stand_frame first_fm) {
    //TODO::将状态发给应用
    if (!lcdserver.isConected) {
        lcdserver.isConected = true;
    }
    switch (first_fm.cmd_id) {
        case DESAYSV_CMDID_TP:break;
        case DESAYSV_CMDID_TP_EXT:break;
        case DESAYSV_CMDID_UPG://触摸升级指令
            lcdserver.resp.tpUpgradeStatus = first_fm.data[0];
            lcdserver.resp.tpUpgradeResult = first_fm.data[1];
            msc_publish(MSC_TOPIC_LCD_RESPONSE, "tp_upg", first_fm.data, sizeof(first_fm.data), false);
            break;
        case DESAYSV_CMDID_UPG_PASSWD:
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "password",
                        first_fm.data,
                        sizeof(first_fm.data),
                        false);
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "upg_mcu",
                        first_fm.data,
                        sizeof(first_fm.data),
                        false);
            break;
        case DESAYSV_CMDID_STATUS://显示屏当前模式
            lcdserver.resp.lcdMode = first_fm.data[0];
            lcdserver.resp.logMode = first_fm.data[1];
            CITOS_ERR("get lcd mode:%d %d", lcdserver.resp.lcdMode, lcdserver.resp.logMode);
            msc_publish(MSC_TOPIC_LCD_RESPONSE, "lcd_mode", first_fm.data, sizeof(first_fm.data), true);
            if (first_fm.data[0] == LCD_MODE_APP) { //startup
                desay_startup();
            }
            osDelay(10);
            desay_set_status(lcdserver.setting.pwrOnOff,
                             lcdserver.setting.brightness,
                             lcdserver.setting.blOnOff);
            desay_query_info();
            break;
        case DESAYSV_CMDID_REQ_BACKLIGHT:
            desay_set_status(lcdserver.setting.pwrOnOff,
                             lcdserver.setting.brightness,
                             lcdserver.setting.blOnOff);
            break;
        case DESAYSV_CMDID_TIMEOUT:lcdserver.resp.lcdReposeTimeout = first_fm.data[0];
            break;
        case DESAYSV_CMDID_INFO://显示屏当期状态
            lcdserver.resp.lcdTemp = first_fm.data[0];
            lcdserver.resp.lcdPowerStatus = first_fm.data[1];
            lcdserver.resp.lcdBrightness = first_fm.data[2] | first_fm.data[3] << 8;
            lcdserver.resp.lcdBlOnOff = first_fm.data[4];
            msc_publish(MSC_TOPIC_LCD_RESPONSE, "lcd_status", first_fm.data, sizeof(first_fm.data), true);
            break;
        case DESAYSV_CMDID_PRODUCT://供应商信息
            msc_publish(MSC_TOPIC_LCD_RESPONSE, "product_id", first_fm.data, sizeof(first_fm.data), true);
            break;
        case DESAYSV_CMDID_VER://显示屏版本号
            msc_publish(MSC_TOPIC_LCD_RESPONSE, "Version", first_fm.data, sizeof(first_fm.data), true);
            break;
        case DESAYSV_CMDID_ERR://显示屏异常
            if (sizeof(first_fm.data) == 13) {
                CITOS_ERR("lcd_abnomal: %x %x %x %x %x %x %x %x %x %x %x %x %x",first_fm.data[0], first_fm.data[1],
                          first_fm.data[2], first_fm.data[3], first_fm.data[4], first_fm.data[5], first_fm.data[6],
                          first_fm.data[7], first_fm.data[8], first_fm.data[9], first_fm.data[10], first_fm.data[11],
                          first_fm.data[12]);
            }else{
            	CITOS_ERR("DESAYSV_CMDID_ERR data error");
            }
            msc_publish(MSC_TOPIC_LCD_RESPONSE, "lcd_abnomal", first_fm.data, sizeof(first_fm.data), true);
            uint8_t eol[66] = {0};
            eol[0] = 0x00;
            eol[1] = 0x00;
            eol[2] = 0x80;
            eol[3] = 0xf0;
            eol[4] = 0x01;
            eol[5] = 0x00;
            eol[6] = 0x01;
            eol[7] = 0x0f;
            //显示模组功能故障
            eol[8] = 0x01;
            eol[9] = first_fm.data[1];
            //显示屏触摸功能故障
            eol[10] = 0x01;
            eol[11] = first_fm.data[2];
            //显示屏背光模块功能故障
            eol[12] = 0x01;
            eol[13] = first_fm.data[3];
            //屏幕欠电压
            eol[24] = 0x01;
            eol[25] = first_fm.data[8];
            //屏幕过电压
            eol[26] = 0x01;
            eol[27] = first_fm.data[7];
            //显示屏整体功能故障
            eol[42] = 0x01;
            eol[43] = first_fm.data[0];
            //显示屏视频信号故障
            eol[44] = 0x01;
            eol[45] = first_fm.data[4];
            //显示屏未收到背光亮度值
            eol[46] = 0x01;
            eol[47] = first_fm.data[5];
            msc_publish(MSC_TOPIC_MCU_MREQUEST, "mp_lcd_dtc", eol, sizeof(eol), false);
            break;
        case DESAYSV_CMDID_TIMESTAMP:
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "timestamp",
                        first_fm.data,
                        sizeof(first_fm.data),
                        true);
            break;
        case DESAYSV_CMDID_LUX:
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "lux",
                        first_fm.data,
                        sizeof(first_fm.data),
                        true);
            break;
        case DESAYSV_CMDID_FPGA:
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "fpga",
                        first_fm.data,
                        sizeof(first_fm.data),
                        true);
            break;
        case DESAYSV_CMDID_FINGERPRINT:
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "fingerprint",
                        first_fm.data,
                        sizeof(first_fm.data),
                        true);
            break;
        case DESAYSV_CMDID_SERIALNUM:
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "serialnum",
                        first_fm.data,
                        sizeof(first_fm.data),
                        true);
            break;
        case DESAYSV_CMDID_PARTNUM:
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "partnum",
                        first_fm.data,
                        sizeof(first_fm.data),
                        true);
            break;
        case DESAYSV_CMDID_HEARTBEAT: //LCD周期性上报状态
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "heartbeat",
                        first_fm.data,
                        sizeof(first_fm.data),
                        true);
            break;
        case DESAYSV_HOST_CMDID_UPG_MCU:
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "upg_mcu",
                        first_fm.data,
                        sizeof(first_fm.data),
                        true);
            break;

        case DESAYSV_CMDID_LOG_STATUS:
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "log_status",
                        first_fm.data,
                        sizeof(first_fm.data),
                        true);
            if(first_fm.data[0] == 0)
            	break;
            uint8_t log_data[253];
            desay_i2c_read(lcdserver.dev->cfg_ptr->i2c_dev,
                           lcdserver.dev->cfg_ptr->i2c_addr,
                           log_data,
                           first_fm.ext_len);
            if (log_data[0] == DESAYSV_CMDID_LOG) {
                msc_publish(MSC_TOPIC_LCD_RESPONSE,
                            "log",
                            log_data + 1,
                            first_fm.ext_len - 3,
                            true);
            }
            break;
        default:CITOS_ERR("not impl cmd_id 0x%x", first_fm.cmd_id);
            break;
    }
}

static void msc_callback(const char *topic, const char *msgid, const unsigned char *data, const int len) {
    //TODO:处理应用的设置
    if (strcmp(topic, MSC_TOPIC_LCD_REQUEST) == 0) {
        CITOS_ERR("get msc %s\n", msgid);
        if (strcmp(msgid, "lcd_pwr") == 0) {
            if (len >= 1) {
                lcdserver.setting.pwrOnOff = data[0] == 0x01 ? true : false;

                desay_set_status(lcdserver.setting.pwrOnOff,
                                 lcdserver.setting.brightness,
                                 lcdserver.setting.blOnOff);
            } else {
                CITOS_ERR("err data for lcd_pwr");
            }
        } else if (strcmp(msgid, "lcd_br") == 0) {
            if (len >= 2) {
                lcdserver.setting.brightness = data[0] | data[1] << 8;
                uint8_t config[3] = {0x01, data[0], data[1]};
                usr_config_set(0, config, sizeof(config));
                desay_set_status(lcdserver.setting.pwrOnOff,
                                 lcdserver.setting.brightness,
                                 lcdserver.setting.blOnOff);
            } else {
                CITOS_ERR("err data for lcd_br");
            }
        } else if (strcmp(msgid, "lcd_bl") == 0) {
            if (len >= 1) {
                lcdserver.setting.blOnOff = data[0] == 0x01 ? true : false;
                desay_set_status(lcdserver.setting.pwrOnOff,
                                 lcdserver.setting.brightness,
                                 lcdserver.setting.blOnOff);
            } else {
                CITOS_ERR("err data for lcd_bl");
            }
        } else if (strcmp(msgid, "queryinfo") == 0) {
            desay_query_info();
        } else if (strcmp(msgid, "password") == 0) {
            if (len >= 3) {
                desay_set_password(data[0], data[1], data[2]);
            } else {
                CITOS_ERR("err data for %s", msgid);
            }
        } else if (strcmp(msgid, "tp_upg") == 0) {
            if (len == 5) {
                desay_send_tp_upgrade(data, len - 1, data[len - 1]);
            } else {
                CITOS_ERR("err data for %s", msgid);
            }
        } else if (strcmp(msgid, "tp_upg_ext") == 0) {
            if (len >= 2) {
                desay_send_tp_upgrade_ext(data, len);
            } else {
                CITOS_ERR("err data for %s", msgid);
            }
        } else if (strcmp(msgid, "mcu_upg") == 0) {
            if (len >= 1) {
                int32_t data_len = len;
                touch_device_control(lcdserver.dev, TS_CONTROL_UPDATE_FIRMWARE, (void *) data, &data_len);
            } else {
                CITOS_ERR("err data for %s", msgid);
            }
        } else if (strcmp(msgid, "timestamp") == 0) {
            if (len == 4) {
                desay_set_timestamp(data);
            } else {
                CITOS_ERR("err data for %s", msgid);
            }
        } else if (strcmp(msgid, "log_status") == 0) {
            if (len >= 1) {
                desay_set_log(data[0]);
            }
        } else if (strcmp(msgid, "lux") == 0) {
            if (len >= 1) {
                desay_set_lightperception(data[0]);
            } else {
                CITOS_ERR("err data for %s", msgid);
            }
        } else if (strcmp(msgid, "image_quality") == 0) {
            if (len >= 4) {
                desay_set_image_quality(data[0], data[1], data[2], data[3]);
            } else {
                CITOS_ERR("err data for %s", msgid);
            }
        } else if (strcmp(msgid, "env_light") == 0) {
            if (len >= 4) {
                desay_set_env_light(data[0], data[1], data[2], data[3]);
            } else {
                CITOS_ERR("err data for %s", msgid);
            }
        } else if (strcmp(msgid, "fpga") == 0) {
            if (len >= 1) {
                desay_set_FPGA(data[0], data[1]);
            } else {
                CITOS_ERR("err data for %s", msgid);
            }
        } else {
            CITOS_WARN("unknow msgid %s", msgid);
        }
    } else if (strcmp(topic, MSC_TOPIC_ANDROID_SYSTEM) == 0) {
        if (strcmp(msgid, "msc") == 0) {
            if (lcdserver.isConected) {
                msc_publish(MSC_TOPIC_LCD_RESPONSE, "lcd_mode", &lcdserver.resp.lcdMode, sizeof(lcdserver.resp.lcdMode), true);
                desay_query_info();
            }
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "lcd_connected",
                        (uint8_t *) &lcdserver.isConected,
                        sizeof(lcdserver.isConected),
                        true);
        }
    }
}

static void lcd_detect(void *arg) {
    touch_dev_t *dev = lcdserver.dev;
    i2c_adap_dev_t *adpt = dev->cfg_ptr->i2c_dev;
    uint8_t touch_addr = dev->cfg_ptr->i2c_addr;
    serdes_dev_t *serdes = *dev->cfg_ptr->serdes_config.serdes;
    uint8_t ser_addr = serdes->cfg_ptr->ser_addr;
    uint8_t des_addr = serdes->cfg_ptr->des_addr;
    bool tpI2cState = true;
    bool desI2cState = true;
    bool isConected = true;
    uint8_t eol[66] = {0};

    printf("%s: touch_addr=0x%x, ser_addr=0x%x, des_addr=0x%x\n", __func__, touch_addr, ser_addr, des_addr);
    while (1) {
        isConected = desay_check_link_state(adpt, ser_addr);
        desI2cState = desay_check_deser_reg(adpt, des_addr);
        desay_check_serializer_reg(adpt, ser_addr);
        if (isConected ^ lcdserver.isConected || tpI2cState^ lcdserver.tpI2cState || desI2cState ^lcdserver.desI2cState) {
            lcdserver.isConected = isConected;
            tpI2cState = lcdserver.tpI2cState;
            lcdserver.desI2cState = desI2cState;
            msc_publish(MSC_TOPIC_LCD_RESPONSE,
                        "lcd_connected",
                        (uint8_t *) &lcdserver.isConected,
                        sizeof(lcdserver.isConected),
                        true);
            eol[0] = 0x00;
            eol[1] = 0x00;
            eol[2] = 0x80;
            eol[3] = 0xf0;
            eol[4] = 0x01;
            eol[5] = 0x00;
            eol[6] = 0x01;
            eol[7] = 0x0f;
            //显示屏与主机IIC通信异常
            eol[14] = 0x01;
            eol[15] = lcdserver.tpI2cState ? 0x00 : 0x01;
            //SOC与前屏解串器通信出现错误
            eol[28] = 0x01;
            eol[29] = lcdserver.desI2cState ? 0x00 : 0x01;
            //与主机屏幕连接故障
            eol[58] = 0x01;
            eol[59] = lcdserver.isConected ? 0x00 : 0x01;
            msc_publish(MSC_TOPIC_MCU_MREQUEST, "mp_lcd_dtc", eol, sizeof(eol), false);
        }

        if (!isConected)
            osDelay(1000);
        else
            osDelay(300);
    }
    //osThreadExit();
    return;
}

void lcd_init(touch_dev_t *dev) {
    lcdserver.dev = dev;
    //TODO::从配置分区获取前屏设置参数
    osThreadAttr_t attr = {
        .name = "lcd_detect",
        .priority = osPriorityAboveNormal2,
//        .stack_size = 2048,
    };
    msc_subscribe(MSC_TOPIC_LCD_REQUEST, msc_callback);
    msc_subscribe(MSC_TOPIC_ANDROID_SYSTEM, msc_callback);
    lcdserver.thread = osThreadNew(lcd_detect, &lcdserver, &attr);
}

