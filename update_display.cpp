#define LOG_TAG  "lcdupgrade"

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "msc.h"
#include "log.h"
#include <pthread.h>
#include <errno.h>
#include <sys/system_properties.h>
#define QUEUE_CAPACITY 128

static char request_boot[7] = {0x5a, 0x05, 0x07, 0x02, 0x31, 0x01, 0x9a};
static char request_boot_ret[6] = {/*0x5A, */0x06, 0x07, 0x02, 0x31, 0x01, 0x9B};

static char key1[9] = {0x5a, 0x05, 0x07, 0x04, 0x31, 0x03, 0xA5, 0x5A, 0x9D};
static char key1_ret[6] = {/*0x5A, */0x06, 0x07, 0x02, 0x31, 0x03, 0x9D};

static char key2[9] = {0x5a, 0x05, 0x07, 0x04, 0x31, 0x04, 0xc3, 0x3C, 0x9E};
static char key2_ret[6] = {/*0x5A, */0x06, 0x07, 0x02, 0x31, 0x04, 0x9E};

static char erasure_app[11] = {0x5a, 0x05, 0x0B, 0x06, 0xB0, 0x01, 0x02, 0x00, 0x00, 0x00, 0x23};
static char erasure_app_ret[10] = {/*0x5A, */0x06, 0x0B, 0x06, 0xB0, 0x02, 0x01, 0x02, 0x00, 0x00, 0x26};

static char program[11] = {0x5a, 0x05, 0x0B, 0x06, 0xB0, 0x01, 0x03, 0x00, 0x00, 0x00, 0x24};
static char program_status[10] = {/*0x5A, */0x06, 0x0B, 0x06, 0xB0, 0x02, 0x01, 0x03, 0x00, 0x00, 0x27};

static char reset[11] = {0x5a, 0x05, 0x0B, 0x06, 0xB0, 0x01, 0x05, 0x00, 0x00, 0x00, 0x26};
static char reset_ret[10] = {/*0x5A, */0x06, 0x0b, 0x06, 0xb0, 0x02, 0x01, 0x05, 0x00, 0x00, 0x29};

static char bootloader_chk_status[11] = {0x5a, 0x05, 0x0B, 0x06, 0xB0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x22};
static char bootloader_ready[10] = {/*0x5A, */0x06, 0x0B, 0x06, 0xB0, 0x02, 0x01, 0x01, 0x00, 0x00, 0x25};
static char bootloader_program_complete[10] = {/*0x5A, */0x06, 0x0B, 0x06, 0xB0, 0x02, 0x01, 0x04, 0x00, 0x00, 0x28};

static char send_ret[10] = {/*0x5A, */0x06, 0x0B, 0x06, 0xB0, 0x03, 0x01, 0x00, 0x00, 0x00, 0x25};
static char fingerprint_ret[16] =
    {0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static char passwd_ret[15] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static unsigned char flash_checksum[11] = {0x5a, 0x05, 0x0B, 0x06, 0xB0, 0x01, 0x04, 0x56, 0x66, 0x00, 0x24};

typedef struct {
  unsigned char cmd;
  unsigned char data[4];
} tp_upg_msg;

typedef struct {
  unsigned char cmd;
  unsigned char data[4];
  unsigned char ext_len;
  unsigned char chk_sum;
} desay_tx_msg;

typedef struct {
  unsigned char cmd;
  unsigned char data[11];
  unsigned char ext_len;
  unsigned char chk_sum;
} desay_tx_msg_ext;

typedef struct {
  unsigned char data[QUEUE_CAPACITY];
  int front;
  int rear;
  int size;
  pthread_mutex_t mutex;
  pthread_cond_t cond;
} Queue;

void initQueue(Queue *q) {
    q->front = 0;
    q->rear = 0;
    q->size = 0;
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->cond, NULL);
}

bool isFull(Queue *q) {
    return q->size == QUEUE_CAPACITY;
}

bool isEmpty(Queue *q) {
    return q->size == 0;
}

bool enqueue(Queue *q, unsigned char value) {
    pthread_mutex_lock(&q->mutex);
    if (isFull(q)) {
        pthread_mutex_unlock(&q->mutex);
        return false;
    }
    q->data[q->rear] = value;
    q->rear = (q->rear + 1) % QUEUE_CAPACITY;
    q->size++;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
    return true;
}

bool dequeue(Queue *q, unsigned char *value) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 2;

    pthread_mutex_lock(&q->mutex);
    while (isEmpty(q)) {
        int res = pthread_cond_timedwait(&q->cond, &q->mutex, &ts);
        if (res == ETIMEDOUT) {
            pthread_mutex_unlock(&q->mutex);
            return false;
        }
    }
    *value = q->data[q->front];
    q->front = (q->front + 1) % QUEUE_CAPACITY;
    q->size--;
    pthread_mutex_unlock(&q->mutex);
    return true;
}

void printQueue(Queue *q) {
    printf("Queue content: ");
    for (int i = 0; i < q->size; i++) {
        printf("%02x ", q->data[(q->front + i) % QUEUE_CAPACITY]);
    }
    printf("\n");
}

#define MSC_TOPIC_LCD_RESPONSE  "lcd.resp"
#define MSC_TOPIC_LCD_REQUEST   "lcd.req"

#define MSC_TOPIC_LCD_UPG_REQUEST "lcd_upg.req"
#define MSC_TOPIC_LCD_UPG_RESPONSE "lcd_upg.resp"

#define MSC_TOPIC_LCD_ICM_UPG_REQUEST "lcd_icm_upg.req"
#define MSC_TOPIC_LCD_ICM_UPG_RESPONSE "lcd_icm_upg.resp"

#define MSC_TOPIC_LCD_ICM_REQUEST "lcd_icm.req"
#define MSC_TOPIC_LCD_ICM_RESPONSE "lcd_icm.resp"

#define ALLOW_POLLING_TIMES  5

#define I2C_ADDR_IVI_LCD 0x1a
#define I2C_ADDR_ICM_LCD 0x1b

struct {
  char tp_path[128];
  char mcu_path[128];
  char fingerprint[9];
  unsigned char upgrade_status[2];
  unsigned char tp_data_status[2];
  unsigned char password_status;
  bool flag;
  pthread_t upgrade_thread;
  char i2c_dev[32];
  char dev_addr;
  const char *topic;
  unsigned char reset_status[2];
} UPGRADE = {0};

AMsc *msc = nullptr;
static char procsuct_val[PROP_VALUE_MAX] = {0};
Queue q;

int dequeueData(Queue *q, char *buff, size_t buff_len) {
    for (size_t i = 0; i < buff_len; i++) {
        if (!dequeue(q, (unsigned char *) &buff[i])) {
            ALOGE("Queue is empty. Cannot dequeue more data.");
            return -1;
        }
    }
    return 0;
}

static int backlight_open() {
    uint8_t bl = 1;
    if (UPGRADE.dev_addr == I2C_ADDR_ICM_LCD) {
        AMsc_publish(msc, MSC_TOPIC_LCD_ICM_REQUEST, "lcd_bl", (unsigned char*)&bl,
                     sizeof(bl), 0);
    } else {
        AMsc_publish(msc, MSC_TOPIC_LCD_REQUEST, "lcd_bl", (unsigned char*)&bl,
                     sizeof(bl), 0);
    }
    return 0;
}

/**
 *i2c_open 打开i2c
 *成功返回0，失败返回-1
 */
static int i2c_open(const char *i2c_bus, uint8_t addr) {
    int fd = open(i2c_bus, O_RDWR/* | O_NOCTTY*/);
    if (fd < 0) {
        printf("i2c_open: can not open\n");
        return -1;
    }
    int ret = ioctl(fd, I2C_SLAVE_FORCE, addr);
    if (ret < 0) {
        printf("i2c_open: set address error 0x%02x\nret = %d\n",
               addr, ret);
        close(fd);
        return -1;
    }
    return fd;
}

/**
*compare_arr_val 比较两个数据的值
*相同返回0，不同返回-1
*/
static int compare_arr_val(const char *buf_orgin, const char *buf, int num) {
    for (int i = 0; i < num; i++) {
        if (buf_orgin[i] != buf[i]) {
            ALOGE("compare err : buf_orgin[%d] = %x, buf[%d] = %x", i, buf_orgin[i], i, buf[i]);
            return -1;
        }
    }
    return 0;
}

/**
 *change_ascil_to_hex 将ascil码转换为16进制
 *输入ascil码，十六进制数
 */
static char change_ascil_to_hex(char ch) {
    if ((ch >= 0x30) && (ch <= 0x39)) {
        return (ch - 0x30);
    }
    if ((ch >= 0x41) && (ch <= 0x5A)) {
        return (ch - 0x37);
    }
    return 0;
}

void notify_upgrade_status(unsigned char status, unsigned char progress, const char *topic, const char *msgid) {
    UPGRADE.upgrade_status[0] = status;
    UPGRADE.upgrade_status[1] = progress;
    AMsc_publish(msc,
                 topic, msgid,
                 UPGRADE.upgrade_status,
                 sizeof(UPGRADE.upgrade_status), true);
}

/**
 *prase_and_send_update_data 解析并发送升级文件
 */
static int prase_and_send_update_data(FILE *fd_file, int fd_i2c) {
    char buf[100];
    char cmd[25];
    unsigned char send_buf[100];
    int last_address = 0x0000C200;
    int last_address_len = 5;
    int address = 0;
    int address_len = 0;
    int free_address_len = 0;
    int ret;
    int file_size = 0;
    unsigned char process = 0;
    unsigned char last_process = 0;
    unsigned int count = 0;
    int checksum = 0;
    int count_fail;

    if (fseek(fd_file, 0, SEEK_END) < 0) {
        ALOGE("%s seek error SEEK_END ", UPGRADE.mcu_path);
        return -1;
    }
    if ((file_size = ftell(fd_file)) < 0) {
        ALOGE("%s tell error ", UPGRADE.mcu_path);
        return -1;
    }
    if (fseek(fd_file, 0, SEEK_SET) < 0) {
        ALOGE("%s seek error SEEK_SET ", UPGRADE.mcu_path);
        return -1;
    }
    if (file_size < 10) {
        ALOGE("file size is %d", file_size);
        return -1;
    }

    while (fgets(buf, 100, fd_file) != nullptr) {
        count += strlen(buf);
        process = count * 100 / file_size;
        send_buf[0] = buf[0];
        send_buf[1] = buf[1] - 0x30;
        for (int i = 2; i < ((strlen(buf) - 2) / 2 + 1); ++i) {
            char ch1 = change_ascil_to_hex(buf[i * 2 - 2]);
            char ch2 = change_ascil_to_hex(buf[i * 2 - 1]);
            send_buf[i] = (ch1 << 4) + ch2;
        }

#if 1
        count_fail = 0;
        while (true) {
            write(fd_i2c, send_buf, (strlen(buf) - 2) / 2 + 1);
            ret = dequeueData(&q, cmd, 13);
            if (ret == 0) {
                ret = compare_arr_val(cmd, send_ret, sizeof(send_ret));
                if (ret == 0) {
                    break;
                }
            }
            count_fail++;
            if (count_fail > ALLOW_POLLING_TIMES) {
                ALOGE("more than the polling times !");
                return -1;
            }
        }
#endif
        address = ((send_buf[3] << 24) | (send_buf[4] << 16) | (send_buf[5] << 8) | (send_buf[6]));
        if ((address >= 0x0000C200) && (buf[0] == 0x53) && ((buf[1] == 0x33)||(buf[1] == 0x31))) {
            address_len = send_buf[2];
            free_address_len = address - last_address - (last_address_len - 5);
            for (int i = 7; i < ((strlen(buf) - 2) / 2); ++i) {
                checksum += send_buf[i];
            }
            if (free_address_len > 0) {
                checksum += free_address_len * 255;
            }
            last_address = address;
            last_address_len = address_len;
        }

        if (last_process != process && process < 100) {
            last_process = process;
            notify_upgrade_status(0x00, process, UPGRADE.topic,"upgrade_mcu");
            ALOGI("process: %d", process);
        }
    }

    if (last_address < 0x0003FFFF) {
      checksum +=
          (0x0003FFFF - last_address - (last_address_len - 5) + 1) * 255;
    } else if (last_address < 0x0005FFFF) {
      checksum +=
          (0x0005FFFF - last_address - (last_address_len - 5) + 1) * 255;
    }
    else {
        // last_address >= 0x0005FFFF ,do nothing
    }

    flash_checksum[7] = checksum & 0xFF;
    flash_checksum[8] = (checksum >> 8) & 0xFF;
    flash_checksum[10] =
        (flash_checksum[0] + flash_checksum[1] + flash_checksum[2] + flash_checksum[3] + flash_checksum[4] +
            flash_checksum[5] + flash_checksum[6] + flash_checksum[7] + flash_checksum[8] +
            flash_checksum[9]) & 0xFF;
    ALOGI("write flash_checksum: %x %x %x %x %x %x %x %x %x %x %x %x\n", flash_checksum[0], flash_checksum[1],
          flash_checksum[2], flash_checksum[3], flash_checksum[4], flash_checksum[5], flash_checksum[6],
          flash_checksum[7], flash_checksum[8], flash_checksum[9], flash_checksum[10], checksum);
    return 0;
}

int tp_upgrade() {
    UPGRADE.password_status = 0xff;
    UPGRADE.tp_data_status[0] = 0xff;
    UPGRADE.tp_data_status[1] = 0xff;
    int line_num = 0;
    char file_buf[512] = {0};
    char password[128] = {0};
    unsigned char password_hex[3] = {0};
    long file_size;
    unsigned int count = 0;
    unsigned char progress, last_progress = 0;
    int ret = -1;
    FILE *fd_file = fopen(UPGRADE.tp_path, "r");
    tp_upg_msg txMsg;
    if (fd_file == nullptr) {
        ALOGE("open update file failed %s !", UPGRADE.tp_path);
        return -1;
    }
    if (fgets(password, sizeof(password), fd_file) == nullptr) {
        ALOGE("get password failed");
        notify_upgrade_status(0xFF, 0, UPGRADE.topic,"upgrade_tp");
        goto ERR;
    }
    for (int i = 0; i < (strlen(password) / 2 - 1); ++i) {
        char ch1 = change_ascil_to_hex(password[2 * i]);
        char ch2 = change_ascil_to_hex(password[2 * i + 1]);
        password_hex[i] = (ch1 << 4) + ch2;
    }

    if (strlen(password) < 6) {
        ALOGE("err password %s", password);
        notify_upgrade_status(0xFF, 0, UPGRADE.topic,"upgrade_tp");
        goto ERR;
    }

    for (int i = 0; i < 10; ++i) {
        AMsc_publish(msc, MSC_TOPIC_LCD_REQUEST, "password",
                     reinterpret_cast<const unsigned char *>(password_hex), sizeof(password_hex), false);
        usleep(100 * 1000);
        if (UPGRADE.password_status == 0xff) {
            if (i == 9) {
                ALOGE("no password callback");
                notify_upgrade_status(0xFF, 0, UPGRADE.topic,"upgrade_tp");
                goto ERR;
            }
            continue;
        } else if (UPGRADE.password_status != 0x00) {
            ALOGE("password error");
            notify_upgrade_status(0xFF, 0, UPGRADE.topic,"upgrade_tp");
            goto ERR;
        } else {
            break;
        }
    }
    if (fseek(fd_file, 0, SEEK_END) < 0) {
        ALOGE("%s seek error SEEK_END ", UPGRADE.tp_path);
        goto ERR;
    }
    if ((file_size = ftell(fd_file)) < 0) {
        ALOGE("%s tell error ", UPGRADE.tp_path);
        goto ERR;
    }
    if (fseek(fd_file, 0, SEEK_SET) < 0) {
        ALOGE("%s seek error SEEK_SET ", UPGRADE.tp_path);
        goto ERR;
    }
    usleep(20 * 1000);

    while (!feof(fd_file)) {
        fgets(file_buf, 300, fd_file);
        count += strlen(file_buf);
        progress = count * 100 / file_size;

        file_buf[strcspn(file_buf, "\n")] = '\0';
        file_buf[strcspn(file_buf, "\r")] = '\0';

        if (++line_num == 1)
            continue;

        //ASIC -> HEX
        unsigned long send_length = strlen(file_buf) % 2 ? (strlen(file_buf) + 1) / 2 : strlen(file_buf) / 2;
        unsigned char ext_buf[send_length];
        for (int i = 0; i < send_length; i++) {
            char ch1 = change_ascil_to_hex(file_buf[2 * i]);
            char ch2 = change_ascil_to_hex(file_buf[2 * i + 1]);
            ext_buf[i] = (ch1 << 4) + ch2;
        }

        txMsg = {0};
        txMsg.cmd = 0x01;
        txMsg.data[0] = line_num >> 8;
        txMsg.data[1] = line_num & 0xff;
        txMsg.data[3] = send_length + 3; //ext_len

        AMsc_publish(msc, MSC_TOPIC_LCD_REQUEST, "tp_upg", (unsigned char *) &txMsg, sizeof(txMsg), false);
        usleep(10 * 1000);
        AMsc_publish(msc,
                     MSC_TOPIC_LCD_REQUEST,
                     "tp_upg_ext",
                     (unsigned char *) &ext_buf,
                     (unsigned char) send_length,
                     false);
        usleep(100 * 1000);
        for (int i = 0; i < 20; ++i) {
            if (i == 19) {
                ALOGE("line %d waiting timeout", line_num);
                notify_upgrade_status(0xFF, 0, UPGRADE.topic,"upgrade_tp");
                goto ERR;
            }
            if (UPGRADE.tp_data_status[0] == 0x01 && UPGRADE.tp_data_status[1] == 0x00) {
                memset(UPGRADE.tp_data_status, 0xff, sizeof(UPGRADE.tp_data_status));
                break;
            } else if (UPGRADE.tp_data_status[0] == 0x02) {
                notify_upgrade_status(0xff, 0,UPGRADE.topic, "upgrade_tp");
                ALOGE("line %d upgrade failed,try again!", line_num);
                goto ERR;
            } else {
                usleep(200 * 1000);
            }
        }

        if (last_progress != progress && progress < 100) {
            last_progress = progress;
            notify_upgrade_status(0x00, progress, UPGRADE.topic,"upgrade_tp");
            ALOGI("tp_upg process: %d", progress);
        }
    }

    txMsg = {0};
    txMsg.cmd = 0x02;

    AMsc_publish(msc, MSC_TOPIC_LCD_REQUEST, "tp_upg", (unsigned char *) &txMsg, sizeof(txMsg), false);
    usleep(1000 * 1000);
    for (int i = 0; i < 5; ++i) {
        if (i == 4) {
            ALOGE("check upgrade waiting timeout");
            notify_upgrade_status(0xFF, 0, UPGRADE.topic,"upgrade_tp");
            goto ERR;
        }
        if (UPGRADE.tp_data_status[0] == 0x00 && UPGRADE.tp_data_status[1] == 0x00) {
            memset(UPGRADE.tp_data_status, 0xff, sizeof(UPGRADE.tp_data_status));
            break;
        } else if (UPGRADE.tp_data_status[0] == 0x02) {
            notify_upgrade_status(0xff, 0,UPGRADE.topic, "upgrade_tp");
            ALOGE("check upgrade failed,try again!");
            goto ERR;
        } else {
            usleep(500 * 1000);
        }
    }
    notify_upgrade_status(0x01, 100, UPGRADE.topic,"upgrade_tp");

    ret = 0;

ERR:
    if (fd_file != nullptr) {
        if(fclose(fd_file) != 0){
            ALOGE("close fd failed\n");
        }
    }
    return ret;
}

int mcu_upgrade(int fd_i2c) {
    int ret;
    int ret_t = -1;
    char buf[100];
    unsigned char passwd_buf[100];
    static int count_fail = 0;
    int file_size;
    int count = 0;
    desay_tx_msg tx_msg = {0};

    FILE *fd_file = fopen(UPGRADE.mcu_path, "r");
    if (fd_file == nullptr) {
        ALOGE("open update file failed %s", UPGRADE.mcu_path);
        return -1;
    }
    //3.查询当前Bootloader状态
    ALOGI("start bootloader_chk_status");
    memset(buf, 0, sizeof(buf));
    write(fd_i2c, bootloader_chk_status, sizeof(bootloader_chk_status));  //check bootloader status
    ret = dequeueData(&q, buf, 13);//check bootloader status
    if (ret == 0) {
        switch (buf[10]) {
            case 0x25:ALOGE("go to S0");
                goto checkS0;
                break;
            case 0x26:ALOGE("go to pro");
                goto programme;
                break;
            case 0x27:ALOGE("go to senddate");
                goto send_data;
                break;
            case 0x28:ALOGE("go to reset");
                goto reset;
                break;
            default:break;
        }
    }

    //4.申请Bootloader权限
    ALOGI("start check bootloader for request_boot");
    count_fail = 0;
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("BootLoader more than the polling times !");
            goto ERR;
        }
        write(fd_i2c, request_boot, sizeof(request_boot));
        ret = dequeueData(&q, buf, 13);
        if (ret == 0) {
            ret = compare_arr_val(buf, request_boot_ret, sizeof(request_boot_ret));
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }

    //5.Key1
    ALOGI("start check key1");
    count_fail = 0;
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("Key1 more than the polling times !");
            goto ERR;
        }
        write(fd_i2c, key1, sizeof(key1));
        ret = dequeueData(&q, buf, 13);
        if (ret == 0) {
            ret = compare_arr_val(buf, key1_ret, sizeof(key1_ret));
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }

    //6.Key2
    ALOGI("start check key2");
    count_fail = 0;
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("Key2 more than the polling times !");
            goto ERR;
        }
        write(fd_i2c, key2, sizeof(key2));
        ret = dequeueData(&q, buf, 13);
        if (ret == 0) {
            ret = compare_arr_val(buf, key2_ret, sizeof(key2_ret));
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }

    //7.1  等待3s，查询Bootloader_status
    ALOGI("start chk bootloader for bootloader_ready");
//    usleep(1 * 1000 * 1000);
    count_fail = 0;
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("bootloader_chk_status more than the polling times !");
            goto ERR;
        }
        write(fd_i2c, bootloader_chk_status, sizeof(bootloader_chk_status));  //check bootloader status
        ret = dequeueData(&q, buf, 13);
        if (ret == 0) {
            ret = compare_arr_val(buf, bootloader_ready, sizeof(bootloader_ready));
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }

    checkS0:
    //7.2 升级过程中开启背光
    backlight_open();
    usleep(10 * 1000);

    //7.3 发cmd 0x87 查询Passwd
    ALOGI("start chk passwd");
    if (fseek(fd_file, 0, SEEK_END) < 0) {
        ALOGE("%s seek error SEEK_END ", UPGRADE.mcu_path);
        goto ERR;
    }
    if ((file_size = ftell(fd_file)) < 0) {
        ALOGE("%s tell error ", UPGRADE.mcu_path);
        goto ERR;
    }
    if (fseek(fd_file, 0, SEEK_SET) < 0) {
        ALOGE("%s seek error SEEK_SET ", UPGRADE.mcu_path);
        goto ERR;
    }
    if (file_size < 10) {
        ALOGE("file size is %d", file_size);
        goto ERR;
    }
    memset(buf, 0, sizeof(buf));
    if (fgets(buf, sizeof(buf), fd_file) == nullptr){
        ALOGE("fgets fail\n");
        goto ERR;
    }
    passwd_buf[0] = buf[0];
    passwd_buf[1] = buf[1] - 0x30;

    for (int i = 2; i < (strlen(buf) / 2 + 1); i++) {
        char ch1 = change_ascil_to_hex(buf[i * 2 - 2]);
        char ch2 = change_ascil_to_hex(buf[i * 2 - 1]);
        passwd_buf[i] = (ch1 << 4) + ch2;
    }

    count_fail = 0;
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("read passwd more than the polling times !");
            goto ERR;
        }
        tx_msg.cmd = 0x87;
        tx_msg.data[0] = passwd_buf[7];
        tx_msg.data[1] = passwd_buf[8];
        tx_msg.data[2] = passwd_buf[9];
        tx_msg.chk_sum =
            tx_msg.cmd + tx_msg.data[0] + tx_msg.data[1] + tx_msg.data[2] + tx_msg.data[3] + tx_msg.ext_len + 1;
        write(fd_i2c, (uint8_t *) &tx_msg, sizeof(tx_msg));
        ret = dequeueData(&q, buf, 13);
        if (ret == 0) {
            ret = compare_arr_val(buf, passwd_ret, 2);
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }

    //8. era_app
    ALOGI("start era_app");
    count_fail = 0;
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("rea_app more than the polling times !");
            goto ERR;
        }
        write(fd_i2c, erasure_app, sizeof(erasure_app));
        usleep(10 * 1000);
        write(fd_i2c, bootloader_chk_status, sizeof(bootloader_chk_status));  //check bootloader status
        ret = dequeueData(&q, buf, 13); //check bootloader status
        if (ret == 0) {
            ret = compare_arr_val(buf, erasure_app_ret, sizeof(erasure_app_ret));
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }

    //9.programming
    ALOGI("start programming");
    programme:
    count_fail = 0;
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("programme more than the polling times !");
            goto ERR;
        }
        write(fd_i2c, program, sizeof(program));
        usleep(100 * 1000);
        write(fd_i2c, bootloader_chk_status, sizeof(bootloader_chk_status));
        ret = dequeueData(&q, buf, 13);
        if (ret == 0) {
            ret = compare_arr_val(buf, program_status, sizeof(program_status));
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }

    //10.逐行发送烧录文件行
    ALOGI("start send file");
    send_data:
    if (prase_and_send_update_data(fd_file, fd_i2c) < 0) {
        ALOGE("prase_and_send_update_data failed !");
        goto ERR;
    }

    //11.flash checksum
    ALOGI("start flash checksum");
    count_fail = 0;
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("flash checksum more than the polling times !");
            goto ERR;
        }

        write(fd_i2c, flash_checksum, sizeof(flash_checksum));
        usleep(100 * 1000);
        write(fd_i2c, bootloader_chk_status, sizeof(bootloader_chk_status));
        ret = dequeueData(&q, buf, 13);
        if (ret == 0) {
            ret = compare_arr_val(buf, bootloader_program_complete, sizeof(bootloader_program_complete));
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }

    //12. reset
    ALOGI("start reset");
    reset:
    count_fail = 0;
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("reset more than the polling times !");
            goto ERR;
        }
        write(fd_i2c, reset, sizeof(reset));
        usleep(100 * 1000);
        write(fd_i2c, bootloader_chk_status, sizeof(bootloader_chk_status));  //check bootloader status
        ret = dequeueData(&q, buf, 13); //check bootloader status
        if (ret == 0) {
            ret = compare_arr_val(buf, reset_ret, sizeof(reset_ret));
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }
#if 0
    //14.R156_fingerprint
    ALOGE("fingerprint start\n");
    count_fail = 0;
    desay_tx_msg_ext txMsg_ext = {0};
    txMsg_ext.cmd = 0x93;

    memcpy(txMsg_ext.data,UPGRADE.fingerprint,9);

    txMsg_ext.chk_sum =
        txMsg_ext.cmd + txMsg_ext.data[0] + txMsg_ext.data[1] + txMsg_ext.data[2] + txMsg_ext.data[3]
            + txMsg_ext.data[4] + txMsg_ext.data[5] +
            txMsg_ext.data[6] + txMsg_ext.data[7] + txMsg_ext.data[8] + txMsg_ext.ext_len + 1;

    ALOGE("fingerprint %02x %02x %02x %02x %02x %02x %02x %02x %02x",txMsg_ext.data[0],txMsg_ext.data[1],txMsg_ext.data[2],txMsg_ext.data[3],txMsg_ext.data[4],txMsg_ext.data[5],txMsg_ext.data[6],txMsg_ext.data[7],txMsg_ext.data[8]);
    while (true) {
        if (count_fail > ALLOW_POLLING_TIMES) {
            ALOGE("fingerprint more than the polling times !");
            goto ERR;
        }
        write(fd_i2c, (uint8_t *) &txMsg_ext, sizeof(txMsg_ext));
        usleep(100 * 1000);
        ret = dequeueData(&q, buf, 13);
        if (ret > 0) {
            ret = compare_arr_val(buf, fingerprint_ret, sizeof(fingerprint_ret));
            if (ret == 0) {
                break;
            }
        }
        count_fail++;
    }
#endif
    ret_t = 0;

ERR:
    if (fd_file != nullptr) {
        if(fclose(fd_file) != 0){
            ALOGE("close fd failed\n");
        }
    }
    return ret_t;

}

void *tp_upgrade_thread(void *arg) {
    if (tp_upgrade()) {
        ALOGE("tp_upgrade failed!");
    } else {
        ALOGI("tp_upgrade success!");
    }
    UPGRADE.upgrade_thread = 0;
    return nullptr;
}

void *mcu_upgrade_thread(void *arg) {
    uint8_t mcu_upg_status = 1;

    AMsc_publish(msc, MSC_TOPIC_LCD_REQUEST, "mcu_upg", (unsigned char *) &mcu_upg_status, sizeof(mcu_upg_status), 0);
    int fd = i2c_open(UPGRADE.i2c_dev, UPGRADE.dev_addr);
    if (fd < 0) {
        ALOGE("i2c_open failed!");
        return nullptr;
    }
    initQueue(&q);

    int ret = mcu_upgrade(fd);
    if (ret < 0) {
        ALOGE("mcu_upgrade failed!");
        backlight_open();
        notify_upgrade_status(0xff, 0, UPGRADE.topic, "upgrade_mcu");
    } else {
        ALOGI("mcu_upgrade success!");
        notify_upgrade_status(0x01, 100, UPGRADE.topic, "upgrade_mcu");
    }
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
    mcu_upg_status = 0;
    AMsc_publish(msc, MSC_TOPIC_LCD_REQUEST, "mcu_upg", (unsigned char *) &mcu_upg_status, sizeof(mcu_upg_status), 0);
    UPGRADE.upgrade_thread = 0;
    return nullptr;
}

void MscCallback(const char *topic, const char *msgid, const unsigned char *data, const int len) {
    if (strcmp(MSC_TOPIC_LCD_RESPONSE, topic) == 0 || strcmp(MSC_TOPIC_LCD_ICM_RESPONSE, topic) == 0) {
        if (strcmp("password", msgid) == 0) {
            if (len > 0) {
                UPGRADE.password_status = data[0];
            }
        } else if (strcmp("tp_upg", msgid) == 0) {
            if (len > 1) {
                UPGRADE.tp_data_status[0] = data[0];
                UPGRADE.tp_data_status[1] = data[1];
            }
        } else if (strcmp("upg_mcu", msgid) == 0) {
            for (int i = 0; i < len; ++i) {
                if (!enqueue(&q, data[i])) {
                    ALOGE("Queue is full. Cannot enqueue more data");
                    break;
                }
            }
        } else if (strcmp("heartbeat", msgid) == 0) {
            ALOGE("get lcd_heartbeat: LVDS:%X,BL:%X,TFT:%X,BAT:%X,TCH:%04X",data[0],data[1],data[2],data[3],(data[12] << 8) | data[11]);
        } else if (strcmp("touch_status", msgid) == 0) {
            ALOGE("touch status: ID[%d]%s x:%d y:%d", data[0], data[1] == 0xc0 ? "DOWN" : "UP  ", data[2] << 8 | data[3],
                  data[4] << 8 | data[5]);
        }
    } else if (strcmp(MSC_TOPIC_LCD_UPG_REQUEST, topic) == 0 || strcmp(MSC_TOPIC_LCD_ICM_UPG_REQUEST, topic) == 0) {
        if (strcmp(MSC_TOPIC_LCD_UPG_REQUEST, topic) == 0) {
            UPGRADE.topic = MSC_TOPIC_LCD_UPG_RESPONSE;
        }
        else if (strcmp(MSC_TOPIC_LCD_ICM_UPG_REQUEST, topic) == 0) {
            UPGRADE.topic = MSC_TOPIC_LCD_ICM_UPG_RESPONSE;
        }
        if (strcmp("upgrade_tp", msgid) == 0) {
            if (UPGRADE.upgrade_thread == 0) {
                if (len < sizeof(UPGRADE.tp_path)) {
                    memcpy(UPGRADE.tp_path, data, len);
                    UPGRADE.tp_path[sizeof(UPGRADE.tp_path) - 1] = '\0';
                    if (pthread_create(&UPGRADE.upgrade_thread, nullptr, tp_upgrade_thread, nullptr) != 0) {
                        notify_upgrade_status(0xFF, 0,UPGRADE.topic, "upgrade_tp");
                    }
                } else {
                    ALOGE("tp upgrade path is too long\n");
                    notify_upgrade_status(0xFF, 0,UPGRADE.topic, "upgrade_tp");
                }
            } else {
                ALOGE("there's one upgrade process existing\n");
            }
        } else if (strcmp("upgrade_mcu", msgid) == 0) {
            if (UPGRADE.upgrade_thread == 0) {
                if (len > sizeof(UPGRADE.fingerprint) && len < sizeof(UPGRADE.mcu_path) + sizeof(UPGRADE.fingerprint)) {
                    memcpy(UPGRADE.fingerprint, data, sizeof(UPGRADE.fingerprint));
                    memcpy(UPGRADE.mcu_path, data + sizeof(UPGRADE.fingerprint), len - sizeof(UPGRADE.fingerprint));
                    UPGRADE.mcu_path[sizeof(UPGRADE.mcu_path) - 1] = '\0';
                    if (strcmp(procsuct_val,"x9h_ms") == 0) {
                        UPGRADE.dev_addr = I2C_ADDR_IVI_LCD;
                        strcpy(UPGRADE.i2c_dev, "/dev/i2c-29");
                    } else if (strcmp(procsuct_val,"x9sp_ms") == 0) {
                        if (strcmp(MSC_TOPIC_LCD_UPG_RESPONSE, UPGRADE.topic)== 0) {
                            UPGRADE.dev_addr = I2C_ADDR_IVI_LCD;
                        }
                        else if (strcmp(MSC_TOPIC_LCD_ICM_UPG_RESPONSE, UPGRADE.topic)== 0) {
                            UPGRADE.dev_addr = I2C_ADDR_ICM_LCD;
                        }
                        strcpy(UPGRADE.i2c_dev, "/dev/i2c-21");
                    }
                    if (pthread_create(&UPGRADE.upgrade_thread, nullptr, mcu_upgrade_thread, nullptr) != 0) {
                        notify_upgrade_status(0xFF, 0,UPGRADE.topic, "upgrade_mcu");
                    }
                } else {
                    ALOGE("mcu upgrade path is too long\n");
                    notify_upgrade_status(0xFF, 0, UPGRADE.topic,"upgrade_mcu");
                }
            } else {
                ALOGE("there's one upgrade process existing\n");
            }
        }  else if (strcmp("mcu_reset", msgid) == 0) {
            if (UPGRADE.upgrade_status[0] != 0xFF) {
                ALOGE("upgrade_status is not failed");
                return;
            }
            ALOGE("start lcd reset");
            char buf[100];
            int fd = i2c_open(UPGRADE.i2c_dev, UPGRADE.dev_addr);
            if (fd < 0) {
                ALOGE("fd failed!");
                return;
            }
            write(fd, reset, sizeof(reset));
            for (int i = 0; i <= 3; i++) {
                usleep(100 * 1000);
                write(fd, bootloader_chk_status, sizeof(bootloader_chk_status));
                int ret = dequeueData(&q, buf, 13);
                if (ret == 0) {
                    ret = compare_arr_val(buf, reset_ret, sizeof(reset_ret));
                    if (ret != 0) {
                        UPGRADE.reset_status[0] = 0x01;
                        ALOGE("mcu reset failed");
                    } else {
                        UPGRADE.reset_status[0] = 0x00;
                        ALOGE("mcu reset success");
                        break;
                    }
                }
            }
            AMsc_publish(msc, MSC_TOPIC_LCD_UPG_RESPONSE, "mcu_rest", UPGRADE.reset_status, sizeof(UPGRADE.reset_status), true);
            if (fd >= 0) {
                close(fd);
                fd = -1;
            }
        }
    }
}

bool MscInit();
void OnServiceDied(void *cookie) {
    if (msc != nullptr) {
        AMsc_delete(msc);
        msc = nullptr;
        usleep(1000 * 500);
        while (!MscInit()) {
            ALOGE("Msc init failed while died");
            usleep(1000 * 500);
        }
    }
}

bool MscInit() {
    msc = AMsc_new(OnServiceDied);
    if (msc == nullptr) {
        return false;
    }
    const char *topics[]{
        MSC_TOPIC_LCD_RESPONSE,        MSC_TOPIC_LCD_UPG_REQUEST,
        MSC_TOPIC_LCD_ICM_UPG_REQUEST, MSC_TOPIC_LCD_ICM_RESPONSE};
    if (AMsc_subscribe(msc, (const char **) (topics), sizeof(topics) / sizeof(topics[0]), MscCallback) != 0) {
        return false;
    }
    return true;
}

int main(int argc, char *argv[]) {
    while (!MscInit()) {
        ALOGE("Msc init failed while died");
        usleep(1000 * 500);
        return -1;
    }

    const prop_info *pi = nullptr;
    for (int i = 0; i < 5; i++) {
        if ((pi = __system_property_find("ro.build.product")) != nullptr) {
            __system_property_read(pi, nullptr, procsuct_val);
            break;
        } else {
            ALOGE("read product val failed");
            usleep(10 * 1000);
        }
    }

    while (true) {
        sleep(100);
    }
}
