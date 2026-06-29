#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 返回值约定：0 = 成功，负数 = 错误码 */
typedef enum {
    RH56_OK              = 0,
    RH56_ERR_PARAM       = -1,
    RH56_ERR_UART        = -2,
    RH56_ERR_TIMEOUT     = -3,
    RH56_ERR_FRAME       = -4,
    RH56_ERR_CHECKSUM    = -5,
    RH56_ERR_RESP_CMD    = -6,
} rh56_result_t;

/* 平台抽象：由自研 OS 提供/实现的 UART 和延时函数
 *
 * write_fn : 写串口，返回写入字节数或负数错误码
 * read_fn  : 读串口，带超时（毫秒），返回读到的字节数或负数错误码
 * sleep_ms : 任务/线程休眠一段时间（毫秒）
 */
typedef int  (*rh56_uart_write_fn)(const uint8_t *buf, size_t len);
typedef int  (*rh56_uart_read_fn)(uint8_t *buf, size_t maxlen, uint32_t timeout_ms);
typedef void (*rh56_sleep_ms_fn)(uint32_t ms);

/* RH56 配置结构，由上层在 init 时提供 */
typedef struct {
    uint8_t             hand_id;      /* 灵巧手 ID，一般是 1 */
    rh56_uart_write_fn  uart_write;   /* 串口写函数 */
    rh56_uart_read_fn   uart_read;    /* 串口读函数（带超时） */
    rh56_sleep_ms_fn    sleep_ms;     /* 延时函数（可用于简单时序控制） */
} rh56_config_t;

/* 初始化：保存配置，后续 API 将使用这些函数操作串口
 *
 * 注意：串口本身的波特率/数据位/停止位配置应由 OS 其他地方完成
 */
rh56_result_t rh56_init(const rh56_config_t *cfg);

rh56_result_t rh56_clear_error(void);
rh56_result_t rh56_save_params(void);
rh56_result_t rh56_force_calibrate(void);
rh56_result_t rh56_set_hand_id(uint8_t new_id, uint8_t persist);
rh56_result_t rh56_set_baudrate(uint8_t redu_ratio, uint8_t persist);
rh56_result_t rh56_reset_factory(void);

rh56_result_t rh56_set_angles(const int16_t angles[6]);
rh56_result_t rh56_get_angles(int16_t angles[6]);
rh56_result_t rh56_set_speeds(const uint16_t speeds[6]);
rh56_result_t rh56_set_forces(const uint16_t forces[6]);
rh56_result_t rh56_set_positions(const int16_t positions[6]);
rh56_result_t rh56_get_positions(int16_t positions[6]);
rh56_result_t rh56_set_current_limits(const uint16_t limits[6]);
rh56_result_t rh56_get_currents(uint16_t currents[6]);
rh56_result_t rh56_set_default_speeds(const uint16_t speeds[6]);
rh56_result_t rh56_set_default_forces(const uint16_t forces[6]);
rh56_result_t rh56_get_forces(uint16_t forces[6]);
rh56_result_t rh56_get_temps(uint8_t temps[6]);
rh56_result_t rh56_get_status(uint8_t status[6]);
rh56_result_t rh56_get_errors(uint8_t errors[6]);

rh56_result_t rh56_run_action_sequence(uint16_t index);
rh56_result_t rh56_save_action_sequence(uint8_t index,
                                        const uint16_t *steps_19short,
                                        uint8_t step_count);

#ifdef __cplusplus
}
#endif
