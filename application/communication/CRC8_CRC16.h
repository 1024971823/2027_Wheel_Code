/**
 * @file    CRC8_CRC16.h
 * @brief   CRC8/CRC16校验算法，用于USB通信帧校验
 *          CRC8: x^8 + x^5 + x^4 + 1 (多项式0x31)
 *          CRC16: x^16 + x^15 + x^2 + 1 (多项式0x8005)
 * @note    从DJI RoboMaster协议标准CRC算法移植
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2026-04-06      Copilot         1. 新建文件，适配达妙MC02
 */

#ifndef CRC8_CRC16_H
#define CRC8_CRC16_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t  get_CRC8_check_sum(const uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t verify_CRC8_check_sum(const uint8_t *pchMessage, uint32_t dwLength);
void     append_CRC8_check_sum(uint8_t *pchMessage, uint32_t dwLength);

uint16_t get_CRC16_check_sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t verify_CRC16_check_sum(const uint8_t *pchMessage, uint32_t dwLength);
void     append_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

#ifdef __cplusplus
}
#endif

#endif /* CRC8_CRC16_H */
