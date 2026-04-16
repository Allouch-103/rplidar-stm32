#ifndef INC_RPLIDAR_H_
#define INC_RPLIDAR_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

// ─── Protocol Constants ────────────────────────────────────────────────────
// Every command sent to the LiDAR starts with these two bytes
#define RPLIDAR_CMD_START_FLAG    0xA5  // always first byte of any command
#define RPLIDAR_CMD_SCAN          0x20  // start continuous scan
#define RPLIDAR_CMD_STOP          0x25  // stop scan
#define RPLIDAR_CMD_RESET         0x40  // soft reset
#define RPLIDAR_CMD_GET_HEALTH    0x52  // request health status

// ─── Health Status Values ──────────────────────────────────────────────────
#define RPLIDAR_HEALTH_GOOD       0x00  // everything ok
#define RPLIDAR_HEALTH_WARNING    0x01  // working but degraded
#define RPLIDAR_HEALTH_ERROR      0x02  // hardware fault

// ─── Packet Size ──────────────────────────────────────────────────────────
// Every scan data packet from LiDAR is exactly 5 bytes
#define RPLIDAR_PACKET_SIZE       5

// ─── Scan Buffer ──────────────────────────────────────────────────────────
// One full revolution = ~360 to 720 points depending on speed
// We allocate 720 to be safe
#define RPLIDAR_MAX_POINTS        720

// ─── Data Structure ───────────────────────────────────────────────────────
// One point in the 2D scan
typedef struct {
    float   angle_deg;    // 0.0 to 359.99 degrees
    float   distance_mm;  // distance in millimeters (0 = invalid)
    uint8_t quality;      // signal strength 0-63 (0 = invalid/noise)
    uint8_t start_bit;    // 1 = this point starts a new revolution
} RPLidarPoint;

// ─── External Variables ───────────────────────────────────────────────────
// These are declared in rplidar.c, accessible from main.c
extern RPLidarPoint rplidar_scan[RPLIDAR_MAX_POINTS]; // full scan buffer
extern uint16_t     rplidar_scan_count;               // points in current scan
extern uint8_t      rplidar_scan_ready;               // 1 = new full scan ready

// ─── Function Declarations ────────────────────────────────────────────────
void    rplidar_init(UART_HandleTypeDef *lidar_uart, GPIO_TypeDef *motor_port, uint16_t motor_pin);
uint8_t rplidar_get_health(void);
void    rplidar_start_scan(void);
void    rplidar_stop(void);
void    rplidar_feed_byte(uint8_t byte); // call this from UART interrupt

#endif /* INC_RPLIDAR_H_ */
