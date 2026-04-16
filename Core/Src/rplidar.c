#include "rplidar.h"
#include <string.h>  // for memset

// ─── Private variables (only used inside this file) ───────────────────────
static UART_HandleTypeDef *_uart;         // pointer to huart1
static GPIO_TypeDef       *_motor_port;   // pointer to PB0 port
static uint16_t            _motor_pin;    // PB0 pin number

// Temporary 5-byte buffer to assemble one incoming packet
static uint8_t  _pkt[RPLIDAR_PACKET_SIZE];
static uint8_t  _pkt_idx = 0;  // how many bytes received so far

// ─── Public variables (declared extern in .h) ─────────────────────────────
RPLidarPoint rplidar_scan[RPLIDAR_MAX_POINTS];
uint16_t     rplidar_scan_count = 0;
uint8_t      rplidar_scan_ready = 0;

// ─── Private helper: send a command to the LiDAR ──────────────────────────
// The LiDAR protocol says every command is:
// [0xA5] [CMD_BYTE]
// That's it — just 2 bytes, no payload for basic commands
static void _send_cmd(uint8_t cmd) {
    uint8_t pkt[2] = {RPLIDAR_CMD_START_FLAG, cmd};
    HAL_UART_Transmit(_uart, pkt, 2, 100);
    // 100ms timeout — more than enough for 2 bytes at 115200
}

// ─── rplidar_init ─────────────────────────────────────────────────────────
// Call this once in main() before anything else
// Stores references to the UART and motor GPIO so the driver can use them
void rplidar_init(UART_HandleTypeDef *lidar_uart,
                  GPIO_TypeDef *motor_port,
                  uint16_t motor_pin)
{
    _uart       = lidar_uart;
    _motor_port = motor_port;
    _motor_pin  = motor_pin;

    // Reset packet assembler state
    _pkt_idx = 0;
    memset(_pkt, 0, sizeof(_pkt));

    // Reset scan buffer
    memset(rplidar_scan, 0, sizeof(rplidar_scan));
    rplidar_scan_count = 0;
    rplidar_scan_ready = 0;
}

// ─── rplidar_get_health ───────────────────────────────────────────────────
// Sends the GET_HEALTH command and reads back 3 data bytes
// Returns: 0=good, 1=warning, 2=error
// Always call this before starting a scan
uint8_t rplidar_get_health(void)
{
    // make sure no streaming RX is ongoing
    HAL_UART_AbortReceive(_uart);
    _send_cmd(RPLIDAR_CMD_STOP);
    HAL_Delay(30);

    _send_cmd(RPLIDAR_CMD_GET_HEALTH);

    uint8_t d[7] = {0};
    if (HAL_UART_Receive(_uart, d, 7, 500) != HAL_OK) return 0xFF;
    if (d[0] != 0xA5 || d[1] != 0x5A) return 0xFE; // bad descriptor

    uint8_t data[3] = {0};
    if (HAL_UART_Receive(_uart, data, 3, 500) != HAL_OK) return 0xFF;

    if (data[0] > 2) return 0xFD; // invalid health byte
    return data[0];
}

// ─── rplidar_start_scan ───────────────────────────────────────────────────
// Turns motor on and sends SCAN command
// After this, the LiDAR streams 5-byte packets continuously
void rplidar_start_scan(void) {

    // 1. Make sure motor is ON
    HAL_GPIO_WritePin(_motor_port, _motor_pin, GPIO_PIN_SET);

    // 2. Wait for motor to reach stable speed
    HAL_Delay(500);

    // 3. Send SCAN command
    _send_cmd(RPLIDAR_CMD_SCAN);

    // 4. Consume the 7-byte response descriptor
    // (same as health check — LiDAR sends descriptor before data stream)
    uint8_t descriptor[7];
    HAL_UART_Receive(_uart, descriptor, 7, 500);

    // 5. Reset packet state — ready to receive scan data
    _pkt_idx = 0;
}

// ─── rplidar_stop ─────────────────────────────────────────────────────────
// Stops the scan and turns off the motor
void rplidar_stop(void) {
    _send_cmd(RPLIDAR_CMD_STOP);
    HAL_Delay(10);
    HAL_GPIO_WritePin(_motor_port, _motor_pin, GPIO_PIN_RESET);
}

// ─── rplidar_feed_byte ────────────────────────────────────────────────────
// THE CORE PARSER — call this every time a byte arrives from UART
// It assembles bytes into 5-byte packets and decodes them
//
// Packet structure (5 bytes):
// Byte 0: [Quality:6][S_bar:1][S:1]
// Byte 1: [Angle_q6 LSB:7][Check_bit:1]
// Byte 2: [Angle_q6 MSB:8]
// Byte 3: [Distance_q2 LSB:8]
// Byte 4: [Distance_q2 MSB:8]
void rplidar_feed_byte(uint8_t byte) {
    _pkt[_pkt_idx++] = byte;
    if (_pkt_idx < RPLIDAR_PACKET_SIZE) return;
    _pkt_idx = 0;

    uint8_t S     = _pkt[0] & 0x01;
    uint8_t S_bar = (_pkt[0] >> 1) & 0x01;
    if (S == S_bar) return;
    if ((_pkt[1] & 0x01) != 1) return;

    uint8_t quality = _pkt[0] >> 2;
    uint16_t angle_q6 = (((uint16_t)_pkt[2] << 7) | (_pkt[1] >> 1));
    uint16_t dist_q2  = (((uint16_t)_pkt[4] << 8) | _pkt[3]);

    float angle = angle_q6 / 64.0f;
    float distance = dist_q2 / 4.0f;

    // If this packet starts NEW revolution, close previous one first
    if (S == 1 && rplidar_scan_count > 0) {
        rplidar_scan_ready = 1;
        // do NOT reset yet if main still reading old buffer in same cycle
        // simplest single-buffer method:
        rplidar_scan_count = 0;
    }

    if (rplidar_scan_count < RPLIDAR_MAX_POINTS) {
        rplidar_scan[rplidar_scan_count].quality = quality;
        rplidar_scan[rplidar_scan_count].start_bit = S;
        rplidar_scan[rplidar_scan_count].angle_deg = angle;
        rplidar_scan[rplidar_scan_count].distance_mm = distance;
        rplidar_scan_count++;
    }
}
