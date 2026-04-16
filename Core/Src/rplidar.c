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
uint8_t rplidar_get_health(void) {

    // 1. Send the command
    _send_cmd(RPLIDAR_CMD_GET_HEALTH);

    // 2. Read the 7-byte response descriptor
    // The LiDAR always sends a descriptor before any data
    // We don't need its content, just consume it
    uint8_t descriptor[7];
    HAL_UART_Receive(_uart, descriptor, 7, 500);

    // 3. Read the 3 actual health data bytes
    // Byte 0 = status (0=good, 1=warn, 2=error)
    // Byte 1,2 = error code (only meaningful if status != 0)
    uint8_t data[3];
    HAL_UART_Receive(_uart, data, 3, 500);

    return data[0]; // return status byte
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

    // Accumulate bytes into our 5-byte packet buffer
    _pkt[_pkt_idx++] = byte;

    // Not enough bytes yet — wait for more
    if (_pkt_idx < RPLIDAR_PACKET_SIZE) return;

    // We have 5 bytes — reset index for next packet
    _pkt_idx = 0;

    // ── Validate the packet ────────────────────────────────────────────
    // Rule 1: S bit (bit0 of byte0) must be inverse of S_bar (bit1 of byte0)
    // This is a sync/validity check built into the protocol
    uint8_t S     = _pkt[0] & 0x01;        // bit 0
    uint8_t S_bar = (_pkt[0] >> 1) & 0x01; // bit 1
    if (S == S_bar) return; // invalid packet, discard

    // Rule 2: Check bit (bit0 of byte1) must always be 1
    if ((_pkt[1] & 0x01) != 1) return; // invalid, discard

    // ── Decode the packet ──────────────────────────────────────────────
    // Quality: upper 6 bits of byte 0
    uint8_t quality = _pkt[0] >> 2;

    // Angle: 15-bit value stored across byte1 and byte2
    // byte1 bits[7:1] = angle bits[6:0]
    // byte2 bits[7:0] = angle bits[14:7]
    // The value is in units of 1/64 degree (Q6 fixed point)
    uint16_t angle_q6 = (((uint16_t)_pkt[2] << 7) | (_pkt[1] >> 1));
    float angle = angle_q6 / 64.0f;

    // Distance: 16-bit value across byte3 and byte4
    // The value is in units of 1/4 mm (Q2 fixed point)
    uint16_t dist_q2 = (((uint16_t)_pkt[4] << 8) | _pkt[3]);
    float distance = dist_q2 / 4.0f;

    // ── Detect start of new revolution ────────────────────────────────
    // S bit = 1 means this point is the START of a new 360° scan
    if (S == 1 && rplidar_scan_count > 0) {
        // Signal main loop that a complete scan is ready
        rplidar_scan_ready = 1;
        rplidar_scan_count = 0; // reset for next revolution
    }

    // ── Store the point ───────────────────────────────────────────────
    if (rplidar_scan_count < RPLIDAR_MAX_POINTS) {
        rplidar_scan[rplidar_scan_count].quality     = quality;
        rplidar_scan[rplidar_scan_count].start_bit   = S;
        rplidar_scan[rplidar_scan_count].angle_deg   = angle;
        rplidar_scan[rplidar_scan_count].distance_mm = distance;
        rplidar_scan_count++;
    }
}
