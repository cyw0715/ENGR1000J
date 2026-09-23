# P4 ↔ PC ↔ Mega framed bridge protocol (v1)

This protocol runs over the P4's `COM5` UART0 connection. It is deliberately
**not** a raw Arduino serial port: opening COM5 must not allow an IDE/uploader
to toggle P4's auto-reset lines.

## Frame format

All multi-byte integers are little-endian.

```text
SOF0  SOF1  TYPE  LENGTH_LO  LENGTH_HI  PAYLOAD[0..LENGTH-1]  CRC_LO  CRC_HI
0xA5  0x5A  u8    u16 LE                         variable       CRC16-CCITT
```

CRC16-CCITT parameters: initial `0xFFFF`, polynomial `0x1021`; CRC covers
`TYPE | LENGTH_LO | LENGTH_HI | PAYLOAD`, not SOF bytes. The maximum payload is
512 bytes. The receiver must discard malformed frames and resynchronize on
`A5 5A`.

## PC → P4 commands

| Type | Name | Payload | P4 behavior |
|---:|---|---|---|
| `0x01` | HELLO | empty | Responds `HELLO_ACK`; client uses this after clearing early boot text. |
| `0x10` | MEGA_TX | 1–256 raw bytes | Writes bytes to Mega CH340 UART, if connected. |
| `0x12` | SET_LINE | `u32 baud`, `u8 stop_bits`, `u8 parity`, `u8 data_bits` | Applies CH340 line coding. Mega bootloader uses `115200,0,0,8`. |
| `0x13` | SET_CONTROL | `u8 flags`: bit0 DTR, bit1 RTS | Applies CH340 DTR/RTS. |
| `0x14` | RESET_MEGA | empty | Pulses CH340 DTR low then high. It is only a reset request; it never writes Mega flash. |
| `0x15` | GET_STATUS | empty | Replies with `STATUS`. |

## P4 → PC responses

| Type | Name | Payload |
|---:|---|---|
| `0x02` | HELLO_ACK | `"P4MEGA1"` plus status flags |
| `0x11` | MEGA_RX | Raw bytes received from Mega. |
| `0x16` | STATUS | connection state and last ESP error code. |
| `0x7F` | ERROR | `u8 command_type`, `i32 esp_err_t` |

## Boundary

This bridge does not implement STK500v2, does not store a Mega image, and does
not autonomously flash Mega. A PC-side uploader can use `MEGA_TX` / `MEGA_RX`
after requesting `SET_LINE` and `RESET_MEGA`.

Do not use a generic Arduino IDE serial upload directly against COM5. It can
toggle the P4's own UART control lines and reset the P4.
