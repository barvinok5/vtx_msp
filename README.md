# Barvinok VTX video MSP protocol (UART)

This document describes the video command protocol used by the Barvinok VTX over its CRSF UART interface.
These commands are expected to be sent by an ELRS-based receiver.
The `msp_test.py` script in this directory can be used to send the same commands from a PC for testing purposes.

```
 Handset ── BLRS TX ═══ RF ═══ BLRS RX ──── UART 420000 8N1 ────► VTX (pico_radio app_tx) ── USB ──► Camera
                                        CRSF frame 0x91, dest 0xF0       app_mspMessageHandler()
```

## 1. Physical layer

| Parameter | Value | Source |
|---|---|---|
| Interface | UART, VTX RX pin (CRSF port, `txCrsf` in `hw_config/*`) | `app/packets/crsf/crsf.c` |
| Baud rate | 420000 | `CRSF_BAUD_RATE`, `app/hw_config/hw_config.h` |
| Format | 8 data bits, no parity, 1 stop bit | `uart_init()` default |
| Logic level | 3.3 V (RP2040/RP2350 GPIO) | |

The VTX only receives on this port; it doesn't reply to these commands.

## 2. Frame structure

Every command is one CRSF extended frame carrying one encapsulated MSP v1 message:

```
 ┌──────┬─────┬──────┬──────┬────────┬────────┬──────┬──────┬───────────┬─────────┬──────────┐
 │ sync │ len │ type │ dest │ origin │ status │ size │ func │ payload…  │ MSP XOR │ CRSF CRC │
 │  C8  │     │  91  │  F0  │   EA   │   30   │      │      │ size bytes│         │          │
 └──────┴─────┴──────┴──────┴────────┴────────┴──────┴──────┴───────────┴─────────┴──────────┘
  ◄─ CRSF header ─►  ◄─ CRSF extended ─►   ◄──────────── encapsulated MSP v1 ────────────►
                     ◄──────────────────────── covered by CRSF CRC ───────────────────────►
                                                   ◄─────── covered by MSP XOR ───────►
```

| Offset | Size | Field | Value | Notes |
|---|---|---|---|---|
| 0 | 1 | sync | `C8` | `CRSF_ADDRESS_FLIGHT_CONTROLLER`. The VTX also accepts `EE` and `EA` |
| 1 | 1 | len | `size + 8` | number of bytes after this one, CRC included |
| 2 | 1 | type | `91` | `CRSF_FRAMETYPE_BARVINOK_VIDEO_MSP`. The VTX also treats `7A`, `7B`, `7C` as MSP |
| 3 | 1 | dest | `F0` | `CRSF_ADDRESS_VIDEO_MODEM`: the VTX handles the command, see section 5 |
| 4 | 1 | origin | `EA` | `CRSF_ADDRESS_RADIO_TRANSMITTER`. Not checked by the VTX |
| 5 | 1 | status | `30` | `CRSF_MSP_STATUS_BYTE`: MSP v1, start of frame, sequence 0. The VTX also accepts `80` |
| 6 | 1 | size | `N` | MSP payload size in bytes |
| 7 | 1 | func | `32`…`36` | MSP function, see section 4 |
| 8 | N | payload | | command data, multi-byte values are little endian |
| 8+N | 1 | MSP XOR | | XOR of bytes 6 … 7+N (size, func, payload) |
| 9+N | 1 | CRSF CRC | | CRC8, poly `D5`, over bytes 2 … 8+N (type … MSP XOR) |

The total frame length is `N + 10` bytes.

### Size limits

| Limit | Max payload `N` | Reason |
|---|---|---|
| ELRS RX | 32 | `ENCAPSULATED_MSP_MAX_PAYLOAD_SIZE` in ELRS `lib/MSP/msptypes.h`. `msp_test.py` uses this limit too |
| VTX parser | 54 | `CRSF_MAX_SIZE` 64: `len` must be ≤ 62 |

## 3. Checksums

### MSP XOR

```c
uint8_t msp_xor(const uint8_t *data, size_t len) {   // data = &frame[6], len = N + 2
    uint8_t crc = 0;
    while (len--) crc ^= *data++;
    return crc;
}
```

### CRSF CRC8 (polynomial 0xD5, init 0, no reflection, no final XOR)

```c
uint8_t crsf_crc8(const uint8_t *data, size_t len) { // data = &frame[2], len = frame[1] - 1
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}
```

### Worked example: `power 50`

```
C8 09 91 F0 EA 30 01 35 32 06 99
```

| Bytes | Field | Value |
|---|---|---|
| `C8` | sync | |
| `09` | len | 9 bytes follow (N = 1: 1 + 8) |
| `91` | type | Barvinok video MSP |
| `F0` | dest | video modem (VTX) |
| `EA` | origin | radio transmitter |
| `30` | status | MSP v1 |
| `01` | size | 1 byte payload |
| `35` | func | `MSP_BLRS_SET_VTX_POWER_ID` |
| `32` | payload | 50 % |
| `06` | MSP XOR | `01 ^ 35 ^ 32 = 06` |
| `99` | CRSF CRC | `crsf_crc8(91 F0 EA 30 01 35 32 06) = 99` |

## 4. Commands

| Func | Name | Payload | Handled by |
|---|---|---|---|
| `32` | `MSP_BLRS_SET_FEC` | `u8 fecId` | VTX + forwarded to camera |
| `33` | `MSP_BLRS_SET_CAMERA_ID` | `u8 cameraId` | forwarded to camera |
| `34` | `MSP_BLRS_SET_NIGH_MODE_ID` | `u8 mode` | forwarded to camera |
| `35` | `MSP_BLRS_SET_VTX_POWER_ID` | `u8 percent` | VTX |
| `36` | `MSP_BLRS_SET_VTX_FREQ_ID` | `u16 beginMHz, u16 endMHz` (LE) | VTX |

The VTX reads only the bytes it needs. Extra payload bytes are ignored.

### 4.1 `0x32` MSP_BLRS_SET_FEC

| Payload byte | Field | Values |
|---|---|---|
| 0 | fecId | `0` none, `1` FEC 1/2, `2` FEC 2/3 (`fecId_t`, `app/packets/fec/fec.h`) |

The VTX sets the FEC id in its settings, re-initializes the FEC encoder, and forwards the command to the camera.
The payload size isn't checked: an empty payload is read as id 0. The id range isn't checked either.

Log: `Requested FEC id <id>`

| Command | Frame |
|---|---|
| FEC none | `C8 09 91 F0 EA 30 01 32 00 33 74` |
| FEC 1/2 | `C8 09 91 F0 EA 30 01 32 01 32 AA` |
| FEC 2/3 | `C8 09 91 F0 EA 30 01 32 02 31 1D` |

### 4.2 `0x33` MSP_BLRS_SET_CAMERA_ID

| Payload byte | Field | Values |
|---|---|---|
| 0 | cameraId | camera index, from the camera switch |

The VTX doesn't interpret it, it forwards the command to the camera.

Log: `Received camera command 0x33, payload len 1`

| Command | Frame |
|---|---|
| camera 0 | `C8 09 91 F0 EA 30 01 33 00 32 22` |
| camera 1 | `C8 09 91 F0 EA 30 01 33 01 33 FC` |

### 4.3 `0x34` MSP_BLRS_SET_NIGH_MODE_ID

| Payload byte | Field | Values |
|---|---|---|
| 0 | mode | `0` day, `1` night (ELRS LUA "Day/Night") |

The VTX doesn't interpret it, it forwards the command to the camera.

Log: `Received camera command 0x34, payload len 1`

| Command | Frame |
|---|---|
| day | `C8 09 91 F0 EA 30 01 34 00 35 55` |
| night | `C8 09 91 F0 EA 30 01 34 01 34 8B` |

### 4.4 `0x35` MSP_BLRS_SET_VTX_POWER_ID

| Payload byte | Field | Values |
|---|---|---|
| 0 | percent | `0` … `100`. ELRS sends `100`, `50` or `0` from the 3-position power switch |

The VTX applies the new power on the radio core's next cycle (both channels).
An empty payload or a value above 100 is rejected.

Log: `Requested power level <percent>`, on error `Invalid VTX power level`

| Command | Frame |
|---|---|
| power 0 | `C8 09 91 F0 EA 30 01 35 00 34 03` |
| power 50 | `C8 09 91 F0 EA 30 01 35 32 06 99` |
| power 100 | `C8 09 91 F0 EA 30 01 35 64 50 E2` |

### 4.5 `0x36` MSP_BLRS_SET_VTX_FREQ_ID

| Payload bytes | Field | Values |
|---|---|---|
| 0–1 | beginMHz | `u16` little endian |
| 2–3 | endMHz | `u16` little endian |

`0 / 0` removes the frequency override, and the provisioned range is used again.

- `beginMHz` and `endMHz` ≤ 4294 (the conversion to Hz must fit `uint32_t`)
- `beginMHz` ≠ 0 and `endMHz` > `beginMHz`
- `(end − begin) / channelWidth + 1` is 4 … 512 channels. `channelWidth` comes from the provisioned configuration

| Command | Payload | Frame |
|---|---|---|
| 2400 – 2480 MHz | `60 09 B0 09` (0x0960, 0x09B0) | `C8 0C 91 F0 EA 30 04 36 60 09 B0 09 E2 4E` |
| remove override | `00 00 00 00` | `C8 0C 91 F0 EA 30 04 36 00 00 00 00 32 F9` |

## 5. Forwarding to the camera

The VTX forwards camera commands over USB.

- **dest `F0`**, func `32`, `33`, `34`: the VTX rebuilds the MSP message with status byte `80`, which is what the camera expects.
  Size, function, payload and XOR are the same as in the received frame:

  | Received on UART (MSP part) | Sent to camera |
  |---|---|
  | `30 01 32 02 31` (FEC 2/3) | `80 01 32 02 31` |
  | `30 01 33 01 33` (camera 1) | `80 01 33 01 33` |
  | `30 01 34 01 34` (night) | `80 01 34 01 34` |

- **dest `F1`** (`CRSF_ADDRESS_CAMERA`): the VTX doesn't parse the MSP part and passes it to the camera unchanged.

## 7. Test tool

`msp_test.py` (Python 3, pyserial) builds and sends these frames:

```
./msp_test.py -p /dev/ttyUSB0              interactive console, 'help <command>' for details
./msp_test.py -p /dev/ttyUSB0 fec 1/2      send one command and exit
./msp_test.py -n freq 2400 2480            dry run: print the frame only
```

Console commands: `fec`, `camera`, `night`, `power`, `freq`, `raw <func> [bytes…]`, `info`, `quit`.
# vtx_msp
