// Copyright 2026 by Frobenius Norm LLC 2026-07-07 00:00:00
//
// OTA llloader -- console rung (UART transport). See src_console.h.

#include "src_console.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"

static const char *TAG = "console";

#define CON_UART        UART_NUM_0
/*! 8192, AND ENLARGING IT DOES NOT HELP -- MEASURED, SO DO NOT TRY IT AGAIN.
    The obvious reading of `stalled with N bytes left` is that this ring overflows while
    `ota_writer` is busy in flash and the unabsorbed inflow is dropped.  At 115200 baud, 8192 B is
    ~0.7 s of inflow and the observed shortfall was ~4.6 s worth, so the arithmetic looked
    compelling.  IT IS WRONG.  Measured 2026-09-26 on the 4WD, same 1,927,632-byte image, same
    sender flags, ring raised 8x to 65536:

        CON_RXBUF  8192  -> stalled with 53113 bytes left
        CON_RXBUF 65536  -> stalled with 52998 bytes left      (0.2% different)

    An 8x larger ring recovered 115 bytes, so ring CAPACITY is not the limiting factor and the loss
    is not an overflow of this buffer.
    LEADING HYPOTHESIS, UNTESTED, recorded so the next attempt starts past the one that failed: the
    bytes are lost BEFORE the ring, in the UART's 128-byte hardware FIFO.  An ESP32 flash erase/write
    can stall an ISR that is not in IRAM, and if the UART interrupt does not run the FIFO overruns
    regardless of how large the ring behind it is -- which is exactly the shape of a result that does
    not respond to ring size.  If that is right the fixes are an IRAM-safe UART ISR, real flow
    control, or not streaming during a flash write at all (an ACK per chunk), and NOT a bigger
    buffer.  See [B644]. */
#define CON_RXBUF       8192
#define CON_MAGIC       "LLIM"
#define CON_HDR_LEN     40            // 4 magic + 4 size + 32 sha256
#define CON_WAIT_HDR_MS 300000        // 5 min for an operator to start the push
#define CON_WAIT_BODY_MS 20000        // per-chunk stall timeout once the stream is flowing

typedef struct {
    size_t remaining;
    bool   installed;
} con_ctx_t;

// Single-shot loader: one console source at a time.
static image_source_t s_src;
static con_ctx_t      s_ctx;

// Read exactly `len` bytes into buf, bounded by an absolute deadline. Returns bytes read
// (== len on success, < len if the deadline passed with the frame incomplete).
static int read_exact(uint8_t *buf, int len, int64_t deadline_us)
{
    int n = 0;
    while (n < len) {
        int64_t left_us = deadline_us - esp_timer_get_time();
        if (left_us <= 0) break;
        TickType_t ticks = pdMS_TO_TICKS((uint32_t) (left_us / 1000));
        if (ticks == 0) ticks = 1;
        int r = uart_read_bytes(CON_UART, buf + n, len - n, ticks);
        if (r < 0) break;
        n += r;
    }
    return n;
}

static int con_read(image_source_t *self, uint8_t *buf, size_t max)
{
    con_ctx_t *c = (con_ctx_t *) self->ctx;
    if (c->remaining == 0) {
        return 0;  // whole framed image delivered
    }
    size_t want = max < c->remaining ? max : c->remaining;
    int r = uart_read_bytes(CON_UART, buf, want, pdMS_TO_TICKS(CON_WAIT_BODY_MS));
    if (r <= 0) {
        ESP_LOGE(TAG, "stalled with %u bytes left", (unsigned) c->remaining);
        return -1;
    }
    c->remaining -= (size_t) r;
    return r;
}

static void con_close(image_source_t *self)
{
    con_ctx_t *c = (con_ctx_t *) self->ctx;
    if (c != NULL && c->installed) {
        uart_driver_delete(CON_UART);
        c->installed = false;
    }
    self->ctx = NULL;
}

image_source_t *llloader_console_open(bool want_golden)
{
    (void) want_golden;  // operator chooses the image

    if (!uart_is_driver_installed(CON_UART)) {
        if (uart_driver_install(CON_UART, CON_RXBUF, 0, 0, NULL, 0) != ESP_OK) {
            ESP_LOGW(TAG, "uart driver install failed -> rung unavailable");
            return NULL;
        }
    }
    // RX (operator->device) is independent of the ESP_LOG TX stream, so log noise never corrupts
    // the inbound frame. Flush any stale bytes before announcing readiness.
    uart_flush_input(CON_UART);
    ESP_LOGW(TAG, "CONSOLE READY -- push LLIM frame within %ds "
                  "(w3_ai_scripts/llloader_console_send.py)", CON_WAIT_HDR_MS / 1000);

    /* RESYNCHRONISE ON THE MAGIC -- DO NOT DEMAND IT AT OFFSET 0.
       This used to read CON_HDR_LEN bytes and reject unless byte 0 began "LLIM", which makes the
       rung fail whenever ANYTHING precedes the header in the RX buffer: a stale byte from an
       earlier attempt, a fragment of a previous push, line noise on the console UART.  One extra
       leading byte was enough to lose the whole recovery, and the report it produced --
       "no valid frame (got 40 bytes)" with CON_HDR_LEN also 40 -- reads like a short read or a
       timeout, which is where the diagnosis went instead.  Observed 2026-09-26: a full 40 bytes
       read well inside the 300 s window and refused on the memcmp.
       This is the LAST rung: INSTALL is exhausted and RECOVER found no golden image, so the board
       is off the network and the console is the only channel left.  It must not be lost to a
       leading byte.  Scanning costs a byte-at-a-time read until the magic lands and nothing after.
       The deadline still bounds the whole search, so a console that never carries a frame still
       gives up exactly as before. */
    int64_t deadline = esp_timer_get_time() + (int64_t) CON_WAIT_HDR_MS * 1000;
    uint8_t hdr[CON_HDR_LEN];
    int matched = 0;                 /* bytes of CON_MAGIC matched so far */
    int skipped = 0;                 /* bytes discarded before the magic -- report them */
    while (matched < 4) {
        uint8_t b;
        if (read_exact(&b, 1, deadline) != 1) {
            ESP_LOGW(TAG, "no frame magic within the window (skipped %d) -> rung unavailable",
                     skipped);
            uart_driver_delete(CON_UART);
            return NULL;
        }
        if (b == (uint8_t) CON_MAGIC[matched]) {
            hdr[matched++] = b;
        }
        else {
            /* A partial match that breaks may itself start a new one ("LLLIM"), so retry this
               byte against position 0 rather than dropping it. */
            skipped += matched ? matched : 1;
            matched = (b == (uint8_t) CON_MAGIC[0]) ? 1 : 0;
            if (matched) hdr[0] = b;
        }
    }
    if (skipped != 0) {
        ESP_LOGW(TAG, "resynchronised on frame magic after %d stray byte(s)", skipped);
    }
    int got = read_exact(hdr + 4, CON_HDR_LEN - 4, deadline);
    if (got != CON_HDR_LEN - 4) {
        ESP_LOGW(TAG, "frame header incomplete (got %d of %d after magic) -> rung unavailable",
                 got, CON_HDR_LEN - 4);
        uart_driver_delete(CON_UART);
        return NULL;
    }

    uint32_t size = (uint32_t) hdr[4]
                  | ((uint32_t) hdr[5] << 8)
                  | ((uint32_t) hdr[6] << 16)
                  | ((uint32_t) hdr[7] << 24);
    if (size == 0) {
        ESP_LOGW(TAG, "frame declares 0 bytes -> ignoring");
        uart_driver_delete(CON_UART);
        return NULL;
    }

    memset(&s_src, 0, sizeof(s_src));
    static const uint8_t zero32[32] = {0};
    if (memcmp(hdr + 8, zero32, 32) != 0) {
        memcpy(s_src.expected_sha256, hdr + 8, 32);
        s_src.have_sha256 = true;
    }

    s_ctx.remaining = size;
    s_ctx.installed = true;

    s_src.name       = "console-uart";
    s_src.total_size = size;
    s_src.ctx        = &s_ctx;
    s_src.read       = con_read;
    s_src.close      = con_close;

    ESP_LOGI(TAG, "frame accepted: %u bytes%s", (unsigned) size,
             s_src.have_sha256 ? " +sha256" : " UNVERIFIED");
    return &s_src;
}
