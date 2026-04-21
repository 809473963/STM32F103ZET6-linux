#include "hello_ack.h"

#include <string.h>
#include "stdio.h"

#define HELLO_LINE_MAX 32

static char s_line_buf[HELLO_LINE_MAX];
static u8 s_line_len = 0;

void HelloAck_Init(void)
{
    s_line_len = 0;
    memset(s_line_buf, 0, sizeof(s_line_buf));
    printf("HELLO_ACK app ready\\r\\n");
}

static void HelloAck_ProcessLine(const char* line)
{
    if (strcmp(line, "hello") == 0) {
        printf("ack\\r\\n");
    }
}

void HelloAck_ParseByte(u8 byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        s_line_buf[s_line_len] = '\0';
        HelloAck_ProcessLine(s_line_buf);
        s_line_len = 0;
        return;
    }

    if (s_line_len < (HELLO_LINE_MAX - 1)) {
        s_line_buf[s_line_len++] = (char)byte;
    } else {
        s_line_len = 0;
    }
}
