#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#define RINGBUFFER_LEN 100
#define RINGBUFFER_MAX_NUM 1

typedef struct ringbuffer {
	char read[RINGBUFFER_LEN];
	char *in_read_ptr;
	char *out_read_ptr;
	char write[RINGBUFFER_LEN];
	char *in_write_ptr;
	char *out_write_ptr;
	FILE *dev;
	FILE *dev_in_as_out;
	uint8_t newlines;
	char block_read;
} ringbuffer_t;

ringbuffer_t *ringbuffers[RINGBUFFER_MAX_NUM];
uint8_t num_ringbuffers;

void ringbuffer_setup(FILE *stream, ringbuffer_t *buffer);
void ringbuffer_setup_in_as_out(FILE *stream, ringbuffer_t *buffer);
void init_ringbuffers(void);
void stream_setup(FILE *uart_stream);
void stream_setup_out_only(FILE *stream);
void stream_setup_in_as_out(FILE *stream, int ringbuffer_putchar_sound(char, FILE*));
uint8_t ringbuffer_busy(void);
void ringbuffer_set_read_noblock(FILE *stream);
void ringbuffer_set_read_block(FILE *stream, char c);

#endif

