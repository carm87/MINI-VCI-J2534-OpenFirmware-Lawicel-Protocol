#ifndef _CRC_DATA_H
#define _CRC_DATA_H


#define CRC_IDENT_STRING		"CRC-Arrays"


typedef struct {

u8_t		Str[16];
u16_t		Version;
u16_t		Day;
u16_t		Month;
u32_t		Year;
u16_t		dummy[3];
u16_t		Mode;
} __attribute__((packed)) crc_array_t;


#endif
