// Created by http://oleddisplay.squix.ch/ Consider a donation
// In case of problems make sure that you are using the font file with the correct version!
const uint8_t Lato_Thin_12Bitmaps[] PROGMEM = {

	// Bitmap Data:
	0x00, // ' '
	0xAA,0xA2, // '!'
	0xDB,0x00, // '"'
	0x29,0x2F,0x14,0x53,0xE9,0x28, // '#'
	0x21,0xEA,0x28,0x60,0xE2,0x8A,0xF0,0x80, // '$'
	0xE2,0x52,0x2A,0x0A,0xC1,0x91,0x49,0x25,0x1C, // '%'
	0x78,0x48,0x40,0x40,0xB4,0x8C,0x84,0xFA, // '&'
	0xA8, // '''
	0x52,0x49,0x24,0x98, // '('
	0x89,0x24,0x92,0x58, // ')'
	0xEE,0x40, // '*'
	0x20,0x82,0x3E,0x20,0x80, // '+'
	0xA0, // ','
	0xE0, // '-'
	0x80, // '.'
	0x08,0x41,0x08,0x20,0x84,0x10,0x80, // '/'
	0x38,0x8A,0x14,0x28,0x50,0x91,0x1C, // '0'
	0x21,0x8A,0x08,0x20,0x82,0x1E, // '1'
	0x72,0x20,0x82,0x10,0x84,0x3E, // '2'
	0x72,0x20,0x8C,0x08,0x28,0xBC, // '3'
	0x08,0x18,0x28,0x48,0x48,0xFE,0x08,0x08, // '4'
	0x7A,0x0F,0x02,0x08,0x20,0xBC, // '5'
	0x10,0x84,0x3C,0x8A,0x28,0x9C, // '6'
	0xF8,0x21,0x04,0x20,0x84,0x10, // '7'
	0x72,0x28,0x9C,0x8A,0x28,0xBC, // '8'
	0x72,0x28,0xA2,0x70,0x42,0x10, // '9'
	0x80,0x20, // ':'
	0x80,0x28, // ';'
	0x39,0x0C,0x0E, // '<'
	0xF8,0x0F,0x80, // '='
	0xE0,0x41,0xB8, // '>'
	0xE2,0x24,0x40,0x04, // '?'
	0x3C,0x21,0xA7,0x54,0xAA,0x55,0x4A,0x59,0x00,0x41,0x1F,0x00, // '@'
	0x10,0x0C,0x0A,0x04,0x84,0x43,0xE1,0x09,0x02, // 'A'
	0xF9,0x0A,0x27,0xC8,0x50,0xA1,0x7C, // 'B'
	0x7E,0x80,0x80,0x80,0x80,0x80,0x82,0x7C, // 'C'
	0xFC,0x82,0x82,0x82,0x82,0x82,0x82,0xFC, // 'D'
	0xFA,0x08,0x3E,0x82,0x08,0x3E, // 'E'
	0xFA,0x08,0x3E,0x82,0x08,0x20, // 'F'
	0x7E,0x80,0x80,0x80,0x86,0x82,0x82,0x7C, // 'G'
	0x82,0x82,0x82,0xFE,0x82,0x82,0x82,0x82, // 'H'
	0xAA,0xAA, // 'I'
	0x10,0x84,0x21,0x08,0x5C, // 'J'
	0x85,0x32,0x86,0x0A,0x12,0x22,0x42, // 'K'
	0x82,0x08,0x20,0x82,0x08,0x3E, // 'L'
	0x81,0x60,0xA8,0xD4,0x69,0x54,0xCA,0x05,0x02, // 'M'
	0x82,0xC2,0xA2,0x92,0x92,0x8A,0x86,0x82, // 'N'
	0x7C,0x41,0x20,0x50,0x28,0x14,0x0A,0x08,0xF8, // 'O'
	0xFA,0x28,0xA2,0xF2,0x08,0x20, // 'P'
	0x7C,0x41,0x20,0x50,0x28,0x14,0x0A,0x08,0xF8,0x02,0x00,0x80, // 'Q'
	0xF9,0x12,0x27,0x89,0x12,0x22,0x42, // 'R'
	0x72,0x08,0x18,0x10,0x20,0xBC, // 'S'
	0xFE,0x10,0x10,0x10,0x10,0x10,0x10,0x10, // 'T'
	0x85,0x0A,0x14,0x28,0x50,0xA1,0x3C, // 'U'
	0x81,0x21,0x10,0x88,0x82,0x41,0x40,0x60,0x20, // 'V'
	0x84,0x12,0x30,0x91,0x88,0x92,0x42,0x92,0x14,0x60,0xC3,0x04,0x10, // 'W'
	0xC2,0x44,0x28,0x10,0x10,0x28,0x44,0x82, // 'X'
	0x82,0x44,0x28,0x28,0x10,0x10,0x10,0x10, // 'Y'
	0x7E,0x04,0x08,0x10,0x10,0x20,0x40,0xFE, // 'Z'
};
const GFXglyph Lato_Thin_12Glyphs[] PROGMEM = {
// bitmapOffset, width, height, xAdvance, xOffset, yOffset
	  {     0,   1,   1,   4,    0,    0 }, // ' '
	  {     1,   2,   8,   4,    1,   -8 }, // '!'
	  {     3,   3,   3,   5,    1,   -8 }, // '"'
	  {     5,   6,   8,   8,    1,   -8 }, // '#'
	  {    11,   6,  10,   8,    1,   -9 }, // '$'
	  {    19,   9,   8,  10,    1,   -8 }, // '%'
	  {    28,   8,   8,   9,    1,   -8 }, // '&'
	  {    36,   2,   3,   3,    1,   -8 }, // '''
	  {    37,   3,  10,   4,    1,   -9 }, // '('
	  {    41,   3,  10,   4,    0,   -9 }, // ')'
	  {    45,   4,   3,   6,    1,   -8 }, // '*'
	  {    47,   6,   6,   8,    1,   -7 }, // '+'
	  {    52,   2,   2,   4,    1,   -1 }, // ','
	  {    53,   4,   1,   5,    1,   -4 }, // '-'
	  {    54,   2,   1,   4,    1,   -1 }, // '.'
	  {    55,   6,   9,   6,    0,   -9 }, // '/'
	  {    62,   7,   8,   8,    0,   -8 }, // '0'
	  {    69,   6,   8,   8,    1,   -8 }, // '1'
	  {    75,   6,   8,   8,    1,   -8 }, // '2'
	  {    81,   6,   8,   8,    1,   -8 }, // '3'
	  {    87,   8,   8,   8,    0,   -8 }, // '4'
	  {    95,   6,   8,   8,    1,   -8 }, // '5'
	  {   101,   6,   8,   8,    1,   -8 }, // '6'
	  {   107,   6,   8,   8,    1,   -8 }, // '7'
	  {   113,   6,   8,   8,    1,   -8 }, // '8'
	  {   119,   6,   8,   8,    1,   -8 }, // '9'
	  {   125,   2,   6,   4,    1,   -6 }, // ':'
	  {   127,   2,   7,   4,    1,   -6 }, // ';'
	  {   129,   6,   4,   8,    1,   -6 }, // '<'
	  {   132,   6,   3,   8,    1,   -6 }, // '='
	  {   135,   6,   4,   8,    1,   -6 }, // '>'
	  {   138,   4,   8,   6,    1,   -8 }, // '?'
	  {   142,   9,  10,  11,    1,   -8 }, // '@'
	  {   154,   9,   8,   9,    0,   -8 }, // 'A'
	  {   163,   7,   8,   9,    1,   -8 }, // 'B'
	  {   170,   8,   8,   9,    1,   -8 }, // 'C'
	  {   178,   8,   8,  10,    1,   -8 }, // 'D'
	  {   186,   6,   8,   8,    1,   -8 }, // 'E'
	  {   192,   6,   8,   8,    1,   -8 }, // 'F'
	  {   198,   8,   8,  10,    1,   -8 }, // 'G'
	  {   206,   8,   8,  10,    1,   -8 }, // 'H'
	  {   214,   2,   8,   4,    1,   -8 }, // 'I'
	  {   216,   5,   8,   6,    0,   -8 }, // 'J'
	  {   221,   7,   8,   8,    1,   -8 }, // 'K'
	  {   228,   6,   8,   7,    1,   -8 }, // 'L'
	  {   234,   9,   8,  12,    1,   -8 }, // 'M'
	  {   243,   8,   8,  10,    1,   -8 }, // 'N'
	  {   251,   9,   8,  10,    1,   -8 }, // 'O'
	  {   260,   6,   8,   8,    1,   -8 }, // 'P'
	  {   266,   9,  10,  10,    1,   -8 }, // 'Q'
	  {   278,   7,   8,   8,    1,   -8 }, // 'R'
	  {   285,   6,   8,   7,    1,   -8 }, // 'S'
	  {   291,   8,   8,   8,    0,   -8 }, // 'T'
	  {   299,   7,   8,  10,    1,   -8 }, // 'U'
	  {   306,   9,   8,   9,    0,   -8 }, // 'V'
	  {   315,  13,   8,  13,    0,   -8 }, // 'W'
	  {   328,   8,   8,   8,    0,   -8 }, // 'X'
	  {   336,   8,   8,   8,    0,   -8 }, // 'Y'
	  {   344,   8,   8,   8,    0,   -8 }, // 'Z'
};
const GFXfont Lato_Thin_12 PROGMEM = {
(uint8_t  *)Lato_Thin_12Bitmaps,(GFXglyph *)Lato_Thin_12Glyphs,0x20, 0x7E, 15};