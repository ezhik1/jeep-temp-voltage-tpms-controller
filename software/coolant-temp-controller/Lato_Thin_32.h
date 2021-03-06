// Created by http://oleddisplay.squix.ch/ Consider a donation
// In case of problems make sure that you are using the font file with the correct version!
const uint8_t Lato_Thin_32Bitmaps[] = {

	// Bitmap Data:
	0x00, // ' '
	0xD8, // '.'
	0x07,0xE0,0x04,0x0C,0x04,0x01,0x04,0x00,0x84,0x00,0x22,0x00,0x11,0x00,0x05,0x00,0x02,0x80,0x01,0x40,0x00,0xA0,0x00,0x50,0x00,0x28,0x00,0x14,0x00,0x0A,0x00,0x04,0x80,0x02,0x40,0x02,0x20,0x01,0x08,0x01,0x02,0x00,0x80,0x81,0x80,0x3F,0x00, // '0'
	0x06,0x00,0x70,0x04,0x80,0x44,0x04,0x20,0x41,0x00,0x08,0x00,0x40,0x02,0x00,0x10,0x00,0x80,0x04,0x00,0x20,0x01,0x00,0x08,0x00,0x40,0x02,0x00,0x10,0x00,0x80,0x04,0x00,0x20,0x7F,0xF8, // '1'
	0x0F,0xC0,0x20,0x60,0x80,0x22,0x00,0x44,0x00,0x48,0x00,0x80,0x01,0x00,0x02,0x00,0x08,0x00,0x10,0x00,0x40,0x01,0x00,0x04,0x00,0x10,0x00,0x40,0x01,0x00,0x04,0x00,0x10,0x00,0x40,0x01,0x00,0x04,0x00,0x1F,0xFF,0x80, // '2'
	0x0F,0xC0,0x20,0x60,0x80,0x22,0x00,0x44,0x00,0x48,0x00,0x80,0x01,0x00,0x04,0x00,0x10,0x00,0x40,0x07,0x00,0x01,0x80,0x00,0x80,0x00,0x80,0x01,0x00,0x02,0x80,0x05,0x00,0x09,0x00,0x12,0x00,0x43,0x03,0x01,0xF8,0x00, // '3'
	0x00,0x10,0x00,0x18,0x00,0x14,0x00,0x12,0x00,0x11,0x00,0x08,0x80,0x08,0x40,0x08,0x20,0x08,0x10,0x08,0x08,0x04,0x04,0x04,0x02,0x04,0x01,0x04,0x00,0x83,0xFF,0xFC,0x00,0x20,0x00,0x10,0x00,0x08,0x00,0x04,0x00,0x02,0x00,0x01,0x00,0x00,0x80, // '4'
	0x3F,0xF0,0x80,0x02,0x00,0x08,0x00,0x40,0x01,0x00,0x04,0x00,0x10,0x00,0x7F,0x81,0x01,0x00,0x02,0x00,0x04,0x00,0x10,0x00,0x20,0x00,0x80,0x02,0x00,0x08,0x00,0x40,0x01,0x00,0x08,0xC0,0xC0,0xFC,0x00, // '5'
	0x00,0x60,0x00,0x40,0x00,0x80,0x01,0x00,0x02,0x00,0x02,0x00,0x04,0x00,0x08,0x00,0x17,0xC0,0x38,0x30,0x20,0x08,0x40,0x04,0x40,0x04,0x40,0x04,0x80,0x02,0x80,0x02,0x80,0x04,0x40,0x04,0x40,0x04,0x20,0x08,0x10,0x30,0x0F,0xC0, // '6'
	0xFF,0xFE,0x00,0x02,0x00,0x04,0x00,0x04,0x00,0x08,0x00,0x10,0x00,0x10,0x00,0x20,0x00,0x20,0x00,0x40,0x00,0x40,0x00,0x80,0x00,0x80,0x01,0x00,0x01,0x00,0x02,0x00,0x02,0x00,0x04,0x00,0x04,0x00,0x08,0x00,0x08,0x00,0x10,0x00, // '7'
	0x0F,0xC0,0x20,0x60,0x80,0x22,0x00,0x44,0x00,0x48,0x00,0x90,0x01,0x20,0x04,0x20,0x08,0x30,0x60,0x1F,0x00,0xC1,0x86,0x00,0x88,0x00,0xA0,0x01,0x40,0x02,0x80,0x05,0x00,0x09,0x00,0x12,0x00,0x43,0x01,0x01,0xFC,0x00, // '8'
	0x0F,0xC0,0x20,0x60,0x80,0x22,0x00,0x48,0x00,0x50,0x00,0xA0,0x01,0x40,0x02,0x40,0x0C,0xC0,0x30,0xC0,0xA0,0x7E,0x80,0x03,0x00,0x04,0x00,0x10,0x00,0x40,0x00,0x80,0x02,0x00,0x08,0x00,0x10,0x00,0x40,0x01,0x00,0x00, // '9'

};
const GFXglyph Lato_Thin_32Glyphs[] = {
// bitmapOffset, width, height, xAdvance, xOffset, yOffset
	  {     0,   1,   1,  10,    0,    0 }, // ' '
	  {   327,   3,   2,   8,    3,   -2 }, // '.'
	  {   364,  17,  22,  20,    1,  -22 }, // '0'
	  {   411,  13,  22,  20,    4,  -22 }, // '1'
	  {   447,  15,  22,  20,    2,  -22 }, // '2'
	  {   489,  15,  22,  20,    2,  -22 }, // '3'
	  {   531,  17,  22,  20,    1,  -22 }, // '4'
	  {   578,  14,  22,  20,    3,  -22 }, // '5'
	  {   617,  16,  22,  20,    2,  -22 }, // '6'
	  {   661,  16,  22,  20,    2,  -22 }, // '7'
	  {   705,  15,  22,  20,    2,  -22 }, // '8'
	  {   747,  15,  22,  20,    2,  -22 } // '9'
};
const GFXfont Lato_Thin_32 = {
(uint8_t  *)Lato_Thin_32Bitmaps,(GFXglyph *)Lato_Thin_32Glyphs,0x20, 0x7E, 39};