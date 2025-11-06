#pragma once
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include <string.h>

#define EEPROM_SIZE 128 // number of bytes to emulate
#define FLASH_TARGET_OFFSET (2*1024*1024 - 4096) // last 4KB sector of 2MB flash

class EEPROM_FLASH {
	private:

		static uint8_t buffer[ 4096 ]; // full flash page buffer
		static bool bufferLoaded;

		// Load flash page into RAM buffer (only once)
		static void loadBuffer(){

			if( !bufferLoaded ){

				memcpy( buffer, (const void*)( XIP_BASE + FLASH_TARGET_OFFSET ), 4096 );
				bufferLoaded = true;
			}
		}

		// Commit buffer to flash
		static void commitBuffer(){

			// Disable interrupts during flash operations
			uint32_t interrupts = save_and_disable_interrupts();

			flash_range_erase( FLASH_TARGET_OFFSET, 4096 );
			flash_range_program( FLASH_TARGET_OFFSET, buffer, 4096 );
			restore_interrupts( interrupts );
		}

	public:
		// Initialize (load flash into RAM buffer)
		static void begin(){

			loadBuffer();
		}

		// Read a byte
		static uint8_t read( int address ){

			if( address < 0 || address >= EEPROM_SIZE ) return 0;

			loadBuffer();
			return buffer[ address ];
		}

		// Write a byte (updates buffer, commits immediately)
		static void write( int address, uint8_t value ){

			if( address < 0 || address >= EEPROM_SIZE ) return;

			loadBuffer();

			if( buffer[ address ] != value ){

				buffer[ address ] = value;
				commitBuffer();  // persist to flash
			}
		}

		// Update (only write if different)
		static void update( int address, uint8_t value ){

			write( address, value );
		}

		// Commit placeholder (for API compatibility)
		static void commit() {

			commitBuffer();
		}

		// Optional: get() for structs/ints/floats
		template<typename T>
		static void get( int address, T &value ){

			loadBuffer();

			if( address < 0 || ( address + sizeof( T )) > EEPROM_SIZE ) return;

			memcpy( &value, &buffer[ address ], sizeof( T  ));
		}

		template<typename T>
		static void put( int address, const T &value ) {

			loadBuffer();

			if( address < 0 || ( address + sizeof( T )) > EEPROM_SIZE ) return;

			memcpy( &buffer[ address ], &value, sizeof(T) );
			commitBuffer();
		}
};

// Static member definitions
uint8_t EEPROM_FLASH::buffer[ 4096 ] = { 0 };
bool EEPROM_FLASH::bufferLoaded = false;
