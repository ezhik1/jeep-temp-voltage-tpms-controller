#ifndef ANALOG_LADDER_BUTTON_H
#define ANALOG_LADDER_BUTTON_H

#include <Arduino.h>

// ======================================================
//  ANALOG LADDER CORE (stable 12-bit discrete decoder)
// ======================================================
class AnalogLadder {
public:
	AnalogLadder( uint8_t pin, int* rawValues, uint8_t count )
	: _pin( pin ), _rawValues( rawValues ), _count( count ) {}

	void begin() {

		pinMode( _pin, INPUT );

		// copy + sort known states
		for( uint8_t i = 0; i < _count; i++ ){

			_sorted[ i ] = _rawValues[ i ];
		}

		for( uint8_t i = 0; i < _count - 1; i++ ){

			for( uint8_t j = i + 1; j < _count; j++ ){

				if( _sorted[ j ] < _sorted[ i ] ){

					int t = _sorted[i];
					_sorted[ i ] = _sorted[ j ];
					_sorted[ j ] = t;
				}
			}
		}
	}

	void update() {

		int raw = analogRead( _pin );

		// light smoothing
		raw = ( raw + analogRead( _pin ) ) / 2;

		int bestKey = 0;
		int bestDiff = _tolerance;

		// ===== nearest-state match =====
		for( uint8_t i = 0; i < _count; i++ ){
			int diff = abs( raw - _sorted[ i ] );

			if( diff < bestDiff ){

				bestDiff = diff;
				bestKey = i + 1;
			}
		}

		// ===== release if too far from all states =====
		if( bestDiff >= _tolerance ){

			bestKey = 0;
		}

		// ===== stability filter =====
		if( bestKey == _lastRawKey ){

			_stableCount++;
		}else{

			_stableCount = 0;
		}

		_lastRawKey = bestKey;

		if( _stableCount >= _requiredStableReads ){

			_currentKey = bestKey;
		}
	}

	int getKey() const {

		return _currentKey;
	}

	void setTolerance( int tolerance ) {

		_tolerance = tolerance;
	}

	void setStableReads( uint8_t numberOfReads ) {

		_requiredStableReads = numberOfReads;
	}

	private:
		uint8_t _pin;
		int* _rawValues;
		uint8_t _count;

		int _sorted[ 10 ];

		int _currentKey = 0;
		int _lastRawKey = -1;

		uint8_t _stableCount = 0;
		uint8_t _requiredStableReads = 2;

		int _tolerance = 600; // IMPORTANT for 12-bit spread
};


// ======================================================
//  BUTTON WRAPPER (ezButton style)
// ======================================================
class AnalogLadderButton {
	public:
		AnalogLadderButton(AnalogLadder &ladder, int id) : _ladder(ladder), _id(id) {}

		void begin() {

			_state = false;
			_lastState = false;
		}

		void update() {

			int key = _ladder.getKey();

			_state = (key == _id);

			if( _state && !_lastState ){

				_pressedEvent = true;
			}

			if( !_state && _lastState ){

				_releasedEvent = true;
			}

			_lastState = _state;
		}

		bool isPressing() const {

			return _state;
		}

		bool isPressed() {

			if( _pressedEvent ){

				_pressedEvent = false;
				return true;
			}

			return false;
		}

		bool isReleased(){
			if( _releasedEvent ){

				_releasedEvent = false;
				return true;
			}

			return false;
		}

		private:
			AnalogLadder &_ladder;
			int _id;

			bool _state = false;
			bool _lastState = false;

			bool _pressedEvent = false;
			bool _releasedEvent = false;
};

#endif
