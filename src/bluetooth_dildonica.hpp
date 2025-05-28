
struct DildonicaBluetoothMessage {
	uint32_t timestamp;
	uint32_t rawSample;
    uint8_t zone;
	uint8_t threshLo;
	uint8_t threshHi;
};

void send_bluetooth_sample(DildonicaBluetoothMessage sample);