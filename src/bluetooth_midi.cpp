#include <zephyr/kernel.h>

#include "circularbuffer.hpp"

extern "C" {
    #include <zephyr/bluetooth/bluetooth.h>
    #include <zephyr/bluetooth/conn.h>
    #include <zephyr/bluetooth/gatt.h>
}

extern "C" bool notif_enabled;
extern "C" struct bt_conn *conn_connected;
extern "C" uint8_t midi_data[5];

extern "C" struct bt_gatt_service_static midi_svc;

struct MidiMessage {
	uint8_t timestampHi;
	uint8_t timestampLo;
	uint8_t midiBytes[3];
	uint8_t midiLen;
};

CircularQueue<MidiMessage, 16> midiMessageQueue;

uint8_t getTimestampHighByte(uint16_t timestampMillis) {
	return 0x80 | ((timestampMillis >> 7) 	& 0b0111111);
}
uint8_t getTimestampLowByte(uint16_t timestampMillis) {
	return 0x80 | (timestampMillis 		    & 0b1111111);
}


void send_midi_control_change(uint32_t timestamp, uint8_t channel, uint8_t controller, uint8_t value)
{
	struct MidiMessage thisMessage;
	timestamp = timestamp % (1<<13);
	thisMessage.timestampHi = getTimestampHighByte(timestamp);
	thisMessage.timestampLo = getTimestampLowByte(timestamp);
	thisMessage.midiBytes[0] = 0xB0 | (channel & 0x0F);
	thisMessage.midiBytes[1] = controller & 0x7F;
	thisMessage.midiBytes[2] = value & 0x7F;
	thisMessage.midiLen = 3;

    midiMessageQueue.enqueue(thisMessage);
}


void send_midi_thread(void) 
{
	while(1) {
		if(notif_enabled && !midiMessageQueue.is_empty()) {
            MidiMessage thisMessage = midiMessageQueue.dequeue();

            struct bt_conn *conn = NULL;

            if (conn_connected) {
                /* Get a connection reference to ensure that a
                    * reference is maintained in case disconnected
                    * callback is called while we perform GATT Write
                    * command.
                    */
                conn = bt_conn_ref(conn_connected);
            }

            if (conn) {
				size_t dataLen = 0;

				//uint8_t timestampHi = thisMessage->timestampHi;
                midi_data[dataLen++] = thisMessage.timestampHi;

                midi_data[dataLen++] = thisMessage.timestampLo;

				midi_data[dataLen++] = thisMessage.midiBytes[0];
				midi_data[dataLen++] = thisMessage.midiBytes[1];
				midi_data[dataLen++] = thisMessage.midiBytes[2];
                // for(size_t i = 0; i != thisMessage->midiLen; i++) {
                // 	midi_data[dataLen++] = thisMessage->midiBytes[i];
                // }

				// while(!k_fifo_is_empty(midi_fifo) {

				// })


                //Send MIDI message as a notification
                int err = bt_gatt_notify(conn, &midi_svc.attrs[1], midi_data, sizeof(midi_data));
                if (err) {
                    printk("Failed to send MIDI notification (err %d)\n", err);
                } 

				if(midi_data[0] == 0) {
					printk("bad sample!!");
				}

                bt_conn_unref(conn);			
            }

        }

		k_yield();
	}
}
