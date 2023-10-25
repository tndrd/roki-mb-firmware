#ifndef BODY_MSGS
#define BODY_MSGS

struct BodyMsgs {
	struct Requests {
		struct GetAllPos {
			const uint8_t Data[10] = { 0xA, 0, 0x20, 0, 0, 0, 0x70, 0, 0x1E,
					0xB8 };
			const size_t RequestSize = sizeof(Data) / sizeof(Data[0]);
			const size_t ResponceSize = 33;
		};
	};

	struct Responces {
		struct KondoNACK {
			const uint8_t Data[4] = { 0x4, 0xFE, 0x15, 0x17 };
			const uint8_t Size = sizeof(Data) / sizeof(Data[0]);
		};
	};
};
#endif
