#ifndef HEADIO
#define HEADIO

#include "Common.hpp"
#include "FixedQueue.hpp"
#include <usart.h>

struct HeadIO {
private:
	static const uint8_t SOM1Val = 0xFF;
	static const uint8_t SOM2Val = 0xAA;
	static const uint8_t SOM3Val = 0xAF;
	static const uint8_t NACKVal = 0xFF;

public:
	struct Request {
		uint8_t ProcID;

		BufferType Data;
		uint8_t Size;

		bool IsBad() const {
			return ProcID == NACKVal;
		}
	};

	struct Responce {
		uint8_t Size;
		uint8_t Error;

		BufferType Data;
	};

public:
	struct Listener {
	private:
		enum class PackageState {
			SOM1, SOM2, PROC_ID, SIZE, DATA, SOM3
		};
		using PS = PackageState;

	private:
		FixedQueue<Request, HeadRequestQueueSize> Requests;

		UART_HandleTypeDef *Uart;
		uint8_t CurrentValue;
		Request CurrentRequest;
		PackageState State;

	private:
		void Receive1() {
			HAL_UART_Receive_IT(Uart, &CurrentValue, 1);
		}

		void ReceiveData() {
			HAL_UART_Receive_IT(Uart, CurrentRequest.Data.data(),
					CurrentRequest.Size);
		}

		void Reset() {
			State = PS::SOM1;
			Receive1();
		}

		uint8_t Get1() {
			return CurrentValue;
		}

		void HandleBadRequest() {
			Request badRequest;
			badRequest.ProcID = NACKVal;

			Requests.Push(badRequest);
			Reset();
		}

		void Callback() {
			switch (State) {
			case PS::SOM1: {
				if (Get1() == SOM1Val) {
					State = PS::SOM2;
					Receive1();
				} else
					HandleBadRequest();
				break;
			}
			case PS::SOM2: {
				if (Get1() == SOM2Val) {
					State = PS::PROC_ID;
					Receive1();
				} else
					HandleBadRequest();
				break;
			}
			case PS::PROC_ID: {
				CurrentRequest.ProcID = Get1();
				State = PS::SIZE;
				Receive1();
				break;
			}
			case PS::SIZE: {
				CurrentRequest.Size = Get1();
				if (CurrentRequest.Size > 0) {
					State = PS::DATA;
					ReceiveData();
				} else {
					State = PS::SOM3;
					Receive1();
				}
				break;
			}
			case PS::DATA: {
				State = PS::SOM3;
				Receive1();
				break;
			}
			case PS::SOM3: {
				if (Get1() == SOM3Val) {
					Requests.Push(CurrentRequest);
					Reset();
				} else
					HandleBadRequest();
				break;
			}
			default:
				HandleBadRequest();
			}
		}
	public:
		explicit Listener(UART_HandleTypeDef *uart) :
				Uart { uart } {
			assert(uart);
		}

		void Init() {
			Reset();
		}

		bool HasRequest() {
			return !Requests.Empty();
		}

		Request GetRequest() {
			assert(HasRequest());
			Request request = Requests.Front();
			Requests.Pop();
			return request;
		}

		void RxCpltCallback() {
			Callback();
		}
	};

	struct Sender {
	private:
		FixedQueue<Responce, HeadResponceQueueSize> Responces;
		UART_HandleTypeDef *Uart;
		BufferType Buffer;

		bool Ready = true;
	private:
		void Send(const Responce &responce) {
			assert(Ready);
			Ready = false;

			size_t bufSize = responce.Size + 5;

			Buffer[0] = SOM1Val;
			Buffer[1] = SOM2Val;
			Buffer[2] = responce.Error;
			Buffer[3] = responce.Size;

			memcpy(&Buffer[4], responce.Data.data(), responce.Size);

			Buffer[bufSize - 1] = SOM3Val;

			HAL_UART_Transmit_IT(Uart, Buffer.data(), bufSize);
		}

		void Callback() {
			Ready = true;
		}

	public:
		explicit Sender(UART_HandleTypeDef *uart) :
				Uart(uart) {
			assert(uart);
		}

		void AddResponce(const Responce &responce) {
			Responces.Push(responce);
		}

		void CheckTimer() {
			if (Responces.Empty())
				return;
			if (!Ready)
				return;

			Send(Responces.Front());
			Responces.Pop();
		}

		void TxCpltCallback() {
			Callback();
		}
	};
public:
	Listener Input;
	Sender Output;

	explicit HeadIO(UART_HandleTypeDef *uart) :
			Input { uart }, Output { uart } {
	}
};

#endif
