/*
 * StarkitMotherboard.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: tndrd
 */

#include <usart.h>

#include <queue>
#include <vector>

namespace Starkit {

class ControllerInterface {
	struct Request {
		std::vector<unsigned char> Data;
		size_t ResponceSize;
		bool ForwardResponce;
	};

	struct Responce {
		std::vector<unsigned char> Data;
	};

	std::queue<Request> Requests;
	std::queue<Responce> Responces;

	size_t SendInterval;
};

struct Quaternion {
	int x;
	int y;
	int z;
	int w;
};

struct IMUFrame {
	Quaternion Orientation;
	int Seq;
};

class IMUInterface {
	std::queue<IMUFrame> Frames;
};

};


