#ifndef COMMON
#define COMMON

#include <array>
#include <cstdlib>

struct MotherboardContext;

static constexpr size_t BufferSize = 256;
using BufferType = std::array<uint8_t, BufferSize>;

static constexpr size_t HeadRequestQueueSize = 16;
static constexpr size_t HeadResponceQueueSize = 16;
static constexpr size_t BodyQueueMaxSize = 500;
static constexpr size_t FrameQueueMaxSize = 300;

#endif
