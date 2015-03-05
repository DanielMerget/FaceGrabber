#pragma once
#include "Buffer.h"

class BufferSynchronizer
{
public:
	BufferSynchronizer();
	~BufferSynchronizer();

private:
	std::vector<Buffer> m_buffers;
};

