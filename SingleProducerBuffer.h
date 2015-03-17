#pragma once
#include "stdafx.h"
#include "Buffer.h"



template < class DataType >
class SingleProducerBuffer :
	public Buffer< DataType >
{
public:
	SingleProducerBuffer();
	~SingleProducerBuffer();

	void pushData(DataType newData);

	
private:
	int m_pushDataPosition;
};

