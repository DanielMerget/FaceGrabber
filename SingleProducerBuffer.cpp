#include "SingleProducerBuffer.h"

template < class DataType >
SingleProducerBuffer< DataType >::SingleProducerBuffer():
	m_currentPosition(0),
	Buffer< DataType >()
{
}

template < class DataType >
SingleProducerBuffer< DataType >::~SingleProducerBuffer()
{
}

template < class DataType >
void SingleProducerBuffer< DataType >::pushData(DataType newData)
{
	this->pushData(newData, m_pushDataPosition);
	m_pushDataPosition = (m_pushDataPosition +1) % this->getBufferSize();
}