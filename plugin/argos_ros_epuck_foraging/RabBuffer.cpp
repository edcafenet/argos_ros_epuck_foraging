#include "RabBuffer.h"

namespace argos {
  /****************************************/
  /****************************************/

  RabBuffer::RabBuffer() {
    m_unCurrentTime = 0;
  }

  /****************************************/
  /****************************************/

  RabBuffer::~RabBuffer() {}

  /****************************************/
  /****************************************/

  void RabBuffer::SetTimeLife(const UInt32& un_max_time_to_live) {
    m_unMaxTimeToLive = un_max_time_to_live;
  }

  /****************************************/
  /****************************************/

  void RabBuffer::Update() {
    m_unCurrentTime += 1;
    for (UInt32 i = 0; i < m_vecBufferElements.size(); i++) {
      if (m_vecBufferElements.at(i).second < (m_unCurrentTime - m_unMaxTimeToLive)) {
        m_vecBufferElements.erase(m_vecBufferElements.begin() + i);
      }
    }
  }

  /****************************************/
	/****************************************/

  void RabBuffer::AddMessage(CCI_EPuckRangeAndBearingSensor::SReceivedPacket* c_packet) {
    m_vecBufferElements.push_back(std::make_pair(*c_packet, m_unCurrentTime));
  }

  /****************************************/
  /****************************************/

  std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> RabBuffer::GetMessages(){
    std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> vecRabMessages;
    std::vector<std::pair<CCI_EPuckRangeAndBearingSensor::SReceivedPacket, UInt32> >::iterator it;
    for (it = m_vecBufferElements.begin(); it != m_vecBufferElements.end(); it++) {
      vecRabMessages.push_back(&(*it).first);
    }
    return vecRabMessages;
  }

  /****************************************/
  /****************************************/

  void RabBuffer::Reset() {
    m_vecBufferElements.clear();
    m_unCurrentTime = 0;
  }
}
