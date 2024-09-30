/**
* \file
* \brief Laser Scanner communication (TCP Helper Class)
* Copyright (C) 2013, Osnabrueck University
* Copyright (C) 2017, Ing.-Buero Dr. Michael Lehning, Hildesheim
* Copyright (C) 2017, SICK AG, Waldkirch
* All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrueck University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*     * Neither the name of SICK AG nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*  Last modified: 12th Dec 2017
*
*      Authors:
*         Michael Lehning <michael.lehning@lehning.de>
*         Jochen Sprickerhof <jochen@sprickerhof.de>
*         Martin Günther <mguenthe@uos.de>
*
* Based on the TiM communication example by SICK AG.
*
*/

#ifdef _MSC_VER
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)
#pragma warning(disable: 4101)   // C4101: "e" : Unreferenzierte lokale Variable
//#define _WIN32_WINNT 0x0501

#endif

#include "sick_scan_common_tcp.h"
#include "abstract_parser.h"
#include "tcp/colaa.hpp"
#include "tcp/colab.hpp"

#include <algorithm>
#include <iterator>
#include <vector>
#include <cassert>
#include <sstream>
#include <limits>


namespace sick_scan_xd
{

/**
 \brief calculate crc-code for last byte of binary message
    XOR-calucation is done ONLY over message content (i.e. skipping the first 8 Bytes holding 0x02020202 <Length Information as 4-byte long>)
 \param msgBlock: message content
 \param len: Length of message content in byte
 \return XOR-calucation abount message content (msgBlock[0] ^ msgBlock[1] .... msgBlock[len-1]
*/
unsigned char sick_crc8(unsigned char *msgBlock, int len)
{
    unsigned char xorVal = 0x00;
    int off = 0;
    for (int i = off; i < len; i++)
    {
        unsigned char val = msgBlock[i];
        xorVal ^= val;
    }
    return (xorVal);
}

/*!
\brief Universal swapping function
\param ptr: Pointer to datablock
\param numBytes : size of variable in bytes
*/
void swap_endian(unsigned char *ptr, int numBytes)
{
  unsigned char *buf = (ptr);
  unsigned char tmpChar;
  for (int i = 0; i < numBytes / 2; i++)
  {
    tmpChar = buf[numBytes - 1 - i];
    buf[numBytes - 1 - i] = buf[i];
    buf[i] = tmpChar;
  }
}

/**
 \brief Converts a SOPAS command to a human readable string
 \param s: ASCII-Sopas command including 0x02 and 0x03
 \return Human readable string 0x02 and 0x02 are converted to "<STX>" and "<ETX>"
*/
std::string stripControl(std::vector<unsigned char> s, int max_strlen = -1)
{
    bool isParamBinary = false;
    int spaceCnt = 0x00;
    int cnt0x02 = 0;

    for (size_t i = 0; i < s.size(); i++)
    {
        if (s[i] != 0x02)
        {
            isParamBinary = false;
        }
        else
        {
            cnt0x02++;
        }
        if (i > 4)
        {
            break;
        }
    }

    if (4 == cnt0x02)
    {
        isParamBinary = true;
    }

    std::string dest;
    if (isParamBinary == true)
    {
        int parseState = 0;

        unsigned long lenId = 0x00;
        char szDummy[255] = {0};
        for (size_t i = 0; i < s.size(); i++)
        {
            switch (parseState)
            {
                case 0:
                {
                    if (s[i] == 0x02)
                    {
                        dest += "<STX>";
                    }
                    else
                    {
                        dest += "?????";
                    }
                    if (i == 3)
                    {
                        parseState = 1;
                    }
                    break;
                }
                case 1:
                {
                    lenId |= s[i] << (8 * (7 - i));
                    if (i == 7)
                    {
                        sprintf(szDummy, "<Len=%04lu>", lenId);
                        dest += szDummy;
                        parseState = 2;
                    }
                    break;
                }
                case 2:
                {
                    unsigned long dataProcessed = i - 8;
                    if (s[i] == ' ')
                    {
                        spaceCnt++;
                    }
                    if (spaceCnt == 2)
                    {
                        parseState = 3;
                    }
                    dest += s[i];
                    if (dataProcessed >= (lenId - 1))
                    {
                        parseState = 4;
                    }
                    break;
                }
                case 3:
                {
                    char ch = dest[dest.length() - 1];
                    if (ch != ' ')
                    {
                    dest += ' ';
                    }
                    sprintf(szDummy, "0x%02x", s[i]);
                    dest += szDummy;

                    unsigned long dataProcessed = i - 8;
                    if (dataProcessed >= (lenId - 1))
                    {
                    parseState = 4;
                    }
                    break;
                }
                case 4:
                {
                    sprintf(szDummy, " CRC:<0x%02x>", s[i]);
                    dest += szDummy;
                    break;
                }
                default:
                    break;
            }
        }
    }
    else
    {
        for (size_t i = 0; i < s.size(); i++)
        {
            if (s[i] >= ' ')
            {
                // <todo> >= 0x80
                dest += s[i];
            }
            else
            {
                switch (s[i])
                {
                    case 0x02:
                        dest += "<STX>";
                        break;
                    case 0x03:
                        dest += "<ETX>";
                    break;
                }
            }
        }
    }

    if(max_strlen > 0 && dest.size() > max_strlen)
    {
        dest.resize(max_strlen);
        dest += "...";
    }

    return (dest);
}

std::vector<unsigned char> stringToVector(std::string s)
{
    std::vector<unsigned char> result;
    for (size_t j = 0; j < s.length(); j++)
    {
        result.push_back(s[j]);
    }
    return result;
}

SickScanCommonTcp::SickScanCommonTcp(const std::string &hostname, const std::string &port, int &timelimit, char cola_dialect_id)
    : hostname_(hostname), port_(port), timelimit_(timelimit)
{
    if((cola_dialect_id == 'a') || (cola_dialect_id == 'A'))
    {
        this->setProtocolType(CoLa_A);
    }
    if((cola_dialect_id == 'b') || (cola_dialect_id == 'B'))
    {
        this->setProtocolType(CoLa_B);
    }

    assert(this->getProtocolType() != CoLa_Unknown);

    m_numberOfBytesInReceiveBuffer = 0;
    m_alreadyReceivedBytes = 0;
    this->setReplyMode(0);
}

SickScanCommonTcp::~SickScanCommonTcp()
{
    // stop_scanner(true);
    this->close_device();
}


int SickScanCommonTcp::reinit(int delay_millisec)
{
    this->close_device();
    // usleep(delay_millisec * 1000);
    return 0;
}


void SickScanCommonTcp::disconnectFunction() {}

void SickScanCommonTcp::disconnectFunctionS(void *obj)
{
    if(obj != NULL)
    {
        ((SickScanCommonTcp *) (obj))->disconnectFunction();
    }
}

void SickScanCommonTcp::readCallbackFunctionS(void *obj, UINT8 *buffer, UINT32 &numOfBytes)
{
    ((SickScanCommonTcp *) obj)->readCallbackFunction(buffer, numOfBytes);
}


void SickScanCommonTcp::setReplyMode(int _mode)
{
    m_replyMode = _mode;
}

int SickScanCommonTcp::getReplyMode()
{
    return (m_replyMode);
}



void SickScanCommonTcp::setReadTimeOutInMs(int timeOutInMs)
{
    readTimeOutInMs = timeOutInMs;
}

int SickScanCommonTcp::getReadTimeOutInMs()
{
    return (readTimeOutInMs);
}


int SickScanCommonTcp::getProtocolType(void)
{
    return m_protocolId;
}

void SickScanCommonTcp::setProtocolType(SopasProtocol cola_dialect_id)
{
    m_protocolId = cola_dialect_id;
}


//
// Look for 23-frame (STX/ETX) in receive buffer.
// Move frame to start of buffer
//
// Return: 0 : No (complete) frame found
//        >0 : Frame length
//
SopasEventMessage SickScanCommonTcp::findFrameInReceiveBuffer()
{
    UINT32 frameLen = 0;
    UINT32 i;

    // Depends on protocol...
    if(this->getProtocolType() == CoLa_A)
    {
        //
        // COLA-A
        //
        // Must start with STX (0x02)
        if(m_receiveBuffer[0] != 0x02)
        {
            // Look for starting STX (0x02)
            for(i = 1; i < m_numberOfBytesInReceiveBuffer; i++)
            {
                if(m_receiveBuffer[i] == 0x02) break;
            }

            // Found beginning of frame?
            if(i >= m_numberOfBytesInReceiveBuffer)
            {
                // No start found, everything can be discarded
                m_numberOfBytesInReceiveBuffer = 0; // Invalidate buffer
                return SopasEventMessage{}; // No frame found
            }

            // Move frame start to index 0
            UINT32 newLen = m_numberOfBytesInReceiveBuffer - i;
            memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[i]), newLen);
            m_numberOfBytesInReceiveBuffer = newLen;
        }

        // Look for ending ETX (0x03)
        for(i = 1; i < m_numberOfBytesInReceiveBuffer; i++)
        {
            if(m_receiveBuffer[i] == 0x03) break;
        }

        // Found end?
        if(i >= m_numberOfBytesInReceiveBuffer)
        {
            // No end marker found, so it's not a complete frame (yet)
            return SopasEventMessage(); // No frame found
        }

        // Calculate frame length in byte
        frameLen = i + 1;

        return SopasEventMessage{ m_receiveBuffer, CoLa_A, frameLen };
    }
    else if(this->getProtocolType() == CoLa_B)
    {
        UINT32 magicWord;
        UINT32 payloadlength;

        if(m_numberOfBytesInReceiveBuffer < 4)
        {
            return SopasEventMessage{};
        }

        UINT16 pos = 0;
        magicWord = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
        if(magicWord != 0x02020202)
        {
            // Look for starting STX (0x02020202)
            for(i = 1; i <= m_numberOfBytesInReceiveBuffer - 4; i++)
            {
                pos = i; // this is needed, as the position value is updated by getIntegerFromBuffer
                magicWord = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
                if(magicWord == 0x02020202)
                {
                    // found magic word
                    break;
                }
            }

            // Found beginning of frame?
            if(i > m_numberOfBytesInReceiveBuffer - 4)
            {
                // No start found, everything can be discarded
                m_numberOfBytesInReceiveBuffer = 0; // Invalidate buffer
                return SopasEventMessage{}; // No frame found
            }
            else
            {
                // Move frame start to index
                UINT32 bytesToMove = m_numberOfBytesInReceiveBuffer - i;
                memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[i]), bytesToMove); // payload+magic+length+s+checksum
                m_numberOfBytesInReceiveBuffer = bytesToMove;
            }
        }

        // Pruefe Laenge des Pufferinhalts
        if(m_numberOfBytesInReceiveBuffer < 9)
        {
            // Es sind nicht genug Daten fuer einen Frame
            printInfoMessage("SickScanCommonNw::findFrameInReceiveBuffer: Frame cannot be decoded yet, only " +
                            ::toString(m_numberOfBytesInReceiveBuffer) + " bytes in the buffer.", m_beVerbose);
            return SopasEventMessage{};
        }

        // Read length of payload
        pos = 4;
        payloadlength = colab::getIntegerFromBuffer<UINT32>(m_receiveBuffer, pos);
        printInfoMessage(
            "SickScanCommonNw::findFrameInReceiveBuffer: Decoded payload length is " + ::toString(payloadlength) +
            " bytes.", m_beVerbose);

        // Ist die Datenlaenge plausibel und wuede in den Puffer passen?
        if(payloadlength > (sizeof(m_receiveBuffer) - 9))
        {
            // magic word + length + checksum = 9
            printWarning(
                "SickScanCommonNw::findFrameInReceiveBuffer: Frame too big for receive buffer. Frame discarded with length:"
                + ::toString(payloadlength) + ".");
            m_numberOfBytesInReceiveBuffer = 0;
            return SopasEventMessage{};
        }
        if((payloadlength + 9) > m_numberOfBytesInReceiveBuffer)
        {
            // magic word + length + s + checksum = 10
            printInfoMessage(
                "SickScanCommonNw::findFrameInReceiveBuffer: Frame not complete yet. Waiting for the rest of it (" +
                ::toString(payloadlength + 9 - m_numberOfBytesInReceiveBuffer) + " bytes missing).", m_beVerbose);
            return SopasEventMessage{}; // frame not complete
        }

        // Calculate the total frame length in bytes: Len = Frame (9 bytes) + Payload
        frameLen = payloadlength + 9;

        //
        // test checksum of payload
        //
        UINT8 temp = 0;
        UINT8 temp_xor = 0;
        UINT8 checkSum;

        // Read original checksum
        pos = frameLen - 1;
        checkSum = colab::getIntegerFromBuffer<UINT8>(m_receiveBuffer, pos);

        // Erzeuge die Pruefsumme zum Vergleich
        for(UINT16 j = 8; j < (frameLen - 1); j++)
        {
            pos = j;
            temp = colab::getIntegerFromBuffer<UINT8>(m_receiveBuffer, pos);
            temp_xor = temp_xor ^ temp;
        }

        // Vergleiche die Pruefsummen
        if(temp_xor != checkSum)
        {
            printWarning("SickScanCommonNw::findFrameInReceiveBuffer: Wrong checksum, Frame discarded.");
            m_numberOfBytesInReceiveBuffer = 0;
            return SopasEventMessage{};
        }

        return SopasEventMessage{ m_receiveBuffer, CoLa_B, frameLen };
    }

    // Return empty frame
    return SopasEventMessage{};
}


/**
* Read callback. Diese Funktion wird aufgerufen, sobald Daten auf der Schnittstelle
* hereingekommen sind.
*/
void SickScanCommonTcp::processFrame(rosTime timeStamp, SopasEventMessage &frame)
{
    if(this->getProtocolType() == CoLa_A)
    {
        printInfoMessage(
            "SickScanCommonNw::processFrame: Calling processFrame_CoLa_A() with " + ::toString(frame.size()) + " bytes.",
            m_beVerbose);
        // processFrame_CoLa_A(frame);
    }
    else if(this->getProtocolType() == CoLa_B)
    {
        printInfoMessage(
            "SickScanCommonNw::processFrame: Calling processFrame_CoLa_B() with " + ::toString(frame.size()) + " bytes.",
            m_beVerbose);
        // processFrame_CoLa_B(frame);
    }

    // Push frame to recvQueue
    DatagramWithTimeStamp dataGramWidthTimeStamp{ timeStamp, std::vector<unsigned char>(frame.getRawData(),
                                                                                        frame.getRawData() +
                                                                                        frame.size()) };
    // recvQueue.push(std::vector<unsigned char>(frame.getRawData(), frame.getRawData() + frame.size()));
    recvQueue.push(dataGramWidthTimeStamp);
}

void SickScanCommonTcp::readCallbackFunction(UINT8 *buffer, UINT32 &numOfBytes)
{
    rosTime rcvTimeStamp = ::rosTimeNow(); // stamp received datagram
    bool beVerboseHere = false;
    printInfoMessage(
        "SickScanCommonNw::readCallbackFunction(): Called with " + toString(numOfBytes) + " available bytes.",
        beVerboseHere);

    ScopedLock lock(&m_receiveDataMutex); // Mutex for access to the input buffer
    UINT32 remainingSpace = sizeof(m_receiveBuffer) - m_numberOfBytesInReceiveBuffer;
    UINT32 bytesToBeTransferred = numOfBytes;
    if(remainingSpace < numOfBytes)
    {
        bytesToBeTransferred = remainingSpace;
        // printWarning("SickScanCommonNw::readCallbackFunction(): Input buffer space is to small, transferring only " +
        //              ::toString(bytesToBeTransferred) + " of " + ::toString(numOfBytes) + " bytes.");
    }
    else
    {
        // printInfoMessage("SickScanCommonNw::readCallbackFunction(): Transferring " + ::toString(bytesToBeTransferred) +
        //                   " bytes from TCP to input buffer.", beVerboseHere);
    }

    if(bytesToBeTransferred > 0)
    {
        // Data can be transferred into our input buffer
        memcpy(&(m_receiveBuffer[m_numberOfBytesInReceiveBuffer]), buffer, bytesToBeTransferred);
        m_numberOfBytesInReceiveBuffer += bytesToBeTransferred;

        UINT32 size = 0;

        while (1)
        {
            // Now work on the input buffer until all received datasets are processed
            SopasEventMessage frame = this->findFrameInReceiveBuffer();

            size = frame.size();
            if(size == 0)
            {
                // Framesize = 0: There is no valid frame in the buffer. The buffer is either empty or the frame
                // is incomplete, so leave the loop
                printInfoMessage("SickScanCommonNw::readCallbackFunction(): No complete frame in input buffer, we are done.",
                                beVerboseHere);

                // Leave the loop
                break;
            }
            else
            {
                // A frame was found in the buffer, so process it now.
                printInfoMessage(
                    "SickScanCommonNw::readCallbackFunction(): Processing a frame of length " + ::toString(frame.size()) +
                    " bytes.", beVerboseHere);
                this->processFrame(rcvTimeStamp, frame);
                UINT32 bytesToMove = m_numberOfBytesInReceiveBuffer - size;
                memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[size]), bytesToMove); // payload+magic+length+s+checksum
                m_numberOfBytesInReceiveBuffer = bytesToMove;
            }
        }
    }
    else
    {
        // There was input data from the TCP interface, but our input buffer was unable to hold a single byte.
        // Either we have not read data from our buffer for a long time, or something has gone wrong. To re-sync,
        // we clear the input buffer here.
        m_numberOfBytesInReceiveBuffer = 0;
    }
}


int SickScanCommonTcp::init_device()
{
    int portInt;
    sscanf(port_.c_str(), "%d", &portInt);
    m_nw.init(hostname_, portInt, disconnectFunctionS, (void *) this);
    m_nw.setReadCallbackFunction(readCallbackFunctionS, (void *) this);
    m_nw.connect();
    return ExitSuccess;
}

int SickScanCommonTcp::close_device()
{
    m_nw.disconnect();
    return 0;
}


int SickScanCommonTcp::numberOfDatagramInInputFifo()
{
    int ret = 0;
    ret = this->recvQueue.getNumberOfEntriesInQueue();
    return(ret);
}

int SickScanCommonTcp::readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read, const std::vector<std::string>& datagram_keywords)
{
    bool retVal = this->recvQueue.waitForIncomingObject(timeout_ms, datagram_keywords);
    if(retVal == false)
    {
        ROS_WARN("Timeout during waiting for new datagram");
        return ExitError;
    }

    DatagramWithTimeStamp datagramWithTimeStamp = this->recvQueue.pop(datagram_keywords);
    if(datagramWithTimeStamp.datagram.size() > buffer_size)
    {
        ROS_WARN_STREAM("Length of received datagram is " << datagramWithTimeStamp.datagram.size() << " byte, exceeds buffer size (" << buffer_size << " byte), datagram truncated");
        datagramWithTimeStamp.datagram.resize(buffer_size);
    }

    *bytes_read = datagramWithTimeStamp.datagram.size();
    memcpy(buffer, &(datagramWithTimeStamp.datagram[0]), datagramWithTimeStamp.datagram.size());
    return (ExitSuccess);
}

/**
* Send a SOPAS command to the device and print out the response to the console.
*/
int SickScanCommonTcp::sendSOPASCommand(const char *request, std::vector<unsigned char> *reply, int cmdLen, bool wait_for_reply)
{
    int sLen = 0;
    int msgLen = 0;
    int preambelCnt = 0;
    bool cmdIsBinary = false;

    if(request != NULL)
    {
        sLen = cmdLen;
        preambelCnt = 0; // count 0x02 bytes to decide between ascii and binary command
        if(sLen >= 4)
        {
            for(int i = 0; i < 4; i++)
            {
                if(request[i] == 0x02)
                {
                    preambelCnt++;
                }
            }
        }

        if(preambelCnt < 4)
        {
            cmdIsBinary = false;
        }
        else
        {
            cmdIsBinary = true;
        }
        if(cmdIsBinary == false)
        {
            msgLen = strlen(request);
        }
        else
        {
            int dataLen = 0;
            for(int i = 4; i < 8; i++)
            {
                dataLen |= ((unsigned char) request[i] << (7 - i) * 8);
            }
            msgLen = 8 + dataLen + 1; // 8 Msg. Header + Packet + CRC
        }

        constexpr bool debugBinCmd = false;
        if constexpr(debugBinCmd)
        {
            printf("=== START HEX DUMP ===\n");
            for(int i = 0; i < msgLen; i++)
            {
                unsigned char *ptr = (UINT8 *) request;
                printf("%02x ", ptr[i]);
            }
            printf("\n=== END HEX DUMP ===\n");
        }
        if(!m_nw.sendCommandBuffer((UINT8 *) request, msgLen))
        {
            ROS_ERROR("## ERROR in sendSOPASCommand(): sendCommandBuffer failed");
            return ExitError;
        }
    }
    if(!wait_for_reply)
    {
        return ExitSuccess;
    }

    // Set timeout in 5 seconds
    const int BUF_SIZE = 65536;
    char buffer[BUF_SIZE];
    int bytes_read;

    std::vector<std::string> response_keywords = { SickScanCommonTcp::getSopasCmdKeyword((uint8_t*)request, msgLen) }; 
    if(this->readWithTimeout(this->getReadTimeOutInMs(), buffer, BUF_SIZE, &bytes_read, response_keywords) == ExitError)
    {
        // sopas timeout
        return ExitError;
    }

    if(reply)
    {
        reply->resize(bytes_read);
        std::copy(buffer, buffer + bytes_read, &(*reply)[0]);
    }

    return ExitSuccess;
}


int SickScanCommonTcp::get_datagram(rosTime &recvTimeStamp, unsigned char *receiveBuffer, int bufferSize,
                                    int *actual_length, bool isBinaryProtocol, int *numberOfRemainingFifoEntries, const std::vector<std::string>& datagram_keywords)
{
    if(NULL != numberOfRemainingFifoEntries)
    {
        // *numberOfRemainingFifoEntries = 0;
    }
    this->setReplyMode(1);

    const int maxWaitInMs = this->getReadTimeOutInMs();
    std::vector<unsigned char> dataBuffer;

    bool retVal = this->recvQueue.waitForIncomingObject(maxWaitInMs, datagram_keywords);
    if(retVal == false)
    {
        ROS_WARN("Timeout during waiting for new datagram");
        return ExitError;
    }
    else
    {
        // Look into receiving queue for new Datagrams
        DatagramWithTimeStamp datagramWithTimeStamp = this->recvQueue.pop(datagram_keywords);
        if(NULL != numberOfRemainingFifoEntries)
        {
            *numberOfRemainingFifoEntries = this->recvQueue.getNumberOfEntriesInQueue();
        }
        recvTimeStamp = datagramWithTimeStamp.timeStamp;
        dataBuffer = datagramWithTimeStamp.datagram;
    }

    // dataBuffer = this->recvQueue.pop();
    long size = dataBuffer.size();
    memcpy(receiveBuffer, &(dataBuffer[0]), size);
    *actual_length = size;

    return ExitSuccess;
}





/*!
 \brief Generate expected answer strings from the command string
 \param requestStr command string (either as ASCII or BINARY)
 \return expected answer string
*/
std::vector<std::string> SickScanCommonTcp::generateExpectedAnswerString(const std::vector<unsigned char> requestStr)
{
    std::string expectedAnswer = "";
    //int i = 0;
    char cntWhiteCharacter = 0;
    int initalTokenCnt = 2; // number of initial token to identify command
    std::map<std::string, int> specialTokenLen;
    char firstToken[1024] = {0};
    specialTokenLen["sRI"] = 1; // for SRi-Command only the first token identify the command
    std::string tmpStr = "";
    int cnt0x02 = 0;
    bool isBinary = false;

    for (size_t i = 0; i < 4; i++)
    {
        if (i < requestStr.size())
        {
            if (requestStr[i] == 0x02)
            {
                cnt0x02++;
            }
        }
    }

    int iStop = requestStr.size();  // copy string until end of string
    if (cnt0x02 == 4)
    {
        int cmdLen = 0;
        for (int i = 0; i < 4; i++)
        {
            cmdLen |= cmdLen << 8;
            cmdLen |= requestStr[i + 4];
        }
        iStop = cmdLen + 8;
        isBinary = true;
    }

    int iStart = (isBinary == true) ? 8 : 0;
    for (int i = iStart; i < iStop; i++)
    {
        tmpStr += (char) requestStr[i];
    }

    if (isBinary)
    {
        tmpStr = "\x2" + tmpStr;
    }

    if (sscanf(tmpStr.c_str(), "\x2%s", firstToken) == 1)
    {
        if (specialTokenLen.find(firstToken) != specialTokenLen.end())
        {
            initalTokenCnt = specialTokenLen[firstToken];
        }
    }

    for (int i = iStart; i < iStop; i++)
    {
        if ((requestStr[i] == ' ') || (requestStr[i] == '\x3'))
        {
            cntWhiteCharacter++;
        }
        if (cntWhiteCharacter >= initalTokenCnt)
        {
            break;
        }
        if (requestStr[i] == '\x2')
        {}
        else
        {
            expectedAnswer += requestStr[i];
        }
    }

    /*!
    * Map that defines expected answer identifiers
    */
    std::map<std::string, std::vector<std::string>> keyWordMap;
    keyWordMap["sWN"] = { "sWA", "sAN" };
    keyWordMap["sRN"] = { "sRA", "sAN" };
    keyWordMap["sRI"] = { "sRA" };
    keyWordMap["sMN"] = { "sAN", "sMA" };
    keyWordMap["sEN"] = { "sEA" };

    std::vector<std::string> expectedAnswers;
    for (std::map<std::string, std::vector<std::string>>::iterator it = keyWordMap.begin(); it != keyWordMap.end(); it++)
    {
        const std::string& keyWord = it->first;
        const std::vector<std::string>& newKeyWords = it->second;

        size_t pos = expectedAnswer.find(keyWord);
        if (pos == 0)  // must be 0, if keyword has been found
        {
            for(int n = 0; n < newKeyWords.size(); n++)
            {
                expectedAnswers.push_back(expectedAnswer);
                expectedAnswers.back().replace(pos, keyWord.length(), newKeyWords[n]);
            }
        }
        else if (pos != std::string::npos) // keyword found at unexpected position
        {
            ROS_WARN("Unexpected position of key identifier.\n");
        }
    }

    if(expectedAnswers.empty())
    {
        expectedAnswers.push_back(expectedAnswer);
    }

    return (expectedAnswers);
}

/**
 \brief Convert ASCII-message to Binary-message
 \param requestAscii holds ASCII-encoded command
 \param requestBinary hold binary command as vector of unsigned char
 \return success = 0
*/
int SickScanCommonTcp::convertAscii2BinaryCmd(const char *requestAscii, std::vector<unsigned char> *requestBinary)
{
    requestBinary->clear();
    if (requestAscii == NULL)
    {
        return (-1);
    }
    int cmdLen = strlen(requestAscii);
    if (cmdLen == 0)
    {
        return (-1);
    }
    if (requestAscii[0] != 0x02)
    {
        return (-1);
    }
    if (requestAscii[cmdLen - 1] != 0x03)
    {
        return (-1);
    }
    // Here we know, that the ascii format is correctly framed with <stx> .. <etx>
    for (int i = 0; i < 4; i++)
    {
        requestBinary->push_back(0x02);
    }

    for (int i = 0; i < 4; i++) // Puffer for Length identifier
    {
        requestBinary->push_back(0x00);
    }

    unsigned long msgLen = cmdLen - 2; // without stx and etx

    // special handling for the following commands
    // due to higher number of command arguments
    std::string keyWord0 = "sMN SetAccessMode";
    std::string keyWord1 = "sWN FREchoFilter";
    std::string keyWord2 = "sEN LMDscandata";
    std::string keyWord3 = "sWN LMDscandatacfg";
    std::string keyWord4 = "sWN SetActiveApplications"; // "sWN SetActiveApplications 2 FEVL <0|1> RANG 1" for MRS-1xxx with firmware >= 2.x
    std::string keyWord5 = "sEN IMUData";
    std::string keyWord6 = "sWN EIIpAddr";
    std::string keyWord7 = "sMN mLMPsetscancfg";
    std::string keyWord8 = "sWN TSCTCupdatetime";
    std::string keyWord9 = "sWN TSCTCSrvAddr";
    std::string keyWord10 = "sWN LICencres";
    std::string keyWord11 = "sWN LFPmeanfilter";
    std::string KeyWord12 = "sRN field";
    std::string KeyWord13 = "sMN mCLsetscancfglist";
    std::string KeyWord14 = "sWN LFPmeanfilter"; // MRS1xxx, LMS1xxx, LMS4xxx, LRS4xxx: "sWN LFPmeanfilter" + { 1 byte 0|1 active/inactive } + { 2 byte 0x02 ... 0x64 number of scans } + { 1 byte 0x00 }
    std::string KeyWord15 = "sWN LFPmedianfilter"; // MRS1xxx, LMS1xxx, LMS4xxx, LRS4xxx: "sWN LFPmedianfilter" (3x1 median filter) + { 1 byte 0|1 active/inactive } + { 2 byte 0x03 }
    std::string KeyWord16 = "sWN LMDscandatascalefactor"; // LRS4xxx: "sWN LMDscandatascalefactor" + { 4 byte float }, e.g. scalefactor 1.0f = 0x3f800000, scalefactor 2.0f = 0x40000000
    std::string KeyWord17 = "sWN GlareDetectionSens"; // LRS4xxx: "sWN GlareDetectionSens"  + { 1 byte sensitivity }  + { 2 byte 0x03 }
    std::string KeyWord18 = "sWN ScanLayerFilter"; // MRS-1000 scan layer activation mask, "sWN ScanLayerFilter <number of layers> <layer 1: on/off> … <layer N: on/off>",  default: all layer activated: "sWN ScanLayerFilter 4 1 1 1 1"
    std::string KeyWord19 = "sMN mNPOSGetData"; // NAV-350 poll data: "sMN mNPOSGetData 1 2" (sopas arguments: wait = 1, i.e. wait for next pose result), mask = 2, i.e. send pose+reflectors+scan)
    std::string KeyWord20 = "sMN mNPOSSetPose"; // Set NAV-350 start pose in navigation mode by "sMN mNPOSSetPose X Y Phi"
    std::string KeyWord21 = "sWN NEVACurrLayer";
    std::string KeyWord22 = "sWN NMAPMapCfg";
    std::string KeyWord23 = "sWN NLMDReflSize";
    std::string KeyWord24 = "sWN NPOSPoseDataFormat";
    std::string KeyWord25 = "sWN NLMDLandmarkDataFormat";
    std::string KeyWord26 = "sWN NAVScanDataFormat";
    std::string KeyWord27 = "sMN mNLAYEraseLayout";

    //BBB

    std::string cmdAscii = requestAscii;


    int copyUntilSpaceCnt = 2;
    int spaceCnt = 0;
    char hexStr[255] = {0};
    int level = 0;
    unsigned char buffer[255];
    int bufferLen = 0;
    if (cmdAscii.find(keyWord0) != std::string::npos) // SetAccessMode
    {
        copyUntilSpaceCnt = 2;
        int keyWord0Len = keyWord0.length();
        sscanf(requestAscii + keyWord0Len + 1, " %d %s", &level, hexStr);
        buffer[0] = (unsigned char) (0xFF & level);
        bufferLen = 1;
        char hexTmp[3] = {0};
        for (int i = 0; i < 4; i++)
        {
            int val;
            hexTmp[0] = hexStr[i * 2];
            hexTmp[1] = hexStr[i * 2 + 1];
            hexTmp[2] = 0x00;
            sscanf(hexTmp, "%x", &val);
            buffer[i + 1] = (unsigned char) (0xFF & val);
            bufferLen++;
        }
    }

    else if (cmdAscii.find(keyWord1) != std::string::npos)
    {
        int echoCodeNumber = 0;
        int keyWord1Len = keyWord1.length();
        sscanf(requestAscii + keyWord1Len + 1, " %d", &echoCodeNumber);
        buffer[0] = (unsigned char) (0xFF & echoCodeNumber);
        bufferLen = 1;
    }

    else if (cmdAscii.find(keyWord2) != std::string::npos)
    {
        int scanDataStatus = 0;
        int keyWord2Len = keyWord2.length();
        sscanf(requestAscii + keyWord2Len + 1, " %d", &scanDataStatus);
        buffer[0] = (unsigned char) (0xFF & scanDataStatus);
        bufferLen = 1;
    }

    else if (cmdAscii.find(keyWord3) != std::string::npos)
    {
        int scanDataStatus = 0;
        int keyWord3Len = keyWord3.length();
        int dummyArr[12] = {0};
        //sWN LMDscandatacfg %02d 00 %d %d 0 0 %02d 0 0 0 1 1\x03"
        int sscanfresult = sscanf(requestAscii + keyWord3Len + 1, " %d %d %d %d %d %d %d %d %d %d %d %d",
                                    &dummyArr[0], // Data Channel Idx LSB
                                    &dummyArr[1], // Data Channel Idx MSB
                                    &dummyArr[2], // Remission
                                    &dummyArr[3], // Remission data format
                                    &dummyArr[4], // Unit
                                    &dummyArr[5], // Encoder Setting LSB
                                    &dummyArr[6], // Encoder Setting MSB
                                    &dummyArr[7], // Position
                                    &dummyArr[8], // Send Name
                                    &dummyArr[9], // Send Comment
                                    &dummyArr[10], // Time information
                                    &dummyArr[11]); // n-th Scan (packed - not sent as single byte sequence) !!!

        if (1 < sscanfresult)
        {
            for (int i = 0; i < 13; i++)
            {
                buffer[i] = 0x00;
            }
            buffer[0] = (unsigned char) (0xFF & dummyArr[0]);  //Data Channel 2 Bytes
            buffer[1] = (unsigned char) (0xFF & dummyArr[1]);; // MSB of Data Channel (here Little Endian!!)
            buffer[2] = (unsigned char) (0xFF & dummyArr[2]);  // Remission
            buffer[3] = (unsigned char) (0xFF & dummyArr[3]);  // Remission data format 0=8 bit 1= 16 bit
            buffer[4] = (unsigned char) (0xFF & dummyArr[4]);  //Unit of remission data
            buffer[5] = (unsigned char) (0xFF & dummyArr[5]);  //encoder Data LSB
            buffer[6] = (unsigned char) (0xFF & dummyArr[6]);  //encoder Data MSB
            buffer[7] = (unsigned char) (0xFF & dummyArr[7]);  // Position
            buffer[8] = (unsigned char) (0xFF & dummyArr[8]);  // Send Scanner Name
            buffer[9] = (unsigned char) (0xFF & dummyArr[9]);  // Comment
            buffer[10] = (unsigned char) (0xFF & dummyArr[10]);  // Time information
            buffer[11] = (unsigned char) (0xFF & (dummyArr[11] >> 8));  // BIG Endian High Byte nth-Scan
            buffer[12] = (unsigned char) (0xFF & (dummyArr[11] >> 0));  // BIG Endian Low Byte nth-Scan
            bufferLen = 13;
        }
    }

    else if (cmdAscii.find(keyWord4) != std::string::npos) // "sWN SetActiveApplications 1 FEVL 1" or "sWN SetActiveApplications 1 RANG 1"
    {
        char tmpStr[1024] = {0};
        char szApplStr[255] = {0};
        int keyWord4Len = keyWord4.length();
        int scanDataStatus = 0;
        int dummy0, dummy1;
        strcpy(tmpStr, requestAscii + keyWord4Len + 2);
        sscanf(tmpStr, "%d %s %d", &dummy0, szApplStr, &dummy1);
        // rebuild string
        buffer[0] = 0x00;
        buffer[1] = dummy0 ? 0x01 : 0x00;
        for (int ii = 0; ii < 4; ii++)
        {
            buffer[2 + ii] = szApplStr[ii]; // idx: 1,2,3,4
        }
        buffer[6] = dummy1 ? 0x01 : 0x00;
        bufferLen = 7;
        // if (parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_MRS_1XXX_NAME) == 0) // activate FEVL and RANG in case of MRS1xxx with firmware version > 1
        // {
        //   std::vector<int> version_id = parseFirmwareVersion("MRS1xxx", deviceIdentStr); // Get MRS1xxx version from device ident string
        //   if (version_id[0] > 1)
        //   {
        //     // buffer[6] = 0x01; // MRS1xxx with firmware version > 1 supports RANG+FEVL -> overwrite with "<STX>sWN{SPC}SetActiveApplications{SPC}1{SPC}FEVL{SPC}1<ETX>"
        //     // MRS1xxx with firmware version > 1 supports RANG+FEVL -> overwrite with "<STX>sWN{SPC}SetActiveApplications{SPC}2{SPC}FEVL{SPC}1{SPC}RANG{SPC}1<ETX>"
        //     // resp. binary "sWN SetActiveApplications \00\02\46\45\56\4C\01\52\41\4e\47\01"
        //     uint8_t field_evaluation_status = isFieldEvaluationActive() ? 0x01: 0x00;
        //     std::vector<uint8_t> binary_parameter = {0x00, 0x02, 0x46, 0x45, 0x56, 0x4C, field_evaluation_status, 0x52, 0x41, 0x4e, 0x47, 0x01};
        //     for (int ii = 0; ii < binary_parameter.size(); ii++)
        //       buffer[ii] = binary_parameter[ii];
        //     bufferLen = binary_parameter.size();
        //   }
        //   // Disable scandatacfg_azimuth_table if firmware version is < 2.2
        //   if (version_id[0] < 2)
        //     parser_->getCurrentParamPtr()->setScandatacfgAzimuthTableSupported(false);
        //   if (version_id[0] == 2 && version_id[1] < 2)
        //     parser_->getCurrentParamPtr()->setScandatacfgAzimuthTableSupported(false);
        // }
        // if (parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_LMS_1XXX_NAME) == 0)
        // {
        //   std::vector<int> version_id = parseFirmwareVersion("LMS1xxx", deviceIdentStr); // Get LMS1xxx version from device ident string
        //   // Disable scandatacfg_azimuth_table if firmware version is < 2.2
        //   if (version_id[0] < 2)
        //     parser_->getCurrentParamPtr()->setScandatacfgAzimuthTableSupported(false);
        //   if (version_id[0] == 2 && version_id[1] < 2)
        //     parser_->getCurrentParamPtr()->setScandatacfgAzimuthTableSupported(false);
        // }
    }

    else if (cmdAscii.find(keyWord5) != std::string::npos)
    {
        int imuSetStatus = 0;
        int keyWord5Len = keyWord5.length();
        sscanf(requestAscii + keyWord5Len + 1, " %d", &imuSetStatus);
        buffer[0] = (unsigned char) (0xFF & imuSetStatus);
        bufferLen = 1;
    }

    else if (cmdAscii.find(keyWord6) != std::string::npos)
    {
        int adrPartArr[4];
        int imuSetStatus = 0;
        int keyWord6Len = keyWord6.length();
        sscanf(requestAscii + keyWord6Len + 1, " %x %x %x %x", &(adrPartArr[0]), &(adrPartArr[1]), &(adrPartArr[2]),
                &(adrPartArr[3]));
        buffer[0] = (unsigned char) (0xFF & adrPartArr[0]);
        buffer[1] = (unsigned char) (0xFF & adrPartArr[1]);
        buffer[2] = (unsigned char) (0xFF & adrPartArr[2]);
        buffer[3] = (unsigned char) (0xFF & adrPartArr[3]);
        bufferLen = 4;
    }
    //\x02sMN mLMPsetscancfg %d 1 %d 0 0\x03";
    //02 02 02 02 00 00 00 25 73 4D 4E 20 6D 4C 4D 50 73 65 74 73 63 61 6E 63 66 67 20
    // 00 00 13 88 4byte freq
    // 00 01 2 byte sectors always 1
    // 00 00 13 88  ang_res
    // FF F9 22 30 sector start always 0
    // 00 22 55 10 sector stop  always 0
    // 21
    else if (cmdAscii.find(keyWord7) != std::string::npos)
    {
#if 1
        bufferLen = 0;
        for (int i = keyWord7.length() + 2, i_max = strlen(requestAscii) - 1; i + 3 < i_max && bufferLen < sizeof(buffer); i += 4, bufferLen++)
        {
            char hex_str[] = { requestAscii[i + 2], requestAscii[i + 3], '\0' };
            buffer[bufferLen] = (std::stoul(hex_str, nullptr, 16) & 0xFF);
        }
#else
        // if (this->parser_->getCurrentParamPtr()->getScannerName().compare(SICK_SCANNER_NAV_31X_NAME) != 0)
        // {
        //     {
        //     bufferLen = 18;
        //     for (int i = 0; i < bufferLen; i++)
        //     {
        //         unsigned char uch = 0x00;
        //         switch (i)
        //         {
        //         case 5:
        //             uch = 0x01;
        //             break;
        //         }
        //         buffer[i] = uch;
        //     }
        //     char tmpStr[1024] = {0};
        //     char szApplStr[255] = {0};
        //     int keyWord7Len = keyWord7.length();
        //     int scanDataStatus = 0;
        //     int dummy0, dummy1;
        //     strcpy(tmpStr, requestAscii + keyWord7Len + 2);
        //     sscanf(tmpStr, "%d 1 %d", &dummy0, &dummy1);

        //     buffer[0] = (unsigned char) (0xFF & (dummy0 >> 24));
        //     buffer[1] = (unsigned char) (0xFF & (dummy0 >> 16));
        //     buffer[2] = (unsigned char) (0xFF & (dummy0 >> 8));
        //     buffer[3] = (unsigned char) (0xFF & (dummy0 >> 0));


        //     buffer[6] = (unsigned char) (0xFF & (dummy1 >> 24));
        //     buffer[7] = (unsigned char) (0xFF & (dummy1 >> 16));
        //     buffer[8] = (unsigned char) (0xFF & (dummy1 >> 8));
        //     buffer[9] = (unsigned char) (0xFF & (dummy1 >> 0));
        //     }
        // }
        // else
        {
            int keyWord7Len = keyWord7.length();
            int dummyArr[14] = {0};
            if (14 == sscanf(requestAscii + keyWord7Len + 1, " %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                            &dummyArr[0], &dummyArr[1], &dummyArr[2],
                            &dummyArr[3], &dummyArr[4], &dummyArr[5],
                            &dummyArr[6], &dummyArr[7], &dummyArr[8],
                            &dummyArr[9], &dummyArr[10], &dummyArr[11], &dummyArr[12], &dummyArr[13]))
            {
            for (int i = 0; i < 54; i++)
            {
                buffer[i] = 0x00;
            }
            int targetPosArr[] = {0, 4, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 50, 54};
            int numElem = (sizeof(targetPosArr) / sizeof(targetPosArr[0])) - 1;
            for (int i = 0; i < numElem; i++)
            {
                int lenOfBytesToRead = targetPosArr[i + 1] - targetPosArr[i];
                int adrPos = targetPosArr[i];
                unsigned char *destPtr = buffer + adrPos;
                memcpy(destPtr, &(dummyArr[i]), lenOfBytesToRead);
                swap_endian(destPtr, lenOfBytesToRead);
            }
            bufferLen = targetPosArr[numElem];
            /**
              00 00 03 20 00 01
              00 00 09 C4 00 00 00 00 00 36 EE 80 00 00 09 C4 00 00 00 00 00 00 00 00 00 00 09 C4 00 00 00 00 00
              00 00 00 00 00 09 C4 00 00 00 00 00 00 00 00 00 00 09 C4 00 00 00 00 00 00 00 00 E4
            */

            }
        }
#endif
    }

    else if (cmdAscii.find(keyWord8) != std::string::npos)
    {
        uint32_t updatetime = 0;
        int keyWord8Len = keyWord8.length();
        sscanf(requestAscii + keyWord8Len + 1, " %d", &updatetime);
        buffer[0] = (unsigned char) (0xFF & (updatetime >> 24));
        buffer[1] = (unsigned char) (0xFF & (updatetime >> 16));
        buffer[2] = (unsigned char) (0xFF & (updatetime >> 8));
        buffer[3] = (unsigned char) (0xFF & (updatetime >> 0));
        bufferLen = 4;
    }

    else if (cmdAscii.find(keyWord9) != std::string::npos)
    {
        int adrPartArr[4];
        int imuSetStatus = 0;
        int keyWord9Len = keyWord9.length();
        sscanf(requestAscii + keyWord9Len + 1, " %x %x %x %x", &(adrPartArr[0]), &(adrPartArr[1]), &(adrPartArr[2]),
                &(adrPartArr[3]));
        buffer[0] = (unsigned char) (0xFF & adrPartArr[0]);
        buffer[1] = (unsigned char) (0xFF & adrPartArr[1]);
        buffer[2] = (unsigned char) (0xFF & adrPartArr[2]);
        buffer[3] = (unsigned char) (0xFF & adrPartArr[3]);
        bufferLen = 4;
    }

    else if (cmdAscii.find(keyWord10) != std::string::npos)
    {
        float EncResolution = 0;
        bufferLen = 4;
        int keyWord10Len = keyWord10.length();
        sscanf(requestAscii + keyWord10Len + 1, " %f", &EncResolution);
        memcpy(buffer, &EncResolution, bufferLen);
        swap_endian(buffer, bufferLen);
    }

    else if (cmdAscii.find(keyWord11) != std::string::npos)
    {
        char tmpStr[1024] = {0};
        char szApplStr[255] = {0};
        int keyWord11Len = keyWord11.length();
        int dummy0, dummy1,dummy2;
        strcpy(tmpStr, requestAscii + keyWord11Len + 2);
        sscanf(tmpStr, "%d %d %d", &dummy0, &dummy1, &dummy2);
        // rebuild string
        buffer[0] = dummy0 ? 0x01 : 0x00;
        buffer[1] =dummy1/256;//
        buffer[2] =dummy1%256;//
        buffer[3] =dummy2;
        bufferLen = 4;
    }

    else if (cmdAscii.find(KeyWord12) != std::string::npos)
    {
        uint32_t fieldID = 0;
        int keyWord12Len = KeyWord12.length();
        sscanf(requestAscii + keyWord12Len + 1, "%d", &fieldID);
        bufferLen = 0;
    }

    else if (cmdAscii.find(KeyWord13) != std::string::npos)
    {
        int scanCfgListEntry = 0;
        int keyWord13Len = KeyWord13.length();
        sscanf(requestAscii + keyWord13Len + 1, " %d", &scanCfgListEntry);
        buffer[0] = (unsigned char) (0xFF & scanCfgListEntry);
        bufferLen = 1;
    }

    else if (cmdAscii.find(KeyWord14) != std::string::npos) // MRS1xxx, LMS1xxx, LMS4xxx, LRS4xxx: "sWN LFPmeanfilter" + { 1 byte 0|1 active/inactive } + { 2 byte 0x02 ... 0x64 number of scans } + { 1 byte 0x00 }
    {
        ROS_INFO_STREAM("convertAscii2BinaryCmd: requestAscii=" << requestAscii);
        int args[3] = { 0, 0, 0 };
        sscanf(requestAscii + KeyWord14.length() + 1, " %d %d %d", &(args[0]), &(args[1]), &(args[2]));
        buffer[0] = (unsigned char) (0xFF & args[0]);
        buffer[1] = (unsigned char) (0xFF & (args[1] >> 8));
        buffer[2] = (unsigned char) (0xFF & (args[1] >> 0));
        buffer[3] = (unsigned char) (0xFF & args[2]);
        bufferLen = 4;
    }

    else if (cmdAscii.find(KeyWord15) != std::string::npos) // MRS1xxx, LMS1xxx, LMS4xxx, LRS4xxx: "sWN LFPmedianfilter" (3x1 median filter) + { 1 byte 0|1 active/inactive } + { 2 byte 0x03 }
    {
        ROS_INFO_STREAM("convertAscii2BinaryCmd: requestAscii=" << requestAscii);
        int args[2] = { 0, 0 };
        sscanf(requestAscii + KeyWord15.length() + 1, " %d %d", &(args[0]), &(args[1]));
        buffer[0] = (unsigned char) (0xFF & args[0]);
        buffer[1] = (unsigned char) (0xFF & (args[1] >> 8));
        buffer[2] = (unsigned char) (0xFF & (args[1] >> 0));
        bufferLen = 3;
    }

    else if (cmdAscii.find(KeyWord16) != std::string::npos) // LRS4xxx: "sWN LMDscandatascalefactor" + { 4 byte float }, e.g. scalefactor 1.0f = 0x3f800000, scalefactor 2.0f = 0x40000000
    {
        ROS_INFO_STREAM("convertAscii2BinaryCmd: requestAscii=" << requestAscii);
        uint32_t args = 0;
        sscanf(requestAscii + KeyWord16.length() + 1, " %x", &args);
        buffer[0] = (unsigned char) (0xFF & (args >> 24));
        buffer[1] = (unsigned char) (0xFF & (args >> 16));
        buffer[2] = (unsigned char) (0xFF & (args >> 8));
        buffer[3] = (unsigned char) (0xFF & (args >> 0));
        bufferLen = 4;
    }

    else if (cmdAscii.find(KeyWord17) != std::string::npos) // LRS4xxx: "sWN GlareDetectionSens"  + { 1 byte sensitivity }  + { 2 byte 0x03 }
    {
        ROS_INFO_STREAM("convertAscii2BinaryCmd: requestAscii=" << requestAscii);
        int args[1] = { 0 };
        sscanf(requestAscii + KeyWord17.length() + 1, " %d", &(args[0]));
        buffer[0] = (unsigned char) (0xFF & args[0]);
        bufferLen = 1;
    }

    else if (cmdAscii.find(KeyWord18) != std::string::npos && strlen(requestAscii) > KeyWord18.length() + 1) // MRS-1000 scan layer activation mask, "sWN ScanLayerFilter <number of layers> <layer 1: on/off> … <layer N: on/off>",  default: all layer activated: "sWN ScanLayerFilter 4 1 1 1 1"
    {
        // Convert ascii integer args to binary, e.g. "4 1 1 1 1" to 0x000401010101
        ScanLayerFilterCfg scan_filter_cfg(requestAscii + KeyWord18.length() + 1);
        int num_layers = scan_filter_cfg.scan_layer_activated.size();
        if (num_layers > 0)
        {
            bufferLen = 0;
            // 2 byte <number of layers>
            buffer[bufferLen++] = (unsigned char) (0xFF & (num_layers >> 8));
            buffer[bufferLen++] = (unsigned char) (0xFF & num_layers);
            for(int n = 0; n < num_layers; n++)
            {
                buffer[bufferLen++] = (unsigned char) (0xFF & (scan_filter_cfg.scan_layer_activated[n])); // 1 byte <layer on/off>
            }
        }
    }

    else if (cmdAscii.find(KeyWord19) != std::string::npos && strlen(requestAscii) > KeyWord19.length() + 1)// NAV-350 poll data: "sMN mNPOSGetData 1 2" (sopas arguments: wait = 1, i.e. wait for next pose result), mask = 2, i.e. send pose+reflectors+scan)
    {
        // Convert ascii integer args to binary, e.g. "1 2" to 0x0102
        int args[2] = { 0, 0 };
        sscanf(requestAscii + KeyWord19.length() + 1, " %d %d", &(args[0]), &(args[1]));
        buffer[0] = (unsigned char) (0xFF & args[0]);
        buffer[1] = (unsigned char) (0xFF & args[1]);
        bufferLen = 2;
    }

    else if (cmdAscii.find(KeyWord20) != std::string::npos && strlen(requestAscii) > KeyWord20.length() + 1) // Set NAV-350 start pose in navigation mode by "sMN mNPOSSetPose X Y Phi", 3 arguments, each int32_t
    {
        int32_t args[3] = { 0, 0, 0 };
        sscanf(requestAscii + KeyWord20.length() + 1, " %d %d %d", &(args[0]), &(args[1]), &(args[2]));
        bufferLen = 0;
        for(int arg_cnt = 0; arg_cnt < 3; arg_cnt++)
        {
            buffer[bufferLen + 0] = (unsigned char) (0xFF & (args[arg_cnt] >> 24));
            buffer[bufferLen + 1] = (unsigned char) (0xFF & (args[arg_cnt] >> 16));
            buffer[bufferLen + 2] = (unsigned char) (0xFF & (args[arg_cnt] >> 8));
            buffer[bufferLen + 3] = (unsigned char) (0xFF & (args[arg_cnt] >> 0));
            bufferLen += 4;
        }
    }

    else if (cmdAscii.find(KeyWord21) != std::string::npos && strlen(requestAscii) > KeyWord21.length() + 1) // "sWN NEVACurrLayer 0"
    {
        int args[1] = { 0 };
        sscanf(requestAscii + KeyWord21.length() + 1, " %d", &(args[0]));
        buffer[0] = (unsigned char) (0xFF & (args[0] >> 8));
        buffer[1] = (unsigned char) (0xFF & (args[0] >> 0));
        bufferLen = 2;
    }

    else if (cmdAscii.find(KeyWord22) != std::string::npos && strlen(requestAscii) > KeyWord22.length() + 1) // "sWN NMAPMapCfg 50 0 0 0 0";
    {
        int args[5] = { 0, 0, 0, 0, 0 };
        sscanf(requestAscii + KeyWord22.length() + 1, " %d %d %d %d %d", &(args[0]), &(args[1]), &(args[2]), &(args[3]), &(args[4]));
        buffer[0] = (unsigned char) (0xFF & args[0]);
        buffer[1] = (unsigned char) (0xFF & args[1]);
        bufferLen = 2;
        for(int arg_cnt = 2; arg_cnt < 5; arg_cnt++)
        {
            buffer[bufferLen + 0] = (unsigned char) (0xFF & (args[arg_cnt] >> 24));
            buffer[bufferLen + 1] = (unsigned char) (0xFF & (args[arg_cnt] >> 16));
            buffer[bufferLen + 2] = (unsigned char) (0xFF & (args[arg_cnt] >> 8));
            buffer[bufferLen + 3] = (unsigned char) (0xFF & (args[arg_cnt] >> 0));
            bufferLen += 4;
        }
    }

    else if (cmdAscii.find(KeyWord23) != std::string::npos && strlen(requestAscii) > KeyWord23.length() + 1) // "sWN NLMDReflSize 80";
    {
        int args[1] = { 0 };
        sscanf(requestAscii + KeyWord23.length() + 1, " %d", &(args[0]));
        buffer[0] = (unsigned char) (0xFF & (args[0] >> 8));
        buffer[1] = (unsigned char) (0xFF & (args[0] >> 0));
        bufferLen = 2;
    }

    else if (cmdAscii.find(KeyWord24) != std::string::npos && strlen(requestAscii) > KeyWord24.length() + 1) // "NPOSPoseDataFormat 1 1";
    {
        int args[2] = { 0, 0 };
        sscanf(requestAscii + KeyWord24.length() + 1, " %d %d", &(args[0]), &(args[1]));
        buffer[0] = (unsigned char) (0xFF & (args[0]));
        buffer[1] = (unsigned char) (0xFF & (args[1]));
        bufferLen = 2;
    }

    else if (cmdAscii.find(KeyWord25) != std::string::npos && strlen(requestAscii) > KeyWord25.length() + 1) // "sWN NLMDLandmarkDataFormat 0 1 1"
    {
        int args[3] = { 0, 0, 0 };
        sscanf(requestAscii + KeyWord25.length() + 1, " %d %d %d", &(args[0]), &(args[1]), &(args[2]));
        buffer[0] = (unsigned char) (0xFF & (args[0]));
        buffer[1] = (unsigned char) (0xFF & (args[1]));
        buffer[2] = (unsigned char) (0xFF & (args[2]));
        bufferLen = 3;
    }

    else if (cmdAscii.find(KeyWord26) != std::string::npos && strlen(requestAscii) > KeyWord26.length() + 1) // "sWN NAVScanDataFormat 1 1"
    {
        int args[2] = { 0, 0 };
        sscanf(requestAscii + KeyWord26.length() + 1, " %d %d", &(args[0]), &(args[1]));
        buffer[0] = (unsigned char) (0xFF & (args[0]));
        buffer[1] = (unsigned char) (0xFF & (args[1]));
        bufferLen = 2;
    }

    else if (cmdAscii.find(KeyWord27) != std::string::npos && strlen(requestAscii) > KeyWord27.length() + 1) // "sMN mNLAYEraseLayout 1"
    {
        int args[1] = { 0 };
        sscanf(requestAscii + KeyWord27.length() + 1, " %d", &(args[0]));
        buffer[0] = (unsigned char) (0xFF & (args[0]));
        bufferLen = 1;
    }

    // copy base command string to buffer
    bool switchDoBinaryData = false;
    for (int i = 1; i <= (int) (msgLen); i++)  // STX DATA ETX --> 0 1 2
    {
        char c = requestAscii[i];
        if (switchDoBinaryData == true)
        {
            if (0 == bufferLen)  // no keyword handling before this point
            {
                if (c >= '0' && c <= '9')  // standard data format expected - only one digit
                {
                    c -= '0';              // convert ASCII-digit to hex-digit
                }                          // 48dez to 00dez and so on.
            }
            else
            {
                break;
            }
        }
        requestBinary->push_back(c);
        if (requestAscii[i] == 0x20) // space
        {
            spaceCnt++;
            if (spaceCnt >= copyUntilSpaceCnt)
            {
                switchDoBinaryData = true;
            }
        }
    }
    // if there are already process bytes (due to special key word handling)
    // copy these data bytes to the buffer
    for (int i = 0; i < bufferLen; i++) // append variable data
    {
        requestBinary->push_back(buffer[i]);
    }

    SickScanCommonTcp::setLengthAndCRCinBinarySopasRequest(requestBinary);

    return (0);
}

void SickScanCommonTcp::setLengthAndCRCinBinarySopasRequest(std::vector<uint8_t>* requestBinary)
{
    int msgLen = (int)requestBinary->size(); // requestBinary = { 4 byte 0x02020202 } + { 4 byte placeholder for payload length } + { payload }
    msgLen -= 8;
    for (int i = 0; i < 4; i++)
    {
        (*requestBinary)[4 + i] = (uint8_t)((msgLen >> (3 - i) * 8) & 0xFF); // payload length is always 4 byte big endian encoded
    }
    unsigned char xorVal = 0x00;
    xorVal = sick_crc8((unsigned char *) (&((*requestBinary)[8])), requestBinary->size() - 8);
    requestBinary->push_back(xorVal);
#if 0
    for (int i = 0; i < requestBinary->size(); i++)
    {
        unsigned char c = (*requestBinary)[i];
        printf("[%c]%02x ", (c < ' ') ? '.' : c, c) ;
    }
    printf("\n");
#endif
}

/*
 * @brief returns the sopas command keyword, e.g.: getSopasCmdKeyword("\x02\x02\x02\x02\x00\x00\x00\x8esSN LFErec \x3", 20) returns "LFErec".
 */
std::string SickScanCommonTcp::getSopasCmdKeyword(const uint8_t* sopasRequest, int requestLength)
{
    const uint32_t binary_stx = 0x02020202;
    bool requestIsBinary = (requestLength >= sizeof(binary_stx) && memcmp(sopasRequest, &binary_stx, sizeof(binary_stx)) == 0); // Cola-B always starts with 0x02020202

    int keyword_start = 0, keyword_end = 0;
    if(requestIsBinary && requestLength > 12) // 0x02020202 + { 4 byte payload length } + { 4 byte command id incl. space }
    {
        keyword_start = 12;
    }
    else if(!requestIsBinary && requestLength > 5)
    {
        keyword_start = 5; // 0x02 + { 4 byte command id incl. space }
    }
    else
    {
        return ""; // no keyword found
    }

    keyword_end = keyword_start;
    while(keyword_end < requestLength-1 && sopasRequest[keyword_end] != 0x03 && !isspace(sopasRequest[keyword_end])) // count until <ETX> or " "
    {
        keyword_end++;
    }

    std::string keyword((const char*)&sopasRequest[keyword_start], keyword_end - keyword_start);
    // ROS_DEBUG_STREAM("SickScanMessages::getSopasCmdKeyword(): keyword=\"" << DataDumper::binDataToAsciiString(&sopasRequest[keyword_start], keyword_end - keyword_start) << "\"=\"" << keyword << "\"");
    return keyword;
}

/**
 \brief send command and check answer
 \param requestStr: Sopas-Command
 \param *reply: Antwort-String
 \param cmdId: Command index to derive the correct error message (optional)
 \return error code
*/
int SickScanCommonTcp::sendSopasAndCheckAnswer(std::string requestStr, std::vector<unsigned char> *reply, int cmdId)
{
    std::vector<unsigned char> requestStringVec;
    for (size_t i = 0; i < requestStr.length(); i++)
    {
        requestStringVec.push_back(requestStr[i]);
    }
    int retCode = this->sendSopasAndCheckAnswer(requestStringVec, reply, cmdId);
    return (retCode);
}

/**
 \brief send command and check answer
 \param requestStr: Sopas-Command given as byte-vector
 \param *reply: Antwort-String
 \param cmdId: Command index to derive the correct error message (optional)
 \return error code
*/
int SickScanCommonTcp::sendSopasAndCheckAnswer(std::vector<unsigned char> requestStr, std::vector<unsigned char> *reply, int cmdId)
{
    std::lock_guard<std::mutex> send_lock_guard(sopasSendMutex); // lock send mutex in case of asynchronous service calls

    reply->clear();
    std::string cmdStr = "";
    int cmdLen = 0;
    for (size_t i = 0; i < requestStr.size(); i++)
    {
        cmdLen++;
        cmdStr += (char) requestStr[i];
    }
    int result = -1;

    // std::string errString;
    // if (cmdId == -1)
    // {
    //     errString = "Error unexpected Sopas answer for request " + stripControl(requestStr, 64);
    // }
    // else
    // {
    //     errString = this->sopasCmdErrMsg[cmdId];
    // }

    // std::vector<std::string> expectedAnswers = generateExpectedAnswerString(requestStr);

    // send sopas cmd

    std::string reqStr = SickScanCommonTcp::sopasReplyToString(requestStr);
    ROS_INFO_STREAM("Sending  : " << stripControl(requestStr));
    result = this->sendSOPASCommand(cmdStr.c_str(), reply, cmdLen);
    std::string replyStr = SickScanCommonTcp::sopasReplyToString(*reply);
    std::vector<unsigned char> replyVec;
    replyStr = "<STX>" + replyStr + "<ETX>";
    replyVec = stringToVector(replyStr);
    ROS_INFO_STREAM("Receiving: " << stripControl(replyVec, 96));

    // if (result != 0)
    // {
    //     std::string tmpStr = "SOPAS Communication -" + errString;
    //     ROS_INFO_STREAM(tmpStr << "\n");
    // }
    // else
    {
        result = -1;
        uint64_t retry_start_timestamp_nsec = ::rosNanosecTimestampNow();
        for(int retry_answer_cnt = 0; result != 0; retry_answer_cnt++)
        {
            std::string answerStr = SickScanCommonTcp::sopasReplyToString(*reply);
            std::stringstream expectedAnswers;
            std::vector<std::string> searchPattern = SickScanCommonTcp::generateExpectedAnswerString(requestStr);

            for(int n = 0; result != 0 && n < searchPattern.size(); n++)
            {
                if (answerStr.find(searchPattern[n]) != std::string::npos)
                {
                    result = 0;
                }
                expectedAnswers << (n > 0 ? "," : "") << "\"" << searchPattern[n] << "\"" ;
            }
            if(result != 0)
            {
                if (cmdId == CMD_START_IMU_DATA)
                {
                    ROS_INFO_STREAM("IMU-Data transfer started. No checking of reply to avoid confusing with LMD Scandata\n");
                    result = 0;
                }
                else
                {
                    if(answerStr.size() > 64)
                    {
                        answerStr.resize(64);
                        answerStr += "...";
                    }
                    // std::string tmpMsg = "Error Sopas answer mismatch: " + errString + ", received answer: \"" + answerStr + "\", expected patterns: " + expectedAnswers.str();
                    // ROS_WARN_STREAM(tmpMsg);
                    result = -1;

                    // Problably we received some scan data message. Ignore and try again...
                    std::vector<std::string> response_keywords = { SickScanCommonTcp::getSopasCmdKeyword((uint8_t*)requestStr.data(), requestStr.size()) };
                    if(retry_answer_cnt < 100 && (::rosNanosecTimestampNow() - retry_start_timestamp_nsec) / 1000000 < m_read_timeout_millisec_default)
                    {
                        char buffer[64*1024];
                        int bytes_read = 0;

                        int read_timeout_millisec = this->getReadTimeOutInMs(); // default timeout: 120 seconds (sensor may be starting up)
                        if (!reply->empty()) // sensor is up and running (i.e. responded with message), try again with 5 sec timeout
                        {
                            read_timeout_millisec = m_read_timeout_millisec_default;
                        }

                        if (this->readWithTimeout(read_timeout_millisec, buffer, sizeof(buffer), &bytes_read, response_keywords) == ExitSuccess)
                        {
                            reply->resize(bytes_read);
                            std::copy(buffer, buffer + bytes_read, &(*reply)[0]);
                        }
                        else
                        {
                            reply->clear();
                        }
                    }
                    else
                    {
                        reply->clear();
                        // ROS_ERROR_STREAM(errString << ", giving up after " << retry_answer_cnt << " unexpected answers.");
                        break;
                    }
                }
            }
        }
    }

    return result;
}

/*!
 \brief convert ASCII or binary reply to a human readable string
 \param reply datablock, which should be converted
 \return human readable string (used for debug/monitoring output)
*/
std::string SickScanCommonTcp::sopasReplyToString(const std::vector<unsigned char> &reply)
{
    std::string reply_str;
    std::vector<unsigned char>::const_iterator it_start, it_end;
    int binLen = SickScanCommonTcp::checkForBinaryAnswer(&reply);

    if (binLen == -1) // ASCII-Cmd
    {
        it_start = reply.begin();
        it_end = reply.end();
    }
    else
    {
        it_start = reply.begin() + 8; // skip header and length id
        it_end = reply.end() - 1; // skip CRC
    }

    bool inHexPrintMode = false;
    for (std::vector<unsigned char>::const_iterator it = it_start; it != it_end; it++)
    {
        // inHexPrintMode means that we should continue printing hex value after we started with hex-Printing
        // That is easier to debug for a human instead of switching between ascii binary and then back to ascii
        if (*it >= 0x20 && (inHexPrintMode == false)) // filter control characters for display
        {
            reply_str.push_back(*it);
        }
        else
        {
            if (binLen != -1) // binary
            {
                char szTmp[255] = {0};
                unsigned char val = *it;
                inHexPrintMode = true;
                sprintf(szTmp, "\\x%02x", val);
                for (size_t ii = 0; ii < strlen(szTmp); ii++)
                {
                    reply_str.push_back(szTmp[ii]);
                }
            }
        }
    }

    return reply_str;
}

/*!
 \brief Convert little endian to big endian (should be replaced by swap-routine)
 \param *vecArr Pointer to 4 byte block
 \return swapped 4-byte-value as long
*/
unsigned long SickScanCommonTcp::convertBigEndianCharArrayToUnsignedLong(const unsigned char *vecArr)
{
    unsigned long val = 0;
    for (int i = 0; i < 4; i++)
    {
        val = val << 8;
        val |= vecArr[i];
    }
    return (val);
}

/*!
 \brief Check block for correct framing and starting sequence of a binary message
 \param reply Pointer to datablock
 \return length of message (-1 if message format is not correct)
*/
int SickScanCommonTcp::checkForBinaryAnswer(const std::vector<unsigned char> *reply)
{
    int retVal = -1;

    if (reply == NULL)
    {}
    else
    {
        if (reply->size() < 8)
        {
            retVal = -1;
        }
        else
        {
            const unsigned char *ptr = &((*reply)[0]);
            unsigned binId = SickScanCommonTcp::convertBigEndianCharArrayToUnsignedLong(ptr);
            unsigned cmdLen = SickScanCommonTcp::convertBigEndianCharArrayToUnsignedLong(ptr + 4);
            if (binId == 0x02020202)
            {
                int replyLen = reply->size();
                if (replyLen == 8 + cmdLen + 1)
                {
                    retVal = cmdLen;
                }
            }
        }
    }

    return (retVal);
}


void SickScanCommonTcp::ScanLayerFilterCfg::parse(const std::string& parameter)
{
    // parse ascii integer args "<number of layers> <layer 1 on/off> ... <layer N on/off>", e.g. "4 1 1 1 1"
    scan_layer_filter = parameter;
    scan_layer_activated.clear();
    first_active_layer = std::numeric_limits<int>::max();
    last_active_layer = -1;
    num_layers = 0;
    num_active_layers = 0;
    std::istringstream ascii_args(parameter);
    std::string ascii_arg;

    for (int arg_cnt = 0; getline(ascii_args, ascii_arg, ' '); arg_cnt++)
    {
        int arg_val = -1;
        if (sscanf(ascii_arg.c_str(), "%d", &arg_val) == 1 && arg_val >= 0)
        {
            if (num_layers == 0) // i.e. parameter <number of layers>
            {
                num_layers = arg_val;
            }
            else // i.e. parameter <layer n on/off>
            {
                int layer = scan_layer_activated.size();
                scan_layer_activated.push_back(arg_val);

                if (arg_val > 0)
                {
                    num_active_layers += 1;
                    first_active_layer = std::min<int>(layer, first_active_layer);
                    last_active_layer = std::max<int>(layer, last_active_layer);
                }
            }
        }
    }
    // print();
}

} /* namespace sick_scan_xd */
