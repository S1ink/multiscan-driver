/*
 * Copyright (C) 2013, Freiburg University
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
 *  Created on: 15.11.2013
 *
 *      Authors:
 *         Christian Dornhege <c.dornhege@googlemail.com>
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #undef NOMINMAX // to get rid off warning C4005: "NOMINMAX": Makro-Neudefinition

#include "tcp/Mutex.hpp"

#include "sick_scan_common_nw.h"
#include "sick_ros_wrapper.h"
#include "template_queue.h"

namespace sick_scan_xd
{

/* class prepared for optimized time stamping */
class DatagramWithTimeStamp
{
public:
    DatagramWithTimeStamp(rosTime timeStamp_, std::vector<unsigned char> datagram_)
    {
        timeStamp = timeStamp_;
        datagram = datagram_;
    }

    virtual std::vector<unsigned char> & data(void) { return datagram; }

// private:
    rosTime timeStamp;
    std::vector<unsigned char> datagram;
};


class SickScanCommonTcp
{
public:
    enum SOPAS_CMD
    {
        CMD_DEVICE_IDENT_LEGACY,
        CMD_DEVICE_IDENT,  // for MRS6124
        CMD_SERIAL_NUMBER,
        CMD_REBOOT,
        CMD_WRITE_EEPROM,
        CMD_FIRMWARE_VERSION,
        CMD_DEVICE_STATE,
        CMD_OPERATION_HOURS,
        CMD_POWER_ON_COUNT,
        CMD_LOCATION_NAME,
        CMD_ACTIVATE_STANDBY,
        CMD_SET_PARTICLE_FILTER,
        CMD_SET_MEAN_FILTER,
        // CMD_ALIGNMENT_MODE,
        CMD_SCAN_LAYER_FILTER,
        CMD_APPLICATION_MODE,
        CMD_APPLICATION_MODE_FIELD_ON,
        CMD_APPLICATION_MODE_FIELD_OFF,
        CMD_APPLICATION_MODE_RANGING_ON,
        CMD_READ_ACTIVE_APPLICATIONS, // "sRN SetActiveApplications"
        CMD_SET_ACCESS_MODE_3,
        CMD_SET_ACCESS_MODE_3_SAFETY_SCANNER,
        CMD_SET_OUTPUT_RANGES,
        CMD_SET_OUTPUT_RANGES_NAV3,
        CMD_GET_OUTPUT_RANGES,
        CMD_RUN,
        CMD_SET_PARTIAL_SCAN_CFG,
        CMD_GET_PARTIAL_SCAN_CFG,
        CMD_GET_PARTIAL_SCANDATA_CFG,
        CMD_SET_PARTIAL_SCANDATA_CFG,
        CMD_STOP_SCANDATA,
        CMD_START_SCANDATA,
        CMD_START_RADARDATA,
        CMD_ACTIVATE_NTP_CLIENT,
        CMD_SET_NTP_INTERFACE_ETH,

        // Encode settings
        CMD_SET_ENCODER_MODE,
        CMD_SET_ENCODER_MODE_NO,
        CMD_SET_ENCODER_MODE_SI,
        CMD_SET_ENCODER_MODE_DP,
        CMD_SET_ENCODER_MODE_DL,
        CMD_SET_ENCODER_MODE_FI,
        CMD_SET_INCREMENTSOURCE_ENC,
        CMD_SET_3_4_TO_ENCODER,
        CMD_SET_ENOCDER_RES_1,
        CMD_SET_ENCODER_RES,

        CMD_START_IMU_DATA, // start of IMU data
        CMD_STOP_IMU_DATA, // start of IMU data

        // start of radar specific commands
        CMD_SET_TRANSMIT_RAWTARGETS_ON,  // transmit raw target for radar
        CMD_SET_TRANSMIT_RAWTARGETS_OFF, // do not transmit raw target for radar

        CMD_SET_TRANSMIT_OBJECTS_ON,  // transmit raw target for radar
        CMD_SET_TRANSMIT_OBJECTS_OFF, // do not transmit raw target for radar

        CMD_SET_TRACKING_MODE_0,  // set radar tracking mode to "BASIC"
        CMD_SET_TRACKING_MODE_1,  // set radar tracking mode to "TRAFFIC"

        CMD_LOAD_APPLICATION_DEFAULT, // load application default
        CMD_DEVICE_TYPE,
        CMD_ORDER_NUMBER,
        // end of radar specific commands
        CMD_START_MEASUREMENT,
        CMD_STOP_MEASUREMENT,
        CMD_SET_ECHO_FILTER,
        CMD_SET_NTP_UPDATETIME,
        CMD_SET_NTP_TIMEZONE,
        CMD_SET_IP_ADDR,
        CMD_SET_GATEWAY,
        CMD_SET_NTP_SERVER_IP_ADDR,
        CMD_SET_SCANDATACONFIGNAV, // "sMN mLMPsetscancfg ..."
        CMD_GET_SCANDATACONFIGNAV, // "sRN LMPscancfg"
        CMD_SEN_SCANDATACONFIGNAV, // "sEN LMPscancfg 1"
        CMD_GET_ANGLE_COMPENSATION_PARAM, // Angle Compensation Parameter for NAV lidar
        CMD_SET_TO_COLA_A_PROTOCOL,  //		sWN EIHstCola 1  // Cola B 	sWN EIHstCola 0  // Cola A
        CMD_SET_TO_COLA_B_PROTOCOL,  //
        CMD_GET_SAFTY_FIELD_CFG,// gets the safty fields cfg olny tim 7xxs supported at the moment

        CMD_SET_LFEREC_ACTIVE,       // activate LFErec messages, send "sEN LFErec 1"
        CMD_SET_LID_OUTPUTSTATE_ACTIVE,  // activate LIDoutputstate messages, send "sEN LIDoutputstate 1"
        CMD_SET_LID_INPUTSTATE_ACTIVE,  // activate LIDinputstate messages, send "sEN LIDinputstate 1"
        CMD_SET_SCAN_CFG_LIST, // "sMN mCLsetscancfglist %d", set scan config from list for NAX310  LD-OEM15xx LD-LRS36xx

        // NAV-350 commands
        CMD_SET_NAV_OPERATIONAL_MODE_0,   // "sMN mNEVAChangeState 0", 0 = power down
        CMD_SET_NAV_OPERATIONAL_MODE_1,   // "sMN mNEVAChangeState 1", 1 = standby
        CMD_SET_NAV_OPERATIONAL_MODE_2,   // "sMN mNEVAChangeState 2", 2 = mapping
        CMD_SET_NAV_OPERATIONAL_MODE_3,   // "sMN mNEVAChangeState 3", 3 = landmark detection
        CMD_SET_NAV_OPERATIONAL_MODE_4,   // "sMN mNEVAChangeState 4", 4 = navigation
        CMD_SET_NAV_CURR_LAYER,           // "sWN NEVACurrLayer 0"
        CMD_SET_NAV_LANDMARK_DATA_FORMAT, // "sWN NLMDLandmarkDataFormat 0 1 1"
        CMD_SET_NAV_SCAN_DATA_FORMAT,     // "sWN NAVScanDataFormat 1 1"
        CMD_SET_NAV_POSE_DATA_FORMAT,     // "sWN NPOSPoseDataFormat 1 1"
        CMD_SET_NAV_MAP_CFG,              // "sWN NMAPMapCfg 50 0 0 0 0"
        CMD_SET_NAV_REFL_SIZE,            // "sWN NLMDReflSize 80"
        CMD_SET_NAV_DO_MAPPING,           // "sMN mNMAPDoMapping"
        CMD_SET_NAV_ADD_LANDMARK,         // "sMN mNLAYAddLandmark landmarkData {x y type subtype size layerID {ID}}"      
        CMD_SET_NAV_ERASE_LAYOUT,         // "sMN mNLAYEraseLayout 1"
        CMD_SET_NAV_STORE_LAYOUT,         // "sMN mNLAYStoreLayout"
        CMD_SET_NAV_POSE,                 // Set NAV-350 start pose in navigation mode by "sMN mNPOSSetPose X Y Phi"

        // Supported by sick_generic_caller version 2.7.3 and above:
        CMD_SET_LFPMEANFILTER, // MRS1xxx, LMS1xxx, LMS4xxx, LRS4xxx: "sWN LFPmeanfilter" + { 1 byte 0|1 active/inactive } + { 2 byte 0x02 ... 0x64 number of scans } + { 1 byte 0x00 }
        CMD_SET_LFPMEDIANFILTER, // MRS1xxx, LMS1xxx, LMS4xxx, LRS4xxx: "sWN LFPmedianfilter" (3x1 median filter) + { 1 byte 0|1 active/inactive } + { 2 byte 0x03 }
        CMD_SET_LMDSCANDATASCALEFACTOR, // LRS4xxx: "sWN LMDscandatascalefactor" + { 4 byte float }, e.g. scalefactor 1.0f = 0x3f800000, scalefactor 2.0f = 0x40000000

        // Supported by sick_generic_caller version 2.8.4 and above:
        CMD_SET_GLARE_DETECTION_SENS, // Glare Detection Sensitivity (LRS4xxx only): glare_detection_sens<0: do not apply, glare_detection_sens==0: deactivate glare_detection_filter, glare_detection_sens==5: medium glare detection sensitivity, glare_detection_sens==10: sensitive glare detection filter

        // CMD_SET_ALIGNMENT_MODE, // Support for MRS-1000 layer activation (alignment mode): do not overwrite: -1, all Layer: 0 (default), red Layer (-2.5 deg): 1, blue Layer (0 deg): 2, green Layer (+2.5 deg): 3, yellow Layer (+5 deg): 4
        CMD_SET_SCAN_LAYER_FILTER, // MRS-1000 scan layer activation mask, "sWN ScanLayerFilter <number of layers> <layer 1: on/off> … <layer N: on/off>"",  default: all layer activated: "sWN ScanLayerFilter 4 1 1 1 1"

        // ML: Add above new CMD-Identifier
        //
        //
        CMD_END // CMD_END is a tag for end of enum - never (re-)move it. It must be the last element.
    };

public:
    SickScanCommonTcp(const std::string &hostname, const std::string &port, int &timelimit, char cola_dialect_id);
    ~SickScanCommonTcp();

    static void disconnectFunctionS(void *obj);
    static void readCallbackFunctionS(void *obj, UINT8 *buffer, UINT32 &numOfBytes);
    void readCallbackFunction(UINT8 *buffer, UINT32 &numOfBytes);

    void setReplyMode(int _mode);
    int getReplyMode();

    int getReadTimeOutInMs();
    void setReadTimeOutInMs(int timeOutInMs);

    int getProtocolType(void);
    void setProtocolType(SopasProtocol cola_dialect_id);

    int numberOfDatagramInInputFifo();

    SopasEventMessage findFrameInReceiveBuffer();

    void processFrame(rosTime timeStamp, SopasEventMessage &frame);

    int reinit(int delay_millisec);
    int init_device();
    int close_device();

    bool isConnected() { return m_nw.isConnected(); }

    /*! Changes the Identifier of a commandstr. to its expected answer counterpart
     * @param requestStr sent request string
     * @return Expected answer
     */
    static std::vector<std::string> generateExpectedAnswerString(const std::vector<unsigned char> requestStr);

    /**
     * \brief Converts a given SOPAS command from ascii to binary (in case of binary communication), sends sopas (ascii or binary) and returns the response (if wait_for_reply:=true)
     * \param [in] request the command to send.
     * \param [in] cmdLen Length of the Comandstring in bytes used for Binary Mode only
     */
    // int convertSendSOPASCommand(const std::string& sopas_ascii_request, std::vector<unsigned char>* reply, bool wait_for_reply = true);
    static int convertAscii2BinaryCmd(const char *requestAscii, std::vector<unsigned char> *requestBinary);
    static void setLengthAndCRCinBinarySopasRequest(std::vector<uint8_t>* requestBinary);

    static std::string getSopasCmdKeyword(const uint8_t* sopasRequest, int requestLength);

    int sendSopasAndCheckAnswer(std::string request, std::vector<unsigned char> *reply, int cmdId = -1);
    int sendSopasAndCheckAnswer(std::vector<unsigned char> request, std::vector<unsigned char> *reply, int cmdId = -1);

    /**
     * \brief Converts reply from sendSOPASCommand to string
     * \param [in] reply reply from sendSOPASCommand
     * \returns reply as string with special characters stripped out
     */
    static std::string sopasReplyToString(const std::vector<unsigned char> &reply);

    /**
    * \param [in] *vecArr to (unsigned) char buffer in big endian byte oder (MSB first)
    *
    * \returns    unsigned long value as interpretation of big endian long value
    */
    static unsigned long convertBigEndianCharArrayToUnsignedLong(const unsigned char *vecArr);

    /**
    * \param [in] reply check reply whether is SOPAS-ASCII or SOPAS-Binary
    *
    * \returns    -1 if ascii otherwise the length of data content following offset 8
    */
    static int checkForBinaryAnswer(const std::vector<unsigned char> *reply);

    SickScanCommonNw m_nw;
    Queue<DatagramWithTimeStamp> recvQueue;
    UINT32 m_alreadyReceivedBytes;
    UINT32 m_lastPacketSize;
    UINT8 m_packetBuffer[480000];

protected:
    void disconnectFunction();

    /// Send a SOPAS command to the device and print out the response to the console.
    int sendSOPASCommand(const char *request, std::vector<unsigned char> *reply, int cmdLen, bool wait_for_reply = true);

    /// Read a datagram from the device.
    /**
     * \param [out] recvTimeStamp timestamp of received datagram
     * \param [in] receiveBuffer data buffer to fill
     * \param [in] bufferSize max data size to write to buffer (result should be 0 terminated)
     * \param [out] actual_length the actual amount of data written
     * \param [in] isBinaryProtocol true=binary False=ASCII
     * \param [in] datagram_keywords keyword in returned datagram, e.g. { "LMDscandata" } to get scandata telegrams, or {} (empty vector) for next received datagram
     */
    int get_datagram(rosTime &recvTimeStamp, unsigned char *receiveBuffer, int bufferSize, int *actual_length,
                bool isBinaryProtocol, int *numberOfRemainingFifoEntries, const std::vector<std::string>& datagram_keywords);

    int readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read, const std::vector<std::string>& datagram_keywords);

private:
    // Response buffer
    UINT32 m_numberOfBytesInResponseBuffer; ///< Number of bytes in buffer
    UINT8 m_responseBuffer[1024]; ///< Receive buffer for everything except scan data and eval case data.
    Mutex m_receiveDataMutex; ///< Access mutex for buffer

    // Receive buffer
    UINT32 m_numberOfBytesInReceiveBuffer; ///< Number of bytes in buffer
    UINT8 m_receiveBuffer[480000]; ///< Low-Level receive buffer for all data

    bool m_beVerbose;

    size_t bytes_transfered_;

    SopasProtocol m_protocolId;
    std::string hostname_;
    std::string port_;
    int timelimit_;
    int m_replyMode;

    int readTimeOutInMs;
    std::mutex sopasSendMutex; // mutex to lock sendSopasAndCheckAnswer
    int m_read_timeout_millisec_default = 5000;
    int m_read_timeout_millisec_startup = 120000;

    struct ScanLayerFilterCfg // Optional ScanLayerFilter setting
    {
        ScanLayerFilterCfg(const std::string& parameter = "") // parameter for ScanLayerFilter, e.g. "4 1 1 1 1"
        {
            if (!parameter.empty()) parse(parameter);
        }

        std::string scan_layer_filter = ""; // Optional ScanLayerFilter setting from launchfile
        std::vector<int> scan_layer_activated = std::vector<int>();
        int first_active_layer = -1;
        int last_active_layer = -1;
        int num_layers = 0;
        int num_active_layers = 0;

        void parse(const std::string& parameter);   // TODO
        // void print();
    };

};


} /* namespace sick_scan_xd */
