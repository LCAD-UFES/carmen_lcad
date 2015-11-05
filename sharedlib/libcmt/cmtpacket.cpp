/*! \file Cmtpacket.cpp

	For information about objects in this file, see the appropriate header:
	\ref Cmtpacket.h

	\section FileCopyright Copyright Notice 
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.
	
	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.
	
	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
*/

#include "cmtpacket.h"

#ifdef CMT_DLL_EXPORT
#include <xsens_math.h>
#include <xsens_settings.h>
#endif

#ifdef LOG_PACKET
#	define PACKETLOG	CMTLOG
#else
#	define PACKETLOG(...)
#endif


namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Packet class //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
#ifndef CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE
#define CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE		0x00000300
#endif
#define FORMAT_DOUBLE		((m_formatList[index].m_outputSettings & CMT_OUTPUTSETTINGS_DATAFORMAT_MASK)|CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE)
#define ISDOUBLE(a)			(m_infoList[index].a >= m_infoList[index].m_doubleBoundary)
#define CHECKIFDOUBLE(a)	ISDOUBLE(a)?FORMAT_DOUBLE:m_formatList[index].m_outputSettings

//////////////////////////////////////////////////////////////////////////////////////////
Packet::Packet(uint16_t items, bool xbus)
{
	PACKETLOG("Create Packet %p\n",this);

	m_itemCount = items;
	m_infoList = NULL;
	m_formatList = new CmtDataFormat[m_itemCount];
	m_toa = 0;
	m_rtc = 0;
	m_xm = xbus;
}

//////////////////////////////////////////////////////////////////////////////////////////
Packet::~Packet()
{
	PACKETLOG("Destroy Packet %p\n",this);
	LSTCHKDELNUL(m_formatList);
	LSTCHKDELNUL(m_infoList);
}

//////////////////////////////////////////////////////////////////////////////////////////
CmtDataFormat Packet::getDataFormat(const uint16_t index) const
{
	if (index >= m_itemCount || m_formatList == NULL)
	{
		CmtDataFormat temp;
		return temp;
	}
	return m_formatList[index];
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Packet::setDataFormat(const CmtDataFormat& format, const uint16_t index)
{
	if (index < m_itemCount)
	{
		m_formatList[index] = format;
		LSTCHKDELNUL(m_infoList);
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Packet::setDataFormat(const CmtOutputMode outputMode, const CmtOutputSettings outputSettings, const uint16_t index)
{
	if (index < m_itemCount)
	{
		m_formatList[index].m_outputMode = outputMode;
		m_formatList[index].m_outputSettings = outputSettings;
		LSTCHKDELNUL(m_infoList);
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Packet::getXbus(void) const
{
	return m_xm;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Packet::setXbus(bool xbus, bool convert)
{
	if (xbus != m_xm)
	{
		if (convert)
		{
			CmtMtTimeStamp stamp = getSampleCounter(0);

			// move the time stamp value(s) around
			if (xbus)
			{
				// new version is Xbus, so old version is regular format -> place timestamp at start of packet and remove from all individual data units
				// remove the timestamp from all data units

				// insert timestamp at the start
				m_msg.insertData(2,0);
				m_msg.setDataShort(stamp,0);
			}
			else
			{
				// new version is regular, so old version is Xbus format -> place timestamp at end of all individual data units and remove from start of packet
				// append the timestamp to all data units

				// remove timestamp from the start
				m_msg.deleteData(2,0);
			}
		}
		// update the state cache
		m_xm = xbus;
		LSTCHKDELNUL(m_infoList);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the floating/fixed point value size
uint16_t Packet::getFPValueSize(const uint16_t index) const
{
	uint16_t ds = 4;
	switch (m_formatList[index].m_outputSettings & CMT_OUTPUTSETTINGS_DATAFORMAT_MASK)
	{
		case CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT:
			ds = 4;
			break;

		case CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE:
			ds = 8;
			break;

		case CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632:
			ds = 6;
			break;

		case CMT_OUTPUTSETTINGS_DATAFORMAT_F1220:
			ds = 4;
			break;
	}
	return ds;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the data size.
uint16_t Packet::getDataSize(const uint16_t index) const
{
	if (m_infoList == NULL)
	{
		// allocate list
		m_infoList = new PacketInfo[m_itemCount];
		uint16_t totalOffset;
		if (m_xm)
			totalOffset = 2;
		else
			totalOffset = 0;

		// fill lists
		for (uint16_t i = 0;i < m_itemCount;++i)
		{
			m_infoList[i].m_offset = totalOffset;
			m_infoList[i].m_size = 0;

			uint16_t ds = getFPValueSize(i);

			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_RAW)
			{
				m_infoList[i].m_rawData = totalOffset;
				m_infoList[i].m_rawAcc = totalOffset;
				m_infoList[i].m_rawGyr = totalOffset+6;
				m_infoList[i].m_rawMag = totalOffset+12;
				m_infoList[i].m_rawTemp = totalOffset+18;

				m_infoList[i].m_size += 20;
				totalOffset += 20;
			}
			else
			{
				m_infoList[i].m_rawData = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawAcc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGyr = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawMag = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawTemp = CMT_DATA_ITEM_NOT_AVAILABLE;
			}


			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_GPSPVT_PRESSURE)
			{
				m_infoList[i].m_gpsPvtData = totalOffset;
				m_infoList[i].m_gpsPvtPressure = totalOffset;
				m_infoList[i].m_gpsPvtPressureAge = totalOffset + 2;
				m_infoList[i].m_size += 3;
				totalOffset += 3;
				
				m_infoList[i].m_gpsPvtGpsData = totalOffset;
				m_infoList[i].m_gpsPvtItow = totalOffset;
				m_infoList[i].m_gpsPvtLatitude = totalOffset + 4;
				m_infoList[i].m_gpsPvtLongitude = totalOffset + 8;
				m_infoList[i].m_gpsPvtHeight = totalOffset + 12;
				m_infoList[i].m_gpsPvtVeln = totalOffset + 16;
				m_infoList[i].m_gpsPvtVele = totalOffset + 20;
				m_infoList[i].m_gpsPvtVeld = totalOffset + 24;
				m_infoList[i].m_gpsPvtHacc = totalOffset + 28;
				m_infoList[i].m_gpsPvtVacc = totalOffset + 32;
				m_infoList[i].m_gpsPvtSacc = totalOffset + 36;
				m_infoList[i].m_gpsPvtGpsAge = totalOffset + 40;
				m_infoList[i].m_size += 41;
				totalOffset += 41;
			}
			else
			{
				m_infoList[i].m_gpsPvtData = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtPressure = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtPressureAge = CMT_DATA_ITEM_NOT_AVAILABLE;

				m_infoList[i].m_gpsPvtGpsData = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtItow = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtLatitude = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtLongitude = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtHeight = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtVeln = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtVele = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtVeld = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtHacc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtVacc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtSacc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_gpsPvtGpsAge = CMT_DATA_ITEM_NOT_AVAILABLE;
			}

			m_infoList[i].m_temp = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_TEMP)
			{
				m_infoList[i].m_temp = totalOffset;
				m_infoList[i].m_size += ds;
				totalOffset += ds;
			}

			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_CALIB)
			{
				m_infoList[i].m_calData = totalOffset;
				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_CALIBMODE_ACC_MASK) == 0)
				{
					m_infoList[i].m_calAcc = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
				else
					m_infoList[i].m_calAcc = CMT_DATA_ITEM_NOT_AVAILABLE;

				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_CALIBMODE_GYR_MASK) == 0)
				{
					m_infoList[i].m_calGyr = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
				else
					m_infoList[i].m_calGyr = CMT_DATA_ITEM_NOT_AVAILABLE;

				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_CALIBMODE_MAG_MASK) == 0)
				{
					m_infoList[i].m_calMag = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
				else
					m_infoList[i].m_calMag = CMT_DATA_ITEM_NOT_AVAILABLE;

				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG_MASK) == CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG_MASK)
					m_infoList[i].m_calData = CMT_DATA_ITEM_NOT_AVAILABLE;
			}
			else
			{
				m_infoList[i].m_calData = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_calAcc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_calGyr = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_calMag = CMT_DATA_ITEM_NOT_AVAILABLE;
			}

			m_infoList[i].m_oriEul = CMT_DATA_ITEM_NOT_AVAILABLE;
			m_infoList[i].m_oriQuat = CMT_DATA_ITEM_NOT_AVAILABLE;
			m_infoList[i].m_oriMat = CMT_DATA_ITEM_NOT_AVAILABLE;

			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_ORIENT)
			{
				switch (m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK)
				{
				case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
					m_infoList[i].m_oriEul = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
					break;
				case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
					m_infoList[i].m_oriQuat = totalOffset;
					m_infoList[i].m_size += 4*ds;
					totalOffset += 4*ds;
					break;
				case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
					m_infoList[i].m_oriMat = totalOffset;
					m_infoList[i].m_size += 9*ds;
					totalOffset += 9*ds;
					break;
				default:
					break;
				}
			}

			m_infoList[i].m_analogIn1 = CMT_DATA_ITEM_NOT_AVAILABLE;
			m_infoList[i].m_analogIn2 = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_AUXILIARY)
			{
				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN1_MASK) == 0)
				{
					m_infoList[i].m_analogIn1 = totalOffset;
					m_infoList[i].m_size += 2;
					totalOffset += 2;
				}
				else
					m_infoList[i].m_analogIn1 = CMT_DATA_ITEM_NOT_AVAILABLE;

				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN2_MASK) == 0)
				{
					m_infoList[i].m_analogIn2 = totalOffset;
					m_infoList[i].m_size += 2;
					totalOffset += 2;
				}
				else
					m_infoList[i].m_analogIn2 = CMT_DATA_ITEM_NOT_AVAILABLE;
			}

			m_infoList[i].m_posLLA = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_POSITION)
			{
				switch (m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_POSITIONMODE_MASK)
				{
				case CMT_OUTPUTSETTINGS_POSITIONMODE_LLA_WGS84:
					m_infoList[i].m_posLLA = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
			}

			m_infoList[i].m_velNEDorNWU = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_VELOCITY)
			{
				switch (m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_VELOCITYMODE_MASK)
				{
				case CMT_OUTPUTSETTINGS_VELOCITYMODE_MS_XYZ:
					m_infoList[i].m_velNEDorNWU = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
			}

			m_infoList[i].m_status = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_STATUS)
			{
				m_infoList[i].m_status = totalOffset;
				m_infoList[i].m_size += 1;
				totalOffset += 1;
			}

			m_infoList[i].m_sc = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_xm)
				m_infoList[i].m_sc = 0;
			if (m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT)
			{
				if (!m_xm)
					m_infoList[i].m_sc = totalOffset;
				m_infoList[i].m_size += 2;
				totalOffset += 2;
			}

			if (m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLEUTC)
			{
				m_infoList[i].m_utcTime = totalOffset;
				m_infoList[i].m_utcNano = totalOffset;
				m_infoList[i].m_utcYear = totalOffset + 4;
				m_infoList[i].m_utcMonth = totalOffset + 6;
				m_infoList[i].m_utcDay = totalOffset + 7;
				m_infoList[i].m_utcHour = totalOffset + 8;
				m_infoList[i].m_utcMinute = totalOffset + 9;
				m_infoList[i].m_utcSecond = totalOffset + 10;
				m_infoList[i].m_utcValid = totalOffset + 11;
				m_infoList[i].m_size += 12;
				totalOffset += 12;
			}
			else {
				m_infoList[i].m_utcTime = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_utcNano = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_utcYear = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_utcMonth = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_utcDay = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_utcHour = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_utcMinute = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_utcSecond = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_utcValid = CMT_DATA_ITEM_NOT_AVAILABLE;
			}

			// post-processing data is never available at this point
			m_infoList[i].m_acc_g = CMT_DATA_ITEM_NOT_AVAILABLE;
			m_infoList[i].m_doubleBoundary = totalOffset;
		}
	}

	if (index < m_itemCount)
		return m_infoList[index].m_size;
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Accelerometer component of a data item.
CmtShortVector Packet::getRawAcc(const uint16_t index) const
{
	CmtShortVector buffer;
	if (containsRawAcc(index))
		for (uint16_t i=0;i<3;++i)
			buffer.m_data[i] = m_msg.getDataShort(m_infoList[index].m_rawAcc + (2*i));
	
	return buffer;
}
bool Packet::containsRawAcc(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawAcc == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawAcc(const CmtShortVector& vec, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawAcc == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawAcc = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3*2);
		m_infoList[index].m_size += 3*2;
	}
	// update
	for (uint16_t i=0;i<3;++i)
		m_msg.setDataShort(vec.m_data[i], m_infoList[index].m_rawAcc + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Gyroscope component of a data item.
CmtShortVector Packet::getRawGyr(const uint16_t index) const
{
	CmtShortVector buffer;
	if (containsRawGyr(index))
		for (uint16_t i=0;i<3;++i)
			buffer.m_data[i] = m_msg.getDataShort(m_infoList[index].m_rawGyr + (2*i));
	
	return buffer;
}
bool Packet::containsRawGyr(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawGyr == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawGyr(const CmtShortVector& vec, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawGyr == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawGyr = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3*2);
		m_infoList[index].m_size += 3*2;
	}
	// update
	for (uint16_t i=0;i<3;++i)
		m_msg.setDataShort(vec.m_data[i], m_infoList[index].m_rawGyr + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Magnetometer component of a data item.
CmtShortVector Packet::getRawMag(const uint16_t index) const
{
	CmtShortVector buffer;
	if (containsRawMag(index))
		for (uint16_t i=0;i<3;++i)
			buffer.m_data[i] = m_msg.getDataShort(m_infoList[index].m_rawMag + (2*i));
	
	return buffer;
}
bool Packet::containsRawMag(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawMag == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawMag(const CmtShortVector& vec, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawMag == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawMag = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3*2);
		m_infoList[index].m_size += 3*2;
	}
	// update
	for (uint16_t i=0;i<3;++i)
		m_msg.setDataShort(vec.m_data[i], m_infoList[index].m_rawMag + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Temperature component of a data item.
uint16_t Packet::getRawTemp(const uint16_t index) const
{
	if (!containsRawTemp(index))
		return 0;
	
	return m_msg.getDataShort(m_infoList[index].m_rawTemp);
}
bool Packet::containsRawTemp(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawTemp == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawTemp(const uint16_t temp, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawTemp == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawTemp = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(temp, m_infoList[index].m_rawTemp);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Data component of a data item.
CmtRawData Packet::getRawData(const uint16_t index) const
{
	CmtRawData buffer;
	if (containsRawData(index))
	{
		const uint8_t* tmp = m_msg.getDataBuffer(m_infoList[index].m_rawData);
		const uint16_t* sh = (const uint16_t*) tmp;
		uint16_t* bare = (uint16_t*) &buffer;

		for (uint16_t i=0;i<10;++i, ++sh, ++bare)
			*bare = swapEndian16(*sh);// m_msg.getDataShort(m_infoList[index].m_rawData + (2*i));
	}
	return buffer;
}
bool Packet::containsRawData(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawData == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawData(const CmtRawData& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawData == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawData = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3* 3*2 + 2);
		m_infoList[index].m_rawAcc = m_infoList[index].m_rawData;
		m_infoList[index].m_rawGyr = m_infoList[index].m_rawData + 3*2;
		m_infoList[index].m_rawMag = m_infoList[index].m_rawData + 6*2;
		m_infoList[index].m_rawTemp= m_infoList[index].m_rawData + 9*2;
		m_infoList[index].m_size += 3* 3*2 + 2;
	}
	// update
	int16_t* bare = (int16_t*) &data;
	for (uint16_t i=0;i<10;++i)
		m_msg.setDataShort(bare[i], m_infoList[index].m_rawData + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Gps PVT Data component of a data item.
CmtGpsPvtData Packet::getGpsPvtData(const uint16_t index) const
{
	CmtGpsPvtData buffer;
	if (containsGpsPvtData(index))
	{
		//const uint8_t* tmp = m_msg.getDataBuffer(m_infoList[index].m_gpsPvtData);
		//const uint16_t* sh = (const uint16_t*) tmp;
		//uint16_t* bare = (uint16_t*) &buffer;

		// pressure data
		buffer.m_pressure = m_msg.getDataShort(m_infoList[index].m_gpsPvtPressure);
		// pressAge
		buffer.m_pressureAge = m_msg.getDataByte(m_infoList[index].m_gpsPvtPressureAge);

		// lon,lat,height,hacc,vacc,veln,vele,veld
		//tmp = m_msg.getDataBuffer(m_infoList[index].m_gpsPvtGpsData);
		//const uint32_t* ln = (const uint32_t*) tmp;
		uint32_t *bareln = (uint32_t*) &buffer.m_itow;
		for (uint16_t i=0; i < 10; ++i)
			bareln[i] = m_msg.getDataLong(m_infoList[index].m_gpsPvtGpsData + (4*i));

		// gpsAge
		buffer.m_gpsAge = m_msg.getDataByte(m_infoList[index].m_gpsPvtGpsAge);
	}
	return buffer;
}
bool Packet::containsGpsPvtData(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_gpsPvtData == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateGpsPvtData(const CmtGpsPvtData& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_gpsPvtData == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_gpsPvtData = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + (2+1) + (40 + 1));
		m_infoList[index].m_gpsPvtPressure = m_infoList[index].m_gpsPvtData;
		m_infoList[index].m_gpsPvtPressureAge = m_infoList[index].m_gpsPvtData + 2;

		m_infoList[index].m_gpsPvtGpsData = m_infoList[index].m_gpsPvtData + 3;
		m_infoList[index].m_gpsPvtItow = m_infoList[index].m_gpsPvtData + 3;
		m_infoList[index].m_gpsPvtLatitude = m_infoList[index].m_gpsPvtData + 3 + 4;
		m_infoList[index].m_gpsPvtLongitude = m_infoList[index].m_gpsPvtData + 3 + 8;
		m_infoList[index].m_gpsPvtHeight = m_infoList[index].m_gpsPvtData + 3 + 12;
		m_infoList[index].m_gpsPvtVeln = m_infoList[index].m_gpsPvtData + 3 + 16;
		m_infoList[index].m_gpsPvtVele = m_infoList[index].m_gpsPvtData + 3 + 20;
		m_infoList[index].m_gpsPvtVeld = m_infoList[index].m_gpsPvtData + 3 + 24;
		m_infoList[index].m_gpsPvtHacc = m_infoList[index].m_gpsPvtData + 3 + 28;
		m_infoList[index].m_gpsPvtVacc = m_infoList[index].m_gpsPvtData + 3 + 32;
		m_infoList[index].m_gpsPvtSacc = m_infoList[index].m_gpsPvtData + 3 + 36;
		m_infoList[index].m_gpsPvtGpsAge = m_infoList[index].m_gpsPvtData + 3 + 40;
		
		m_infoList[index].m_size += (2+1) + (40 + 1);
	}
	// update
	m_msg.setDataShort(data.m_pressure, m_infoList[index].m_gpsPvtPressure);
	m_msg.setDataByte(data.m_pressureAge, m_infoList[index].m_gpsPvtPressureAge);

	// lon,lat,height,hacc,vacc,veln,vele,veld
	int32_t* bareln = (int32_t*)&data.m_itow;
	for (uint16_t i=0; i<10;++i)
		m_msg.setDataLong(bareln[i],m_infoList[index].m_gpsPvtGpsData + (4*i));

	// gpsAge
	m_msg.setDataByte(data.m_gpsAge,  m_infoList[index].m_gpsPvtGpsAge);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Pressure Data component of a data item.
CmtRawPressureData Packet::getRawPressureData(const uint16_t index) const
{
	CmtRawPressureData buffer;
	if (containsRawPressureData(index))
	{
		// pressure data
		buffer.m_pressure = m_msg.getDataShort(m_infoList[index].m_gpsPvtPressure);
		// pressAge
		buffer.m_pressureAge = m_msg.getDataByte(m_infoList[index].m_gpsPvtPressureAge);
	}
	return buffer;
}
bool Packet::containsRawPressureData(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_gpsPvtPressure == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawPressureData(const CmtRawPressureData& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_gpsPvtPressure == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_gpsPvtData = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + (2+1));
		m_infoList[index].m_gpsPvtPressure = m_infoList[index].m_gpsPvtData;
		m_infoList[index].m_gpsPvtPressureAge = m_infoList[index].m_gpsPvtData + 2;

		m_infoList[index].m_size += (2+1);
	}
	// update
	m_msg.setDataShort(data.m_pressure, m_infoList[index].m_gpsPvtPressure);
	m_msg.setDataByte(data.m_pressureAge, m_infoList[index].m_gpsPvtPressureAge);

	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Temperature component of a data item.
double Packet::getTemp(const uint16_t index) const
{
	if (containsTemp(index))
		return m_msg.getDataFPValue(CHECKIFDOUBLE(m_temp), m_infoList[index].m_temp);

	return 0.0;
}
bool Packet::containsTemp(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_temp == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateTemp(const double& temp, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_temp == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_temp))
	{
		// add
		ds = 8;
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_temp = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + ds);
		m_infoList[index].m_size += ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_temp), temp, m_infoList[index].m_temp);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Accelerometer component of a data item.
CmtVector Packet::getCalAcc(const uint16_t index) const
{
	CmtVector buffer;
	if (containsCalAcc(index))
		m_msg.getDataFPValue(&buffer.m_data[0], CHECKIFDOUBLE(m_calAcc), m_infoList[index].m_calAcc, 3);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsCalAcc(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_calAcc == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateCalAcc(const CmtVector& vec, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_calAcc == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_calAcc))
	{
		// add
		ds = 8;
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_calAcc = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_calAcc), &vec.m_data[0], m_infoList[index].m_calAcc, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Gyroscope component of a data item.
CmtVector Packet::getCalGyr(const uint16_t index) const
{
	CmtVector buffer;
	if (containsCalGyr(index))
		m_msg.getDataFPValue(&buffer.m_data[0], CHECKIFDOUBLE(m_calGyr), m_infoList[index].m_calGyr, 3);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsCalGyr(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_calGyr == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateCalGyr(const CmtVector& vec, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_calGyr == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_calGyr))
	{
		// add
		ds = 8;
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_calGyr = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_calGyr), &vec.m_data[0], m_infoList[index].m_calGyr, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Magnetometer component of a data item.
CmtVector Packet::getCalMag(const uint16_t index) const
{
	CmtVector buffer;
	if (containsCalMag(index))
		m_msg.getDataFPValue(&buffer.m_data[0], CHECKIFDOUBLE(m_calMag), m_infoList[index].m_calMag, 3);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsCalMag(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_calMag == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateCalMag(const CmtVector& vec, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_calMag == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_calMag))
	{
		// add
		ds = 8;
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_calMag = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_calMag), &vec.m_data[0], m_infoList[index].m_calMag, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Accelerometer component of a data item.
CmtCalData Packet::getCalData(const uint16_t index) const
{
	CmtCalData buffer;
	double* bare = (double*) &buffer;
	if (containsCalData(index))
	{
		if (m_infoList[index].m_calAcc == CMT_DATA_ITEM_NOT_AVAILABLE)
			memset(bare, 0, 3*sizeof(double));
		else
			m_msg.getDataFPValue(bare, CHECKIFDOUBLE(m_calAcc) , m_infoList[index].m_calAcc, 3);

		bare += 3;
		if (m_infoList[index].m_calGyr == CMT_DATA_ITEM_NOT_AVAILABLE)
			memset(bare, 0, 3*sizeof(double));
		else
			m_msg.getDataFPValue(bare, CHECKIFDOUBLE(m_calGyr), m_infoList[index].m_calGyr, 3);

		bare += 3;
		if (m_infoList[index].m_calMag == CMT_DATA_ITEM_NOT_AVAILABLE)
			memset(bare, 0, 3*sizeof(double));
		else
			m_msg.getDataFPValue(bare, CHECKIFDOUBLE(m_calMag), m_infoList[index].m_calMag, 3);
	}
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsCalData(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_calData == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateCalData(const CmtCalData& data, const uint16_t index)
{
	const uint16_t numValues = 9;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_calData == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_calData))
	{
		// add
		ds = 8;	// added values are always in double precision
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_calData = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_calAcc = m_infoList[index].m_calData;
		m_infoList[index].m_calGyr = m_infoList[index].m_calData + 3*ds;
		m_infoList[index].m_calMag = m_infoList[index].m_calData + 6*ds;
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	double* bare = (double*) &data;
	if (m_infoList[index].m_calAcc != CMT_DATA_ITEM_NOT_AVAILABLE)
		m_msg.setDataFPValue(CHECKIFDOUBLE(m_calAcc), bare, m_infoList[index].m_calAcc, 3);
	bare += 3;
	if (m_infoList[index].m_calGyr != CMT_DATA_ITEM_NOT_AVAILABLE)
		m_msg.setDataFPValue(CHECKIFDOUBLE(m_calGyr), bare, m_infoList[index].m_calGyr, 3);
	bare += 3;
	if (m_infoList[index].m_calMag != CMT_DATA_ITEM_NOT_AVAILABLE)
		m_msg.setDataFPValue(CHECKIFDOUBLE(m_calMag), bare, m_infoList[index].m_calMag, 3);

	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Orientation component of a data item as a Quaternion.
CmtQuat Packet::getOriQuat(const uint16_t index) const
{
	CmtQuat buffer;
	if (containsOriQuat(index))
		m_msg.getDataFPValue(&buffer.m_data[0], CHECKIFDOUBLE(m_oriQuat), m_infoList[index].m_oriQuat, 4);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}

bool Packet::containsOriQuat(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_oriQuat == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateOriQuat(const CmtQuat& data, const uint16_t index)
{
	const uint16_t numValues = 4;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_oriQuat == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_oriQuat))
	{
		// add
		ds = 8;	// added values are always in double precision
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_oriQuat = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;

		double* bare = (double*) &data;
		m_msg.setDataFPValue((m_formatList[index].m_outputSettings & CMT_OUTPUTSETTINGS_DATAFORMAT_MASK)|CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE,
							bare, m_infoList[index].m_oriQuat, numValues);
		return true;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_oriQuat), &data.m_data[0], m_infoList[index].m_oriQuat, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Orientation component of a data item as CmtEuler angles.
CmtEuler Packet::getOriEuler(const uint16_t index) const
{
	CmtEuler buffer;
	if (containsOriEuler(index))
		m_msg.getDataFPValue(&buffer.m_roll, CHECKIFDOUBLE(m_oriEul), m_infoList[index].m_oriEul, 3);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsOriEuler(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_oriEul == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateOriEuler(const CmtEuler& data, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_oriEul == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_oriEul))
	{
		// add
		ds = 8;	// added values are always in double precision
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_oriEul = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_oriEul), &data.m_roll, m_infoList[index].m_oriEul, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Orientation component of a data item as an Orientation Matrix.
CmtMatrix Packet::getOriMatrix(const uint16_t index) const
{
	CmtMatrix buffer;
	uint16_t k = 0;
	if (containsOriMatrix(index))
	{
		uint16_t ds = getFPValueSize(index);
		for (int32_t i=0;i<3;++i)
			for (int32_t j=0;j<3;++j, k+=ds)
				buffer.m_data[i][j] = m_msg.getDataFPValue(CHECKIFDOUBLE(m_oriMat), m_infoList[index].m_oriMat+k);
	}	
	else
		memset(&buffer, 0, sizeof(buffer));

	return buffer;
}
bool Packet::containsOriMatrix(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_oriMat == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateOriMatrix(const CmtMatrix& data, const uint16_t index)
{
	const uint16_t numValues = 9;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_oriMat == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_oriMat))
	{
		// add
		ds = 8;	// added values are always in double precision
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_oriMat = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	uint16_t k = 0;
	for (int32_t i=0;i<3;++i)
		for (int32_t j=0;j<3;++j, k+=ds)
			m_msg.setDataFPValue(CHECKIFDOUBLE(m_oriMat), data.m_data[i][j], m_infoList[index].m_oriMat+k);
	return true;
}

bool Packet::containsOri(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_oriEul == CMT_DATA_ITEM_NOT_AVAILABLE &&
		m_infoList[index].m_oriMat == CMT_DATA_ITEM_NOT_AVAILABLE &&
		m_infoList[index].m_oriQuat == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the AnalogIn 1 component of a data item.
CmtAnalogInData Packet::getAnalogIn1(const uint16_t index) const
{
	CmtAnalogInData buffer;
	if (containsAnalogIn1(index))
		buffer.m_data = m_msg.getDataShort(m_infoList[index].m_analogIn1);

	return buffer;
}
bool Packet::containsAnalogIn1(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_analogIn1 == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateAnalogIn1(const CmtAnalogInData& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_analogIn1 == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_analogIn1 = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(data.m_data, m_infoList[index].m_analogIn1);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the AnalogIn 2 component of a data item.
CmtAnalogInData Packet::getAnalogIn2(const uint16_t index) const
{
	CmtAnalogInData buffer;
	if (containsAnalogIn2(index))
		buffer.m_data = m_msg.getDataShort(m_infoList[index].m_analogIn2);

	return buffer;
}
bool Packet::containsAnalogIn2(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_analogIn2 == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateAnalogIn2(const CmtAnalogInData& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_analogIn2 == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_analogIn2 = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(data.m_data, m_infoList[index].m_analogIn2);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Position LLA component of a data item.
CmtVector Packet::getPositionLLA(const uint16_t index) const
{
	CmtVector buffer;
	if (containsPositionLLA(index))
		m_msg.getDataFPValue(&buffer.m_data[0], CHECKIFDOUBLE(m_posLLA), m_infoList[index].m_posLLA, 3);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsPositionLLA(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_posLLA == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updatePositionLLA(const CmtVector& data, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_posLLA == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_posLLA))
	{
		// add
		ds = 8;	// added values are always in double precision
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_posLLA = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_posLLA), &data.m_data[0], m_infoList[index].m_posLLA, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Velocity NED component of a data item.
CmtVector Packet::getVelocity(const uint16_t index) const
{
	CmtVector buffer;
	if (containsVelocity(index))
		m_msg.getDataFPValue(&buffer.m_data[0], CHECKIFDOUBLE(m_velNEDorNWU), m_infoList[index].m_velNEDorNWU, 3);
	else
		memset(&buffer, 0, sizeof(buffer));

	return buffer;
}
bool Packet::containsVelocity(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_velNEDorNWU == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateVelocity(const CmtVector& data, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_velNEDorNWU == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_posLLA))
	{
		// add
		ds = 8;	// added values are always in double precision
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_velNEDorNWU = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_velNEDorNWU), &data.m_data[0], m_infoList[index].m_velNEDorNWU, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Status component of a data item.
uint8_t Packet::getStatus(const uint16_t index) const
{
	if (containsStatus(index))
		return m_msg.getDataByte(m_infoList[index].m_status);
	return 0;
}
bool Packet::containsStatus(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_status == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateStatus(const uint8_t data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;

	if (m_infoList[index].m_status == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_status = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 1);
		m_infoList[index].m_size += 1;
	}
	// update
	m_msg.setDataByte(data,m_infoList[index].m_status);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Sample Counter component of the packet.
uint16_t Packet::getSampleCounter(const uint16_t index) const
{
	if (!containsSampleCounter(index))
		return 0;
	return m_msg.getDataShort(m_infoList[index].m_sc);
}
bool Packet::containsSampleCounter(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_sc == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateSampleCounter(const uint16_t counter, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_sc == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_sc = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(counter, m_infoList[index].m_sc);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the UTC Time component of the packet.
CmtUtcTime Packet::getUtcTime(const uint16_t index) const
{
	CmtUtcTime buffer;
	if (containsUtcTime(index))
	{
		buffer.m_nano = m_msg.getDataLong(m_infoList[index].m_utcNano);
		buffer.m_year = m_msg.getDataShort(m_infoList[index].m_utcYear);

		// month, day, hour, minute, second and valid
		uint8_t *bareByte = (uint8_t*) &buffer.m_month;
		for (uint16_t i=0; i < 6; ++i)
			bareByte[i] = m_msg.getDataByte(m_infoList[index].m_utcMonth + i);
	}
	return buffer;
}
bool Packet::containsUtcTime(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_utcTime == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateUtcTime(const CmtUtcTime& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_utcTime == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_utcTime = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 12);
		m_infoList[index].m_utcNano = m_infoList[index].m_utcTime;
		m_infoList[index].m_utcYear = m_infoList[index].m_utcTime + 4;
		m_infoList[index].m_utcMonth = m_infoList[index].m_utcTime + 6;
		m_infoList[index].m_utcDay = m_infoList[index].m_utcTime + 7;
		m_infoList[index].m_utcHour = m_infoList[index].m_utcTime + 8;
		m_infoList[index].m_utcMinute = m_infoList[index].m_utcTime + 9;
		m_infoList[index].m_utcSecond = m_infoList[index].m_utcTime + 10;
		m_infoList[index].m_utcValid = m_infoList[index].m_utcTime + 11;
		
		m_infoList[index].m_size += 12;
	}
	// update
	m_msg.setDataLong(data.m_nano, m_infoList[index].m_utcNano);
	m_msg.setDataShort(data.m_year, m_infoList[index].m_utcYear);

	// month, day, hour, minute, second and valid
	int8_t* bareByte = (int8_t*)&data.m_month;
	for (uint16_t i=0; i<6;++i)
		m_msg.setDataByte(bareByte[i],m_infoList[index].m_utcMonth + i);

	return true;
}

TimeStamp Packet::getRtc(const uint16_t index) const
{
	(void)index;
	return m_rtc;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the XKF-3 Acc G component of the packet.
CmtVector Packet::getAccG(const uint16_t index) const
{
	CmtVector buffer;
	if (containsAccG(index))
		m_msg.getDataFPValue(&buffer.m_data[0], CHECKIFDOUBLE(m_acc_g), m_infoList[index].m_acc_g, 3);
	else
		memset(&buffer, 0, sizeof(buffer));

	return buffer;
}
bool Packet::containsAccG(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_acc_g == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateAccG(const CmtVector& g, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_acc_g == CMT_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_acc_g))
	{
		// add
		ds = 8;	// added values are always in double precision
		m_msg.m_autoUpdateChecksum = false;

		m_infoList[index].m_acc_g = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_acc_g), &g.m_data[0], m_infoList[index].m_acc_g, numValues);
	return true;
}

Packet::Packet(const Packet& pack)
{
	PACKETLOG("Create new packet from Packet %p\n",&pack);

	m_itemCount = 0;
	m_formatList = NULL;
	m_infoList = NULL;
	*this = pack;
}

void Packet::operator = (const Packet& pack)
{
	PACKETLOG("Copy packet from Packet %p\n",this);

	if (m_itemCount != pack.m_itemCount)
	{
		LSTCHKDELNUL(m_formatList);
		m_itemCount = pack.m_itemCount;
		m_formatList = new CmtDataFormat[m_itemCount];
	}
	
	LSTCHKDELNUL(m_infoList);
	m_infoList = new PacketInfo[m_itemCount];
	for (uint16_t i = 0; i < m_itemCount; ++i)
	{
		m_formatList[i] = pack.m_formatList[i];
		m_infoList[i] = pack.m_infoList[i];
	}
	
	m_toa = pack.m_toa;
	m_rtc = pack.m_rtc;
	m_msg = pack.m_msg;
	m_xm = pack.m_xm;
}


} // end of xsens namespace
